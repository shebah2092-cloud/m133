#include "mhe_estimator.h"

#include <cstring>
#include <cstdio>
#include <cmath>
#include <drivers/drv_hrt.h>

#ifndef MODULE_NAME
# define MODULE_NAME "rocket_mpc"
#endif
#include <px4_platform_common/log.h>

// ===================================================================
MheEstimator::~MheEstimator()
{
	destroy();
}

void MheEstimator::destroy()
{
	if (_capsule) {
		m130_mhe_acados_free(_capsule);
		m130_mhe_acados_free_capsule(_capsule);
		_capsule = nullptr;
	}

	_solver_ready = false;
}

bool MheEstimator::init(const MheConfig &cfg)
{
	_cfg = cfg;

	_capsule = m130_mhe_acados_create_capsule();

	if (!_capsule) {
		PX4_ERR("MHE create_capsule failed");
		return false;
	}

	int status = m130_mhe_acados_create(_capsule);

	if (status != 0) {
		PX4_ERR("MHE acados_create returned %d", status);
		m130_mhe_acados_free_capsule(_capsule);
		_capsule = nullptr;
		return false;
	}

	_nlp_config = m130_mhe_acados_get_nlp_config(_capsule);
	_nlp_dims   = m130_mhe_acados_get_nlp_dims(_capsule);
	_nlp_in     = m130_mhe_acados_get_nlp_in(_capsule);
	_nlp_out    = m130_mhe_acados_get_nlp_out(_capsule);

	_solver_ready = true;

	// Initialize last_valid to zero
	memset(&_last_valid, 0, sizeof(_last_valid));
	_last_valid.quality = 0.0f;
	_last_valid.valid = false;

	PX4_INFO("MHE solver init N=%d dt=%.3f",
		 _cfg.horizon_steps, (double)_cfg.horizon_dt);
	return true;
}

void MheEstimator::init_state(const double x0[MHE_NX])
{
	PX4_INFO("MHE init_state: ptrs config=%p dims=%p out=%p in=%p",
		 (void*)_nlp_config, (void*)_nlp_dims, (void*)_nlp_out, (void*)_nlp_in);

	if (!_nlp_config || !_nlp_dims || !_nlp_out || !_nlp_in) {
		PX4_ERR("MHE null solver pointers in init_state");
		return;
	}

	PX4_INFO("MHE init_state: x0=[%.3f, %.3f, %.3f, ...] N=%d",
		 x0[0], x0[1], x0[2], _cfg.horizon_steps);

	memcpy(_x_bar, x0, MHE_NX * sizeof(double));

	// Warm-start all solver nodes with x0
	for (int k = 0; k <= _cfg.horizon_steps; k++) {
		ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "x", (void *)x0);
	}

	PX4_INFO("MHE init_state: x set");

	double u_zero[MHE_NU] = {};

	for (int k = 0; k < _cfg.horizon_steps; k++) {
		ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "u", u_zero);
	}

	PX4_INFO("MHE init_state: u set");

	// Seed last_valid with x0 as a best-guess state, but DO NOT mark it
	// valid: no solve has run yet, so downstream consumers must not treat
	// it as an authoritative estimate (would grant mhe_authority=true and
	// poison the EKF↔MHE blend before the first solve).
	memcpy(_last_valid.x_hat, x0, MHE_NX * sizeof(double));
	_last_valid.gyro_bias[0] = (float)x0[IX_BGX];
	_last_valid.gyro_bias[1] = (float)x0[IX_BGY];
	_last_valid.gyro_bias[2] = (float)x0[IX_BGZ];
	_last_valid.wind_ne[0]   = (float)x0[IX_WN];
	_last_valid.wind_ne[1]   = (float)x0[IX_WE];
	_last_valid.quality = 0.0f;
	_last_valid.valid   = false;
	_last_valid.status  = -1;
	_last_valid.solve_time_ms = 0.0f;

	_initialized = true;
	PX4_INFO("MHE state initialized (not valid until first solve)");
}

// ===================================================================
// Re-initialisation after in-flight divergence (XVAL reset).
// Unlike init_state() which assumes a fresh seed on an empty buffer,
// this variant wipes the sliding window and re-engages the startup
// ramp so the first post-recovery solve cannot promote mhe_authority
// with a half-filled or pre-divergence window.
// ===================================================================
void MheEstimator::reinit_state(const double x0[MHE_NX])
{
	// Wipe the sliding window so the first post-recovery solve does not
	// re-consume the measurements that caused the divergence.
	memset(_meas_buf,  0, sizeof(_meas_buf));
	memset(_param_buf, 0, sizeof(_param_buf));
	_meas_count   = 0;
	_param_count  = 0;
	_meas_head    = 0;
	_param_head   = 0;

	// Re-engage the startup ramp so mhe_authority cannot fire on the
	// first solve after recovery.
	_solve_count  = 0;
	_consec_fails = 0;
	_last_solve_t = -1.0f;

	// Delegate x_bar seed, solver warm-start, and last_valid handling.
	init_state(x0);
}

// ===================================================================
// Full reset for re-arm — wipe all flight state, keep solver alive
// ===================================================================
void MheEstimator::reset()
{
	_initialized = false;
	_solve_count = 0;
	_consec_fails = 0;
	_last_solve_t = -1.0f;

	memset(_x_bar, 0, sizeof(_x_bar));
	memset(_meas_buf, 0, sizeof(_meas_buf));
	memset(_param_buf, 0, sizeof(_param_buf));
	_meas_count = 0;
	_param_count = 0;
	_meas_head = 0;
	_param_head = 0;

	memset(&_last_valid, 0, sizeof(_last_valid));
	_last_valid.quality = 0.0f;
	_last_valid.valid = false;

	if (_capsule) {
		m130_mhe_acados_reset(_capsule, 1);
	}

	PX4_INFO("MHE reset for re-arm");
}

// ===================================================================
// Ring buffer push
// ===================================================================
void MheEstimator::push_measurement(float t, const double y_meas[MHE_NMEAS])
{
	// Gap detection: the acados-generated MHE solver assumes every stage
	// is spaced by exactly horizon_dt (20 ms).  If measurements stop (e.g.
	// GPS drops for > GPS_STALE_TIMEOUT_US) and then resume, naively
	// indexing the ring buffer by count mixes pre-gap and post-gap samples
	// into a single window and the solver interprets the jump as giant
	// process noise, corrupting the estimate before quality can gate it.
	//
	// Threshold 3 x horizon_dt tolerates normal scheduler jitter while
	// catching any real discontinuity.  On detection we flush both the
	// measurement and parameter ring buffers (kept in lockstep by
	// RocketMPC.cpp) and invalidate the last output so downstream
	// consumers cannot promote a stale estimate to authority.
	if (_meas_count > 0) {
		int last_idx = (_meas_head - 1 + BUF_SIZE) % BUF_SIZE;
		float dt_actual = t - _meas_buf[last_idx].t;

		if (dt_actual > 3.0f * _cfg.horizon_dt) {
			// Snapshot validity BEFORE clearing it — the reseed block
			// below needs to know whether _last_valid.x_hat was
			// populated by a successful solve. Checking _last_valid.valid
			// after `_last_valid.valid = false` always reads false.
			const bool had_valid_estimate = _last_valid.valid;

			_meas_count   = 0;
			_meas_head    = 0;
			_param_count  = 0;
			_param_head   = 0;
			_last_valid.valid   = false;
			_last_valid.quality = 0.0f;
			_consec_fails = 0;
			_last_solve_t = -1.0f;

			// Re-seed the arrival cost mean.  Without this, the first solve
			// after the gap anchors the NEW window to the pre-gap arrival
			// state via P0 — up to hundreds of metres / tens of degrees of
			// stale bias depending on how long the gap lasted.  The
			// cross-validation / startup ramp gates will still suppress the
			// output until the solver re-converges, but the optimiser burns
			// several cycles fighting a phantom arrival target.
			//
			// Best available seed is the last valid estimate (the state we
			// had just before losing measurements).  If we never had one,
			// fall back to zero — the measurement likelihood alone will
			// dominate once `min_init_meas` samples accumulate.
			if (had_valid_estimate) {
				memcpy(_x_bar, _last_valid.x_hat, MHE_NX * sizeof(double));
			} else {
				memset(_x_bar, 0, sizeof(_x_bar));
			}

			// Re-engage the startup ramp so mhe_authority cannot fire on
			// the first solve after the gap, matching reinit_state().
			_solve_count = 0;
		}
	}

	int idx = _meas_head % BUF_SIZE;
	_meas_buf[idx].t = t;
	memcpy(_meas_buf[idx].y, y_meas, MHE_NMEAS * sizeof(double));
	_meas_head++;

	if (_meas_count < BUF_SIZE) { _meas_count++; }
}

void MheEstimator::push_params(float t, const double params[MHE_NP])
{
	int idx = _param_head % BUF_SIZE;
	_param_buf[idx].t = t;
	memcpy(_param_buf[idx].p, params, MHE_NP * sizeof(double));
	_param_head++;

	if (_param_count < BUF_SIZE) { _param_count++; }
}

// ===================================================================
// Frozen output
// ===================================================================
MheOutput MheEstimator::_frozen_output() const
{
	MheOutput out;

	if (_last_valid.valid) {
		out = _last_valid;

	} else {
		memset(&out, 0, sizeof(out));
	}

	out.quality = 0.0f;
	out.valid   = false;
	out.status  = -1;
	return out;
}

// ===================================================================
// Main update
// ===================================================================
MheOutput MheEstimator::update(float t)
{
	if (!_initialized) { return _frozen_output(); }

	// Rate limiting — return last valid estimate (or frozen if none yet).
	// Before the first successful solve _last_valid.valid is false, so this
	// naturally returns a non-authoritative output.
	float solve_period = 1.0f / _cfg.solve_rate_hz;

	if (_last_solve_t >= 0.0f && (t - _last_solve_t) < solve_period * 0.9f) {
		return _last_valid;
	}

	// Not enough measurements yet to run a solve — explicitly return a
	// frozen (valid=false, quality=0) output so consumers don't mistake
	// the seeded x0 for an authoritative estimate.
	if (_meas_count < _cfg.min_init_meas) { return _frozen_output(); }

	// Determine how many measurements to use
	int n_use = _meas_count;

	if (n_use > _cfg.horizon_steps + 1) {
		n_use = _cfg.horizon_steps + 1;
	}

	int N_use = n_use - 1; // number of intervals

	// Get buffer indices for the window
	int start_idx = (_meas_head - n_use);

	if (start_idx < 0) { start_idx += BUF_SIZE; }

	// Stage 0: arrival cost + measurement + noise
	{
		int idx0 = start_idx % BUF_SIZE;
		double yref_0[MHE_NY0]; // y_meas[13] + w_noise[17] + x_bar[17] = 47
		memset(yref_0, 0, sizeof(yref_0));
		memcpy(yref_0, _meas_buf[idx0].y, MHE_NMEAS * sizeof(double));
		// w_noise = zeros → already zero
		memcpy(&yref_0[MHE_NMEAS + MHE_NU], _x_bar, MHE_NX * sizeof(double));
		ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, 0, "yref", yref_0);
	}

	// Stages 1..horizon_steps-1: measurement + noise
	// Fill ALL intermediate stages so the NLP cost is well-defined.
	// For stages beyond our measurement window, repeat last measurement.
	for (int k = 1; k < _cfg.horizon_steps; k++) {
		double yref_k[MHE_NY]; // y_meas[13] + w_noise[17] = 30
		memset(yref_k, 0, sizeof(yref_k));

		if (k < n_use) {
			int idx_k = (start_idx + k) % BUF_SIZE;
			memcpy(yref_k, _meas_buf[idx_k].y, MHE_NMEAS * sizeof(double));
		} else {
			// Use last available measurement
			int idx_last = (_meas_head - 1 + BUF_SIZE) % BUF_SIZE;
			memcpy(yref_k, _meas_buf[idx_last].y, MHE_NMEAS * sizeof(double));
		}

		ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, k, "yref", yref_k);
	}

	// Parameters for each interval — ALIGNED WITH THE MEASUREMENT WINDOW.
	//
	// push_measurement() and push_params() are called atomically from
	// RocketMPC::Run() inside the same `if (smeas.valid)` block, and the
	// gap detector in push_measurement() flushes both ring buffers
	// together, so _meas_head == _param_head and _meas_count == _param_count
	// at all times.  That means the correct start index for params is the
	// same `start_idx` used for the measurements above.
	//
	// The old code used `_param_head - _param_count` (the OLDEST sample in
	// the buffer), which drifts up to (_param_count - n_use) * horizon_dt
	// away from the measurement window.  Once the 64-slot buffer is
	// saturated and n_use == horizon_steps + 1 == 21, the offset is
	// 43 * 20 ms = 860 ms: stage-0 params (δ_e_act/δ_r_act/δ_a_act, mass,
	// thrust, inertias) came from ~860 ms before the stage-0 measurement,
	// so the solver evaluated aero forces with fin positions that didn't
	// match the observation.  That biased α/β/b_gyro and inflated w_norm,
	// pushing quality below qg_thr for no real reason.
	//
	// Stages [0, n_use-1] now take time-aligned params; stages beyond the
	// measurement count (only reachable if min_init_meas < horizon_steps+1)
	// repeat the newest params, mirroring the measurement loop above.
	for (int k = 0; k < _cfg.horizon_steps; k++) {
		double p_k[MHE_NP];

		if (k < n_use) {
			int p_idx = (start_idx + k) % BUF_SIZE;
			memcpy(p_k, _param_buf[p_idx].p, MHE_NP * sizeof(double));

		} else if (_param_count > 0) {
			int p_last = (_param_head - 1 + BUF_SIZE) % BUF_SIZE;
			memcpy(p_k, _param_buf[p_last].p, MHE_NP * sizeof(double));

		} else {
			memset(p_k, 0, sizeof(p_k));
		}

		m130_mhe_acados_update_params(_capsule, k, p_k, MHE_NP);
	}

	// Terminal stage params
	{
		int p_last = (_param_head - 1) % BUF_SIZE;

		if (p_last < 0) { p_last += BUF_SIZE; }

		double p_term[MHE_NP];

		if (_param_count > 0) {
			memcpy(p_term, _param_buf[p_last].p, MHE_NP * sizeof(double));
		} else {
			memset(p_term, 0, sizeof(p_term));
		}

		m130_mhe_acados_update_params(_capsule, _cfg.horizon_steps, p_term, MHE_NP);
	}

	// Solve
	hrt_abstime t0 = hrt_absolute_time();
	int status = m130_mhe_acados_solve(_capsule);
	float solve_ms = (float)(hrt_absolute_time() - t0) * 1e-3f;

	_last_solve_t = t;

	// Extract estimate from final node
	double x_hat[MHE_NX];
	ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out,
			_cfg.horizon_steps, "x", x_hat);

	// Check finite
	bool finite_ok = true;

	for (int i = 0; i < MHE_NX; i++) {
		if (!std::isfinite(x_hat[i])) {
			finite_ok = false;
			break;
		}
	}

	if (!finite_ok) {
		_consec_fails++;
		PX4_WARN("MHE NaN at t=%.3f, fail #%d", (double)t, _consec_fails);

		if (_consec_fails >= _cfg.max_consec_fails) {
			PX4_ERR("MHE frozen: too many consecutive failures");
			return _frozen_output();
		}

		// Mid-streak recovery: once consecutive failures pass half the
		// hard limit, re-seed the arrival cost from the last valid
		// estimate (the NaN x_hat from this solve is unusable) and
		// re-engage the startup ramp.  Without this the optimiser keeps
		// linearising around the same bad x_bar every cycle and the NaN
		// streak can ride all the way to max_consec_fails, freezing MHE
		// for the rest of the flight.
		const int reseed_threshold = _cfg.max_consec_fails / 2;

		if (reseed_threshold > 0 && _consec_fails == reseed_threshold) {
			if (_last_valid.valid) {
				memcpy(_x_bar, _last_valid.x_hat, MHE_NX * sizeof(double));
				// Re-warm-start all solver nodes from the last good
				// estimate so SQP linearises from a finite point
				// instead of whatever junk the NaN solve left behind.
				for (int k = 0; k <= _cfg.horizon_steps; k++) {
					ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out,
							_nlp_in, k, "x", (void *)_x_bar);
				}
				double u_zero[MHE_NU] = {};
				for (int k = 0; k < _cfg.horizon_steps; k++) {
					ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out,
							_nlp_in, k, "u", u_zero);
				}
			}
			_solve_count = 0;
			PX4_WARN("MHE mid-streak reseed at fail #%d", _consec_fails);
		}

		// Progressive quality decay on consecutive failures.
		// CRITICAL: also mark the output INVALID so downstream control code
		// falls back to EKF immediately.  Any non-finite solve means we have
		// no fresh estimate; returning stale x_hat with valid=true would let
		// MPC solve on a frozen pose (e.g. ~6 m position error per 20 ms
		// stale cycle at 300 m/s).  Quality is still decayed for telemetry
		// continuity, but `valid` is the authoritative gate.
		MheOutput degraded = _last_valid;
		degraded.valid = false;
		float decay = 1.0f - (float)_consec_fails / (float)_cfg.max_consec_fails;

		if (decay < 0.0f) { decay = 0.0f; }

		degraded.quality *= decay;
		degraded.status  = status;
		degraded.solve_time_ms = solve_ms;

		return degraded;
	}

	// Update arrival cost: shift window
	if (N_use >= 2) {
		ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, 1, "x", _x_bar);
	}

	// Quality metric
	double w_norm_sq = 0.0;

	for (int k = 0; k < N_use && k < _cfg.horizon_steps; k++) {
		double wk[MHE_NU];
		ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, k, "u", wk);

		for (int j = 0; j < MHE_NU; j++) {
			w_norm_sq += wk[j] * wk[j];
		}
	}

	double w_norm = sqrt(w_norm_sq / (double)(N_use > 0 ? N_use : 1));

	float quality = 1.0f;

	if (status == 3 || status == 4) {
		quality *= 0.5f;
	}

	if (w_norm > 5.0) {
		float q_fac = 1.0f - (float)(w_norm - 5.0) / 15.0f;

		if (q_fac < 0.1f) { q_fac = 0.1f; }

		quality *= q_fac;
	}

	// Bound saturation check
	static constexpr double WIND_BOUND = 5.0;
	static constexpr double GBIAS_BOUND = 0.03;   // matches acados solver bound (±0.03 rad/s)
	static constexpr double SAT_THRESH = 0.90;

	double wind_sat = fmax(fabs(x_hat[IX_WN]), fabs(x_hat[IX_WE])) / WIND_BOUND;
	double gbias_sat = fmax(fmax(fabs(x_hat[IX_BGX]), fabs(x_hat[IX_BGY])),
				fabs(x_hat[IX_BGZ])) / GBIAS_BOUND;
	double max_sat = fmax(wind_sat, gbias_sat);

	if (max_sat > SAT_THRESH) {
		float sat_penalty = 1.0f - (float)((max_sat - SAT_THRESH) / (1.0 - SAT_THRESH)) * 0.7f;

		if (sat_penalty < 0.3f) { sat_penalty = 0.3f; }

		quality *= sat_penalty;
	}

	// Startup ramp
	_solve_count++;

	if (_solve_count <= _cfg.startup_ramp_solves) {
		float ramp = (float)_solve_count / (float)_cfg.startup_ramp_solves;
		quality *= ramp;
	}

	_consec_fails = 0;

	// Build output
	MheOutput output;
	memcpy(output.x_hat, x_hat, MHE_NX * sizeof(double));
	output.gyro_bias[0] = (float)x_hat[IX_BGX];
	output.gyro_bias[1] = (float)x_hat[IX_BGY];
	output.gyro_bias[2] = (float)x_hat[IX_BGZ];
	output.wind_ne[0]   = (float)x_hat[IX_WN];
	output.wind_ne[1]   = (float)x_hat[IX_WE];
	output.quality      = quality;
	output.status       = status;
	output.solve_time_ms = solve_ms;
	output.valid        = true;

	_last_valid = output;
	return output;
}
