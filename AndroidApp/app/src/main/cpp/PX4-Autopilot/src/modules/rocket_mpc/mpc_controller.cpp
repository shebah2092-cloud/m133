#include "mpc_controller.h"

#include <cstring>
#include <cstdio>
#include <drivers/drv_hrt.h>

#ifndef MODULE_NAME
# define MODULE_NAME "rocket_mpc"
#endif
#include <px4_platform_common/log.h>

// Helper: Hermite smooth step  3s² - 2s³
static inline float hermite(float s)
{
	s = (s < 0.0f) ? 0.0f : ((s > 1.0f) ? 1.0f : s);
	return 3.0f * s * s - 2.0f * s * s * s;
}

// ===================================================================
MpcController::~MpcController()
{
	destroy();
}

void MpcController::destroy()
{
	if (_capsule) {
		m130_rocket_acados_free(_capsule);
		m130_rocket_acados_free_capsule(_capsule);
		_capsule = nullptr;
	}

	_solver_ready = false;
}

bool MpcController::init(const MpcConfig &cfg)
{
	_cfg = cfg;

	_capsule = m130_rocket_acados_create_capsule();

	if (!_capsule) {
		PX4_ERR("MPC create_capsule failed");
		return false;
	}

	int status = m130_rocket_acados_create(_capsule);

	if (status != 0) {
		PX4_ERR("MPC acados_create returned %d", status);
		m130_rocket_acados_free_capsule(_capsule);
		_capsule = nullptr;
		return false;
	}

	_nlp_config = m130_rocket_acados_get_nlp_config(_capsule);
	_nlp_dims   = m130_rocket_acados_get_nlp_dims(_capsule);
	_nlp_in     = m130_rocket_acados_get_nlp_in(_capsule);
	_nlp_out    = m130_rocket_acados_get_nlp_out(_capsule);
	_nlp_solver = m130_rocket_acados_get_nlp_solver(_capsule);

	_solver_ready = true;
	PX4_INFO("MPC solver init N=%d tf=%.1f", _cfg.N_horizon, (double)_cfg.tf);
	return true;
}

// ===================================================================
// Runtime parameters: [mass, thrust]
// ===================================================================
void MpcController::get_params(float t, double p[MPC_NP]) const
{
	float mass, thrust;

	if (t < _cfg.burn_time) {
		float f = t / _cfg.burn_time;
		mass = _cfg.mass_full - f * (_cfg.mass_full - _cfg.mass_dry);
		float t_tailoff_start = _cfg.burn_time - _cfg.t_tail;

		if (t > t_tailoff_start) {
			float s = (t - t_tailoff_start) / _cfg.t_tail;
			thrust = _cfg.thrust_plateau * (1.0f - s) * (1.0f - s) * (1.0f - s);

		} else {
			thrust = _cfg.thrust_plateau;
		}

	} else {
		mass = _cfg.mass_dry;
		thrust = 0.0f;
	}

	p[0] = (double)mass;
	p[1] = (double)thrust;
}

// ===================================================================
// Weight scheduling — exact port from Python _update_weights_for_phase
// ===================================================================
void MpcController::_compute_weights(float t, float gamma, float gamma_ref_prev,
				     float phi,
				     float alpha, float q_rate, float x_pos,
				     bool cruise_alt_set, float h_ref_scaled,
				     double W[MPC_NY], double W_e[MPC_NYN])
{
	const bool is_boost = (t < _cfg.burn_time);
	// Weight-blend window scales with t_tail so the transition tracks the
	// thrust curve in get_params() (thrust tail-off spans exactly t_tail).
	// Post-burnout the window is 2*t_tail to let weights settle smoothly.
	const float t_tailoff_start = _cfg.burn_time - _cfg.t_tail;
	const float t_tailoff_end   = _cfg.burn_time + 2.0f * _cfg.t_tail;
	const bool in_tailoff = (t_tailoff_start < t) && (t < t_tailoff_end);

	float gamma_base, q_w, r_w, de_rate_w, dr_rate_w, alpha_w_boost;

	if (is_boost && !in_tailoff) {
		gamma_base   = 250.0f;
		q_w          = 200.0f;
		r_w          = 80.0f;
		de_rate_w    = 120.0f;
		dr_rate_w    = 60.0f;
		alpha_w_boost = 120.0f;

	} else if (in_tailoff) {
		float s_to = (t - t_tailoff_start) / (t_tailoff_end - t_tailoff_start);
		s_to = (s_to < 0.0f) ? 0.0f : ((s_to > 1.0f) ? 1.0f : s_to);
		const float s_peak = 0.67f;
		float s_up = s_to / s_peak;
		s_up = (s_up > 1.0f) ? 1.0f : s_up;

		float gamma_act = 250.0f * (1.0f - s_up) * (1.0f - s_up) + 2.0f;
		float q_act     = 200.0f + 800.0f * s_up;
		float r_act     = 80.0f + 200.0f * s_up;
		float de_act    = 120.0f * (1.0f - s_up) + 200.0f * s_up;
		float dr_act    = 60.0f + 100.0f * s_up;
		float alpha_act = 120.0f + 800.0f * s_up;

		if (s_to > s_peak) {
			float s_f = (s_to - s_peak) / (1.0f - s_peak);
			float h = hermite(s_f);
			gamma_base    = gamma_act + (100.0f - gamma_act) * h;
			q_w           = q_act     + (60.0f  - q_act)     * h;
			r_w           = r_act     + (40.0f  - r_act)     * h;
			de_rate_w     = de_act    + (20.0f  - de_act)    * h;
			dr_rate_w     = dr_act    + (20.0f  - dr_act)    * h;
			alpha_w_boost = alpha_act + (40.0f  - alpha_act) * h;

		} else {
			gamma_base    = gamma_act;
			q_w           = q_act;
			r_w           = r_act;
			de_rate_w     = de_act;
			dr_rate_w     = dr_act;
			alpha_w_boost = alpha_act;
		}

	} else {
		// Coast/terminal
		gamma_base    = 100.0f;
		q_w           = 60.0f;
		r_w           = 40.0f;
		de_rate_w     = 20.0f;
		dr_rate_w     = 20.0f;
		alpha_w_boost = 40.0f;
	}

	// Adaptive gamma error boost
	// NOTE: This must be the tracking error |gamma - gamma_ref|, NOT |gamma|.
	// Python reference: m130_mpc_autopilot.py:644
	//   gamma_err_deg = abs(math.degrees(gamma - self._gamma_ref_prev))
	float gamma_err_deg = fabsf((gamma - gamma_ref_prev) * 180.0f / (float)M_PI);
	float alpha_deg_abs = fabsf(alpha * 180.0f / (float)M_PI);
	float q_rate_abs    = fabsf(q_rate * 180.0f / (float)M_PI);

	if (is_boost && !in_tailoff) {
		if (gamma_err_deg < 5.0f && q_rate_abs > 3.0f) {
			q_w = q_w + 300.0f * (1.0f - gamma_err_deg / 5.0f);
		}

		if (alpha_deg_abs > 1.0f) {
			float a_sc = alpha_deg_abs / 3.0f;
			alpha_w_boost = alpha_w_boost + 200.0f * ((a_sc > 3.0f) ? 3.0f : a_sc);
		}
	}

	// Altitude cruise weight: active after burnout when cruise alt is captured,
	// fades out during dive phase (matches Python baseline).
	float h_cruise_w = 0.0f;
	if (!is_boost && cruise_alt_set) {
		float t_since_burn = t - _cfg.burn_time;
		float early_boost = (1.0f - t_since_burn / 3.0f);
		if (early_boost < 0.0f) { early_boost = 0.0f; }

		// Compute s_dive from progress (same as LOS guidance)
		float dx_safe = _cfg.target_x - x_pos;
		if (dx_safe < 0.5f) { dx_safe = 0.5f; }
		float progress = 1.0f - dx_safe / ((_cfg.target_x > 1.0f) ? _cfg.target_x : 1.0f);
		progress = (progress < 0.0f) ? 0.0f : ((progress > 1.0f) ? 1.0f : progress);

		// Python reference: m130_mpc_autopilot.py:530
		//   p1 = min(p0 + 0.10, 0.95)  # 10% window, clipped at 95%
		float cp0 = _cfg.cruise_progress;
		float cp1 = cp0 + 0.10f;
		if (cp1 > 0.95f) { cp1 = 0.95f; }
		float s_dive;
		if (progress < cp0) { s_dive = 0.0f; }
		else if (progress > cp1) { s_dive = 1.0f; }
		else {
			float sf = (progress - cp0) / (cp1 - cp0);
			s_dive = hermite(sf);
		}

		h_cruise_w = (15.0f + 35.0f * early_boost) * (1.0f - s_dive);
	}

	// Roll recovery weighting
	const float roll_recovery_start = 5.0f * (float)M_PI / 180.0f;
	const float roll_recovery_full  = 20.0f * (float)M_PI / 180.0f;
	float roll_abs = fabsf(phi);
	float roll_recovery;

	if (roll_abs <= roll_recovery_start) {
		roll_recovery = 0.0f;

	} else if (roll_abs >= roll_recovery_full) {
		roll_recovery = 1.0f;

	} else {
		roll_recovery = (roll_abs - roll_recovery_start) /
				(roll_recovery_full - roll_recovery_start);
	}

	float chi_w     = 25.0f > (120.0f * (1.0f - 0.55f * roll_recovery)) ?
			  25.0f : (120.0f * (1.0f - 0.55f * roll_recovery));
	float p_w       = 20.0f + 160.0f * roll_recovery;
	float phi_w     = 80.0f + 520.0f * roll_recovery;
	float da_rate_w = 10.0f - 7.0f * roll_recovery;

	// W: diagonal for [h, gamma, chi, p, q, r, alpha, beta, phi, de_rate, dr_rate, da_rate]
	W[0]  = (double)h_cruise_w;
	W[1]  = (double)(gamma_base + 400.0f);
	W[2]  = (double)chi_w;
	W[3]  = (double)p_w;
	W[4]  = (double)q_w;
	W[5]  = (double)r_w;
	W[6]  = (double)alpha_w_boost;
	W[7]  = 60.0;
	W[8]  = (double)phi_w;
	W[9]  = (double)de_rate_w;
	W[10] = (double)dr_rate_w;
	W[11] = (double)da_rate_w;

	// W_e: diagonal for [h, gamma, chi, p, q, r, alpha, beta, phi]
	W_e[0] = (double)(h_cruise_w * 0.5f);
	W_e[1] = 300.0;
	W_e[2] = (double)((40.0f > chi_w) ? 40.0f : chi_w);
	W_e[3] = (double)((40.0f > p_w) ? 40.0f : p_w);
	W_e[4] = 60.0;
	W_e[5] = 40.0;
	W_e[6] = 30.0;
	W_e[7] = 40.0;
	W_e[8] = (double)((40.0f > phi_w) ? 40.0f : phi_w);
}

// ===================================================================
// Forward guess for initial trajectory
// ===================================================================
void MpcController::_forward_guess(const double x0[MPC_NX], float t0)
{
	const int N = _cfg.N_horizon;
	const float dt_h = _cfg.tf / (float)N;

	memcpy(&_x_traj[0], x0, MPC_NX * sizeof(double));

	for (int k = 0; k < N; k++) {
		const double *x = &_x_traj[k * MPC_NX];
		double *xn = &_x_traj[(k + 1) * MPC_NX];
		float t_k = t0 + k * dt_h;

		double p[MPC_NP];
		get_params(t_k, p);
		float mass = (float)p[0];
		float thrust = (float)p[1];

		float V_k = (float)x[0];

		if (V_k < 10.0f) { V_k = 10.0f; }

		float gam = (float)x[1];
		float sg = sinf(gam);
		float cg = cosf(gam);
		float al = (float)x[6];

		float V_dot = (thrust * cosf(al) - 0.5f * 1.225f * V_k * V_k * 0.0133f * 0.30f
			       - mass * G_ACCEL * sg) / mass;
		float gam_dot = -G_ACCEL * cg / V_k + thrust * sinf(al) / (mass * V_k);

		memcpy(xn, x, MPC_NX * sizeof(double));
		xn[0] = (double)(V_k + V_dot * dt_h);

		if (xn[0] < 10.0) { xn[0] = 10.0; }

		xn[1] = (double)(gam + gam_dot * dt_h);

		for (int j = 3; j < 9; j++) { xn[j] *= 0.95; }

		xn[9]  = x[9]  + (double)(V_k * sg * 0.01f * dt_h);
		xn[10] = x[10] + (double)(V_k * cg * cosf((float)x[2]) * 0.001f * dt_h);
		xn[11] = x[11] + (double)(V_k * cg * sinf((float)x[2]) * 0.001f * dt_h);

		float tau = _cfg.tau_servo;
		if (tau < 1e-4f) { tau = 1e-4f; }   // guard against div-by-zero / NaN
		float decay = expf(-dt_h / tau);

		for (int j = 0; j < 3; j++) {
			xn[15 + j] = x[12 + j] + (x[15 + j] - x[12 + j]) * (double)decay;
		}
	}
}

// ===================================================================
// Full reset for re-arm — wipe all flight state
// ===================================================================
void MpcController::reset()
{
	if (_capsule) {
		m130_rocket_acados_reset(_capsule, 1);
	}

	_warm = false;
	_consec_fails = 0;
	_consec_ok = 0;
	_solve_count = 0;
	_last_delta_e = 0.0f;
	_last_delta_r = 0.0f;
	_last_delta_a = 0.0f;
	memset(_last_fins, 0, sizeof(_last_fins));
	_last_solve_t = -1.0f;
	_last_solve_time_ms = 0.0f;
	memset(_x_traj, 0, sizeof(_x_traj));

	PX4_INFO("MPC reset for re-arm");
}

// ===================================================================
// Reinitialize solver after consecutive failures
// ===================================================================
void MpcController::_reinit(const double x_mpc[MPC_NX])
{
	m130_rocket_acados_reset(_capsule, 1);
	_forward_guess(x_mpc, _last_solve_t > 0.0f ? _last_solve_t : 0.0f);

	for (int k = 0; k <= _cfg.N_horizon; k++) {
		ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "x",
				&_x_traj[k * MPC_NX]);
	}

	double u_zero[MPC_NU] = {0.0, 0.0, 0.0};

	for (int k = 0; k < _cfg.N_horizon; k++) {
		ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "u", u_zero);
	}

	_warm = false;
	_consec_fails = 0;
	_consec_ok = 0;
	PX4_WARN("MPC solver reinitialized");
}

// ===================================================================
// Main solve
// ===================================================================
MpcSolveResult MpcController::solve(const double x_mpc[MPC_NX],
				    float gamma_ref, float chi_ref, float phi_ref,
				    float h_ref_scaled, bool cruise_alt_set,
				    float t_flight, float dt,
				    float x_pos, float altitude)
{
	MpcSolveResult result{};
	memcpy(result.fins, _last_fins, sizeof(_last_fins));
	result.delta_e = _last_delta_e;
	result.delta_r = _last_delta_r;
	result.delta_a = _last_delta_a;
	result.status = -1;
	result.valid = false;

	if (!_solver_ready) { return result; }

	const int N = _cfg.N_horizon;
	float V = (float)x_mpc[0];

	// Adaptive time horizon
	float dx_to_target = _cfg.target_x - x_pos;

	if (dx_to_target < 0.5f) { dx_to_target = 0.5f; }

	float tf_use = _cfg.tf;

	if (V > 30.0f && dx_to_target > 0.0f) {
		float t_to_target = dx_to_target / V;
		tf_use = t_to_target * 0.8f;

		if (tf_use < 2.0f) { tf_use = 2.0f; }

		if (tf_use > _cfg.tf) { tf_use = _cfg.tf; }
	}

	float dt_h = tf_use / (float)N;

	// Set initial state constraint
	if (_solve_count < 2) { PX4_DEBUG("MPC solve: N=%d V=%.1f tf=%.1f dt=%.4f", N, (double)V, (double)tf_use, (double)dt_h); }
	ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, _nlp_out, 0,
				      "lbx", (void *)x_mpc);
	ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, _nlp_out, 0,
				      "ubx", (void *)x_mpc);
	if (_solve_count < 2) { PX4_DEBUG("MPC constraints set"); }

	// Compute weight schedule
	double W_diag[MPC_NY];
	double W_e_diag[MPC_NYN];
	_compute_weights(t_flight, (float)x_mpc[1], gamma_ref, (float)x_mpc[8],
			 (float)x_mpc[6], (float)x_mpc[4], x_pos,
			 cruise_alt_set, h_ref_scaled,
			 W_diag, W_e_diag);

	// Build full diagonal weight matrices
	double W_mat[MPC_NY * MPC_NY];
	memset(W_mat, 0, sizeof(W_mat));

	for (int i = 0; i < MPC_NY; i++) { W_mat[i * MPC_NY + i] = W_diag[i]; }

	double W_e_mat[MPC_NYN * MPC_NYN];
	memset(W_e_mat, 0, sizeof(W_e_mat));

	for (int i = 0; i < MPC_NYN; i++) { W_e_mat[i * MPC_NYN + i] = W_e_diag[i]; }

	float h_ref = h_ref_scaled;   // cruise altitude from LOS (scaled)
	if (_solve_count < 2) { PX4_DEBUG("MPC weights computed"); }

	// Set stage references and parameters
	for (int k = 0; k < N; k++) {
		float t_k = t_flight + k * dt_h;
		double p[MPC_NP];
		get_params(t_k, p);
		m130_rocket_acados_update_params(_capsule, k, p, MPC_NP);

		// Cost weight
		ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, k, "W", W_mat);

		// yref = [h_ref, gamma_ref, chi_ref, 0,0,0, 0,0, phi_ref, 0,0,0]
		double yref[MPC_NY] = {};
		yref[0] = (double)h_ref;
		yref[1] = (double)gamma_ref;
		yref[2] = (double)chi_ref;
		yref[8] = (double)phi_ref;
		ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, k, "yref", yref);
	}

	// Terminal stage
	{
		double p_term[MPC_NP];
		get_params(t_flight + tf_use, p_term);
		m130_rocket_acados_update_params(_capsule, N, p_term, MPC_NP);

		ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, N, "W", W_e_mat);

		double yref_e[MPC_NYN] = {};
		yref_e[0] = (double)h_ref;
		yref_e[1] = (double)gamma_ref;
		yref_e[2] = (double)chi_ref;
		yref_e[8] = (double)phi_ref;
		ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, N, "yref", yref_e);
	}

	if (_solve_count < 2) { PX4_DEBUG("MPC params+costs set for all stages"); }

	// Update time steps
	double new_time_steps[MPC_N];

	for (int k = 0; k < N; k++) { new_time_steps[k] = (double)dt_h; }

	int ts_status = m130_rocket_acados_update_time_steps(_capsule, N, new_time_steps);

	if (ts_status != 0) {
		// Silent failure would leave the integrator on stale dt values
		// while upstream logic (cost scaling, trajectory prediction) uses
		// the new dt_h — the solver reports success on a physically
		// inconsistent problem. Count as a fail and log with throttling.
		_consec_fails++;

		if (_consec_fails <= 3 || (_consec_fails % 50) == 0) {
			PX4_WARN("MPC update_time_steps failed status=%d dt_h=%.4f N=%d",
				 ts_status, (double)dt_h, N);
		}
	}

	if (_solve_count < 2) { PX4_DEBUG("MPC time_steps updated"); }

	// Warm start / initialization
	if (!_warm) {
		_forward_guess(x_mpc, t_flight);

		for (int k = 0; k <= N; k++) {
			ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "x",
					&_x_traj[k * MPC_NX]);
		}

		double u_zero[MPC_NU] = {0.0, 0.0, 0.0};

		for (int k = 0; k < N; k++) {
			ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "u", u_zero);
		}

	} else {
		// Shift trajectory one step
		double x_buf[MPC_NX];

		for (int k = 0; k < N - 1; k++) {
			ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, k + 1, "x", x_buf);
			ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "x", x_buf);

			double u_buf[MPC_NU];
			ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, k + 1, "u", u_buf);
			ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "u", u_buf);
		}

		ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, N, "x", x_buf);
		ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, N - 1, "x", x_buf);
	}

	// Set current state
	ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, 0, "x", (void *)x_mpc);
	if (_solve_count < 2) { PX4_DEBUG("MPC init guess set, warm=%d", _warm); }

	// RTI iterations
	int n_rti = _warm ? 3 : 5;

	if (_consec_fails > 3) { n_rti = 8; }

	hrt_abstime t0 = hrt_absolute_time();
	bool ok = true;
	int status = 0;

	for (int i = 0; i < n_rti; i++) {
		if (_solve_count < 2) { PX4_DEBUG("MPC RTI iter %d/%d", i, n_rti); }
		status = m130_rocket_acados_solve(_capsule);
		if (_solve_count < 2) { PX4_DEBUG("MPC RTI iter %d done, status=%d", i, status); }

		if (status != 0 && status != 2) {
			ok = false;
			break;
		}
	}

	float solve_ms = (float)(hrt_absolute_time() - t0) * 1e-3f;

	_last_solve_time_ms = solve_ms;
	_solve_count++;

	// Read solution
	double u_opt[MPC_NU];
	ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, 0, "u", u_opt);

	double x1[MPC_NX];
	ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, 1, "x", x1);

	bool finite_check = true;

	for (int i = 0; i < MPC_NU; i++) {
		if (!std::isfinite(u_opt[i])) { finite_check = false; }
	}

	for (int i = 0; i < MPC_NX; i++) {
		if (!std::isfinite(x1[i])) { finite_check = false; }
	}

	float de, dr, da;

	if (!ok || !finite_check) {
		_consec_fails++;
		_warm = false;       // force fresh forward_guess next solve — don't shift corrupted trajectory
		_consec_ok = 0;      // require 3 consecutive OK before re-enabling warm shift
		de = _last_delta_e;
		dr = _last_delta_r;
		da = _last_delta_a;

		// Diagnostic: dump input state on first few failures and every 50th after
		if (_consec_fails <= 3 || (_consec_fails % 50) == 0) {
			PX4_WARN("MPC fail #%d status=%d t=%.2f V=%.1f gam=%.4f chi=%.4f",
				 _consec_fails, status, (double)t_flight,
				 x_mpc[0], x_mpc[1], x_mpc[2]);
			PX4_WARN("  alpha=%.4f beta=%.4f phi=%.4f h=%.4f xg=%.4f",
				 x_mpc[6], x_mpc[7], x_mpc[8], x_mpc[9], x_mpc[10]);
			PX4_WARN("  refs: gam=%.4f chi=%.4f phi=%.4f h=%.4f",
				 (double)gamma_ref, (double)chi_ref, (double)phi_ref, (double)h_ref);
		}

		if (_consec_fails >= 10) {
			_reinit(x_mpc);
		}

	} else {
		_consec_fails = 0;
		de = (float)x1[12];
		dr = (float)x1[13];
		da = (float)x1[14];

		if (_solve_count <= 3) {
			PX4_DEBUG("MPC u_opt=[%.5f,%.5f,%.5f] x1_de=%.5f dr=%.5f da=%.5f",
				  u_opt[0], u_opt[1], u_opt[2],
				  x1[12], x1[13], x1[14]);
		}

		if (!_warm) {
			_consec_ok++;

			if (_consec_ok >= 3) {
				_warm = true;
			}

		}
	}

	_last_delta_e = de;
	_last_delta_r = dr;
	_last_delta_a = da;
	_last_solve_t = t_flight;

	// X-fin mixing: [da-de-dr, da-de+dr, da+de+dr, da+de-dr]
	float fins[4] = {
		da - de - dr,
		da - de + dr,
		da + de + dr,
		da + de - dr,
	};
	memcpy(_last_fins, fins, sizeof(fins));

	result.fins[0] = fins[0];
	result.fins[1] = fins[1];
	result.fins[2] = fins[2];
	result.fins[3] = fins[3];
	result.delta_e = de;
	result.delta_r = dr;
	result.delta_a = da;
	result.status  = status;
	result.solve_time_ms = solve_ms;
	// Report real solver iteration count from acados, matching Python
	// m130_mpc_autopilot.py:776 which reads solver.get_stats("sqp_iter")
	// after the RTI loop.  Previously we reported n_rti (the *planned*
	// loop bound, 3/5/8), which stayed fixed even when the loop broke
	// on the very first iteration — misleading in failure logs.
	int sqp_iter_real = 0;
	ocp_nlp_get(_nlp_solver, "sqp_iter", &sqp_iter_real);
	result.sqp_iter = sqp_iter_real;
	result.valid   = ok && finite_check;

	return result;
}
