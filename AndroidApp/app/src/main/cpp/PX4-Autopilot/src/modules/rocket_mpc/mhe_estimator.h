#pragma once

#include <cstdint>
#include <cmath>

#ifdef UNUSED
#undef UNUSED
#endif
extern "C" {
#include "acados_solver_m130_mhe.h"
#include "acados_c/ocp_nlp_interface.h"
}

static constexpr int MHE_NX  = M130_MHE_NX;    // 17
static constexpr int MHE_NU  = M130_MHE_NU;    // 17 (process noise / measurements as "controls")
static constexpr int MHE_NP  = M130_MHE_NP;    // 9
static constexpr int MHE_N   = M130_MHE_N;     // 20
static constexpr int MHE_NY0 = M130_MHE_NY0;   // 47  (y_meas[13] + w_noise[17] + x_bar[17])
static constexpr int MHE_NY  = M130_MHE_NY;    // 30  (y_meas[13] + w_noise[17])
static constexpr int MHE_NMEAS = 13;

// MHE state indices
static constexpr int IX_V     = 0;
static constexpr int IX_GAMMA = 1;
static constexpr int IX_CHI   = 2;
static constexpr int IX_P     = 3;
static constexpr int IX_Q     = 4;
static constexpr int IX_R     = 5;
static constexpr int IX_ALPHA = 6;
static constexpr int IX_BETA  = 7;
static constexpr int IX_PHI   = 8;
static constexpr int IX_H     = 9;
static constexpr int IX_XG    = 10;
static constexpr int IX_YG    = 11;
static constexpr int IX_BGX   = 12;
static constexpr int IX_BGY   = 13;
static constexpr int IX_BGZ   = 14;
static constexpr int IX_WN    = 15;
static constexpr int IX_WE    = 16;

struct MheOutput {
	double x_hat[MHE_NX];
	float  gyro_bias[3];
	float  wind_ne[2];
	float  quality;
	int    status;
	float  solve_time_ms;
	bool   valid;
};

struct MheConfig {
	int   horizon_steps      = 20;
	float horizon_dt         = 0.02f;
	float solve_rate_hz      = 50.0f;
	int   max_consec_fails   = 10;
	float quality_gate       = 0.3f;
	int   min_init_meas      = 10;
	int   startup_ramp_solves = 5;
};

class MheEstimator {
public:
	MheEstimator() = default;
	~MheEstimator();

	bool init(const MheConfig &cfg);
	void destroy();

	MheConfig &mutable_config() { return _cfg; }
	const MheConfig &config() const { return _cfg; }

	void init_state(const double x0[MHE_NX]);
	void reinit_state(const double x0[MHE_NX]);
	void reset();
	void push_measurement(float t, const double y_meas[MHE_NMEAS]);
	void push_params(float t, const double params[MHE_NP]);

	MheOutput update(float t);
	const MheOutput &last_output() const { return _last_valid; }
	bool ready() const { return _solver_ready && _initialized; }
	int  solve_count() const { return _solve_count; }

private:
	MheOutput _frozen_output() const;

	MheConfig _cfg;
	m130_mhe_solver_capsule *_capsule{nullptr};
	ocp_nlp_config *_nlp_config{nullptr};
	ocp_nlp_dims   *_nlp_dims{nullptr};
	ocp_nlp_in     *_nlp_in{nullptr};
	ocp_nlp_out    *_nlp_out{nullptr};

	bool _solver_ready{false};
	bool _initialized{false};
	int  _solve_count{0};
	int  _consec_fails{0};
	float _last_solve_t{-1.0f};

	// Arrival cost
	double _x_bar[MHE_NX]{};

	// Sliding window buffers (ring buffer approach)
	static constexpr int BUF_SIZE = 64;

	struct MeasEntry {
		float  t;
		double y[MHE_NMEAS];
	};

	struct ParamEntry {
		float  t;
		double p[MHE_NP];
	};

	MeasEntry  _meas_buf[BUF_SIZE]{};
	ParamEntry _param_buf[BUF_SIZE]{};
	int _meas_count{0};
	int _param_count{0};
	int _meas_head{0};
	int _param_head{0};

	MheOutput _last_valid{};
};
