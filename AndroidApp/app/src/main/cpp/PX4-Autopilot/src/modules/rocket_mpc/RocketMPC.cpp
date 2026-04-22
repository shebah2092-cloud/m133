/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file RocketMPC.cpp
 * Rocket M130 MPC+MHE control module — replaces RocketGNC.
 *
 * Implements:
 *   - MPC controller (acados SQP_RTI, N=80, 18 states, 3 controls)
 *   - MHE estimator  (acados, N=20, 17 states, sliding window)
 *   - LOS guidance   (geometric, feedforward only)
 *   - Sensor bridge   (uORB → MHE measurement vector)
 *   - X-fin mixer     (4-fin, same as MPC formulation)
 */

#include "RocketMPC.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <sched.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/resource.h>

// Maximum fin deflection is baked into the acados solver
// (see m130_ocp_setup.py::delta_max = np.radians(20.0)).
// To change it, modify the Python OCP and regenerate the solver.
static constexpr float SOLVER_DELTA_MAX_RAD = 0.3490658503988659f;   // 20° in radians

// Natural gamma from thrust/mass/gravity (called with param values)
static float compute_gamma_natural(float total_impulse, float burn_time,
				   float launch_pitch_deg, float mass_full)
{
	float T_avg = total_impulse / burn_time;
	float th = launch_pitch_deg * (float)M_PI / 180.0f;
	float a_v = T_avg * sinf(th) - mass_full * 9.80665f;
	float a_h = T_avg * cosf(th);
	return atan2f(a_v, (a_h > 1.0f) ? a_h : 1.0f);
}

// Plateau thrust derived from total impulse, burn time and tail-off duration.
// Mirrors the Python reference (m130_mpc_autopilot.py:81-85):
//   T_plateau = total_impulse / (burn_time - 0.75 * t_tail)
// This is ~15-20% higher than the simple average (impulse/burn_time) and
// compensates for the cubic tail-off profile used by get_params(): the
// plateau is the steady-state thrust during the flat propulsive phase, so
// that the integral over [0, burn_time] matches the measured total impulse.
//
// Fallback to impulse/burn_time when the tail-off window would invalidate
// the formula (burn_time <= 0.75 * t_tail), matching the Python branch.
static float compute_thrust_plateau(float total_impulse, float burn_time,
				    float t_tail)
{
	if (burn_time <= 0.0f) {
		return 0.0f;
	}

	const float denom = burn_time - 0.75f * t_tail;

	if (denom > 1e-3f) {
		return total_impulse / denom;
	}

	// Degenerate geometry: tail-off longer than burn_time — fall back to
	// average thrust rather than emitting an over-scaled plateau.
	return total_impulse / burn_time;
}

// ===================================================================
//  Custom work queue — acados NLP solver (N=80, NX=18) needs ~48KB
//  stack at runtime.  The default nav_and_controllers queue only
//  provides ~32KB after PX4_STACK_ADJUSTED().  We create a dedicated
//  queue with a raw Kconfig-equivalent value of 20000 which yields
//  ~64KB on POSIX (20000*2 + 24576, rounded to page boundary).
// ===================================================================
static constexpr px4::wq_config_t rocket_mpc_wq{"wq:rocket_mpc", 20000, -14};

// ===================================================================
//  Constructor / Destructor
// ===================================================================

RocketMPC::RocketMPC() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, rocket_mpc_wq)
{
	_actuator_outputs_sim_pub.advertise();
	_actuator_servos_pub.advertise();
	_rocket_gnc_status_pub.advertise();
	_timing_debug_pub.advertise();

	parameters_update(true);
}

RocketMPC::~RocketMPC()
{
	ScheduleClear();
	_mpc.destroy();
	_mhe.destroy();
	perf_free(_loop_perf);
	perf_free(_mpc_perf);
	perf_free(_mhe_perf);
}

bool RocketMPC::init()
{
	_sensor.set_launch_alt(_param_launch_alt.get());

	// HITL mode: switch to groundtruth topics
	int32_t hitl_val = 0;
	param_get(param_find("SYS_HITL"), &hitl_val);

	_hitl = (hitl_val == 1);

	if (_hitl) {
		_vehicle_attitude_sub       = uORB::Subscription{ORB_ID(vehicle_attitude_groundtruth)};
		_vehicle_local_position_sub = uORB::Subscription{ORB_ID(vehicle_local_position_groundtruth)};
		PX4_INFO("HITL mode: using groundtruth topics + lpos reference for MHE");

	} else {
		PX4_INFO("Real flight mode: using raw GPS (sensor_gps) for MHE");
	}

	// Read all params once
	const float mass_full      = _param_mass_full.get();
	const float mass_dry       = _param_mass_dry.get();
	const float burn_time      = _param_burn_time.get();
	const float impulse        = _param_impulse.get();
	const float thrust_override = _param_thrust.get();
	const float t_tail         = _param_t_tail.get();
	const float tau_servo      = _param_tau_servo.get();
	const float target_x       = _param_xtrgt.get();
	const float target_h       = _param_htrgt.get();
	const float impact_ang_deg = _param_imp_ang.get();
	const float launch_pitch   = _param_launch_pitch.get();

	// Derive plateau thrust from propulsion params (impulse / (t_burn - 0.75·t_tail))
	// to stay consistent with the Python reference. ROCKET_THRUST is treated as
	// an optional manual override: 0 (or negative) means "auto-derive", any
	// positive value overrides the derivation (advanced/experimental only).
	const float thrust_derived = compute_thrust_plateau(impulse, burn_time, t_tail);
	const bool  thrust_overridden = (thrust_override > 0.0f);
	const float thrust_plateau = thrust_overridden ? thrust_override : thrust_derived;

	if (thrust_overridden) {
		PX4_WARN("ROCKET_THRUST override active: using %.1f N (derived would be %.1f N)",
			 (double)thrust_override, (double)thrust_derived);

	} else {
		PX4_INFO("Thrust plateau derived: %.1f N (impulse=%.1f, t_burn=%.3f, t_tail=%.3f)",
			 (double)thrust_derived, (double)impulse, (double)burn_time, (double)t_tail);
	}

	// Initialize MPC solver
	MpcConfig mpc_cfg;
	mpc_cfg.N_horizon       = MPC_N;
	mpc_cfg.tf              = _param_mpc_tf.get();
	mpc_cfg.t_ctrl          = _param_t_ctrl.get();
	mpc_cfg.dt_solve        = 0.02f;
	mpc_cfg.target_x        = target_x;
	mpc_cfg.target_h        = target_h;
	mpc_cfg.mass_full       = mass_full;
	mpc_cfg.mass_dry        = mass_dry;
	mpc_cfg.burn_time       = burn_time;
	mpc_cfg.thrust_plateau  = thrust_plateau;
	mpc_cfg.t_tail          = t_tail;
	mpc_cfg.Ixx_full = _param_ixx_f.get(); mpc_cfg.Ixx_dry = _param_ixx_d.get();
	mpc_cfg.Iyy_full = _param_iyy_f.get(); mpc_cfg.Iyy_dry = _param_iyy_d.get();
	mpc_cfg.Izz_full = _param_izz_f.get(); mpc_cfg.Izz_dry = _param_izz_d.get();
	mpc_cfg.tau_servo       = tau_servo;
	mpc_cfg.impact_angle_deg = impact_ang_deg;
	mpc_cfg.gamma_natural_rad = compute_gamma_natural(impulse, burn_time, launch_pitch, mass_full);
	mpc_cfg.cruise_progress  = _param_cruise_p.get();
	mpc_cfg.quality_gate_thr = _param_mhe_qg.get();

	if (!_mpc.init(mpc_cfg)) {
		PX4_ERR("MPC solver init FAILED");
		return false;
	}

	// Initialize MHE solver
	MheConfig mhe_cfg;
	mhe_cfg.horizon_steps = MHE_N;
	mhe_cfg.horizon_dt    = 0.02f;
	mhe_cfg.solve_rate_hz = 50.0f;
	mhe_cfg.max_consec_fails = 10;
	mhe_cfg.quality_gate  = _param_mhe_qg.get();
	// Require a FULL measurement window before the first solve so that every
	// horizon stage (0..N) carries a real measurement. Previously =10 which
	// allowed stages N/2..N-1 to repeat the last measurement, injecting a
	// "temporal freeze" bias during the first ~200 ms after launch.
	mhe_cfg.min_init_meas = MHE_N + 1;
	mhe_cfg.startup_ramp_solves = 5;

	if (!_mhe.init(mhe_cfg)) {
		PX4_ERR("MHE solver init FAILED");
		return false;
	}

	// Initialize LOS guidance
	LosConfig los_cfg;
	los_cfg.target_x         = mpc_cfg.target_x;
	los_cfg.target_h         = mpc_cfg.target_h;
	los_cfg.impact_angle_deg = mpc_cfg.impact_angle_deg;
	los_cfg.impact_blend_start = mpc_cfg.impact_blend_start;
	los_cfg.impact_blend_end   = mpc_cfg.impact_blend_end;
	los_cfg.cruise_progress  = mpc_cfg.cruise_progress;
	los_cfg.gamma_natural_rad = mpc_cfg.gamma_natural_rad;
	los_cfg.burn_time        = mpc_cfg.burn_time;
	_los.configure(los_cfg);

	// Register callback on sensor_combined so we run every IMU sample.
	// This is critical for SITL lockstep: SimulatorMavlink needs
	// actuator_outputs_sim published on every HIL_SENSOR tick, not
	// just every 20ms.  MPC solve is rate-limited to 50Hz internally.
	if (!_sensor_combined_sub.registerCallback()) {
		PX4_WARN("sensor_combined callback registration failed — falling back to 50Hz timer");
		ScheduleOnInterval(20_ms);
	}

	// Publish initial status so Logger picks up the topic
	rocket_gnc_status_s s{};
	s.timestamp = hrt_absolute_time();
	_rocket_gnc_status_pub.publish(s);

	PX4_INFO("RocketMPC initialized — MPC(N=%d) + MHE(N=%d) @ sensor_combined callback",
		 MPC_N, MHE_N);
	return true;
}

void RocketMPC::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();

		// Propagate runtime-changeable params to subsystems
		MpcConfig &mcfg = _mpc.mutable_config();
		mcfg.target_x        = _param_xtrgt.get();
		mcfg.target_h        = _param_htrgt.get();
		mcfg.impact_angle_deg = _param_imp_ang.get();
		mcfg.t_ctrl          = _param_t_ctrl.get();
		mcfg.tf              = _param_mpc_tf.get();
		mcfg.quality_gate_thr = _param_mhe_qg.get();
		mcfg.cruise_progress = _param_cruise_p.get();
		_los.set_cruise_progress(mcfg.cruise_progress);

		// Recompute gamma_natural from current propulsion/launch params so
		// LOS feedforward and MPC config stay consistent with the param set.
		const float gamma_natural = compute_gamma_natural(
			_param_impulse.get(), _param_burn_time.get(),
			_param_launch_pitch.get(), _param_mass_full.get());
		mcfg.gamma_natural_rad = gamma_natural;
		mcfg.burn_time         = _param_burn_time.get();
		mcfg.t_tail            = _param_t_tail.get();
		mcfg.mass_full         = _param_mass_full.get();
		mcfg.mass_dry          = _param_mass_dry.get();

		// Re-derive the plateau thrust so changes to ROCKET_IMPULS,
		// ROCKET_TBURN or ROCKET_T_TAIL take effect on the next solve
		// instead of silently leaving the old plateau in place. Any
		// positive ROCKET_THRUST still wins as a manual override.
		const float thrust_override = _param_thrust.get();
		const float thrust_derived  = compute_thrust_plateau(
			_param_impulse.get(), _param_burn_time.get(),
			_param_t_tail.get());
		mcfg.thrust_plateau = (thrust_override > 0.0f) ? thrust_override
								: thrust_derived;

		_los.configure_target(_param_xtrgt.get(), _param_htrgt.get(),
				      _param_imp_ang.get());
		_los.set_gamma_natural(gamma_natural);
		_los.set_burn_time(_param_burn_time.get());

		// Keep MHE quality gate in sync (used by any consumer that reads
		// MheConfig directly; the blend gate in Run() uses mcfg.quality_gate_thr).
		_mhe.mutable_config().quality_gate = _param_mhe_qg.get();

		_sensor.set_launch_alt(_param_launch_alt.get());
		_target_downrange = _param_xtrgt.get();
	}
}

// ===================================================================
//  Main Run loop — called on every sensor_combined update (IMU rate)
//  MPC/MHE solve is rate-limited to ~50Hz (20ms).
//  actuator_outputs_sim is ALWAYS published (critical for lockstep).
// ===================================================================

void RocketMPC::_reset_flight_state()
{
	// Flight-progress flags & timers
	_launched    = false;
	_launch_dv   = 0.0f;
	_time_launch = 0;
	_prev_run_time = 0;
	_dt_measured = 0.02f;
	_dt_min = 1.0f;
	_dt_max = 0.0f;
	_dt_sum = 0.0f;
	_dt_count = 0;

	// Actuator / servo caches (physical commands from last solve)
	_de_act = 0.0f; _dr_act = 0.0f; _da_act = 0.0f;
	memset(_last_fins, 0, sizeof(_last_fins));
	_last_de = 0.0f; _last_dr = 0.0f; _last_da = 0.0f;
	_last_mpc_solve_time = 0;

	// Solver & guidance sub-modules
	_mpc.reset();
	_mhe.reset();
	_los.reset();

	// MHE telemetry snapshot + publishing authority
	_mhe_publishing = false;
	_mhe_phi = 0.0f; _mhe_theta = 0.0f; _mhe_psi = 0.0f;
	_mhe_x = 0.0f; _mhe_y = 0.0f; _mhe_z = 0.0f;
	_mhe_vx = 0.0f; _mhe_vy = 0.0f; _mhe_vz = 0.0f;
	_mhe_vm = 0.0f;
	_blend_alpha = 0.0f;

	// Cross-validation state
	_xval = {};
}

void RocketMPC::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// ==========================================================
	// Real-time priority setup — once per thread lifetime.
	// يُثبّت MPC على النوى الكبيرة (Big cores) ويمنحه أولوية
	// SCHED_FIFO لتفادي spikes Android scheduler (PIL أظهر
	// أقصى = 69ms مقابل p50 = 15ms).
	// ==========================================================
	static std::once_flag s_rt_flag;
	std::call_once(s_rt_flag, []() {
		pid_t tid = (pid_t)syscall(SYS_gettid);

		// Pin to big cores by reading cpu_capacity from sysfs. On ARM
		// big.LITTLE, the kernel exposes a normalized capacity per CPU
		// (1024 = biggest core). Cores with capacity >= half of the max
		// are classified as "big". Fallback: if sysfs is unreadable, use
		// the top half of online CPUs (which is the big cluster on most
		// Snapdragon / Dimensity layouts). Legacy hard-coded range [4,8)
		// failed on devices where big cores live elsewhere (e.g. SD865:
		// cores 5-7; Dimensity 9000: core 7 only).
		cpu_set_t cs;
		CPU_ZERO(&cs);
		long ncpu = sysconf(_SC_NPROCESSORS_CONF);
		if (ncpu < 1) { ncpu = 8; }

		long cap[64] = {};
		long cap_max = 0;
		bool have_cap = false;
		for (long c = 0; c < ncpu && c < 64; ++c) {
			char path[96];
			snprintf(path, sizeof(path),
				 "/sys/devices/system/cpu/cpu%ld/cpu_capacity", c);
			FILE *f = fopen(path, "r");
			if (f) {
				if (fscanf(f, "%ld", &cap[c]) == 1 && cap[c] > 0) {
					have_cap = true;
					if (cap[c] > cap_max) { cap_max = cap[c]; }
				}
				fclose(f);
			}
		}

		int n_big = 0;
		if (have_cap && cap_max > 0) {
			long thr = cap_max / 2;
			for (long c = 0; c < ncpu && c < 64; ++c) {
				if (cap[c] >= thr) { CPU_SET((int)c, &cs); ++n_big; }
			}
		}
		if (n_big == 0) {
			// Fallback: top half of CPUs (usually big cluster).
			long start = ncpu / 2;
			for (long c = start; c < ncpu && c < 64; ++c) {
				CPU_SET((int)c, &cs); ++n_big;
			}
		}
		int aff_rc = sched_setaffinity(tid, sizeof(cs), &cs);

		// SCHED_FIFO highest safe priority (below kernel threads).
		struct sched_param sp;
		sp.sched_priority = 80;
		int sch_rc = sched_setscheduler(tid, SCHED_FIFO, &sp);

		// Fallback for non-root Android: lower nice value (higher priority).
		// Nice range is [-20, 19]; -20 is highest. Android allows this for
		// own-process threads without CAP_SYS_NICE.
		int nice_rc = setpriority(PRIO_PROCESS, tid, -20);
		int nice_now = getpriority(PRIO_PROCESS, tid);

		PX4_INFO("RT config: affinity(big)=%s, SCHED_FIFO=%s, nice=%d (set rc=%d)",
			 aff_rc == 0 ? "OK" : "FAIL",
			 sch_rc == 0 ? "OK" : "FAIL",
			 nice_now, nice_rc);
	});

	const hrt_abstime _cycle_t0 = hrt_absolute_time();
	perf_begin(_loop_perf);
	parameters_update();

	// Read sensor data
	sensor_combined_s sc{};
	_sensor_combined_sub.copy(&sc);

	vehicle_attitude_s att{};
	_vehicle_attitude_sub.copy(&att);

	vehicle_local_position_s lpos{};
	_vehicle_local_position_sub.copy(&lpos);

	vehicle_air_data_s air{};
	_vehicle_air_data_sub.copy(&air);

	// Read GPS for MHE — source depends on mode:
	//  Real flight: raw sensor_gps (bypasses EKF2)
	//  HITL/SITL:   lpos reference (same frame as groundtruth/EKF2 states)
	if (_hitl || _param_sitl_gps.get() == 1) {
		// Use EKF2 local position — ensures same reference frame as attitude/position states.
		// First ensure GPS origin is set from lpos reference (not raw GPS).
		if (!_sensor.gps_origin_set() && lpos.xy_global && lpos.z_global) {
			_sensor.set_gps_origin(lpos.ref_lat, lpos.ref_lon, (double)lpos.ref_alt);
			_actual_launch_alt_msl = lpos.ref_alt;
			_launch_alt_captured = true;
		}
		if (lpos.xy_valid && lpos.z_valid) {
			_sensor.update_from_lpos(lpos);
		}
	} else {
		// Real flight: raw GPS for independent MHE position measurements
		sensor_gps_s gps{};
		if (_sensor_gps_sub.copy(&gps) && gps.fix_type >= sensor_gps_s::FIX_TYPE_3D) {
			_sensor.update_gps(gps);
		}
	}

	// Arming detection
	vehicle_status_s vstatus{};
	_vehicle_status_sub.copy(&vstatus);
	const bool is_armed = (vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	if (is_armed && !_armed) {
		if (att.timestamp == 0) {
			mavlink_log_critical(&_mavlink_log_pub,
				"ROCKET MPC: NO ATTITUDE DATA");
			PX4_ERR("NO ATTITUDE DATA at arming");
		}

		if (!lpos.xy_valid || !lpos.z_valid) {
			mavlink_log_critical(&_mavlink_log_pub,
				"ROCKET MPC: NO POSITION FIX");
			PX4_ERR("NO POSITION FIX at arming");
		}

		// Reset launch-altitude capture BEFORE the conditional GPS/lpos branches
		// below. If no GPS fix is available at arm, these must NOT carry over from
		// a previous flight — otherwise MHE's baro reference (mhe_p[8]) would use a
		// stale MSL altitude until a fresh GPS fix arrives.
		_launch_alt_captured   = false;
		_actual_launch_alt_msl = 0.0f;

		_arm_origin_x = lpos.x;
		_arm_origin_y = lpos.y;
		_arm_origin_z = lpos.z;

		// Set GPS origin for MHE — source depends on mode
		if (_hitl || _param_sitl_gps.get() == 1) {
			// HITL/SITL: derive origin from EKF2 reference (same frame as lpos)
			if (lpos.xy_global && lpos.z_global) {
				_sensor.set_gps_origin(lpos.ref_lat, lpos.ref_lon, (double)lpos.ref_alt);
				_actual_launch_alt_msl = lpos.ref_alt;
				_launch_alt_captured = true;
				PX4_INFO("GPS origin (from lpos ref): lat=%.6f lon=%.6f alt=%.1f",
					 lpos.ref_lat, lpos.ref_lon, (double)lpos.ref_alt);
			} else {
				// lpos ref not yet available — use param as fallback
				_actual_launch_alt_msl = _param_launch_alt.get();
				_launch_alt_captured = true;
				_sensor.set_gps_origin(0.0, 0.0, (double)_actual_launch_alt_msl);
				PX4_WARN("lpos ref not available at arm — using param launch_alt=%.1f",
					 (double)_actual_launch_alt_msl);
			}
		} else {
			// Real flight: raw GPS origin (independent of EKF2)
			sensor_gps_s gps{};
			_sensor_gps_sub.copy(&gps);
			if (gps.fix_type >= sensor_gps_s::FIX_TYPE_3D) {
				_sensor.set_gps_origin(gps.latitude_deg, gps.longitude_deg, gps.altitude_msl_m);
				_actual_launch_alt_msl = (float)gps.altitude_msl_m;
				_launch_alt_captured = true;
				PX4_INFO("GPS origin: lat=%.6f lon=%.6f alt=%.1f (launch_alt_msl=%.1f)",
					 gps.latitude_deg, gps.longitude_deg, gps.altitude_msl_m,
					 (double)_actual_launch_alt_msl);
			} else {
				PX4_WARN("No GPS 3D fix at arm — MHE will wait for fix");
			}
		}

		// Bearing from heading at arming
		const Quatf q_arm(att.q[0], att.q[1], att.q[2], att.q[3]);
		const Eulerf euler_arm(q_arm);
		const float bearing = euler_arm.psi();
		_target_downrange = _param_xtrgt.get();
		_cos_bearing = cosf(bearing);
		_sin_bearing = sinf(bearing);

		PX4_INFO("rocket_mpc: armed — origin=(%.2f,%.2f,%.2f) heading=%.1f°  range=%.0fm",
			 (double)_arm_origin_x, (double)_arm_origin_y, (double)_arm_origin_z,
			 (double)(bearing * 180.0f / M_PI_F), (double)_target_downrange);

		// Reset all in-flight state (flags, actuator caches, solvers,
		// MHE telemetry, xval). Centralised to keep arm / disarm in
		// lockstep and guarantee no state bleeds across flights.
		_reset_flight_state();
	}

	if (!is_armed && _armed) {
		_arm_origin_x = 0.0f;
		_arm_origin_y = 0.0f;
		_arm_origin_z = 0.0f;

		// Mirror the arm-time reset on disarm. Without this, _launched
		// stays true and the MPC/actuator publish path at the bottom
		// of Run() keeps driving the servos while the vehicle is
		// disarmed — a mechanical hazard on the ground and a source
		// of misleading telemetry.
		_reset_flight_state();
	}

	_armed = is_armed;

	const hrt_abstime now = hrt_absolute_time();

	// Compute actual dt EARLY so launch detection integrates \u0394v with the
	// current-cycle dt (previously used stale _dt_measured from prior cycle).
	float dt = 0.02f;

	if (_prev_run_time > 0) {
		float dt_raw = (float)(now - _prev_run_time) * 1e-6f;
		dt = math::constrain(dt_raw, 0.001f, 0.05f);
	}

	_prev_run_time = now;
	_dt_measured = dt;

	// Pre-launch: continuously update origin and bearing
	if (_armed && !_launched && sc.timestamp > 0) {
		if (lpos.xy_valid && lpos.z_valid) {
			_arm_origin_x = lpos.x;
			_arm_origin_y = lpos.y;
			_arm_origin_z = lpos.z;

			// Update GPS origin for MHE
			if (_hitl || _param_sitl_gps.get() == 1) {
				// HITL/SITL: update from lpos reference
				if (lpos.xy_global && lpos.z_global) {
					_sensor.set_gps_origin(lpos.ref_lat, lpos.ref_lon, (double)lpos.ref_alt);
					_actual_launch_alt_msl = lpos.ref_alt;
					_launch_alt_captured = true;
				}
			} else {
				// Real flight: raw GPS
				sensor_gps_s gps{};
				_sensor_gps_sub.copy(&gps);
				if (gps.fix_type >= sensor_gps_s::FIX_TYPE_3D) {
					_sensor.set_gps_origin(gps.latitude_deg, gps.longitude_deg, gps.altitude_msl_m);
					_actual_launch_alt_msl = (float)gps.altitude_msl_m;
					_launch_alt_captured = true;
				}
			}

			const Quatf q_pre(att.q[0], att.q[1], att.q[2], att.q[3]);
			const Eulerf euler_pre(q_pre);
			const float bearing = euler_pre.psi();
			_cos_bearing = cosf(bearing);
			_sin_bearing = sinf(bearing);
		}
	}

	// Launch detection
	if (!_launched && _armed && sc.timestamp > 0) {
		const float ax = sc.accelerometer_m_s2[0];

		if (ax > 1.0f * 9.80665f) {
			_launch_dv += ax * _dt_measured;

		} else {
			_launch_dv = 0.0f;
		}

		if (_launch_dv > 2.0f) {
			_launched    = true;
			_time_launch = now;
			PX4_WARN(">>> LAUNCH DETECTED <<<  ax=%.1f m/s2", (double)ax);
			mavlink_log_critical(&_mavlink_log_pub, "LAUNCH DETECTED ax=%.1f", (double)ax);

			// Initialize MHE with current state
			double x0_mhe[MHE_NX] = {};
			float V0 = sqrtf(lpos.vx * lpos.vx + lpos.vy * lpos.vy + lpos.vz * lpos.vz);
			x0_mhe[IX_V] = (V0 > 10.0f) ? (double)V0 : 10.0;

			const Quatf q0(att.q[0], att.q[1], att.q[2], att.q[3]);
			const Eulerf euler0(q0);

			// Compute flight-path angle gamma from velocity (not body pitch).
			// At low speed, fall back to body pitch as gamma is poorly defined.
			float v_h0 = sqrtf(lpos.vx * lpos.vx + lpos.vy * lpos.vy);
			float gamma0 = (V0 > 10.0f) ? atan2f(-lpos.vz, v_h0) : euler0.theta();
			x0_mhe[IX_GAMMA] = (double)gamma0;
			// IX_CHI = velocity heading in NED. Use atan2(vy,vx) when moving;
			// fall back to body yaw (NOT launch bearing) at low speed so any
			// misalignment between rail and heading doesn't inject an initial error.
			float chi0 = (V0 > 10.0f) ? atan2f(lpos.vy, lpos.vx) : euler0.psi();
			x0_mhe[IX_CHI]   = (double)chi0;
			x0_mhe[IX_P]     = sc.gyro_rad[0];
			x0_mhe[IX_Q]     = sc.gyro_rad[1];
			x0_mhe[IX_R]     = sc.gyro_rad[2];
			x0_mhe[IX_PHI]   = euler0.phi();
			// Position: in SITL/HITL lpos is relative to EKF2 origin, not arm origin.
			// Use _arm_origin so the MHE initial state matches the GPS measurement frame.
			x0_mhe[IX_H]  = 0.0;
			x0_mhe[IX_XG] = (double)(_arm_origin_x / X_SCALE);
			x0_mhe[IX_YG] = (double)(_arm_origin_y / Y_SCALE);
			// Biases + wind: zero
			_mhe.init_state(x0_mhe);

			// Initialize LOS with current gamma (flight-path angle, not body pitch)
			_los.set_gamma_prev(gamma0);
			_los.set_chi_prev(0.0f);
		}
	}

	// Flight time
	const float t = _launched ? (float)(now - _time_launch) * 1e-6f : 0.0f;

	if (_launched) {
		_dt_min = fminf(_dt_min, dt);
		_dt_max = fmaxf(_dt_max, dt);
		_dt_sum += dt;
		_dt_count++;
	}

	// Attitude
	const Quatf quat(att.q[0], att.q[1], att.q[2], att.q[3]);
	const Eulerf euler(quat);
	const float phi = euler.phi();

	// Position relative to arm origin in NED
	const float pos_north = lpos.x - _arm_origin_x;
	const float pos_east  = lpos.y - _arm_origin_y;

	// Rotate NED → downrange/crossrange using bearing captured at arming.
	// This makes MPC/LOS work regardless of the rocket's compass heading.
	const float Xm  =  _cos_bearing * pos_north + _sin_bearing * pos_east;   // downrange
	const float Ym  = -(lpos.z - _arm_origin_z);                              // up
	const float Zm  = -_sin_bearing * pos_north + _cos_bearing * pos_east;    // crossrange

	const float Vxm =  _cos_bearing * lpos.vx + _sin_bearing * lpos.vy;   // downrange velocity
	const float Vym =  lpos.vz;                                            // down velocity
	const float Vzm = -_sin_bearing * lpos.vx + _cos_bearing * lpos.vy;   // crossrange velocity

	const float vm = sqrtf(lpos.vx * lpos.vx + lpos.vy * lpos.vy + lpos.vz * lpos.vz);
	const float v_h = sqrtf(Vxm * Vxm + Vzm * Vzm);

	// Default: use cached fins from last solve
	float fin[4] = {_last_fins[0], _last_fins[1], _last_fins[2], _last_fins[3]};
	float delta_e = _last_de, delta_r = _last_dr, delta_a = _last_da;
	int   mpc_status = -1;
	float mpc_solve_ms = 0.0f;
	float mhe_solve_ms = 0.0f;
	float mhe_quality = 0.0f;
	float gamma_ref = 0.0f, chi_ref = 0.0f;

	// Rate-limit MPC/MHE solve to ~50Hz (20ms).  On intermediate
	// sensor_combined ticks we republish the cached fins so that
	// SimulatorMavlink lockstep can proceed.
	const bool do_mpc_this_cycle = (now - _last_mpc_solve_time >= 19_ms);

	if (_launched && t > _param_t_ctrl.get() && vm > 10.0f && do_mpc_this_cycle) {
		_last_mpc_solve_time = now;

		if (_mpc.solve_count() == 0) {
			PX4_INFO("First MPC cycle: t=%.3f vm=%.1f", (double)t, (double)vm);
			PX4_INFO("  lpos: vx=%.3f vy=%.3f vz=%.3f", (double)lpos.vx, (double)lpos.vy, (double)lpos.vz);
			PX4_INFO("  rotated: Vxm=%.3f Vym=%.3f Vzm=%.3f v_h=%.3f", (double)Vxm, (double)Vym, (double)Vzm, (double)v_h);
			PX4_INFO("  bearing: cos=%.4f sin=%.4f euler.theta=%.4f",
				 (double)_cos_bearing, (double)_sin_bearing, (double)euler.theta());
		}
		// ---- MHE: push measurements & solve ----
		SensorMeasurement smeas = _sensor.build_measurement(sc, air);

		if (smeas.valid) {
			_mhe.push_measurement(t, smeas.y);

			// Update actual fin positions (first-order lag) BEFORE passing to MHE.
			// MHE model expects delta_*_act (physical fin position), not the command.
			{
				// Guard: if tau_servo is misconfigured (=0) or dt is 0, the division
				// would produce inf or NaN. Clamp tau to a minimum of 0.1 ms.
				float tau = _mpc.config().tau_servo;
				if (tau < 1e-4f) { tau = 1e-4f; }
				float decay = expf(-dt / tau);
				_de_act = _de_act * decay + delta_e * (1.0f - decay);
				_dr_act = _dr_act * decay + delta_r * (1.0f - decay);
				_da_act = _da_act * decay + delta_a * (1.0f - decay);
			}

			// MHE parameters must match m130_mhe_model.py:
			// [delta_e_act, delta_r_act, delta_a_act, mass, thrust, Ixx, Iyy, Izz, launch_alt]
			double mhe_p[MHE_NP] = {};
			double mpc_p[MPC_NP];
			_mpc.get_params(t, mpc_p);
			float mass   = (float)mpc_p[0];
			float thrust = (float)mpc_p[1];

			const MpcConfig &cfg = _mpc.config();
			float frac = (t < cfg.burn_time) ? (t / cfg.burn_time) : 1.0f;
			float Ixx = cfg.Ixx_full - frac * (cfg.Ixx_full - cfg.Ixx_dry);
			float Iyy = cfg.Iyy_full - frac * (cfg.Iyy_full - cfg.Iyy_dry);
			float Izz = cfg.Izz_full - frac * (cfg.Izz_full - cfg.Izz_dry);

			mhe_p[0] = (double)_de_act;
			mhe_p[1] = (double)_dr_act;
			mhe_p[2] = (double)_da_act;
			mhe_p[3] = (double)mass;
			mhe_p[4] = (double)thrust;
			mhe_p[5] = (double)Ixx;
			mhe_p[6] = (double)Iyy;
			mhe_p[7] = (double)Izz;
			// Use actual captured altitude MSL (not hardcoded param) to match baro/GPS observations
			mhe_p[8] = (double)(_launch_alt_captured ? _actual_launch_alt_msl : _param_launch_alt.get());
			_mhe.push_params(t, mhe_p);
		}

		perf_begin(_mhe_perf);
		if (_mhe.solve_count() < 3) {
			PX4_INFO("MHE update #%d: t=%.3f ready=%d init=%d",
				 _mhe.solve_count(), (double)t, _mhe.ready(), 1);
		}
		MheOutput mhe_out = _mhe.update(t);
		perf_end(_mhe_perf);
		mhe_solve_ms = mhe_out.solve_time_ms;
		if (_mhe.solve_count() < 3) {
			PX4_INFO("MHE update done: status=%d quality=%.2f valid=%d",
				 mhe_out.status, (double)mhe_out.quality, mhe_out.valid);
		}
		mhe_quality = mhe_out.quality;

		// ---- Publish MHE as vehicle_attitude + vehicle_local_position ----
		// Once MHE has authority, it replaces EKF2 as the system estimator.
		// Before that, EKF2 keeps publishing (pre-launch / early flight).
		const float qg_thr = _mpc.config().quality_gate_thr;

		// Finite check on x_hat: MHE's internal guard downgrades quality on
		// NaN, but belt-and-braces here prevents any non-finite entry from
		// leaking into the blend if a partial failure slipped through.
		bool mhe_finite = mhe_out.valid;

		if (mhe_finite) {
			for (int i = 0; i < MHE_NX; i++) {
				if (!std::isfinite(mhe_out.x_hat[i])) { mhe_finite = false; break; }
			}
		}

		const bool mhe_authority = mhe_finite && mhe_out.quality >= qg_thr;

		if (mhe_authority) {
			_mhe_publishing = true;

			// Convert MHE flight-dynamics states → Euler angles
			const float gamma = (float)mhe_out.x_hat[IX_GAMMA];
			const float chi   = (float)mhe_out.x_hat[IX_CHI];
			const float alpha = (float)mhe_out.x_hat[IX_ALPHA];
			const float beta  = (float)mhe_out.x_hat[IX_BETA];
			const float phi_m = (float)mhe_out.x_hat[IX_PHI];
			const float V     = (float)mhe_out.x_hat[IX_V];

			_mhe_theta = gamma + alpha;          // pitch = flight-path + AoA
			_mhe_psi   = chi - beta;             // yaw   = heading - sideslip
			_mhe_phi   = phi_m;                  // roll

			// Position NED → rotate to downrange / crossrange
			const float ned_x = (float)(mhe_out.x_hat[IX_XG]) * X_SCALE;  // north
			const float ned_y = (float)(mhe_out.x_hat[IX_YG]) * Y_SCALE;  // east
			_mhe_x =  _cos_bearing * ned_x + _sin_bearing * ned_y;  // downrange
			_mhe_y = -_sin_bearing * ned_x + _cos_bearing * ned_y;  // crossrange
			_mhe_z = -(float)(mhe_out.x_hat[IX_H]) * H_SCALE;      // down (NED)

			// Velocity NED → rotate to downrange / crossrange
			const float vn = V * cosf(gamma) * cosf(chi);   // north
			const float ve = V * cosf(gamma) * sinf(chi);   // east
			_mhe_vx =  _cos_bearing * vn + _sin_bearing * ve;  // downrange
			_mhe_vy = -_sin_bearing * vn + _cos_bearing * ve;  // crossrange
			_mhe_vz = -V * sinf(gamma);                        // down
			_mhe_vm = V;
		}

		// ---- Build MPC state [18] ----

		float V_mpc, gamma_mpc, chi_mpc, alpha_mpc, beta_mpc, phi_mpc;
		float cur_alt, cur_x, cur_y;

		// === EKF-MHE Cooperative Blend ===
		// Instead of a hard switch, blend smoothly from EKF2→MHE as MHE matures.
		// blend_alpha ramps 0→1 over [burn_time - 0.5, burn_time + 2.0] seconds.
		// Gated by MHE quality and altitude (GPS multipath near ground).

		const float burn_time = _mpc.config().burn_time;
		const float t_blend_start = burn_time - 0.5f;
		const float t_blend_end   = burn_time + 2.0f;

		// Hermite smoothstep: smooth s-curve from 0 to 1
		float time_blend = 0.0f;
		if (t > t_blend_start) {
			float s = (t - t_blend_start) / (t_blend_end - t_blend_start);
			s = (s < 0.0f) ? 0.0f : ((s > 1.0f) ? 1.0f : s);
			time_blend = s * s * (3.0f - 2.0f * s);  // smoothstep
		}

		// Quality gate: scale blend by MHE confidence (0 below qg_thr, full at qg_thr+0.2)
		float quality_factor = 0.0f;
		if (mhe_authority) {
			quality_factor = (mhe_out.quality - qg_thr) / 0.2f;
			quality_factor = (quality_factor < 0.0f) ? 0.0f : ((quality_factor > 1.0f) ? 1.0f : quality_factor);
		}

		// Compute effective blend (time × quality)
		float blend = time_blend * quality_factor;

		// Altitude fadeout: reduce MHE influence below 150m AGL (GPS multipath)
		constexpr float ALT_FADE_HI = 150.0f;
		constexpr float ALT_FADE_LO = 50.0f;
		// Use EKF2 altitude for fadeout decision (available before MHE)
		float alt_for_fade = Ym;
		if (blend > 0.01f) {
			// Once MHE is partially active, use blended altitude for smooth fadeout
			float mhe_alt = (float)(mhe_out.x_hat[IX_H]) * H_SCALE;
			alt_for_fade = (1.0f - blend) * Ym + blend * mhe_alt;
		}
		float alt_factor = (alt_for_fade - ALT_FADE_LO) / (ALT_FADE_HI - ALT_FADE_LO);
		alt_factor = (alt_factor < 0.0f) ? 0.0f : ((alt_factor > 1.0f) ? 1.0f : alt_factor);
		blend *= alt_factor;

		// ----- EKF2 fallback states (always computed) -----
		float ekf_gamma, ekf_chi, ekf_alpha, ekf_beta;
		if (vm > 10.0f) {
			ekf_gamma = atan2f(-Vym, v_h);
			ekf_chi   = atan2f(Vzm, Vxm);  // bearing-frame χ (downrange=0), used by MPC state
			// β is frame-invariant only when both ψ and χ are in the SAME frame.
			// euler.psi() is NED-absolute, so compute χ in NED for β (matches Python reference
			// m130_mpc_autopilot.py::_extract_mpc_state where β = -(ψ_NED - χ_NED)).
			const float chi_ned_vel = atan2f(lpos.vy, lpos.vx);
			ekf_beta = -(euler.psi() - chi_ned_vel);
			ekf_beta = atan2f(sinf(ekf_beta), cosf(ekf_beta));
		} else {
			ekf_gamma = euler.theta();
			ekf_chi   = 0.0f;              // bearing-frame (low-V MPC fallback)
			ekf_beta  = 0.0f;              // match Python: no sideslip when V is near zero
		}
		ekf_alpha = euler.theta() - ekf_gamma;

		// ----- MHE states (valid when mhe_authority) -----
		float mhe_gamma = ekf_gamma, mhe_chi = ekf_chi;
		float mhe_alpha = ekf_alpha, mhe_beta = ekf_beta, mhe_phi = phi;
		float mhe_alt = Ym, mhe_x_dr = Xm, mhe_y_cr = Zm;

		if (mhe_authority) {
			mhe_gamma = (float)mhe_out.x_hat[IX_GAMMA];
			mhe_alpha = (float)mhe_out.x_hat[IX_ALPHA];
			mhe_beta  = (float)mhe_out.x_hat[IX_BETA];
			mhe_phi   = (float)mhe_out.x_hat[IX_PHI];
			mhe_alt   = (float)(mhe_out.x_hat[IX_H]) * H_SCALE;

			// MHE chi in NED → rotate to downrange frame
			const float chi_ned = (float)mhe_out.x_hat[IX_CHI];
			const float bearing_angle = atan2f(_sin_bearing, _cos_bearing);
			mhe_chi = atan2f(sinf(chi_ned - bearing_angle), cosf(chi_ned - bearing_angle));

			// MHE position NED → rotate to downrange/crossrange
			const float mhe_north = (float)(mhe_out.x_hat[IX_XG]) * X_SCALE;
			const float mhe_east  = (float)(mhe_out.x_hat[IX_YG]) * Y_SCALE;
			mhe_x_dr =  _cos_bearing * mhe_north + _sin_bearing * mhe_east;
			mhe_y_cr = -_sin_bearing * mhe_north + _cos_bearing * mhe_east;
		}

		// === Cross-Validation: EKF↔MHE consistency check ===
		// Compare MHE estimates against EKF2 and penalize blend if they diverge.
		// This protects against MHE divergence corrupting the MPC.
		constexpr float XVAL_GAMMA_THRESH = 0.2618f;  // 15 deg
		constexpr float XVAL_CHI_THRESH   = 0.3491f;  // 20 deg
		constexpr float XVAL_ALT_THRESH   = 50.0f;    // 50 m
		constexpr int   XVAL_CONSEC_LIMIT = 5;        // cycles before penalty kicks in
		constexpr float XVAL_RESET_TIME   = 3.0f;     // seconds of sustained divergence → reset MHE

		if (mhe_authority && blend > 0.01f) {
			_xval.gamma_err = fabsf(mhe_gamma - ekf_gamma);
			_xval.chi_err   = fabsf(atan2f(sinf(mhe_chi - ekf_chi), cosf(mhe_chi - ekf_chi)));
			_xval.alt_err   = fabsf(mhe_alt - Ym);

			bool diverged = (_xval.gamma_err > XVAL_GAMMA_THRESH)
				     || (_xval.chi_err   > XVAL_CHI_THRESH)
				     || (_xval.alt_err   > XVAL_ALT_THRESH);

			if (diverged) {
				_xval.consec_diverge++;
				if (_xval.consec_diverge == 1) {
					_xval.diverge_start_t = t;
				}

				// Progressive penalty: ramps from 0 to 0.8 over 5 consecutive cycles
				if (_xval.consec_diverge >= XVAL_CONSEC_LIMIT) {
					float pen = (float)(_xval.consec_diverge - XVAL_CONSEC_LIMIT) / 10.0f;
					_xval.penalty = (pen > 0.8f) ? 0.8f : pen;
				}

				// Auto-reset MHE if sustained divergence exceeds threshold
				if ((t - _xval.diverge_start_t) > XVAL_RESET_TIME && _xval.consec_diverge > 20) {
					// Reset MHE with current EKF2-derived state
					double x0_reset[MHE_NX] = {};
					x0_reset[IX_V]     = (double)vm;
					x0_reset[IX_GAMMA] = (double)ekf_gamma;
					// MHE stores IX_CHI in NED-absolute frame (see model:
					// x_ground_dot = V*cos(gamma)*cos(chi) producing NED north,
					// and init at launch uses atan2f(lpos.vy, lpos.vx) which is
					// NED-absolute). The previous "bearing angle" seed was wrong
					// because it ignored the velocity direction. ekf_chi cannot
					// be used here either — it is in bearing-frame (built from
					// rotated Vxm/Vzm). Re-derive NED heading from raw lpos vel.
					float chi_reset_ned;
					if (vm > 10.0f) {
						chi_reset_ned = atan2f(lpos.vy, lpos.vx);
					} else {
						chi_reset_ned = euler.psi();
					}
					x0_reset[IX_CHI]   = (double)chi_reset_ned;
					x0_reset[IX_P]     = (double)sc.gyro_rad[0];
					x0_reset[IX_Q]     = (double)sc.gyro_rad[1];
					x0_reset[IX_R]     = (double)sc.gyro_rad[2];
					x0_reset[IX_ALPHA] = (double)ekf_alpha;
					x0_reset[IX_BETA]  = (double)ekf_beta;
					x0_reset[IX_PHI]   = (double)phi;
					x0_reset[IX_H]     = (double)(Ym / H_SCALE);
					x0_reset[IX_XG]    = (double)(Xm / X_SCALE);
					x0_reset[IX_YG]    = (double)(Zm / Y_SCALE);
					_mhe.reinit_state(x0_reset);

					PX4_WARN("XVAL: MHE reset (diverged %.1fs, gamma_err=%.1f° alt_err=%.0fm)",
						 (double)(t - _xval.diverge_start_t),
						 (double)(_xval.gamma_err * 57.2958f),
						 (double)_xval.alt_err);

					_xval.consec_diverge = 0;
					_xval.penalty = 0.0f;
					_xval.diverge_start_t = 0.0f;
				}
			} else {
				// Agreement: decay penalty and reset counter
				_xval.consec_diverge = 0;
				_xval.diverge_start_t = 0.0f;
				_xval.penalty *= 0.9f;  // smooth decay
				if (_xval.penalty < 0.01f) { _xval.penalty = 0.0f; }
			}

			// Apply cross-validation penalty to blend
			blend *= (1.0f - _xval.penalty);
		} else {
			// When MHE not active, decay state
			_xval.consec_diverge = 0;
			_xval.penalty *= 0.95f;
		}

		// Update stored blend after cross-validation
		_blend_alpha = blend;

		// ----- Smooth blend: state = (1-blend)*EKF + blend*MHE -----
		V_mpc     = vm;  // Velocity always from EKF2 (MHE has known bias)
		gamma_mpc = (1.0f - blend) * ekf_gamma + blend * mhe_gamma;
		alpha_mpc = (1.0f - blend) * ekf_alpha + blend * mhe_alpha;
		beta_mpc  = (1.0f - blend) * ekf_beta  + blend * mhe_beta;
		phi_mpc   = (1.0f - blend) * phi       + blend * mhe_phi;
		cur_alt   = (1.0f - blend) * Ym        + blend * mhe_alt;
		cur_x     = (1.0f - blend) * Xm        + blend * mhe_x_dr;
		cur_y     = (1.0f - blend) * Zm        + blend * mhe_y_cr;

		// Chi cross-check: reduce chi blend when MHE and EKF2 heading diverge.
		// V comes from EKF2, so if chi_mhe is inconsistent with velocity direction
		// we should trust EKF2 chi more to maintain V-chi coherence.
		{
			constexpr float CHI_COHERENCE_THRESH = 0.1745f;  // 10 deg
			float chi_divergence = fabsf(atan2f(sinf(mhe_chi - ekf_chi), cosf(mhe_chi - ekf_chi)));
			float chi_blend = blend;
			if (chi_divergence > CHI_COHERENCE_THRESH) {
				// Scale down chi blend proportionally to divergence
				float chi_penalty = (chi_divergence - CHI_COHERENCE_THRESH) / 0.3491f;  // 0→1 over 20°
				chi_penalty = (chi_penalty > 0.7f) ? 0.7f : chi_penalty;
				chi_blend *= (1.0f - chi_penalty);
			}
			chi_mpc = (1.0f - chi_blend) * ekf_chi + chi_blend * mhe_chi;
		}

		// ---- LOS guidance ----
		if (_mpc.solve_count() < 3) {
			PX4_INFO("LOS compute: cur_x=%.1f alt=%.1f t=%.3f", (double)cur_x, (double)cur_alt, (double)t);
		}
		LosResult los = _los.compute(cur_x, cur_y, cur_alt,
					     _los.gamma_ref_prev(), _los.chi_ref_prev(),
					     t, dt);
		if (_mpc.solve_count() < 3) {
			PX4_INFO("LOS done: gamma_ref=%.3f chi_ref=%.3f dx=%.1f",
				 (double)los.gamma_ref, (double)los.chi_ref, (double)los.dx_to_target);
		}
		gamma_ref = los.gamma_ref;
		chi_ref   = los.chi_ref;

		// When dx < 50 m (near target) and MPC has solved at least once,
		// skip a new solve. The cached fin[] / delta_* already hold the
		// last command via the default-initialisation above, so no
		// action is needed inside this branch.
		if (los.dx_to_target < 50.0f && _mpc.solve_count() > 0) {
			// intentionally empty: keep cached _last_fins / _last_d*
		} else {
			// Build x_mpc[18]
			double x_mpc[MPC_NX];
			bool skip_solve = false;
			x_mpc[0]  = (double)V_mpc;
			x_mpc[1]  = (double)gamma_mpc;
			x_mpc[2]  = (double)chi_mpc;

			// Angular rates: blend MHE bias-corrected with raw gyro
			if (mhe_authority && blend > 0.01f) {
				x_mpc[3] = (1.0 - (double)blend) * (double)sc.gyro_rad[0] + (double)blend * mhe_out.x_hat[IX_P];
				x_mpc[4] = (1.0 - (double)blend) * (double)sc.gyro_rad[1] + (double)blend * mhe_out.x_hat[IX_Q];
				x_mpc[5] = (1.0 - (double)blend) * (double)sc.gyro_rad[2] + (double)blend * mhe_out.x_hat[IX_R];
			} else {
				x_mpc[3] = (double)sc.gyro_rad[0];
				x_mpc[4] = (double)sc.gyro_rad[1];
				x_mpc[5] = (double)sc.gyro_rad[2];
			}

			x_mpc[6]  = (double)alpha_mpc;
			x_mpc[7]  = (double)beta_mpc;
			x_mpc[8]  = (double)phi_mpc;
			x_mpc[9]  = (double)(cur_alt / H_SCALE);
			x_mpc[10] = (double)(cur_x / X_SCALE);
			x_mpc[11] = (double)(cur_y / Y_SCALE);
			// Commanded fin deflections (from last solve)
			x_mpc[12] = (double)delta_e;
			x_mpc[13] = (double)delta_r;
			x_mpc[14] = (double)delta_a;
			// Actual fin deflections — already updated above (before MHE push)
			x_mpc[15] = (double)_de_act;
			x_mpc[16] = (double)_dr_act;
			x_mpc[17] = (double)_da_act;

			float h_ref_scaled = _los.cruise_alt_set()
					     ? (_los.cruise_alt_target() / H_SCALE)
					     : 0.0f;

			// NaN guard: reject x_mpc before it reaches the solver
			{
				bool x_finite = true;

				for (int i = 0; i < MPC_NX; i++) {
					if (!std::isfinite(x_mpc[i])) { x_finite = false; break; }
				}

				if (!x_finite) {
					PX4_ERR("MPC state NaN at t=%.2f — skipping solve", (double)t);
					skip_solve = true;
				}
			}

			if (!skip_solve) {

			if (_mpc.solve_count() < 3) {
				PX4_INFO("MPC solve #%d: V=%.1f gamma=%.3f x_mpc[0..3]=[%.2f,%.3f,%.3f,%.4f]",
					 _mpc.solve_count(), (double)V_mpc, (double)gamma_mpc,
					 x_mpc[0], x_mpc[1], x_mpc[2], x_mpc[3]);
			}

			perf_begin(_mpc_perf);
			MpcSolveResult res = _mpc.solve(x_mpc,
							gamma_ref, chi_ref, 0.0f,
							h_ref_scaled, _los.cruise_alt_set(),
							t, dt, cur_x, cur_alt);
			perf_end(_mpc_perf);

			if (res.valid) {
				// Clamp individual deflections to physical servo limit.
				// NOTE: max_d MUST match the delta_max baked into the acados solver
				// (see SOLVER_DELTA_MAX_RAD above and m130_ocp_setup.py::delta_max).
				const float max_d = SOLVER_DELTA_MAX_RAD;
				delta_e = math::constrain(res.delta_e, -max_d, max_d);
				delta_r = math::constrain(res.delta_r, -max_d, max_d);
				delta_a = math::constrain(res.delta_a, -max_d, max_d);

				// Recompute X-fin mixing with clamped values
				fin[0] = math::constrain(delta_a - delta_e - delta_r, -max_d, max_d);
				fin[1] = math::constrain(delta_a - delta_e + delta_r, -max_d, max_d);
				fin[2] = math::constrain(delta_a + delta_e + delta_r, -max_d, max_d);
				fin[3] = math::constrain(delta_a + delta_e - delta_r, -max_d, max_d);

				// Cache for lockstep republish on intermediate ticks
				_last_fins[0] = fin[0];
				_last_fins[1] = fin[1];
				_last_fins[2] = fin[2];
				_last_fins[3] = fin[3];
				// Back-solve the actual (de, dr, da) that the CLAMPED fins
				// implement. When any fin saturates at the second clamp
				// above, the physical composition diverges from the
				// commanded (delta_e, delta_r, delta_a). MHE must see what
				// the fins actually produce, not the pre-mix command —
				// otherwise delta_*_act drives a biased aero force/moment
				// model which MHE silently absorbs into alpha or gyro_bias.
				_last_da = (fin[0] + fin[1] + fin[2] + fin[3]) * 0.25f;
				_last_de = (-fin[0] - fin[1] + fin[2] + fin[3]) * 0.25f;
				_last_dr = (-fin[0] + fin[1] + fin[2] - fin[3]) * 0.25f;
			}

			mpc_status   = res.status;
			mpc_solve_ms = res.solve_time_ms;

			} // !skip_solve
		}
	}

	// ---- Publish actuator outputs (always, to be sole authority) ----
	// HIL/SITL: publish radians to actuator_outputs_sim (XqpowerCan HIL branch expects rad).
	// Real flight: publish normalized [-1,+1] to actuator_servos (XqpowerCan multiplies by _angle_limit).
	{
		const bool sim_path = _hitl || (_param_sitl_gps.get() == 1);

		if (sim_path) {
			actuator_outputs_s ao{};
			ao.timestamp = now;
			ao.noutputs  = 4;
			ao.output[0] = fin[0];
			ao.output[1] = fin[1];
			ao.output[2] = fin[2];
			ao.output[3] = fin[3];
			_actuator_outputs_sim_pub.publish(ao);

		} else {
			const float max_d = SOLVER_DELTA_MAX_RAD;
			const float inv_max_d = 1.0f / max_d;
			actuator_servos_s as{};
			as.timestamp        = now;
			as.timestamp_sample = sc.timestamp;

			for (int i = 0; i < 4; ++i) {
				float n = fin[i] * inv_max_d;
				if (n >  1.0f) { n =  1.0f; }
				if (n < -1.0f) { n = -1.0f; }
				as.control[i] = n;
			}

			for (int i = 4; i < actuator_servos_s::NUM_CONTROLS; ++i) {
				as.control[i] = NAN;
			}

			_actuator_servos_pub.publish(as);
		}

		// Servo feedback (debug_array id=1, name="SRV_FB") is published
		// exclusively by the xqpower_can driver from real CAN measurements.
		// In HIL without hardware, the Python simulator applies its own
		// first-order servo lag model to the commands in _actuator_servos_pub
		// and logs the model trace on its side — PX4 does not emit a
		// synthetic copy, keeping id=1 a single source of truth.
	}

	// ---- Publish status (reuse rocket_gnc_status topic) ----
	{
		rocket_gnc_status_s status{};
		status.timestamp        = now;
		status.timestamp_sample = sc.timestamp;
		status.stage            = _launched ? 2 : 1;
		status.t_flight         = t;
		status.q_dyn            = (air.rho > 0.0f) ? (0.5f * air.rho * vm * vm) : 0.0f;
		status.q_kgf            = status.q_dyn / 10000.0f;
		status.pitch_accel_cmd  = gamma_ref;
		status.yaw_accel_cmd    = chi_ref;
		status.delta_roll       = delta_a;
		status.delta_pitch      = delta_e;
		status.delta_yaw        = delta_r;
		status.fin1             = fin[0];
		status.fin2             = fin[1];
		status.fin3             = fin[2];
		status.fin4             = fin[3];
		status.altitude         = _mhe_publishing ? -(float)_mhe_z : Ym;
		status.airspeed         = _mhe_publishing ? _mhe_vm : vm;
		status.rho              = air.rho;
		status.phi              = _mhe_publishing ? _mhe_phi : euler.phi();
		status.wx_filter        = mpc_solve_ms;   // repurpose: MPC solve time
		status.du_roll          = mhe_quality;     // repurpose: MHE quality
		status.out_integ_roll   = (float)_mpc.solve_count();
		status.yaw_los_deg      = _blend_alpha;    // repurpose: EKF-MHE blend factor [0..1]

		status.theta            = _mhe_publishing ? _mhe_theta : euler.theta();
		status.psi              = _mhe_publishing ? _mhe_psi   : euler.psi();

		// Compute alpha_est and gamma_rad for logging
		float gamma_rad_log = _mhe_publishing
			? atan2f(-(-_mhe_vz), sqrtf(_mhe_vx * _mhe_vx + _mhe_vy * _mhe_vy))
			: ((v_h > 1.0f) ? atan2f(-Vym, v_h) : euler.theta());
		float theta_log = _mhe_publishing ? _mhe_theta : euler.theta();
		status.alpha_est    = theta_log - gamma_rad_log;
		status.gamma_rad    = gamma_rad_log;

		status.pos_downrange  = _mhe_publishing ? _mhe_x  : Xm;
		status.pos_crossrange = _mhe_publishing ? _mhe_y  : Zm;
		status.vel_downrange  = _mhe_publishing ? _mhe_vx : Vxm;
		status.vel_down       = _mhe_publishing ? _mhe_vz : Vym;
		status.vel_crossrange = _mhe_publishing ? _mhe_vy : Vzm;

		status.bearing_deg    = atan2f(_sin_bearing, _cos_bearing) * 180.0f / M_PI_F;
		status.target_range_remaining = _target_downrange - Xm;
		status.launched       = _launched;
		status.dt_actual      = _dt_measured;
		status.dt_min         = _dt_min;
		status.dt_max         = (_dt_count > 0) ? _dt_max : 0.0f;

		// Servo mask — only trust fresh readings (< 500 ms old) so a
		// dead xqpower_can driver doesn't leave a stale mask in telemetry.
		debug_array_s dbg_srv{};

		if (_debug_array_sub.copy(&dbg_srv) && dbg_srv.id == 1
		    && (now - dbg_srv.timestamp) < 500_ms) {
			status.servo_online_mask = (uint8_t)dbg_srv.data[12];
		}

		_rocket_gnc_status_pub.publish(status);
	}

	// Periodic dt logging
	{
		static uint32_t dt_log_count = 0;
		static float    dt_log_sum   = 0.0f;
		static float    dt_log_min   = 1.0f;
		static float    dt_log_max   = 0.0f;
		dt_log_sum += dt;
		dt_log_min = fminf(dt_log_min, dt);
		dt_log_max = fmaxf(dt_log_max, dt);
		dt_log_count++;

		if (dt_log_count >= 250) {  // every ~5s at 50Hz
			PX4_INFO("dt: avg=%.3f min=%.3f max=%.3f ms (%.0f Hz)  mpc=%.1fms mhe_q=%.2f",
				 (double)(dt_log_sum / dt_log_count * 1000.0f),
				 (double)(dt_log_min * 1000.0f),
				 (double)(dt_log_max * 1000.0f),
				 (double)(dt_log_count / dt_log_sum),
				 (double)_mpc.last_solve_time_ms(),
				 (double)mhe_quality);
			dt_log_sum = 0.0f;
			dt_log_min = 1.0f;
			dt_log_max = 0.0f;
			dt_log_count = 0;
		}
	}

	// ---- Publish DEBUG_VECT("TIMING") for PIL instrumentation ----
	// Forwarded over MAVLink as DEBUG_VECT (msg 250) to pil_runner.
	//   x = mhe_us, y = mpc_us, z = total cycle_us
	// Only emitted on cycles where MPC/MHE actually ran (mpc_solve_ms > 0),
	// to keep timing CSV aligned with actual control decisions.
	const hrt_abstime cycle_end = hrt_absolute_time();
	const float cycle_us = (float)(cycle_end - _cycle_t0);
	if (mpc_solve_ms > 0.0f) {
		debug_vect_s dbg{};
		dbg.timestamp = cycle_end;
		// char[10] name — use plain memcpy on fixed buffer
		static const char kName[] = "TIMING";
		for (size_t i = 0; i < sizeof(dbg.name); i++) {
			dbg.name[i] = (i < sizeof(kName) - 1) ? kName[i] : 0;
		}
		dbg.x = mhe_solve_ms * 1000.0f;   // ms → µs
		dbg.y = mpc_solve_ms * 1000.0f;   // ms → µs
		dbg.z = cycle_us;                 // already µs
		_timing_debug_pub.publish(dbg);
	}

	perf_end(_loop_perf);
}

// ===================================================================
//  ModuleBase boilerplate
// ===================================================================

int RocketMPC::task_spawn(int argc, char *argv[])
{
	RocketMPC *instance = new RocketMPC();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int RocketMPC::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RocketMPC::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rocket M130 MPC+MHE control module.
Implements acados-based MPC controller with MHE estimator and LOS guidance.
Replaces RocketGNC for optimal trajectory tracking flights.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rocket_mpc", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rocket_mpc_main(int argc, char *argv[])
{
	return RocketMPC::main(argc, argv);
}
