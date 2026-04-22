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

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/rocket_gnc_status.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/debug_vect.h>
#include <systemlib/mavlink_log.h>

#include "mpc_controller.h"
#include "mhe_estimator.h"
#include "los_guidance.h"
#include "sensor_bridge.h"

using namespace time_literals;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

#define MODULE_NAME "rocket_mpc"

class RocketMPC : public ModuleBase<RocketMPC>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	RocketMPC();
	~RocketMPC() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	void parameters_update(bool force = false);

	// Reset all in-flight state (_launched, actuator caches, solver
	// sub-modules, MHE publishing snapshot, xval). Invoked on both
	// arm→launch transitions and disarm to prevent stale state from
	// one flight bleeding into the next and to stop the MPC/actuator
	// pipeline when the vehicle is disarmed.
	void _reset_flight_state();

	// ---------------------------------------------------------------
	// Sub-systems
	// ---------------------------------------------------------------
	MpcController  _mpc;
	MheEstimator   _mhe;
	LosGuidance    _los;
	SensorBridge   _sensor;

	// ---------------------------------------------------------------
	// uORB Subscriptions
	// ---------------------------------------------------------------
	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};   /* RAW GPS — independent of EKF2 for MHE redundancy */
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};

	// ---------------------------------------------------------------
	// uORB Publications
	// ---------------------------------------------------------------
	uORB::Publication<actuator_outputs_s>  _actuator_outputs_sim_pub{ORB_ID(actuator_outputs_sim)};
	uORB::Publication<actuator_servos_s>   _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<rocket_gnc_status_s> _rocket_gnc_status_pub{ORB_ID(rocket_gnc_status)};
	uORB::Publication<debug_vect_s>        _timing_debug_pub{ORB_ID(debug_vect)};


	// ---------------------------------------------------------------
	// State variables
	// ---------------------------------------------------------------
	hrt_abstime _time_launch{0};
	bool        _launched{false};
	float       _launch_dv{0.0f};

	bool  _armed{false};
	float _arm_origin_x{0.0f};
	float _arm_origin_y{0.0f};
	float _arm_origin_z{0.0f};

	float _cos_bearing{1.0f};
	float _sin_bearing{0.0f};
	float _target_downrange{0.0f};

	// Actual launch altitude MSL captured at arming (replaces hardcoded param for MHE)
	float _actual_launch_alt_msl{0.0f};
	bool  _launch_alt_captured{false};

	bool  _hitl{false};  // HITL mode: use lpos reference frame for MHE (avoid GPS origin mismatch)

	// Cached fin commands for lockstep republish
	float _last_fins[4]{0.0f, 0.0f, 0.0f, 0.0f};
	float _last_de{0.0f}, _last_dr{0.0f}, _last_da{0.0f};
	hrt_abstime _last_mpc_solve_time{0};

	// Tracked actual fin deflections (first-order lag filter)
	float _de_act{0.0f}, _dr_act{0.0f}, _da_act{0.0f};

	// MHE-derived state cache (for logging & publish)
	bool  _mhe_publishing{false};
	float _mhe_phi{0.0f}, _mhe_theta{0.0f}, _mhe_psi{0.0f};
	float _mhe_x{0.0f}, _mhe_y{0.0f}, _mhe_z{0.0f};
	float _mhe_vx{0.0f}, _mhe_vy{0.0f}, _mhe_vz{0.0f};
	float _mhe_vm{0.0f};

	// EKF-MHE cooperative blend state
	float _blend_alpha{0.0f};  // current blend factor 0=EKF, 1=MHE

	// Cross-validation: EKF↔MHE consistency monitor
	struct {
		int   consec_diverge{0};       // consecutive cycles where MHE diverges from EKF
		float diverge_start_t{0.0f};   // flight time when divergence streak began
		float penalty{0.0f};           // cross-validation penalty applied to blend [0..1]
		float gamma_err{0.0f};         // last |gamma_mhe - gamma_ekf|
		float chi_err{0.0f};           // last |chi_mhe - chi_ekf|
		float alt_err{0.0f};           // last |alt_mhe - alt_ekf|
	} _xval;

	// dt measurement
	hrt_abstime _prev_run_time{0};
	float       _dt_measured{0.02f};
	float       _dt_min{1.0f};
	float       _dt_max{0.0f};
	float       _dt_sum{0.0f};
	uint32_t    _dt_count{0};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _mpc_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": mpc_solve")};
	perf_counter_t _mhe_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": mhe_solve")};
	orb_advert_t   _mavlink_log_pub{nullptr};

	// ---------------------------------------------------------------
	// Parameters
	// ---------------------------------------------------------------
	DEFINE_PARAMETERS(
		// Timing
		(ParamFloat<px4::params::ROCKET_T_CTRL>)    _param_t_ctrl,

		// Target
		(ParamFloat<px4::params::ROCKET_XTRGT>)     _param_xtrgt,
		(ParamFloat<px4::params::ROCKET_HTRGT>)     _param_htrgt,
		(ParamFloat<px4::params::ROCKET_IMP_ANG>)   _param_imp_ang,
		(ParamFloat<px4::params::ROCKET_CRUISE_P>)  _param_cruise_p,

		// Mass / propulsion
		(ParamFloat<px4::params::ROCKET_MASS_F>)    _param_mass_full,
		(ParamFloat<px4::params::ROCKET_MASS_D>)    _param_mass_dry,
		(ParamFloat<px4::params::ROCKET_TBURN>)     _param_burn_time,
		(ParamFloat<px4::params::ROCKET_IMPULS>)    _param_impulse,
		(ParamFloat<px4::params::ROCKET_THRUST>)    _param_thrust,
		(ParamFloat<px4::params::ROCKET_T_TAIL>)    _param_t_tail,

		// Inertias
		(ParamFloat<px4::params::ROCKET_IXX_F>)     _param_ixx_f,
		(ParamFloat<px4::params::ROCKET_IXX_D>)     _param_ixx_d,
		(ParamFloat<px4::params::ROCKET_IYY_F>)     _param_iyy_f,
		(ParamFloat<px4::params::ROCKET_IYY_D>)     _param_iyy_d,
		(ParamFloat<px4::params::ROCKET_IZZ_F>)     _param_izz_f,
		(ParamFloat<px4::params::ROCKET_IZZ_D>)     _param_izz_d,

		// Servo
		(ParamFloat<px4::params::ROCKET_TAU_SRV>)   _param_tau_servo,

		// Launch site
		(ParamFloat<px4::params::ROCKET_L_ALT>)     _param_launch_alt,
		(ParamFloat<px4::params::ROCKET_L_PITCH>)   _param_launch_pitch,

		// MHE
		(ParamFloat<px4::params::ROCKET_MHE_QG>)    _param_mhe_qg,

		// MPC solver
		(ParamFloat<px4::params::ROCKET_MPC_TF>)    _param_mpc_tf,

		// SITL: use EKF2 lpos as GPS substitute for MHE (0=disabled, 1=enabled)
		(ParamInt<px4::params::ROCKET_SITL_GPS>)     _param_sitl_gps
	)
};
