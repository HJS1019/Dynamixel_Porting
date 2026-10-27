/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "MulticopterPositionControl.hpp"

#include <float.h>
#include <cmath>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>
#include "PositionControl/ControlMath.hpp"

using namespace matrix;

MulticopterPositionControl::MulticopterPositionControl(bool vtol) :
	SuperBlock(nullptr, "MPC"),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_vel_x_deriv(this, "VELD"),  /**< velocity derivative in x */
	_vel_y_deriv(this, "VELD"),  /**< velocity derivative in y*/
	_vel_z_deriv(this, "VELD")   /**< velocity derivative in z */
{
	parameters_update(true);
	_tilt_limit_slew_rate.setSlewRate(.2f);
}

MulticopterPositionControl::~MulticopterPositionControl()
{
	perf_free(_cycle_perf);
}

bool MulticopterPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void MulticopterPositionControl::parameters_update(bool force)
{
	// check for parameter updates

	// ############################### PARAMETER UPDATE START ####################################### //
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		SuperBlock::updateParams();
		// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //


		int num_changed =
			0; // 파라미터를 카운트 해서 0보다 크면 파라미터가 바뀌었다고 알려주는 부분

		if (_param_sys_vehicle_resp.get() >= 0.f) {
			// make it less sensitive at the lower end
			float responsiveness = _param_sys_vehicle_resp.get() * _param_sys_vehicle_resp.get();

			num_changed += _param_mpc_acc_hor.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_hor_max.commit_no_notification(math::lerp(2.f, 15.f, responsiveness));
			num_changed += _param_mpc_man_y_max.commit_no_notification(math::lerp(80.f, 450.f, responsiveness));

			if (responsiveness > 0.6f) {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(0.f);

			} else {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(math::lerp(0.5f, 0.f, responsiveness / 0.6f));
			}

			if (responsiveness < 0.5f) {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(45.f);

			} else {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(math::min(MAX_SAFE_TILT_DEG, math::lerp(45.f, 70.f,
						(responsiveness - 0.5f) * 2.f)));
			}

			num_changed += _param_mpc_acc_down_max.commit_no_notification(math::lerp(0.8f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_up_max.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_jerk_max.commit_no_notification(math::lerp(2.f, 50.f, responsiveness));
			num_changed += _param_mpc_jerk_auto.commit_no_notification(math::lerp(1.f, 25.f, responsiveness));
		}

		if (_param_mpc_xy_vel_all.get() >= 0.f) {
			float xy_vel = _param_mpc_xy_vel_all.get();
			num_changed += _param_mpc_vel_manual.commit_no_notification(xy_vel);
			num_changed += _param_mpc_vel_man_back.commit_no_notification(-1.f);
			num_changed += _param_mpc_vel_man_side.commit_no_notification(-1.f);
			num_changed += _param_mpc_xy_cruise.commit_no_notification(xy_vel);
			num_changed += _param_mpc_xy_vel_max.commit_no_notification(xy_vel);
		}

		if (_param_mpc_z_vel_all.get() >= 0.f) {
			float z_vel = _param_mpc_z_vel_all.get();
			num_changed += _param_mpc_z_v_auto_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_vel_max_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_v_auto_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_z_vel_max_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_tko_speed.commit_no_notification(z_vel * 0.6f);
			num_changed += _param_mpc_land_speed.commit_no_notification(z_vel * 0.5f);
		}

		if (num_changed > 0) {
			param_notify_changes();
		}


		// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

		_takeoff.setSpoolupTime(_param_com_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.setInitialRampValue(0.f);

		// Position, Velocity Controller Gain Definition Part
		_control.setPositionGains(
			Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get()),
			Vector3f(_param_mpc_xy_d.get(), _param_mpc_xy_d.get(), _param_mpc_z_d.get()));

		_control.setVelocityGains(
			Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get()),
			Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get()),
			Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get()));

		gain_check = {_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get()};

		// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

		_control.setHorizontalThrustMargin(_param_mpc_thr_xy_marg.get()); // normalized value ( -1~1 )
		_control.decoupleHorizontalAndVecticalAcceleration(_param_mpc_acc_decouple.get());

		// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
		// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

		// Parameter 들 중에 딱히 Translational Force의 정규화에 영향을 미치는 애들은 없어 보임

		// Check that the design parameters are inside the absolute maximum constraints
		if (_param_mpc_xy_cruise.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_xy_cruise.set(_param_mpc_xy_vel_max.get());
			_param_mpc_xy_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_XY_CRUISE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_cruise_set"), events::Log::Warning,
					    "Cruise speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_vel_manual.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_vel_manual.set(_param_mpc_xy_vel_max.get());
			_param_mpc_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MANUAL</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_set"), events::Log::Warning,
					    "Manual speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_vel_man_back.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_back.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_back.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual backward speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_BACK</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_back_set"), events::Log::Warning,
					    "Manual backward speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		if (_param_mpc_vel_man_side.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_side.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_side.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual sideways speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_SIDE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_side_set"), events::Log::Warning,
					    "Manual sideways speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		if (_param_mpc_z_v_auto_up.get() > _param_mpc_z_vel_max_up.get()) {
			_param_mpc_z_v_auto_up.set(_param_mpc_z_vel_max_up.get());
			_param_mpc_z_v_auto_up.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Ascent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_UP</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_up_vel_set"), events::Log::Warning,
					    "Ascent speed has been constrained by max speed", _param_mpc_z_vel_max_up.get());
		}

		if (_param_mpc_z_v_auto_dn.get() > _param_mpc_z_vel_max_dn.get()) {
			_param_mpc_z_v_auto_dn.set(_param_mpc_z_vel_max_dn.get());
			_param_mpc_z_v_auto_dn.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Descent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_DN</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_down_vel_set"), events::Log::Warning,
					    "Descent speed has been constrained by max speed", _param_mpc_z_vel_max_dn.get());
		}


		//HOVER_THRUST_MIN = 0.05f;
		//HOVER_THRUST_MAX = 0.9f;

		// 일반적으로 hover thrust는 0.5로 되어있음 --> 조종기 스틱 중간임 --> 그런데 이제 무거우면 intergratio쌓여서
		// hover thrust 비율이 점차 올라가고 if "hover thrust > thrust max"--> hover_thrust = thrust_max가 되는 로직임
		// 반대로는 이제 hover_thrust < thrust_min --> hover_thrust = thrust_min이 됨

		if (_param_mpc_thr_hover.get() > _param_mpc_thr_max.get() ||
		    _param_mpc_thr_hover.get() < _param_mpc_thr_min.get()) {
			_param_mpc_thr_hover.set(math::constrain(_param_mpc_thr_hover.get(), _param_mpc_thr_min.get(),
						 _param_mpc_thr_max.get()));
			_param_mpc_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max\t");
			/* EVENT
			 * @description <param>MPC_THR_HOVER</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_hover_thrust_set"), events::Log::Warning,
					    "Hover thrust has been constrained by min/max thrust", _param_mpc_thr_hover.get());
		}



		if (!_param_mpc_use_hte.get() || !_hover_thrust_initialized) {
			_control.setHoverThrust(_param_mpc_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		// initialize vectors from params and enforce constraints
		_param_mpc_tko_speed.set(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get()));
		_param_mpc_land_speed.set(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get()));


		_control.setVelocityLimits(_param_mpc_xy_vel_max.get(),
					   _param_mpc_z_vel_max_up.get(),
					   _param_mpc_z_vel_max_dn.get());

		_control.setAccelerationLimits(_param_mpc_xy_acc_max.get(),
					       _param_mpc_z_acc_max_up.get());

	}

	// ############################### PARAMETER UPDATE END ####################################### //
}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //


PositionControlStates MulticopterPositionControl::set_vehicle_states(const vehicle_local_position_s
		&vehicle_local_position)
{
	PositionControlStates states;

	const Vector2f position_xy(vehicle_local_position.x, vehicle_local_position.y);

	// only set position states if valid and finite
	if (vehicle_local_position.xy_valid && position_xy.isAllFinite()) {
		states.position.xy() = position_xy;

	} else {
		states.position(0) = states.position(1) = NAN;
	}

	if (PX4_ISFINITE(vehicle_local_position.z) && vehicle_local_position.z_valid) {
		states.position(2) = vehicle_local_position.z;

	} else {
		states.position(2) = NAN;
	}

	const Vector2f velocity_xy(vehicle_local_position.vx, vehicle_local_position.vy);

	if (vehicle_local_position.v_xy_valid && velocity_xy.isAllFinite()) {
		states.velocity.xy() = velocity_xy;
		states.acceleration(0) = _vel_x_deriv.update(velocity_xy(0));
		states.acceleration(1) = _vel_y_deriv.update(velocity_xy(1));

	} else {
		states.velocity(0) = states.velocity(1) = NAN;
		states.acceleration(0) = states.acceleration(1) = NAN;

		// reset derivatives to prevent acceleration spikes when regaining velocity
		_vel_x_deriv.reset();
		_vel_y_deriv.reset();
	}

	if (PX4_ISFINITE(vehicle_local_position.vz) && vehicle_local_position.v_z_valid) {
		states.velocity(2) = vehicle_local_position.vz;
		states.acceleration(2) = _vel_z_deriv.update(states.velocity(2));

	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;

		// reset derivative to prevent acceleration spikes when regaining velocity
		_vel_z_deriv.reset();
	}

	states.yaw = vehicle_local_position.heading;

	return states;
}

bool MulticopterPositionControl::latchCurrentPose(matrix::Vector4f &setpoint) const
{
	if (!_states.position.isAllFinite() || !PX4_ISFINITE(_states.yaw)) {
		return false;
	}

	setpoint = {_states.position(0), _states.position(1), _states.position(2), _states.yaw};
	return true;
}

bool MulticopterPositionControl::isAutoNavigationState(uint8_t nav_state)
{
	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		return true;

	default:
		return false;
	}
}

bool MulticopterPositionControl::isTrajectorySetpointValid(const trajectory_setpoint_s &trajectory_setpoint)
{
	for (int axis = 0; axis < 3; ++axis) {
		if (PX4_ISFINITE(trajectory_setpoint.position[axis])
		    || PX4_ISFINITE(trajectory_setpoint.velocity[axis])) {
			return true;
		}
	}

	return false;
}

void MulticopterPositionControl::updateAutoSetpoint(const trajectory_setpoint_s &trajectory_setpoint, float dt)
{
	for (int axis = 0; axis < 2; ++axis) {
		if (PX4_ISFINITE(trajectory_setpoint.position[axis])) {
			_auto_pose_setpoint(axis) = trajectory_setpoint.position[axis];
		}
	}

	if (PX4_ISFINITE(trajectory_setpoint.yaw)) {
		_auto_pose_setpoint(3) = trajectory_setpoint.yaw;
	}

	for (int axis = 0; axis < 3; ++axis) {
		_auto_velocity_ff(axis) = PX4_ISFINITE(trajectory_setpoint.velocity[axis])
					  ? trajectory_setpoint.velocity[axis] : 0.f;
	}

	_auto_velocity_ff(3) = 0.f;

	float speed_up = _param_mpc_z_vel_max_up.get();
	float speed_down = _param_mpc_z_vel_max_dn.get();

	if (PX4_ISFINITE(_vehicle_constraints.speed_up)) {
		speed_up = math::min(speed_up, math::max(_vehicle_constraints.speed_up, 0.f));
	}

	if (PX4_ISFINITE(_vehicle_constraints.speed_down)) {
		speed_down = math::min(speed_down, math::max(_vehicle_constraints.speed_down, 0.f));
	}

	if (PX4_ISFINITE(trajectory_setpoint.velocity[2])) {
		_auto_velocity_ff(2) = math::constrain(trajectory_setpoint.velocity[2], -speed_up, speed_down);
	}

	if (PX4_ISFINITE(trajectory_setpoint.position[2])) {
		_auto_pose_setpoint(2) = trajectory_setpoint.position[2];

	} else if (PX4_ISFINITE(trajectory_setpoint.velocity[2])) {
		_auto_pose_setpoint(2) += _auto_velocity_ff(2) * dt;
	}
}

void MulticopterPositionControl::publishControllerStatus(hrt_abstime now)
{
	if (!_mode_changed && _last_status_publish != 0 && now >= _last_status_publish
	    && now - _last_status_publish < 200_ms) {
		return;
	}

	multicopter_position_control_status_s status{};
	status.timestamp = now;
	status.nav_state = _vehicle_status.nav_state;
	status.auto_takeoff = _auto_takeoff;
	status.auto_land = _auto_land;
	status.mode_changed = _mode_changed;
	status.trajectory_fresh = _trajectory_fresh;
	status.takeoff_intent = _takeoff_intent;
	status.takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());
	status.raw_takeoff_ramp = _raw_takeoff_ramp;
	status.velocity_limit_up = _velocity_limit_up;
	status.active_position_z = _active_pose_setpoint(2);
	status.active_velocity_z = _active_velocity_ff(2);
	status.velocity_setpoint_z = _control.getVelocitySetpoint()(2);
	status.acceleration_setpoint_z = _control.getAccelerationSetpoint()(2);
	status.force_setpoint_z = _control.getThrustSetpoint()(2);
	status.landed = _vehicle_land_detected.landed;
	status.ground_contact = _vehicle_land_detected.ground_contact;
	status.attitude_valid = _control.hasValidAttitude();
	status.controller_update_success = _controller_update_success;
	status.failsafe_update_success = _failsafe_update_success;
	_controller_status_pub.publish(status);
	_last_status_publish = now;
}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

void MulticopterPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);
	vehicle_local_position_s vehicle_local_position{};

	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	// 먼저 최신 LPOS를 받아와야 timestamp_sample을 쓸 수 있다
	if (!_local_pos_sub.update(&vehicle_local_position)) {
		// 새 샘플이 없으면 현재 시간 기준으로 dt 보정 후 다음 사이클로
		const hrt_abstime now = hrt_absolute_time();
		const hrt_abstime elapsed = now >= _time_stamp_last_loop ? now - _time_stamp_last_loop : 0;
		const float dt = math::constrain(elapsed * 1e-6f, 0.002f, 0.04f);
		_time_stamp_last_loop = now;
		setDt(dt);
		perf_end(_cycle_perf);
		return;
	}

	// 새 샘플이 있으므로 그 타임스탬프로 dt 계산
	const hrt_abstime elapsed = vehicle_local_position.timestamp_sample >= _time_stamp_last_loop
				    ? vehicle_local_position.timestamp_sample - _time_stamp_last_loop : 0;
	const float dt = math::constrain(elapsed * 1e-6f, 0.002f, 0.04f);
	_time_stamp_last_loop = vehicle_local_position.timestamp_sample;
	setDt(dt);


	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// matrix::Vector3f imu_att;
	vehicle_attitude_s vehicle_attitude{};

	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
		matrix::Quatf q(vehicle_attitude.q);  // PX4에서는 q0 = w, q1 = x, ...

		if (q.isAllFinite() && q.norm() > FLT_EPSILON) {
			q.normalize();
			const matrix::Eulerf euler(q);  // RPY 변환
			roll  = euler.phi();    // x
			pitch = euler.theta();  // y
			yaw   = euler.psi();    // z
			_control.updateAttitude(matrix::Dcmf(q));
		}
	}

	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	_vehicle_control_mode_sub.update(&_vehicle_control_mode); // For vehicle armed state check
	_vehicle_status_sub.update(&_vehicle_status);

	_custom_control_mode_sub.update(&_custom_control_mode);

	manual_control_setpoint_s rpyz_cmd{};


	//FROM SBUS
	if (_manual_control_setpoint_sub.update(&rpyz_cmd)) {
		const Vector4f manual_candidate{rpyz_cmd.pitch, rpyz_cmd.roll, -rpyz_cmd.throttle, rpyz_cmd.yaw};

		if (manual_candidate.isAllFinite()) {
			manual_setpoint = manual_candidate;
		}
	}

	// FROM ROS SEUK. position command
	if (_custom_command_position_mode_sub.update(&_custom_command_position_mode)) {
		const Vector4f position_candidate{_custom_command_position_mode.setpoint[0],
						  _custom_command_position_mode.setpoint[1],
						  _custom_command_position_mode.setpoint[2],
						  _custom_command_position_mode.setpoint[3]};

		if (position_candidate.isAllFinite()) {
			custom_command_position = position_candidate;
		}
	}


	{
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		_states = set_vehicle_states(vehicle_local_position);

		if (vehicle_local_position.xy_global && PX4_ISFINITE(vehicle_local_position.ref_lat)
		    && PX4_ISFINITE(vehicle_local_position.ref_lon)
		    && PX4_ISFINITE(vehicle_local_position.x) && PX4_ISFINITE(vehicle_local_position.y)) {
			MapProjection mp;
			mp.initReference(vehicle_local_position.ref_lat, vehicle_local_position.ref_lon, hrt_absolute_time());
			mp.reproject(vehicle_local_position.x, vehicle_local_position.y, lat, lon);  // local → global
		}

		alt = vehicle_local_position.z;
	}

	const bool armed = _vehicle_control_mode.flag_armed;

	if ((!armed || (armed && !_previous_armed)) && _states.position.isAllFinite() && PX4_ISFINITE(_states.yaw)) {
		base_setpoint = {_states.position(0), _states.position(1), _states.position(2), _states.yaw};
	}

	_previous_armed = armed;

	/*
	if(_custom_control_mode.sine_motion_flag){

		wave_z = wave_amp*sin(wave_freq*time_count);
		time_count += dt;
	}
	else{
		wave_z = 0.f;
		time_count = 0.f;
	}*/



	//float wave_duration = 3.0f;       // 전체 파형 주기 (초)

	/*
	if (_custom_control_mode.sine_motion_flag) {

		float phase = fmodf(time_count / wave_duration, 1.0f);  // 주기적 phase [0, 1)

		// 반복 가능한 부드러운 코사인 파형: 0.3 * cos(2π * t/T)
		wave_z = wave_amp * sinf(2.f * (float)M_PI * phase);  // 시작: +0.3 → 0 → -0.3 → ...

		time_count += dt;

	} else {
		wave_z = 0.f;
		time_count = 0.f;
	}*/

	//float segment_duration = wave_duration / 2.f;  // 각 단계는 전체의 1/4 주기
	//float step_amp = 0.1f;

	if (_custom_control_mode.sine_motion_flag) {

		wave_z = 1.0f;

	} else {
		wave_z = 0.f;
		time_count = 0.f;
	}


	// SEUK
	// pose_setpoint: 최종 커맨드
	// base_setpoint: 암 이전에 픽스호크 위치. 땅바닥에서 흘러가는거 대응
	// manual_setpoint: sbus 신호로부터 생성
	// custom_command_position: ros2에서 날라오는 커맨드 위치
	// 각각 인덱스별로 x, y, z, yaw임
	/*const bool auto_enabled = _vehicle_control_mode.flag_control_auto_enabled;

	if (!auto_enabled) {
		const float altitude_limit = 0.5f;

		if (manual_setpoint(2) < -0.4f) { pose_z_setpoint -= 0.001f; }
		else if (manual_setpoint(2) > 0.4f) { pose_z_setpoint += 0.001f; }

		pose_z_setpoint = math::constrain(pose_z_setpoint, -altitude_limit, 0.f);
		pose_setpoint(0) = base_setpoint(0) + manual_setpoint(0) + custom_command_position(0);
		pose_setpoint(1) = base_setpoint(1) + manual_setpoint(1) + custom_command_position(1);
	//pose_setpoint(2) = base_setpoint(2) + manual_setpoint(2);

		pose_setpoint(2) = base_setpoint(2) + pose_z_setpoint + custom_command_position(2);
		pose_setpoint(3) = base_setpoint(3) + manual_setpoint(3) + custom_command_position(3);
	}

	// if a goto setpoint available this publishes a trajectory setpoint to go there

	if (_goto_control.checkForSetpoint(vehicle_local_position.timestamp_sample,
		_vehicle_control_mode.flag_multicopter_position_control_enabled)) {
			_goto_control.update(dt, _states.position, _states.yaw, pose_setpoint);}*/

	const hrt_abstime now = hrt_absolute_time();

	// Update both AUTO inputs before mode handling and takeoff intent are evaluated.
	_vehicle_constraints_sub.update(&_vehicle_constraints);
	_trajectory_setpoint_sub.update(&_trajectory_setpoint);

	// FROM ROS SEUK. velocity command (feed forward)
	if (_custom_command_velocity_mode_sub.update(&_custom_command_velocity_mode)) {
		const Vector4f velocity_candidate{_custom_command_velocity_mode.setpoint[0],
						  _custom_command_velocity_mode.setpoint[1],
						  _custom_command_velocity_mode.setpoint[2],
						  _custom_command_velocity_mode.setpoint[3]};

		if (velocity_candidate.isAllFinite()) {
			custom_command_velocity = velocity_candidate;
		}
	}

	const uint8_t nav_state = _vehicle_status.nav_state;
	const bool auto_enabled = isAutoNavigationState(nav_state);
	_auto_takeoff = nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF;
	_auto_land = nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
	_mode_changed = nav_state != _previous_nav_state;

	if (auto_enabled && (!_auto_mode_active || _mode_changed)) {
		_auto_velocity_ff.zero();
		_auto_pose_latched = latchCurrentPose(_auto_pose_setpoint);
		_auto_mode_entry_time = now;

	} else if (auto_enabled && !_auto_pose_latched) {
		// If local position was invalid at mode entry, latch the first valid pose
		// before accepting a trajectory for the new mode.
		_auto_pose_latched = latchCurrentPose(_auto_pose_setpoint);

	} else if (!auto_enabled && _auto_mode_active) {
		_auto_velocity_ff.zero();
		_auto_pose_latched = false;
		_auto_mode_entry_time = 0;

		if (_states.position.isAllFinite() && PX4_ISFINITE(_states.yaw)) {
			base_setpoint(0) = _states.position(0) - manual_setpoint(0) - custom_command_position(0);
			base_setpoint(1) = _states.position(1) - manual_setpoint(1) - custom_command_position(1);
			base_setpoint(2) = _states.position(2) - custom_command_position(2);
			base_setpoint(3) = _states.yaw - manual_setpoint(3) - custom_command_position(3);
			pose_z_setpoint = 0.f;
		}
	}

	_auto_mode_active = auto_enabled;
	_previous_nav_state = nav_state;

	const hrt_abstime trajectory_timestamp = _trajectory_setpoint.timestamp;
	const bool trajectory_not_from_previous_mode = trajectory_timestamp != 0
			&& trajectory_timestamp >= _auto_mode_entry_time;
	const bool trajectory_not_in_future = trajectory_timestamp <= now;
	const bool trajectory_within_timeout = trajectory_not_in_future
					       && now - trajectory_timestamp <= TRAJECTORY_STREAM_TIMEOUT_US;
	_trajectory_fresh = auto_enabled && _auto_pose_latched
			    && trajectory_not_from_previous_mode && trajectory_within_timeout
			    && isTrajectorySetpointValid(_trajectory_setpoint);

	if (!auto_enabled) {
		//const float altitude_limit = 30.f;   // 운용 고도에 맞춰 조정
		const float altitude_limit = 1.5f;   // 운용 고도에 맞춰 조정
		const float manual_z_rate  = 0.2f;   // [m/s]

		if (manual_setpoint(2) < -0.4f) {
			pose_z_setpoint -= manual_z_rate * dt;

		} else if (manual_setpoint(2) > 0.4f) {
			pose_z_setpoint += manual_z_rate * dt;
		}

		pose_z_setpoint = math::constrain(pose_z_setpoint, -altitude_limit, 0.f);

		pose_setpoint(0) = base_setpoint(0) + manual_setpoint(0) + custom_command_position(0);
		pose_setpoint(1) = base_setpoint(1) + manual_setpoint(1) + custom_command_position(1);
		pose_setpoint(2) = base_setpoint(2) + pose_z_setpoint    + custom_command_position(2);
		pose_setpoint(3) = base_setpoint(3) + manual_setpoint(3) + custom_command_position(3);
	}

	_takeoff_intent = _vehicle_constraints.want_takeoff || (_auto_takeoff && _trajectory_fresh);
	const float takeoff_target_speed =
		math::max(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get()), 0.f);
	_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed,
				    _vehicle_land_detected.landed,
				    _takeoff_intent,
				    takeoff_target_speed,
				    _param_com_throw_en.get(),
				    now);

	_raw_takeoff_ramp = _takeoff.updateRamp(dt, takeoff_target_speed);
	const bool takeoff_ramp_active = _takeoff.getTakeoffState() < TakeoffState::flight;
	_velocity_limit_up = takeoff_ramp_active ? _raw_takeoff_ramp : _param_mpc_z_vel_max_up.get();
	_control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _velocity_limit_up, _param_mpc_z_vel_max_dn.get());

	takeoff_status_s takeoff_status{};
	takeoff_status.takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());
	takeoff_status.tilt_limit    = math::radians(_param_mpc_tiltmax_air.get());
	takeoff_status.timestamp     = now;
	_takeoff_status_pub.publish(takeoff_status);

	if (auto_enabled) {
		if (_trajectory_fresh) {
			if (_auto_land) {
				// LAND position[2] can be the ground altitude. Integrate the limited
				// positive-down velocity from the mode-entry height instead.
				for (int axis = 0; axis < 2; ++axis) {
					if (PX4_ISFINITE(_trajectory_setpoint.position[axis])) {
						_auto_pose_setpoint(axis) = _trajectory_setpoint.position[axis];
					}
				}

				if (PX4_ISFINITE(_trajectory_setpoint.yaw)) {
					_auto_pose_setpoint(3) = _trajectory_setpoint.yaw;
				}

				_auto_velocity_ff.zero();

				if (PX4_ISFINITE(_trajectory_setpoint.velocity[2])) {
					float speed_down =
						math::max(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get()), 0.f);

					if (PX4_ISFINITE(_vehicle_constraints.speed_down)) {
						speed_down = math::min(speed_down, math::max(_vehicle_constraints.speed_down, 0.f));
					}

					const float descent_speed = math::constrain(_trajectory_setpoint.velocity[2], 0.f,
								    speed_down);
					_auto_velocity_ff(2) = descent_speed;
					_auto_pose_setpoint(2) += descent_speed * dt;
				}

			} else {
				updateAutoSetpoint(_trajectory_setpoint, dt);
			}

		} else {
			// Freeze the latched/last active pose. Do not chase the current position
			// and do not continue integrating a stale velocity command.
			_auto_velocity_ff.zero();
		}

		_active_pose_setpoint = _auto_pose_setpoint;
		_active_velocity_ff = _auto_velocity_ff;

	} else {
		if (!_vehicle_control_mode.flag_armed) {
			custom_command_velocity.zero();
		}

		_active_pose_setpoint = pose_setpoint;
		_active_velocity_ff = custom_command_velocity;
	}

	_control.setInputSetpoint(_active_pose_setpoint);
	_control.setVelocityFeedforward(_active_velocity_ff);

	_control.setState(_states); // translational actual value setting (position, velocity, acceleration)
	const bool controller_enabled = _vehicle_control_mode.flag_armed;

	// ########################################################################################################### //
	// ########################################### Run Position Control ########################################## //
	// ########################################################################################################### //

	_controller_update_success = _control.update(dt, controller_enabled,
				     _vehicle_control_mode.flag_control_position_enabled);
	_failsafe_update_success = true;

	if (!_controller_update_success) {
		// Failsafe
		_control.setInputSetpoint(_active_pose_setpoint);
		_control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _velocity_limit_up, _param_mpc_z_vel_max_dn.get());
		_control.setAccelerationLimits(_param_mpc_xy_acc_max.get(), _param_mpc_z_acc_max_up.get());
		_failsafe_update_success = _control.update(dt, controller_enabled,
					   _vehicle_control_mode.flag_control_position_enabled);

		if (!_failsafe_update_success && hrt_elapsed_time(&_last_warn) > 1_s) {
			PX4_WARN("position controller input invalid");
			_last_warn = now;
		}
	}


	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	// Publish internal position control setpoints
	// on top of the input/feed-forward setpoints these containt the PID corrections
	// This message is used by other modules (such as Landdetector) to determine vehicle intention.
	vehicle_local_position_setpoint_s local_pos_sp{};
	_control.getLocalPositionSetpoint(local_pos_sp);
	local_pos_sp.timestamp = hrt_absolute_time();
	_local_pos_sp_pub.publish(local_pos_sp);

	// Publish attitude setpoint output
	vehicle_attitude_setpoint_s attitude_setpoint{};
	_control.getAttitudeSetpoint(attitude_setpoint, _vehicle_control_mode.flag_control_position_enabled, manual_setpoint);
	attitude_setpoint.timestamp = hrt_absolute_time();



	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
	publishControllerStatus(now);

	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	perf_end(_cycle_perf);
}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //


int MulticopterPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterPositionControl *instance = new MulticopterPositionControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterPositionControl::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("nav_state: %" PRIu8 ", source_id: %" PRIu8 ", auto_takeoff: %s, auto_land: %s, mode_changed: %s",
		 _vehicle_status.nav_state,
		 _vehicle_control_mode.source_id,
		 _auto_takeoff ? "true" : "false",
		 _auto_land ? "true" : "false",
		 _mode_changed ? "true" : "false");
	PX4_INFO("trajectory_fresh: %s, takeoff_intent: %s, trajectory z: %.3f, vz: %.3f",
		 _trajectory_fresh ? "true" : "false",
		 _takeoff_intent ? "true" : "false",
		 (double)_trajectory_setpoint.position[2], (double)_trajectory_setpoint.velocity[2]);
	PX4_INFO("takeoff_state: %" PRIu8 ", raw_ramp: %.3f, velocity_limit_up: %.3f",
		 static_cast<uint8_t>(_takeoff.getTakeoffState()),
		 (double)_raw_takeoff_ramp,
		 (double)_velocity_limit_up);
	PX4_INFO("active pose z: %.3f, velocity ff z: %.3f, controller vz: %.3f",
		 (double)_active_pose_setpoint(2), (double)_active_velocity_ff(2),
		 (double)_control.getVelocitySetpoint()(2));
	PX4_INFO("controller acceleration z: %.3f, force/thrust z: %.3f",
		 (double)_control.getAccelerationSetpoint()(2),
		 (double)_control.getThrustSetpoint()(2));
	PX4_INFO("landed: %s, ground_contact: %s, armed: %s, attitude_valid: %s",
		 _vehicle_land_detected.landed ? "true" : "false",
		 _vehicle_land_detected.ground_contact ? "true" : "false",
		 _vehicle_control_mode.flag_armed ? "true" : "false",
		 _control.hasValidAttitude() ? "true" : "false");
	PX4_INFO("controller update: %s, failsafe update: %s",
		 _controller_update_success ? "valid" : "invalid",
		 _failsafe_update_success ? "valid" : "invalid");
	PX4_INFO("vx P : %f | vy P : %f | vz P : %f", (double)gain_check(0), (double)gain_check(1), (double)gain_check(2));
	const matrix::Vector3f &fxyz = _control.getThrustSetpoint();
	PX4_INFO("force_cmd = Fx : %f| Fy : %f| Fz : %f|", (double)fxyz(0), (double)fxyz(1), (double)fxyz(2));

	PX4_INFO("XYZ = x : %f| y : %f| z : %f|", (double)lat, (double)lon, (double)alt);

	// PX4_INFO("dt check = %f", (double)dt_check);
	//PX4_INFO("armed_flag check : %s", _vehicle_control_mode.flag_armed ? "true" : "false" );

	return 0;
}


int MulticopterPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[])
{
	return MulticopterPositionControl::main(argc, argv);
}
