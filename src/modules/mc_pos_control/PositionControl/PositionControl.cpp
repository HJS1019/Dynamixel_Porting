/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/custom_dt.h>
#include <drivers/drv_hrt.h>

//lib/geo/geo.h float CONSTANTS_ONE_G = 9.80665f;

using namespace matrix;


void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setAccelerationLimits(const float acc_horizontal, const float acc_vertical)
{

	_lim_acc_horizontal = acc_horizontal;
	_lim_acc_vertical = acc_vertical;

}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 0.5f);//math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}


bool PositionControl::updateAttitude(const matrix::Dcmf &R)
{
	if (!R.isAllFinite()) {
		return false;
	}

	_w2b = R.transpose();
	_attitude_valid = true;
	return true;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position; // world
	_vel = states.velocity; // world
	_yaw = states.yaw; // RPY
	_vel_dot = states.acceleration; // world??
}

void PositionControl::setInputSetpoint(const matrix::Vector4f &pose_setpoint)
{
	_pos_sp(0) = pose_setpoint(0);
	_pos_sp(1) = pose_setpoint(1);
	_pos_sp(2) = pose_setpoint(2);

	_yaw_sp = pose_setpoint(3);
	_yawspeed_sp = 0.f;
}

void PositionControl::setVelocityFeedforward(const matrix::Vector4f &vel_ff)
{
	_vel_ff = vel_ff;
} // feedforward velocity

bool PositionControl::update(const float dt, bool arm_flag, bool pos_flag)
{
	if (arm_flag != _armed_sign) {
		resetIntegral();
		_armed_sign = arm_flag;
	}

	if (!arm_flag) {
		_vel_sp.zero();
		_acc_sp.zero();
		_thr_sp.zero();
		return true;
	}

	const bool valid = _inputValid(dt);

	if (valid) {

		{
			_positionControl();

			// custom_dt_s m{};
			// m.timestamp = now0;
			// m.stage = custom_dt_s::STAGE_POSITION;
			// m.dt = pos_dt;
			// g_custom_dt_pub.publish(m);
		}

		{

			_velocityControl(dt);

			// custom_dt_s m{};
			// m.timestamp = now1;
			// m.stage = custom_dt_s::STAGE_VELOCITY;
			// m.dt = vel_dt;
			// g_custom_dt_pub.publish(m);
		}

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control

	}

	if (!pos_flag) {
		_vel_sp(0) = 0.f;
		_vel_sp(1) = 0.f;

		_acc_sp(0) = 0.f;
		_acc_sp(1) = 0.f;

		_thr_sp(0) = 0.f;
		_thr_sp(1) = 0.f;
	}



	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void PositionControl::_positionControl()
{
	// 0901 Default
	// Vector3f vel_sp_position;

	// vel_sp_position(0) = (_pos_sp(0) - _pos(0))*static_cast<float>(1.0001) - _vel(0)*static_cast<float>(0.0);  //SEUK 여기는 하드코딩 되어있음.
	// vel_sp_position(1) = (_pos_sp(1) - _pos(1))*static_cast<float>(1.0001) - _vel(1)*static_cast<float>(0.0);
	// vel_sp_position(2) = (_pos_sp(2) - _pos(2))*static_cast<float>(0.5001) - _vel(2)*static_cast<float>(0.1);
	// _vel_sp(0) = math::constrain(vel_sp_position(0), -_lim_vel_horizontal, _lim_vel_horizontal);
	// _vel_sp(1) = math::constrain(vel_sp_position(1), -_lim_vel_horizontal, _lim_vel_horizontal);
	// _vel_sp(2) = math::constrain(vel_sp_position(2), static_cast<float>(-0.5), static_cast<float>(0.5));//_lim_vel_down -0.5


	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p) - _vel.emult(_gain_pos_d);

	vel_sp_position(0) += _vel_ff(0);
	vel_sp_position(1) += _vel_ff(1);
	vel_sp_position(2) += _vel_ff(2);

	_vel_sp(0) = math::constrain(vel_sp_position(0), -_lim_vel_horizontal, _lim_vel_horizontal);
	_vel_sp(1) = math::constrain(vel_sp_position(1), -_lim_vel_horizontal, _lim_vel_horizontal);
	_vel_sp(2) = math::constrain(vel_sp_position(2), -_lim_vel_up, _lim_vel_down);

}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

void PositionControl::_velocityControl(const float dt)
{
	Vector3f vel_error =  _w2b * (_vel_sp - _vel); // body-frame 속도 오차

	// I gain Calculate
	_vel_int += vel_error.emult(_gain_vel_i) * dt; // body
	_vel_int(0) = math::constrain(_vel_int(0), -3.0001f, 3.0001f);
	_vel_int(1) = math::constrain(_vel_int(1), -3.0001f, 3.0001f);
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);


	const Vector3f acc_b_est = _w2b * _vel_dot;

	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - acc_b_est.emult(_gain_vel_d);


	_acc_sp(0) = acc_sp_velocity(0);
	_acc_sp(1) = acc_sp_velocity(1);
	_acc_sp(2) = acc_sp_velocity(2);

	_acc_sp(0) = math::constrain(_acc_sp(0), -3.0f, 3.0f);
	_acc_sp(1) = math::constrain(_acc_sp(1), -3.0f, 3.0f);
	_acc_sp(2) = math::constrain(_acc_sp(2), -_lim_acc_vertical, _lim_acc_vertical);

	float mass = 8.0f;



	_thr_sp(0) = mass * _acc_sp(0);
	_thr_sp(1) = mass * _acc_sp(1);
	_thr_sp(2) = mass * _acc_sp(2);


	// const Vector3f g_world(0.f, 0.f, CONSTANTS_ONE_G); // 중력보상 할려면 이거써라
	// const Vector3f g_body = _w2b * g_world; // 중력보상 할려면 이거써라
	// _thr_sp(0) = mass*_acc_sp(0) + 0.5f * mass*g_body(0); // 중력보상 할려면 이거써라
	// _thr_sp(1) = mass*_acc_sp(1) + 0.5f * mass*g_body(1); // 중력보상 할려면 이거써라
	// _thr_sp(2) = mass*_acc_sp(2) + 0.5f * mass*g_body(2); // 중력보상 할려면 이거써라 => 무게추 달고 말고 모델 달라지는거 + step PWM 걱정되서 걍 안씀

	if ((_thr_sp(2) <= _lim_thr_min && vel_error(2) <= 0.f) ||
	    (_thr_sp(2) >= _lim_thr_max && vel_error(2) >= 0.f)) {
		vel_error(2) = 0.f;
	}

}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) { // 수직방향 추력과 병진방향 추력이 서로 영향을 준다면 --> true

		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		z_specific_force += _acc_sp(
					    2); // 병진방향으로 이동하면서 발생하는 수직방향의 추력손실을 매꿔주는 텀?
	}

	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1),
				   -z_specific_force).normalized(); // 좌표계를 비행좌표계로 바꾼거
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

	// NED = North-East-Down 기준 좌표계 (PX4 기본 좌표계)
	// ned_z = "아래 방향", 즉 중력 방향 (지면 방향)
	// 아무래도 가속도 커맨드가 곧 translational force command라고 하고 servo allocation 해야할듯
	// Convert to thrust assuming hover thrust produces standard gravity
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;

	// Project thrust to planned body attitude
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust; //body_z
}

bool PositionControl::_inputValid(const float dt) const
{
	const bool gains_finite = _gain_pos_p.isAllFinite() && _gain_pos_d.isAllFinite()
				  && _gain_vel_p.isAllFinite() && _gain_vel_i.isAllFinite()
				  && _gain_vel_d.isAllFinite();
	const bool limits_finite = PX4_ISFINITE(_lim_vel_horizontal) && PX4_ISFINITE(_lim_vel_up)
				   && PX4_ISFINITE(_lim_vel_down) && PX4_ISFINITE(_lim_thr_min)
				   && PX4_ISFINITE(_lim_thr_max) && PX4_ISFINITE(_lim_thr_xy_margin)
				   && PX4_ISFINITE(_lim_tilt) && PX4_ISFINITE(_lim_acc_horizontal)
				   && PX4_ISFINITE(_lim_acc_vertical) && PX4_ISFINITE(_hover_thrust);
	const bool state_finite = _pos.isAllFinite() && _vel.isAllFinite() && _vel_dot.isAllFinite()
				  && PX4_ISFINITE(_yaw);
	const bool setpoints_finite = _pos_sp.isAllFinite() && _vel_ff.isAllFinite()
				      && PX4_ISFINITE(_yaw_sp);

	return PX4_ISFINITE(dt) && dt > 0.f && _attitude_valid && _w2b.isAllFinite()
	       && gains_finite && limits_finite && state_finite && setpoints_finite
	       && _vel_int.isAllFinite();
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint, bool &pos_flag,
		Vector4f &manual_input) const
{
	//ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	matrix::Vector4f blank_vec{0.f, 0.f, 0.f, 0.f};

	if (pos_flag) {
		ControlMath::setAttitude(_yaw_sp, attitude_setpoint, blank_vec);

	} else {
		ControlMath::setAttitude(_yaw_sp, attitude_setpoint, manual_input);
	}

	_thr_sp.copyTo(attitude_setpoint.thrust_body); // custom
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
