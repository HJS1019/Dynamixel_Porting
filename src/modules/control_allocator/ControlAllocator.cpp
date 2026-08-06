/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocator.cpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocator.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <cmath> // isnan()
#include <uORB/Publication.hpp>
#include <uORB/topics/custom_dt.h>
#include <cstdlib>
#include <cstring>

using namespace matrix;
using namespace time_literals;


ControlAllocator::ControlAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_control_allocator_status_pub[0].advertise();
	_control_allocator_status_pub[1].advertise();

	_actuator_motors_pub.advertise();


	parameters_updated();
}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

ControlAllocator::~ControlAllocator()
{

	perf_free(_loop_perf);
}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //


bool
ControlAllocator::init()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	if (!_vehicle_thrust_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	ScheduleDelayed(50_ms);
#endif

	return true;
}
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

void
ControlAllocator::parameters_updated()
{

	update_effectiveness_matrix_if_needed();
}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //


void
ControlAllocator::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		_vehicle_thrust_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	perf_begin(_loop_perf);

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	// Push backup schedule
	ScheduleDelayed(50_ms);
#endif

	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// Check if parameters have changed
	if (_parameter_update_sub.updated() /*&& !_armed*/) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		if (_handled_motor_failure_bitmask == 0) {
			// We don't update the geometry after an actuator failure, as it could lead to unexpected results
			// (e.g. a user could add/remove motors, such that the bitmask isn't correct anymore)
			updateParams();
			parameters_updated();
		}
	}
	// ㅡㅡㅡㅡ (g) armed 상태 갱신 : 서보 얼로케이션 안전 조건 ㅡㅡㅡㅡ //
	// 기존 코드에 _armed 를 갱신하는 부분이 없어 항상 false 였다.
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	}

	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ custom part ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	servo_angle_s servo_angle{};

	if (_servo_angle_sub.update(&servo_angle)) {
		const hrt_abstime servo_now = hrt_absolute_time();
		const bool timestamp_fresh = servo_angle.timestamp != 0
					     && servo_now >= servo_angle.timestamp
					     && servo_now - servo_angle.timestamp <= 200000;
		const bool all_channels_valid = (servo_angle.valid_mask & 0x0F) == 0x0F;
		bool angles_finite = true;

		for (int i = 0; i < 4; ++i) {
			angles_finite = angles_finite && PX4_ISFINITE(servo_angle.servo_angle[i]);
		}

		if (timestamp_fresh && all_channels_valid && angles_finite) {
			for (int i = 0; i < 4; ++i) {
				_servo_ang(i) = servo_angle.servo_angle[i];
			}

			update_effectiveness_matrix_if_needed();
		}

	}

	center_of_mass_s com_update;

	if(_center_of_mass_sub.update(&com_update)){

		xc = com_update.com_update[0];
		yc = com_update.com_update[1]; //-0.04f
		zc = com_update.com_update[2];

	}


	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;


	// Run allocator on torque changes
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);
		_servo_yaw_trim = vehicle_torque_setpoint.yaw_trim;   // ★ 추가 (서보 얼로케이션용)


		do_update = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;

	}

	// Also run allocator on thrust setpoint changes if the torque setpoint
	// has not been updated for more than 5ms
	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);

		if (dt > 0.005f) {
			do_update = true;
			_timestamp_sample = vehicle_thrust_setpoint.timestamp_sample;
		}
	}
/*
	manual_control_setpoint_s rpyz_cmd;
	wrench_command_s wrench_cmd;
	if (_manual_control_setpoint_sub.update(&rpyz_cmd)){
		_torque_sp(0) = 0;//rpyz_cmd.roll;
		_torque_sp(1) = 0;//rpyz_cmd.pitch;
		_torque_sp(2) = rpyz_cmd.yaw;

		_thrust_sp(0) = 2*rpyz_cmd.pitch;
		_thrust_sp(1) =	2*rpyz_cmd.roll;
		_thrust_sp(2) = 55.0f*rpyz_cmd.throttle;
		do_update = true;

		wrench_cmd.tr_d=_torque_sp(0);
		wrench_cmd.tp_d=_torque_sp(1);
		wrench_cmd.ty_d=_torque_sp(2);

		wrench_cmd.fx_d=_thrust_sp(0);
		wrench_cmd.fy_d=_thrust_sp(1);
		wrench_cmd.fz_d=-_thrust_sp(2);

		_wrench_command_pub.publish(wrench_cmd);
	}*/

	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ CONTROL ALLOCATION START ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	if (do_update) {

			_last_run = now;

			update_effectiveness_matrix_if_needed();

	}

	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	// Publish actuator setpoint and allocator status
	publish_actuator_controls();

	// ㅡㅡㅡㅡ 서보 얼로케이션 (CA_SERVO_ALLOC == 1 일 때만 동작) ㅡㅡㅡㅡ //
	// publish_actuator_controls() 가 _prev_thrust 를 갱신한 뒤 호출되므로,
	// 이번 주기의 계산에는 "이전 주기" 추력이 쓰인다. (e) 대수 루프 회피
	updateServoAllocation(dt);

	perf_end(_loop_perf);
}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

void
ControlAllocator::update_effectiveness_matrix_if_needed()
{
	// control wrench update
	_control_sp(0) = _torque_sp(0);

	_control_sp(1) = _torque_sp(1);

	_control_sp(2) = _torque_sp(2);

	_control_sp(3) = _thrust_sp(2); // -22.0 : 음수여야함

	// Assign control effectiveness matrix
	float xi = - 0.01;	// so called b-over-k
	float r2 = sqrt(2);
	float r_arm = 0.23; // 0.21 [0923]
	float l_servo = -0.005; // 0.006; [260222 출근]
	bool allocation_version_1 = false;
	/*
	float th1 = 0.0; //rad
	float th2 = 0.0; //rad
	float th3 = 0.0; //rad
	float th4 = 0.0; //rad*/
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ custom_part ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// 현재 모터는 1(CW),2(CCW),3(CW),4(CCW) 반시계방향으로 왼쪽 맨위 부터 1번
	//
	// (CW)    (CCW)
	//  m1      m4
	//    +	  +
	//      +
	//    +   +
	//  m2     m3
	//(CCW)    (CW)
	//
	//



	if (allocation_version_1)
	{
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// ------------------------------------------------- [Version. 1] ------------------------------------------------- //
		// ----------------------------------- Com bias로 인한 yaw torque 보상을 여기서 수행  ---------------------------------- //
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// 1x1
		_custom_effectiveness(0,0) = (yc+r_arm/r2)*cosf(_servo_ang(0))+(-(l_servo-zc)+xi)*sinf(_servo_ang(0))/r2;
		// 1x2
		_custom_effectiveness(0,1) = (yc+r_arm/r2)*cosf(_servo_ang(1))+((l_servo-zc)-xi)*sinf(_servo_ang(1))/r2;
		// 1x3
		_custom_effectiveness(0,2) = (yc-r_arm/r2)*cosf(_servo_ang(2))+((l_servo-zc)-xi)*sinf(_servo_ang(2))/r2;
		// 1x4
		_custom_effectiveness(0,3) = (yc-r_arm/r2)*cosf(_servo_ang(3))+(-(l_servo-zc)+xi)*sinf(_servo_ang(3))/r2;
		// 2x1
		_custom_effectiveness(1,0) =-(xc-r_arm/r2)*cosf(_servo_ang(0))+((l_servo-zc)+xi)*sinf(_servo_ang(0))/r2;
		// 2x2
		_custom_effectiveness(1,1) =-(xc+r_arm/r2)*cosf(_servo_ang(1))+((l_servo-zc)+xi)*sinf(_servo_ang(1))/r2;
		// 2x3
		_custom_effectiveness(1,2) =-(xc+r_arm/r2)*cosf(_servo_ang(2))+(-(l_servo-zc)-xi)*sinf(_servo_ang(2))/r2;
		// 2x4
		_custom_effectiveness(1,3) =-(xc-r_arm/r2)*cosf(_servo_ang(3))+(-(l_servo-zc)-xi)*sinf(_servo_ang(3))/r2;
		// 3x1
		_custom_effectiveness(2,0) =-xi*cosf(_servo_ang(0))+(-(xc-yc)/r2)*sinf(_servo_ang(0));
		// 3x2
		_custom_effectiveness(2,1) = xi*cosf(_servo_ang(1))+((xc+yc)/r2)*sinf(_servo_ang(1));
		// 3x3
		_custom_effectiveness(2,2) =-xi*cosf(_servo_ang(2))+((xc-yc)/r2)*sinf(_servo_ang(2));
		// 3x4
		_custom_effectiveness(2,3) = xi*cosf(_servo_ang(3))+(-(xc+yc)/r2)*sinf(_servo_ang(3));

		// 4x1
		_custom_effectiveness(3,0) =-cosf(_servo_ang(0));
		// 4x2
		_custom_effectiveness(3,1) =-cosf(_servo_ang(1));
		// 4x3
		_custom_effectiveness(3,2) =-cosf(_servo_ang(2));
		// 4x4
		_custom_effectiveness(3,3) =-cosf(_servo_ang(3));
	}
	else
	{
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// ------------------------------------------------- [Version. 2] ------------------------------------------------- //
		// ---------------------------------- Com bias로 인한 yaw torque 보상을 servo에서 수행  -------------------------------- //
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// 1x1
		_custom_effectiveness(0,0) = (yc+r_arm/r2)*cosf(_servo_ang(0))+(-(l_servo-zc)+xi)*sinf(_servo_ang(0))/r2;
		// 1x2
		_custom_effectiveness(0,1) = (yc+r_arm/r2)*cosf(_servo_ang(1))+((l_servo-zc)-xi)*sinf(_servo_ang(1))/r2;
		// 1x3
		_custom_effectiveness(0,2) = (yc-r_arm/r2)*cosf(_servo_ang(2))+((l_servo-zc)-xi)*sinf(_servo_ang(2))/r2;
		// 1x4
		_custom_effectiveness(0,3) = (yc-r_arm/r2)*cosf(_servo_ang(3))+(-(l_servo-zc)+xi)*sinf(_servo_ang(3))/r2;
		// 2x1
		_custom_effectiveness(1,0) =-(xc-r_arm/r2)*cosf(_servo_ang(0))+((l_servo-zc)+xi)*sinf(_servo_ang(0))/r2;
		// 2x2
		_custom_effectiveness(1,1) =-(xc+r_arm/r2)*cosf(_servo_ang(1))+((l_servo-zc)+xi)*sinf(_servo_ang(1))/r2;
		// 2x3
		_custom_effectiveness(1,2) =-(xc+r_arm/r2)*cosf(_servo_ang(2))+(-(l_servo-zc)-xi)*sinf(_servo_ang(2))/r2;
		// 2x4
		_custom_effectiveness(1,3) =-(xc-r_arm/r2)*cosf(_servo_ang(3))+(-(l_servo-zc)-xi)*sinf(_servo_ang(3))/r2;
		// 3x1
		_custom_effectiveness(2,0) =-xi*cosf(_servo_ang(0));
		// 3x2
		_custom_effectiveness(2,1) = xi*cosf(_servo_ang(1));
		// 3x3
		_custom_effectiveness(2,2) =-xi*cosf(_servo_ang(2));
		// 3x4
		_custom_effectiveness(2,3) = xi*cosf(_servo_ang(3));

		// 4x1
		_custom_effectiveness(3,0) =-cosf(_servo_ang(0));
		// 4x2
		_custom_effectiveness(3,1) =-cosf(_servo_ang(1));
		// 4x3
		_custom_effectiveness(3,2) =-cosf(_servo_ang(2));
		// 4x4
		_custom_effectiveness(3,3) =-cosf(_servo_ang(3));
	}



	matrix::geninv(_custom_effectiveness, _mix);

	_actuator_sp = _mix * (_control_sp);

	for(int i = 0; i<4; ++i){
		if(_actuator_sp(i) > 55.0f){_actuator_sp(i) = 55.0f;}
		if(_actuator_sp(i) < 0.5f){_actuator_sp(i) = 0.5f;}
	}


}


// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ Servo Allocation 보조 함수 ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

float
ControlAllocator::servoClampf(float x, float lo, float hi)
{
	return (x < lo) ? lo : (x > hi) ? hi : x;
}

// 원본 listen_and_speak_ros.cpp 의 setThrustLimitation 과 동일 (2.0 ~ 55.0)
float
ControlAllocator::setThrustLimitation(float f)
{
	if (!PX4_ISFINITE(f)) {
		return kThrustMin;
	}

	return servoClampf(f, kThrustMin, kThrustMax);
}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ Servo Allocation 본체 ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// 원본: listen_and_speak_ros.cpp timer_callback (395~490행)
//   SA * sin(theta) = wrench_servo  를 풀어 틸트 서보 각도를 구한다.
//   wrench_servo = (fx, fy, tz_trim, 0)
//     fx, fy   : 기체를 기울이지 않고 내는 수평력
//     tz_trim  : MulticopterRateControl 에서 모터(RT)가 감당하지 못한 yaw 몫

// ㅡㅡㅡㅡㅡㅡㅡ 서보 각도 계산 (순수 함수, LPF/발행 없음) ㅡㅡㅡㅡㅡㅡㅡ //
// updateServoAllocation() 과 servotest 커맨드가 공유한다.
void
ControlAllocator::solveServoAngles(float fx, float fy, float tz_trim,
				   float f1, float f2, float f3, float f4,
				   float xc_in, float yc_in, float th_out[4]) const
{
	const float r2 = 1.41421356f;   // sqrt(2)
	const float r_arm = kServoRArm;

	const float xc_s = PX4_ISFINITE(xc_in) ? xc_in : 0.f;
	const float yc_s = PX4_ISFINITE(yc_in) ? yc_in : 0.f;

	const float r_arm1 = r_arm - ((xc_s - yc_s) / r2);
	const float r_arm2 = r_arm + ((xc_s + yc_s) / r2);
	const float r_arm3 = r_arm + ((xc_s - yc_s) / r2);
	const float r_arm4 = r_arm - ((xc_s + yc_s) / r2);

	matrix::SquareMatrix<float, 4> SA;
	SA(0, 0) =  f1 / r2;     SA(0, 1) =  f2 / r2;     SA(0, 2) = -f3 / r2;     SA(0, 3) = -f4 / r2;
	SA(1, 0) =  f1 / r2;     SA(1, 1) = -f2 / r2;     SA(1, 2) = -f3 / r2;     SA(1, 3) =  f4 / r2;
	SA(2, 0) = r_arm1 * f1;  SA(2, 1) =  r_arm2 * f2; SA(2, 2) = r_arm3 * f3;  SA(2, 3) =  r_arm4 * f4;
	SA(3, 0) = r_arm1 * f1;  SA(3, 1) = -r_arm2 * f2; SA(3, 2) = r_arm3 * f3;  SA(3, 3) = -r_arm4 * f4;

	matrix::Vector<float, 4> wrench_servo;
	wrench_servo(0) = fx;
	wrench_servo(1) = fy;
	wrench_servo(2) = tz_trim;
	wrench_servo(3) = 0.f;

	// 특이행렬이면 .I() 가 0 행렬 → sin=0 → 각도 0 (중립)
	const matrix::Vector<float, 4> sine_theta_command = SA.I() * wrench_servo;

	const float sin_max = sinf(kServoThetaMax);

	for (int i = 0; i < 4; ++i) {
		const float s = servoClampf(sine_theta_command(i), -sin_max, sin_max);
		th_out[i] = asinf(servoClampf(s, -1.f, 1.f));
		th_out[i] = servoClampf(th_out[i], -kServoThetaMax, kServoThetaMax);
	}
}


void
ControlAllocator::updateServoAllocation(float dt)
{
	// (a) 파라미터 스위치
	//   0 = off, 1 = 정상(armed 필요), 2 = 테스트(arm 무시, 발행 없음)
	const int32_t alloc_mode = _param_ca_servo_alloc.get();

	if (alloc_mode == 0) {
		return;
	}

	const bool test_mode = (alloc_mode == 2);

	float th_cmd[4] {0.f, 0.f, 0.f, 0.f};

	// (g) disarmed 이면 서보를 움직이지 않는다 (중립 유지)
	// (e) 한 주기 이전 추력이 아직 없으면 역시 중립
	//     단 테스트 모드에서는 arm 상태를 무시하고 계산을 수행한다.
	const bool allocation_active = (test_mode || _armed) && _prev_thrust_valid;

	if (allocation_active) {
		// ── 입력 준비 ────────────────────────────────────────────
		const float fx = PX4_ISFINITE(_thrust_sp(0)) ? _thrust_sp(0) : 0.f;
		const float fy = PX4_ISFINITE(_thrust_sp(1)) ? _thrust_sp(1) : 0.f;
		const float tz_trim = PX4_ISFINITE(_servo_yaw_trim) ? _servo_yaw_trim : 0.f;

		// (d)(e) f1~f4 : 한 주기 이전 할당 결과 + setThrustLimitation
		const float f1 = setThrustLimitation(_prev_thrust(0));
		const float f2 = setThrustLimitation(_prev_thrust(1));
		const float f3 = setThrustLimitation(_prev_thrust(2));
		const float f4 = setThrustLimitation(_prev_thrust(3));

		// ── SA 풀이 → asin → 포화 ──────────────────────────────
		solveServoAngles(fx, fy, tz_trim, f1, f2, f3, f4, xc, yc, th_cmd);
	}

	// ── payload (5번 서보) : 현재 기체에는 서보가 4개뿐이라 사용하지 않음 ──
	//
	// [주석 처리됨 - 서보 5번을 장착하게 되면 되살릴 것]
	//
	// 원본 listen_and_speak_ros.cpp 의 payload_angle_trajectory() 는
	// 얼로케이션과 무관한 "시간 기반 시퀀스" 이다:
	//   - 부팅 후 10초에 걸쳐 0 -> 180 deg 이동
	//   - payload_flag == 0 : 3초에 걸쳐 180 deg 로 복귀
	//   - payload_flag == 1 : 180->0 (20s) -> 0 유지 (45s) -> 0->180 (20s) 1회 시퀀스
	// SA 행렬 계산이나 wrench 에 전혀 영향을 주지 않으므로 제거해도 무방하다.
	//
	// 되살릴 때 함께 해야 할 일:
	//   1) ServoCommand.msg 를 float32[5] 로 확장
	//   2) dynamixel 드라이버를 -n 5 로 기동, _servo_ids 5개로 확장
	//   3) 아래 LPF 배열 _lpf_th[4] -> _lpf_th[5] 로 확장
	//
	// payload_angle_trajectory();
	// float th5_cmd = servoClampf(payload_angle_command, 0.f, 3.14159265f);
	// float th5_lpf = _lpf_th[4].step(th5_cmd);
	// servo_cmd.servo_command[4] = servoClampf(th5_lpf, 0.f, 3.14159265f);

	// ── (c) LPF 계수: dt 이동평균으로 fs 추정 ────────────────────
	//   ControlAllocator 는 setpoint 도착 시 실행되어 주기가 불규칙하다.
	//   MulticopterRateControl.cpp 의 yaw trimming 과 동일한 방식으로
	//   dt 를 이동평균하여 안정적인 fs 를 구한 뒤 버터워스 계수를 맞춘다.
	const float dt_guard = math::constrain(dt, 1.25e-4f, 2.0e-2f);

	_servo_dt_hist[_servo_dt_head] = dt_guard;
	_servo_dt_head = (_servo_dt_head + 1) % kServoMaMaxN;

	if (_servo_dt_count < kServoMaMaxN) {
		_servo_dt_count++;
	}

	int K = static_cast<int>(kServoMaWindow_s / dt_guard + 0.5f);
	K = math::constrain(K, 8, kServoMaMaxN);

	if (K > _servo_dt_count) {
		K = _servo_dt_count;
	}

	float sum_dt = 0.f;

	for (int i = 0; i < K; ++i) {
		int idx = _servo_dt_head - 1 - i;

		if (idx < 0) {
			idx += kServoMaMaxN;
		}

		sum_dt += _servo_dt_hist[idx];
	}

	const float dt_ma = (K > 0) ? (sum_dt / static_cast<float>(K)) : dt_guard;
	float fs = (dt_ma > 0.f) ? (1.f / dt_ma) : kServoMinFs_Hz;
	fs = math::constrain(fs, kServoMinFs_Hz, kServoMaxFs_Hz);

	// 나이퀴스트 여유 확보 (fc <= 0.45 * fs)
	const float fc_clamped = math::constrain(kServoLpfFc_Hz, 0.1f, 0.45f * fs);

	if (!_servo_lpf_initialized) {
		for (int i = 0; i < 4; ++i) {
			_lpf_th[i].setButter2Lowpass(fc_clamped, fs, true);

			// 초기 과도 억제 워밍업 3회 (원본과 동일)
			// 필터 상태(s1,s2)가 0이면 첫 출력이 작게 나와 서보가 튄다.
			for (int k = 0; k < 3; ++k) {
				(void)_lpf_th[i].step(th_cmd[i]);
			}
		}

		_servo_lpf_prev_fs = fs;
		_servo_lpf_initialized = true;

	} else if (fabsf(fs - _servo_lpf_prev_fs) > kServoFsUpdFrac * _servo_lpf_prev_fs) {
		// fs 가 25% 이상 변했을 때만 계수 갱신 (필터 상태는 유지)
		for (int i = 0; i < 4; ++i) {
			_lpf_th[i].setButter2Lowpass(fc_clamped, fs, false);
		}

		_servo_lpf_prev_fs = fs;
	}

	// ── LPF 통과 → NaN 가드 → 최종 포화 → 발행 ───────────────────
	servo_command_s servo_cmd{};

	for (int i = 0; i < 4; ++i) {
		float y = _lpf_th[i].step(th_cmd[i]);

		if (!PX4_ISFINITE(y)) {
			y = 0.f;
		}

		// 2차 필터의 오버슈트 대비 최종 포화
		servo_cmd.servo_command[i] = servoClampf(y, -kServoThetaMax, kServoThetaMax);
	}

	servo_cmd.timestamp = hrt_absolute_time();

	if (test_mode) {
		// 테스트 모드: 서보로 내보내지 않고 1초에 한 번만 로그 출력
		static hrt_abstime last_log = 0;
		const hrt_abstime now_log = hrt_absolute_time();

		if (now_log - last_log > 1000000) {
			last_log = now_log;
			PX4_INFO("[servo alloc TEST] armed=%d fx=%.3f fy=%.3f tz_trim=%.4f",
				 _armed ? 1 : 0,
				 (double)_thrust_sp(0), (double)_thrust_sp(1), (double)_servo_yaw_trim);
			PX4_INFO("    f=[%.2f %.2f %.2f %.2f]  com=[%.4f %.4f]",
				 (double)_prev_thrust(0), (double)_prev_thrust(1),
				 (double)_prev_thrust(2), (double)_prev_thrust(3),
				 (double)xc, (double)yc);
			PX4_INFO("    th_raw=[%.4f %.4f %.4f %.4f]  th_lpf=[%.4f %.4f %.4f %.4f] rad",
				 (double)th_cmd[0], (double)th_cmd[1], (double)th_cmd[2], (double)th_cmd[3],
				 (double)servo_cmd.servo_command[0], (double)servo_cmd.servo_command[1],
				 (double)servo_cmd.servo_command[2], (double)servo_cmd.servo_command[3]);
		}

	} else {
		_servo_command_pub.publish(servo_cmd);
	}
}
void
ControlAllocator::publish_actuator_controls()
{
    if (!_publish_controls) {
        return;
    }

    actuator_motors_s actuator_motors{};     // ✅ zero-init
    thrust_command_s thrust_commands{};       // ✅ zero-init

    actuator_motors.timestamp = hrt_absolute_time();
    actuator_motors.timestamp_sample = _timestamp_sample;
    actuator_motors.reversible_flags = _param_r_rev.get();

    // 원하는 saturation 범위
    constexpr float u_min = 0.0f;
    constexpr float u_max = 0.85f;

    for (int i = 0; i < 4; i++) {

        const float force_cmd = _actuator_sp(i);

        // raw thrust command
        if (PX4_ISFINITE(force_cmd)) {
            thrust_commands.thrust_command[i] = force_cmd;
        } else {
            thrust_commands.thrust_command[i] = 0.f;
        }
	// (e) 서보 얼로케이션이 다음 주기에 사용할 값으로 보관
        _prev_thrust(i) = thrust_commands.thrust_command[i];

        // force -> pwm scale
        float u = force_to_pwm_scale(force_cmd);

        // ✅ publish 직전 최종 saturation (2중 안전)
        u = math::constrain(u, u_min, u_max);

        // 혹시 NaN이면 0으로
        if (!PX4_ISFINITE(u)) {
            u = 0.f;
        }

        actuator_motors.control[i] = u;
    }

    _prev_thrust_valid = true;

    _actuator_motors_pub.publish(actuator_motors);
    _thrust_command_pub.publish(thrust_commands);
}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

int ControlAllocator::task_spawn(int argc, char *argv[])
{
	ControlAllocator *instance = new ControlAllocator();

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
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

int ControlAllocator::print_status()
{
	PX4_INFO("Running");

	PX4_INFO("Dumi : %f", (double)dumi);

	PX4_INFO("torque_cmd = roll : %f| pitch : %f| yaw : %f| fz : %f|",(double)_control_sp(0),(double)_control_sp(1),(double)_control_sp(2),(double)_control_sp(3));

	// Print perf
	perf_print_counter(_loop_perf);

	return 0;
}

int ControlAllocator::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("unknown command");
	}

	// servotest <fx> <fy> <tz_trim> <f1> <f2> <f3> <f4> [xc] [yc]
	//   주어진 입력으로 서보 각도를 1회 계산하여 출력한다 (발행하지 않음).
	//   라떼판다 결과와 숫자를 직접 비교하기 위한 검증용.
	if (strcmp(argv[0], "servotest") == 0) {
		// 마지막 인자가 --publish 이면 실제로 servo_command 를 발행한다.
		bool do_publish = false;
		int  nargs = argc;

		if (nargs >= 2 && strcmp(argv[nargs - 1], "--publish") == 0) {
			do_publish = true;
			nargs--;                 // --publish 를 인자 수에서 제외
		}

		if (nargs != 8 && nargs != 10) {
			PX4_ERR("usage: servotest <fx> <fy> <tz_trim> <f1> <f2> <f3> <f4> [xc] [yc] [--publish]");
			return 1;
		}

		if (!is_running()) {
			PX4_ERR("control_allocator is not running");
			return 1;
		}

		ControlAllocator *instance = get_instance();

		if (instance == nullptr) {
			PX4_ERR("instance unavailable");
			return 1;
		}

		const float fx      = strtof(argv[1], nullptr);
		const float fy      = strtof(argv[2], nullptr);
		const float tz_trim = strtof(argv[3], nullptr);

		const float f1 = setThrustLimitation(strtof(argv[4], nullptr));
		const float f2 = setThrustLimitation(strtof(argv[5], nullptr));
		const float f3 = setThrustLimitation(strtof(argv[6], nullptr));
		const float f4 = setThrustLimitation(strtof(argv[7], nullptr));

		// xc, yc 를 생략하면 현재 center_of_mass 값을 사용
		const float xc_in = (nargs == 10) ? strtof(argv[8], nullptr) : instance->xc;
		const float yc_in = (nargs == 10) ? strtof(argv[9], nullptr) : instance->yc;

		float th[4] {0.f, 0.f, 0.f, 0.f};
		instance->solveServoAngles(fx, fy, tz_trim, f1, f2, f3, f4, xc_in, yc_in, th);

		PX4_INFO("input : fx=%.3f fy=%.3f tz_trim=%.4f", (double)fx, (double)fy, (double)tz_trim);
		PX4_INFO("        f=[%.2f %.2f %.2f %.2f] (after 2.0~55.0 limit)",
			 (double)f1, (double)f2, (double)f3, (double)f4);
		PX4_INFO("        com=[%.4f %.4f]", (double)xc_in, (double)yc_in);
		PX4_INFO("result: th=[%.5f %.5f %.5f %.5f] rad",
			 (double)th[0], (double)th[1], (double)th[2], (double)th[3]);
		PX4_INFO("        th=[%.2f %.2f %.2f %.2f] deg",
			 (double)(th[0] * 57.2957795f), (double)(th[1] * 57.2957795f),
			 (double)(th[2] * 57.2957795f), (double)(th[3] * 57.2957795f));
		if (do_publish) {
			servo_command_s servo_cmd{};

			for (int i = 0; i < 4; ++i) {
				servo_cmd.servo_command[i] = th[i];
			}

			servo_cmd.timestamp = hrt_absolute_time();
			instance->_servo_command_pub.publish(servo_cmd);
			PX4_INFO("--> published to servo_command (LPF 미적용, 계산값 직접 발행)");
		}

		return 0;
	}

	return print_usage("unknown command");
}

int ControlAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements control allocation. It takes torque and thrust setpoints
as inputs and outputs actuator setpoint messages.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("control_allocator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("servotest",
					 "servotest <fx> <fy> <tz_trim> <f1> <f2> <f3> <f4> [xc] [yc]");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

/**
 * Control Allocator app start / stop handling function
 */
extern "C" __EXPORT int control_allocator_main(int argc, char *argv[]);

int control_allocator_main(int argc, char *argv[])
{
	return ControlAllocator::main(argc, argv);
}
