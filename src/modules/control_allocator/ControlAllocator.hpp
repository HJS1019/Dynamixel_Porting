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
 * @file ControlAllocator.hpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once


#include <ControlAllocationUtils.hpp> // custom

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_motors.h>

#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/failure_detector_status.h>

#include <uORB/topics/manual_control_setpoint.h> // custom

//ㅡㅡㅡㅡㅡㅡㅡㅡㅡ25.03.18.Song Yeong Inㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ//
#include <cmath>
#include <uORB/topics/servo_angle.h>
#include <uORB/topics/thrust_command.h>
#include <uORB/topics/wrench_command.h>
#include <uORB/topics/center_of_mass.h> // custom

#include <uORB/topics/servo_command.h>


class ControlAllocator : public ModuleBase<ControlAllocator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:



	ControlAllocator();

	virtual ~ControlAllocator();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();



private:


	/**
	 * initialize some vectors/matrices from parameters
	 */

	void parameters_updated();

	void update_effectiveness_matrix_if_needed();


	void publish_control_allocator_status(int matrix_index);

	void publish_actuator_controls();

	hrt_abstime _last_effectiveness_update{0};
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //

	// Inputs
	uORB::SubscriptionCallbackWorkItem _vehicle_torque_setpoint_sub{this, ORB_ID(vehicle_torque_setpoint)};  /**< vehicle torque setpoint subscription */
	uORB::SubscriptionCallbackWorkItem _vehicle_thrust_setpoint_sub{this, ORB_ID(vehicle_thrust_setpoint)};	 /**< vehicle thrust setpoint subscription */

	uORB::Subscription _vehicle_torque_setpoint1_sub{ORB_ID(vehicle_torque_setpoint), 1};  /**< vehicle torque setpoint subscription (2. instance) */
	uORB::Subscription _vehicle_thrust_setpoint1_sub{ORB_ID(vehicle_thrust_setpoint), 1};	 /**< vehicle thrust setpoint subscription (2. instance) */

	// Outputs
	uORB::PublicationMulti<control_allocator_status_s> _control_allocator_status_pub[2] {ORB_ID(control_allocator_status), ORB_ID(control_allocator_status)};

	uORB::Publication<actuator_motors_s>	_actuator_motors_pub{ORB_ID(actuator_motors)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1}; //ms

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _failure_detector_status_sub{ORB_ID(failure_detector_status)};

	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ custom subscriber | publisher ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	uORB::SubscriptionCallbackWorkItem _servo_angle_sub{this, ORB_ID(servo_angle)};
	uORB::Publication<thrust_command_s> _thrust_command_pub{ORB_ID(thrust_command)};
	uORB::Publication<wrench_command_s> _wrench_command_pub{ORB_ID(wrench_command)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _center_of_mass_sub{ORB_ID(center_of_mass)};


	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	matrix::Matrix<float, 4,4> _custom_effectiveness;
	matrix::Vector3f _torque_sp;
	matrix::Vector3f _thrust_sp;
	matrix::Vector4f _servo_ang{0.0,0.0,0.0,0.0}; // custom
	bool _publish_controls{true};

	// Reflects motor failures that are currently handled, not motor failures that are reported.
	// For example, the system might report two motor failures, but only the first one is handled by CA
	uint16_t _handled_motor_failure_bitmask{0};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	bool _armed{false};
	hrt_abstime _last_run{0};
	hrt_abstime _timestamp_sample{0};
	hrt_abstime _last_status_pub{0};

	bool _has_slew_rate{false};
	float dumi = 0.f;

	float xc = 0.0;
	float yc = 0.0;
	float zc = 0.0;


	// ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ Servo Allocation (라떼판다 이식) ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ //
	// 원본: listen_and_speak_ros.cpp 의 timer_callback (395~490행, 100Hz)
	//   입력: fx, fy, tz_trim, f1~f4, xc, yc
	//   출력: servo_command (th1~th4, 단위 rad, ±0.7)

	// 2차 버터워스 LPF (Direct Form II Transposed)
	struct ServoBiquad {
		float b0{0.f}, b1{0.f}, b2{0.f}, a1{0.f}, a2{0.f};
		float s1{0.f}, s2{0.f};

		void setButter2Lowpass(float fc_hz, float fs_hz, bool reset)
		{
			const float PI_F = 3.14159265358979323846f;
			const float K = tanf(PI_F * fc_hz / fs_hz);
			const float KK = K * K;
			const float denom_inv = 1.f / (1.f + 1.41421356f * K + KK);
			b0 = KK * denom_inv;
			b1 = 2.f * b0;
			b2 = b0;
			a1 = 2.f * (KK - 1.f) * denom_inv;
			a2 = (1.f - 1.41421356f * K + KK) * denom_inv;

			if (reset) {
				s1 = 0.f;
				s2 = 0.f;
			}
		}

		float step(float x)
		{
			const float y = b0 * x + s1;
			s1 = b1 * x - a1 * y + s2;
			s2 = b2 * x - a2 * y;
			return y;
		}
	};

	void updateServoAllocation(float dt);

	// 테스트용: 주어진 입력으로 서보 각도를 1회 계산 (발행하지 않음)
	//   solveServoAngles() 는 순수 계산 함수이므로 LPF/발행과 분리되어 있다.
	void solveServoAngles(float fx, float fy, float tz_trim,
			      float f1, float f2, float f3, float f4,
			      float xc_in, float yc_in, float th_out[4]) const;

	static float servoClampf(float x, float lo, float hi);
	static float setThrustLimitation(float f);

	uORB::Publication<servo_command_s> _servo_command_pub{ORB_ID(servo_command)};

	// LPF 4채널 (payload 5번째 채널은 미사용 - 현재 기체 서보 4개)
	ServoBiquad _lpf_th[4];
	bool  _servo_lpf_initialized{false};
	float _servo_lpf_prev_fs{0.f};

	// dt 이동평균 (fs 추정)
	static constexpr int kServoMaMaxN = 64;
	float _servo_dt_hist[kServoMaMaxN] {};
	int   _servo_dt_count{0};
	int   _servo_dt_head{0};

	// (e) 대수 루프 회피: 한 주기 이전 추력
	matrix::Vector4f _prev_thrust{0.f, 0.f, 0.f, 0.f};
	bool _prev_thrust_valid{false};

	// MulticopterRateControl 이 채운 yaw_trim (RT 포화 초과분 + 저역)
	float _servo_yaw_trim{0.f};

	// 서보 얼로케이션 상수 (원본과 동일)
	static constexpr float kServoRArm       = 0.23f;   // 팔 길이 [m]
	static constexpr float kServoThetaMax   = 0.7f;    // 서보 포화각 [rad] (±40deg)
	static constexpr float kServoLpfFc_Hz   = 8.0f;    // LPF 차단 주파수
	static constexpr float kServoMinFs_Hz   = 50.f;    // fs 하한
	static constexpr float kServoMaxFs_Hz   = 2000.f;  // fs 상한
	static constexpr float kServoFsUpdFrac  = 0.25f;   // 계수 갱신 임계
	static constexpr float kServoMaWindow_s = 0.10f;   // dt 이동평균 창
	static constexpr float kThrustMin       = 2.0f;    // setThrustLimitation 하한
	static constexpr float kThrustMax       = 55.0f;   // setThrustLimitation 상한


	matrix::Matrix<float, 4, 4> _mix;
	matrix::Vector<float, 4> _actuator_min; 	///< Minimum actuator values
	matrix::Vector<float, 4> _actuator_max; 	///< Maximum actuator values
	matrix::Vector<float, 4> _actuator_sp;  	///< Actuator setpoint
	matrix::Vector<float, 4> _control_sp;   	///< Control setpoint

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_AIRFRAME>) _param_ca_airframe,
		(ParamInt<px4::params::CA_METHOD>) _param_ca_method,
		(ParamInt<px4::params::CA_FAILURE_MODE>) _param_ca_failure_mode,
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev,
		(ParamInt<px4::params::CA_SERVO_ALLOC>) _param_ca_servo_alloc
	)

};
