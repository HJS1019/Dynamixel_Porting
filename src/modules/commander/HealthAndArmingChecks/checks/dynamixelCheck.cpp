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

#include "dynamixelCheck.hpp"

using namespace time_literals;

DynamixelChecks::DynamixelChecks()
{
	_param_dxl_en = param_find("DXL_EN");
	_param_ca_servo_alloc = param_find("CA_SERVO_ALLOC");
}

void DynamixelChecks::checkAndReport(const Context &context, Report &reporter)
{
	(void)context;

	// DXL_EN 이 없거나(드라이버 미포함 빌드) 0 이면 점검하지 않는다
	int32_t dxl_enabled = 0;

	if (_param_dxl_en == PARAM_INVALID || param_get(_param_dxl_en, &dxl_enabled) != PX4_OK || dxl_enabled == 0) {
		return;
	}

	servo_angle_s servo_angle{};
	const bool received = _servo_angle_sub.copy(&servo_angle);
	const bool fresh = received && servo_angle.timestamp != 0
			   && hrt_elapsed_time(&servo_angle.timestamp) < 500_ms;

	if (!fresh) {
		/* EVENT
		 * @description
		 * The dynamixel tilt-servo driver is not publishing servo feedback.
		 * Check that the driver is running (dynamixel status) and the servo bus is powered.
		 *
		 * <profile name="dev">
		 * This check is enabled by the <param>DXL_EN</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system,
					    events::ID("check_dynamixel_driver_not_running"),
					    events::Log::Error, "Tilt servo driver not running");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Tilt servo driver not running");
		}

		return;
	}

	const uint8_t active_mask = servo_angle.active_mask;
	const uint8_t ready_mask = servo_angle.ready_mask & active_mask;

	if (active_mask == 0 || ready_mask != active_mask) {
		/* EVENT
		 * @description
		 * Not all tilt servos have torque enabled. Check the QGC messages or 'dynamixel status'
		 * for the servo that is not responding, not in position mode or in hardware error.
		 *
		 * <profile name="dev">
		 * This check is enabled by the <param>DXL_EN</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system,
					    events::ID("check_dynamixel_servos_not_ready"),
					    events::Log::Error, "Tilt servos not ready");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(),
					     "Preflight Fail: Tilt servos not ready (ready 0x%02x of 0x%02x)",
					     static_cast<unsigned>(ready_mask), static_cast<unsigned>(active_mask));
		}
	}

	// 서보 얼로케이션이 꺼져 있으면 서보는 중립에 고정된 채로 비행하게 된다 → arm 은 허용, 경고만
	int32_t servo_alloc = 1;

	if (_param_ca_servo_alloc != PARAM_INVALID && param_get(_param_ca_servo_alloc, &servo_alloc) == PX4_OK
	    && servo_alloc == 0) {
		/* EVENT
		 * @description
		 * The tilt servos are held at neutral because servo allocation is disabled.
		 *
		 * <profile name="dev">
		 * Set <param>CA_SERVO_ALLOC</param> to 1 to let the control allocator command the tilt servos.
		 * </profile>
		 */
		reporter.armingCheckFailure(NavModes::None, health_component_t::system,
					    events::ID("check_dynamixel_allocation_off"),
					    events::Log::Warning, "Tilt servo allocation disabled (CA_SERVO_ALLOC=0)");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_warning(reporter.mavlink_log_pub(), "Preflight: Tilt servo allocation disabled (CA_SERVO_ALLOC=0)");
		}
	}
}
