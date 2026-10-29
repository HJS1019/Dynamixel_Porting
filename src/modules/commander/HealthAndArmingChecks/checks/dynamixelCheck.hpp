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

#include "../Common.hpp"

#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/servo_angle.h>

/**
 * 틸트 서보(Dynamixel) arm 차단 점검
 *
 * 라떼판다 시절에는 사람이 ssh로 접속해 토크를 켜고 손으로 눌러 확인했다.
 * Pixhawk 단독 운용에서는 그 확인을 이 점검이 대신한다.
 *
 * DXL_EN = 1 일 때만 동작하며, 다음 중 하나라도 해당하면 arm 을 거부한다.
 *   - dynamixel 드라이버의 servo_angle 이 0.5초 넘게 안 들어옴 (드라이버 미실행)
 *   - 설정된 서보(active_mask) 중 하나라도 ready 가 아님 (무응답/토크 OFF/모드 불일치/HW 에러)
 * CA_SERVO_ALLOC = 0 이면 arm 은 막지 않고 경고만 띄운다 (서보가 중립에 고정된 채 비행).
 *
 * 파라미터는 param_find 로 찾는다: dynamixel 드라이버가 빌드에 없는 보드(SITL 등)에서도
 * 컴파일이 깨지지 않고, 그 경우 이 점검은 아무것도 하지 않는다.
 */
class DynamixelChecks : public HealthAndArmingCheckBase
{
public:
	DynamixelChecks();
	~DynamixelChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	uORB::Subscription _servo_angle_sub{ORB_ID(servo_angle)};

	param_t _param_dxl_en{PARAM_INVALID};
	param_t _param_ca_servo_alloc{PARAM_INVALID};
};
