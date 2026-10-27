/****************************************************************************
 *
 *   Copyright (c) 2018 - 2026 PX4 Development Team. All rights reserved.
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

#include "PositionControl.hpp"

#include <gtest/gtest.h>

#include <limits>

using namespace matrix;

class PositionControlTest : public ::testing::Test
{
protected:
	void configureController(const bool set_valid_attitude = true)
	{
		_control.setPositionGains(Vector3f{1.f, 1.f, 1.f}, Vector3f{0.f, 0.f, 0.f});
		_control.setVelocityGains(Vector3f{1.f, 1.f, 1.f}, Vector3f{0.f, 0.f, 0.f},
					  Vector3f{0.f, 0.f, 0.f});
		_control.setVelocityLimits(10.f, 10.f, 10.f);
		_control.setAccelerationLimits(10.f, 10.f);
		_control.setThrustLimits(0.1f, 0.9f);
		_control.setHorizontalThrustMargin(0.3f);
		_control.setTiltLimit(1.f);
		_control.setHoverThrust(0.5f);

		PositionControlStates state{};
		_control.setState(state);
		_control.setInputSetpoint(Vector4f{0.f, 0.f, 0.f, 0.f});
		_control.setVelocityFeedforward(Vector4f{0.f, 0.f, 0.f, 0.f});

		if (set_valid_attitude) {
			ASSERT_TRUE(_control.updateAttitude(Dcmf{}));
		}
	}

	PositionControl _control;
};

TEST_F(PositionControlTest, DisarmedOutputIsFiniteAndZero)
{
	configureController(false);

	EXPECT_TRUE(_control.update(0.01f, false, true));
	EXPECT_TRUE(_control.getVelocitySetpoint().isAllFinite());
	EXPECT_TRUE(_control.getAccelerationSetpoint().isAllFinite());
	EXPECT_TRUE(_control.getThrustSetpoint().isAllFinite());
	EXPECT_FLOAT_EQ(_control.getThrustSetpoint()(0), 0.f);
	EXPECT_FLOAT_EQ(_control.getThrustSetpoint()(1), 0.f);
	EXPECT_FLOAT_EQ(_control.getThrustSetpoint()(2), 0.f);
}

TEST_F(PositionControlTest, ArmedUpdateRequiresValidAttitude)
{
	configureController(false);

	Dcmf invalid_attitude{};
	invalid_attitude(0, 0) = std::numeric_limits<float>::quiet_NaN();
	EXPECT_FALSE(_control.updateAttitude(invalid_attitude));
	EXPECT_FALSE(_control.hasValidAttitude());
	EXPECT_FALSE(_control.update(0.01f, true, true));
	EXPECT_TRUE(_control.getThrustSetpoint().isAllFinite());

	EXPECT_TRUE(_control.updateAttitude(Dcmf{}));
	EXPECT_TRUE(_control.hasValidAttitude());
	EXPECT_TRUE(_control.update(0.01f, true, true));
}

TEST_F(PositionControlTest, ZeroErrorHasNoGravityCompensation)
{
	configureController();

	ASSERT_TRUE(_control.update(0.01f, true, true));
	EXPECT_FLOAT_EQ(_control.getAccelerationSetpoint()(0), 0.f);
	EXPECT_FLOAT_EQ(_control.getAccelerationSetpoint()(1), 0.f);
	EXPECT_FLOAT_EQ(_control.getAccelerationSetpoint()(2), 0.f);
	EXPECT_FLOAT_EQ(_control.getThrustSetpoint()(0), 0.f);
	EXPECT_FLOAT_EQ(_control.getThrustSetpoint()(1), 0.f);
	EXPECT_FLOAT_EQ(_control.getThrustSetpoint()(2), 0.f);
}

TEST_F(PositionControlTest, UpwardVelocityLimitProducesNegativeNedForce)
{
	configureController();
	_control.setVelocityLimits(10.f, 0.4f, 10.f);
	_control.setInputSetpoint(Vector4f{0.f, 0.f, -10.f, 0.f});

	ASSERT_TRUE(_control.update(0.01f, true, true));
	EXPECT_NEAR(_control.getVelocitySetpoint()(2), -0.4f, 1e-6f);
	EXPECT_NEAR(_control.getAccelerationSetpoint()(2), -0.4f, 1e-6f);
	EXPECT_NEAR(_control.getThrustSetpoint()(2), -3.2f, 1e-5f);
}

TEST_F(PositionControlTest, ArmTransitionResetsVelocityIntegrator)
{
	configureController();
	_control.setVelocityGains(Vector3f{}, Vector3f{0.f, 0.f, 1.f}, Vector3f{});
	_control.setInputSetpoint(Vector4f{0.f, 0.f, -1.f, 0.f});

	ASSERT_TRUE(_control.update(0.5f, true, true));
	EXPECT_NEAR(_control.getThrustSetpoint()(2), -4.f, 1e-5f);

	ASSERT_TRUE(_control.update(0.5f, false, true));
	EXPECT_FLOAT_EQ(_control.getThrustSetpoint()(2), 0.f);

	ASSERT_TRUE(_control.update(0.5f, true, true));
	EXPECT_NEAR(_control.getThrustSetpoint()(2), -4.f, 1e-5f);
}

TEST_F(PositionControlTest, NonFiniteStateIsRejectedBeforeForceCalculation)
{
	configureController();
	ASSERT_TRUE(_control.update(0.01f, true, true));

	PositionControlStates invalid_state{};
	invalid_state.velocity(2) = std::numeric_limits<float>::quiet_NaN();
	_control.setState(invalid_state);

	EXPECT_FALSE(_control.update(0.01f, true, true));
	EXPECT_TRUE(_control.getVelocitySetpoint().isAllFinite());
	EXPECT_TRUE(_control.getAccelerationSetpoint().isAllFinite());
	EXPECT_TRUE(_control.getThrustSetpoint().isAllFinite());
}

TEST_F(PositionControlTest, AttitudeSetpointCarriesCurrentForceInterface)
{
	configureController();
	_control.setVelocityLimits(10.f, 0.4f, 10.f);
	_control.setInputSetpoint(Vector4f{0.f, 0.f, -10.f, 0.f});
	ASSERT_TRUE(_control.update(0.01f, true, true));

	vehicle_attitude_setpoint_s attitude_setpoint{};
	bool position_control_enabled = true;
	Vector4f manual_input{};
	_control.getAttitudeSetpoint(attitude_setpoint, position_control_enabled, manual_input);

	EXPECT_TRUE(PX4_ISFINITE(attitude_setpoint.thrust_body[0]));
	EXPECT_TRUE(PX4_ISFINITE(attitude_setpoint.thrust_body[1]));
	EXPECT_TRUE(PX4_ISFINITE(attitude_setpoint.thrust_body[2]));
	EXPECT_NEAR(attitude_setpoint.thrust_body[2], -3.2f, 1e-5f);
}
