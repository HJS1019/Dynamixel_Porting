#include "DynamixelPositionMapping.hpp"

#include <gtest/gtest.h>
#include <limits>

using DynamixelPositionMapping::Calibration;

TEST(DynamixelPositionMapping, NeutralIsZeroRaw)
{
	const Calibration calibration{2048, 1, 1592, 2504};
	uint32_t position = 0;

	ASSERT_TRUE(DynamixelPositionMapping::angleToPosition(calibration, 0.f, position));
	EXPECT_EQ(position, 2048u);
}

TEST(DynamixelPositionMapping, MapsPointSevenRadiansToCalibratedLimits)
{
	const Calibration calibration{2048, 1, 1592, 2504};
	uint32_t position = 0;

	ASSERT_TRUE(DynamixelPositionMapping::angleToPosition(calibration, 0.7f, position));
	EXPECT_EQ(position, 2504u);

	ASSERT_TRUE(DynamixelPositionMapping::angleToPosition(calibration, -0.7f, position));
	EXPECT_EQ(position, 1592u);
}

TEST(DynamixelPositionMapping, SaturatesCommandsBeyondLimits)
{
	const Calibration calibration{2048, 1, 1592, 2504};
	uint32_t position = 0;

	ASSERT_TRUE(DynamixelPositionMapping::angleToPosition(calibration, 1000.f, position));
	EXPECT_EQ(position, 2504u);

	ASSERT_TRUE(DynamixelPositionMapping::angleToPosition(calibration, -1000.f, position));
	EXPECT_EQ(position, 1592u);
}

TEST(DynamixelPositionMapping, ReversesDirection)
{
	const Calibration calibration{2048, -1, 1592, 2504};
	uint32_t position = 0;

	ASSERT_TRUE(DynamixelPositionMapping::angleToPosition(calibration, 0.7f, position));
	EXPECT_EQ(position, 1592u);

	ASSERT_TRUE(DynamixelPositionMapping::angleToPosition(calibration, -0.7f, position));
	EXPECT_EQ(position, 2504u);
}

TEST(DynamixelPositionMapping, RawAngleRoundTripIsWithinOneCount)
{
	const Calibration calibration{2048, -1, 1592, 2504};
	constexpr uint32_t original_position = 2200;
	const float angle = DynamixelPositionMapping::positionToAngle(calibration, original_position);
	uint32_t round_trip_position = 0;

	ASSERT_TRUE(std::isfinite(angle));
	ASSERT_TRUE(DynamixelPositionMapping::angleToPosition(calibration, angle, round_trip_position));
	EXPECT_NEAR(static_cast<double>(round_trip_position), static_cast<double>(original_position), 1.0);
}

TEST(DynamixelPositionMapping, RejectsNonFiniteAngles)
{
	const Calibration valid{2048, 1, 1592, 2504};
	uint32_t position = 1234;

	EXPECT_FALSE(DynamixelPositionMapping::angleToPosition(valid,
			std::numeric_limits<float>::quiet_NaN(), position));
	EXPECT_EQ(position, 1234u);
	EXPECT_FALSE(DynamixelPositionMapping::angleToPosition(valid,
			std::numeric_limits<float>::infinity(), position));
	EXPECT_EQ(position, 1234u);
}

TEST(DynamixelPositionMapping, RejectsInvalidCalibration)
{
	const Calibration invalid_calibrations[] {
		{2048, 0, 1592, 2504},
		{2048, 2, 1592, 2504},
		{2048, 1, -1, 2504},
		{2048, 1, 1592, 4096},
		{2048, 1, 2000, 2000},
		{1500, 1, 1592, 2504},
		{2600, 1, 1592, 2504},
	};

	for (const Calibration &calibration : invalid_calibrations) {
		uint32_t position = 1234;
		EXPECT_FALSE(DynamixelPositionMapping::validCalibration(calibration));
		EXPECT_FALSE(DynamixelPositionMapping::angleToPosition(calibration, 0.f, position));
		EXPECT_EQ(position, 1234u);
		EXPECT_FALSE(std::isfinite(DynamixelPositionMapping::positionToAngle(calibration, 2048)));
	}
}

TEST(DynamixelPositionMapping, RejectsOutOfRangeRawFeedback)
{
	const Calibration valid{2048, 1, 1592, 2504};
	EXPECT_FALSE(std::isfinite(DynamixelPositionMapping::positionToAngle(valid, 4096)));
}
