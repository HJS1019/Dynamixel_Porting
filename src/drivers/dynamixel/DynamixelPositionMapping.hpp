#pragma once

#include <cmath>
#include <cstdint>

namespace DynamixelPositionMapping
{

static constexpr uint32_t PositionMinimum = 0;
static constexpr uint32_t PositionMaximum = 4095;
static constexpr double TwoPi = 2.0 * 3.14159265358979323846;
static constexpr double CountsPerRadian = 4096.0 / TwoPi;
static constexpr float RadiansPerCount = static_cast<float>(TwoPi / 4096.0);

struct Calibration {
	int32_t zero;
	int32_t direction;
	int32_t minimum;
	int32_t maximum;
};

inline bool validCalibration(const Calibration &calibration)
{
	return (calibration.direction == -1 || calibration.direction == 1)
	       && calibration.minimum >= static_cast<int32_t>(PositionMinimum)
	       && calibration.maximum <= static_cast<int32_t>(PositionMaximum)
	       && calibration.minimum < calibration.maximum
	       && calibration.zero >= calibration.minimum
	       && calibration.zero <= calibration.maximum;
}

inline bool angleToPosition(const Calibration &calibration, float angle_radians, uint32_t &position)
{
	if (!validCalibration(calibration) || !std::isfinite(angle_radians)) {
		return false;
	}

	const double rounded = round(static_cast<double>(calibration.zero)
				     + static_cast<double>(calibration.direction)
				     * static_cast<double>(angle_radians) * CountsPerRadian);

	if (rounded <= calibration.minimum) {
		position = static_cast<uint32_t>(calibration.minimum);

	} else if (rounded >= calibration.maximum) {
		position = static_cast<uint32_t>(calibration.maximum);

	} else {
		position = static_cast<uint32_t>(rounded);
	}

	return true;
}

inline float positionToAngle(const Calibration &calibration, uint32_t position)
{
	if (!validCalibration(calibration) || position > PositionMaximum) {
		return NAN;
	}

	return calibration.direction
	       * (static_cast<int32_t>(position) - calibration.zero)
	       * RadiansPerCount;
}

} // namespace DynamixelPositionMapping
