#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "core/units/units.h"
#include "core/utils/math_util.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"

class TankVoltageConstraint : public TrajectoryConstraint {
 public:
  TankVoltageConstraint(LinearVelocityFeedforward Kv, LinearAccelerationFeedforward Ka, Voltage maxVoltage, Length trackWidth)
      : m_Kv(Kv),
        m_Ka(Ka),
        m_maxVoltage(maxVoltage),
        m_trackWidth(trackWidth) {}

  Velocity max_velocity(
      const Pose2d& pose, Curvature curvature,
      Velocity velocity) const override {
    return Velocity(std::numeric_limits<double>::max());
  }

  MinMax min_max_acceleration(
      const Pose2d& pose, Curvature curvature,
      Velocity speed) const override {
    Velocity leftVelocity = (speed - (m_trackWidth / 2 * (speed * curvature / 1_rad)));
    Velocity rightVelocity = (speed + (m_trackWidth / 2 * (speed * curvature / 1_rad)));

    Velocity maxWheelSpeed = units::max(leftVelocity, rightVelocity);
    Velocity minWheelSpeed = units::min(leftVelocity, rightVelocity);

    Acceleration maxWheelAcceleration = (m_maxVoltage - (m_Kv * maxWheelSpeed)) / m_Ka;
    Acceleration minWheelAcceleration = (-m_maxVoltage - m_Kv * minWheelSpeed) / m_Ka;

    Acceleration maxChassisAcceleration;
    Acceleration minChassisAcceleration;

    const double speedVal = speed.inps();
    const double speedSgn = std::abs(speedVal) <= 1e-9 ? 1.0 : (speedVal < 0.0 ? -1.0 : 1.0);
    const double curvd = abs(curvature / 1_rad).canonical_value();
    const double twd = m_trackWidth.canonical_value();
    const double maxDenom = 1.0 + (twd * curvd * speedSgn / 2.0);
    const double minDenom = 1.0 - (twd * curvd * speedSgn / 2.0);

    maxChassisAcceleration = maxWheelAcceleration / (std::abs(maxDenom) < 1e-6 ? 1e-6 : maxDenom);
    minChassisAcceleration = minWheelAcceleration / (std::abs(minDenom) < 1e-6 ? 1e-6 : minDenom);

    if (abs(curvature) > 1E-9_radpm && (m_trackWidth / 2.0) > 1_rad / abs(curvature)) {
      if (speed > 0_mps && minChassisAcceleration > 0_inps2) {
        minChassisAcceleration = -minChassisAcceleration;
      } else if (speed < 0_mps && maxChassisAcceleration < 0_inps2) {
        maxChassisAcceleration = -maxChassisAcceleration;
      }
    }

    return {minChassisAcceleration, maxChassisAcceleration};
  }

  std::unique_ptr<TrajectoryConstraint> clone() const override {
    return std::unique_ptr<TrajectoryConstraint>(new TankVoltageConstraint(*this));
  }

 private:
  LinearVelocityFeedforward m_Kv;
  LinearAccelerationFeedforward m_Ka;
  Voltage m_maxVoltage;
  Length m_trackWidth;
};
