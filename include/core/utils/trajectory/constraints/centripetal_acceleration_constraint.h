#pragma once

#include <cmath>
#include <memory>

#include "core/units/units.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"

class CentripetalAccelerationConstraint : public TrajectoryConstraint {
 public:
  explicit CentripetalAccelerationConstraint(Acceleration maxCentripetalAcceleration)
      : m_maxCentripetalAcceleration(maxCentripetalAcceleration) {}

  Velocity max_velocity(
      const Pose2d& pose, Curvature curvature,
      Velocity velocity) const override {
    if (abs(curvature) < 1e-9_radpm) {
      return Velocity::from_canonical(std::numeric_limits<double>::max());
    }
    return sqrt(m_maxCentripetalAcceleration / abs(curvature / 1_rad));
  }

  MinMax min_max_acceleration(
      const Pose2d& pose, Curvature curvature,
      Velocity speed) const override {
    return {};
  }

  std::unique_ptr<TrajectoryConstraint> clone() const override {
    return std::unique_ptr<TrajectoryConstraint>(new CentripetalAccelerationConstraint(*this));
  }

 private:
  Acceleration m_maxCentripetalAcceleration;
};
