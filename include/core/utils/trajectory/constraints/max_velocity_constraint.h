#pragma once

#include <cmath>
#include <memory>

#include "core/units/units.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"

class MaxVelocityConstraint : public TrajectoryConstraint {
 public:
  explicit MaxVelocityConstraint(double maxVelocity)
      : m_maxVelocity(std::abs(maxVelocity)) {}

  Velocity max_velocity(
      const Pose2d& pose, Curvature curvature,
      Velocity velocity) const override {
    return m_maxVelocity;
  }

  MinMax min_max_acceleration(
      const Pose2d& pose, Curvature curvature,
      Velocity speed) const override {
    return {};
  }

  std::unique_ptr<TrajectoryConstraint> clone() const override {
    return std::unique_ptr<TrajectoryConstraint>(new MaxVelocityConstraint(*this));
  }

 private:
  Velocity m_maxVelocity;
};
