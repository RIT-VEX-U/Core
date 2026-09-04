#pragma once

#include <algorithm>
#include <cmath>
#include <memory>

#include "core/units/units.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"

class TankKinematicsConstraint : public TrajectoryConstraint {
 public:
  TankKinematicsConstraint(Length trackWidth, Velocity maxSpeed)
      : m_trackWidth(trackWidth), m_maxSpeed(maxSpeed) {}

  Velocity max_velocity(
      const Pose2d& pose, Curvature curvature,
      Velocity velocity) const override {
    Velocity leftVelocity = (velocity - (m_trackWidth / 2 * (velocity * curvature / 1_rad)));
    Velocity rightVelocity = (velocity + (m_trackWidth / 2 * (velocity * curvature / 1_rad)));

    Velocity realMaxSpeed = units::max(abs(leftVelocity), abs(rightVelocity));

    if (realMaxSpeed > m_maxSpeed) {
      leftVelocity = leftVelocity / realMaxSpeed * m_maxSpeed;
      rightVelocity = rightVelocity / realMaxSpeed * m_maxSpeed;
    }

    return (leftVelocity + rightVelocity) / 2.0;
  }

  MinMax min_max_acceleration(
      const Pose2d& pose, Curvature curvature,
      Velocity speed) const override {
    return {};
  }

  std::unique_ptr<TrajectoryConstraint> clone() const override {
    return std::unique_ptr<TrajectoryConstraint>(new TankKinematicsConstraint(*this));
  }

 private:
  Length m_trackWidth;
  Velocity m_maxSpeed;
};
