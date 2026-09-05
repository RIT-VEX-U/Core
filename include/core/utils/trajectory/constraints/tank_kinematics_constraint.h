#pragma once

#include <algorithm>
#include <cmath>
#include <memory>

#include "core/units/units.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"

/**
 * @brief Trajectory constraint that limits velocity based on tank drive kinematics and maximum wheel speed.
 */
class TankKinematicsConstraint : public TrajectoryConstraint {
 public:
  /**
   * @brief Constructs a TankKinematicsConstraint.
   * @param trackWidth Robot track width (distance between left and right wheels).
   * @param maxSpeed Maximum allowed speed of a single wheel.
   */
  TankKinematicsConstraint(Length trackWidth, Velocity maxSpeed)
      : m_trackWidth(trackWidth), m_maxSpeed(maxSpeed) {}

  /**
   * @brief Computes maximum allowed chassis velocity to keep wheel speeds within limits during turns.
   * @param pose 2D position and orientation.
   * @param curvature Path curvature in rad/meter.
   * @param velocity Candidate linear velocity.
   * @return Constrained Velocity limit.
   */
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

  /**
   * @brief Computes minimum and maximum allowed linear accelerations.
   * @param pose 2D position and orientation.
   * @param curvature Path curvature in rad/meter.
   * @param speed Current scalar speed.
   * @return MinMax acceleration bounds struct.
   */
  MinMax min_max_acceleration(
      const Pose2d& pose, Curvature curvature,
      Velocity speed) const override {
    return {};
  }

  /**
   * @brief Polymorphic deep-copy factory method.
   * @return Unique pointer to cloned TankKinematicsConstraint instance.
   */
  std::unique_ptr<TrajectoryConstraint> clone() const override {
    return std::unique_ptr<TrajectoryConstraint>(new TankKinematicsConstraint(*this));
  }

 private:
  Length m_trackWidth; ///< Robot track width
  Velocity m_maxSpeed; ///< Maximum allowed speed of a single wheel
};
