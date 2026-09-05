#pragma once

#include <cmath>
#include <memory>

#include "core/units/units.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"

/**
 * @brief Trajectory constraint that enforces a strict upper limit on velocity.
 */
class MaxVelocityConstraint : public TrajectoryConstraint {
 public:
  /**
   * @brief Constructs a MaxVelocityConstraint.
   * @param maxVelocity Maximum allowed linear velocity.
   */
  explicit MaxVelocityConstraint(Velocity maxVelocity)
      : m_maxVelocity(abs(maxVelocity)) {}

  /**
   * @brief Computes maximum allowed velocity.
   * @param pose 2D position and orientation.
   * @param curvature Path curvature in rad/meter.
   * @param velocity Candidate linear velocity.
   * @return Constrained Velocity limit.
   */
  Velocity max_velocity(
      const Pose2d& pose, Curvature curvature,
      Velocity velocity) const override {
    return m_maxVelocity;
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
   * @return Unique pointer to cloned MaxVelocityConstraint instance.
   */
  std::unique_ptr<TrajectoryConstraint> clone() const override {
    return std::unique_ptr<TrajectoryConstraint>(new MaxVelocityConstraint(*this));
  }

 private:
  Velocity m_maxVelocity; ///< Maximum allowed linear velocity
};
