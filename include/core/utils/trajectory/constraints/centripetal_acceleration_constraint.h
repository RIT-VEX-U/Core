#pragma once

#include <cmath>
#include <memory>

#include "core/units/units.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"

/**
 * @brief Trajectory constraint that limits maximum velocity to prevent excessive centripetal acceleration.
 */
class CentripetalAccelerationConstraint : public TrajectoryConstraint {
 public:
  /**
   * @brief Constructs a CentripetalAccelerationConstraint.
   * @param maxCentripetalAcceleration Maximum allowed centripetal acceleration.
   */
  explicit CentripetalAccelerationConstraint(Acceleration maxCentripetalAcceleration)
      : m_maxCentripetalAcceleration(maxCentripetalAcceleration) {}

  /**
   * @brief Computes maximum allowed velocity based on path curvature and centripetal acceleration limits.
   * @param pose 2D position and orientation.
   * @param curvature Path curvature in rad/meter.
   * @param velocity Candidate linear velocity.
   * @return Constrained Velocity limit.
   */
  Velocity max_velocity(
      const Pose2d& pose, Curvature curvature,
      Velocity velocity) const override {
    if (abs(curvature) < 1e-9_radpm) {
      return Velocity::from_canonical(std::numeric_limits<double>::max());
    }
    return sqrt(m_maxCentripetalAcceleration / abs(curvature / 1_rad));
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
   * @return Unique pointer to cloned CentripetalAccelerationConstraint instance.
   */
  std::unique_ptr<TrajectoryConstraint> clone() const override {
    return std::unique_ptr<TrajectoryConstraint>(new CentripetalAccelerationConstraint(*this));
  }

 private:
  Acceleration m_maxCentripetalAcceleration; ///< Maximum centripetal acceleration
};
