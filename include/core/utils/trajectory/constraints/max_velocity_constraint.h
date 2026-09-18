#pragma once

#include <cmath>
#include <memory>

#include "core/utils/trajectory/constraints/trajectory_constraint.h"
#include "core/utils/units.h"

using namespace units::literals;

/**
 * @brief Trajectory constraint that enforces a strict upper limit on velocity.
 */
class MaxVelocityConstraint : public TrajectoryConstraint {
   public:
    /**
     * @brief Constructs a MaxVelocityConstraint.
     * @param maxVelocity Maximum allowed linear velocity.
     */
    explicit MaxVelocityConstraint(units::Velocity maxVelocity) : maxVelocity_(abs(maxVelocity)) {}

    /**
     * @brief Computes maximum allowed velocity.
     * @param pose 2D position and orientation.
     * @param curvature Path curvature in rad/meter.
     * @param velocity Candidate linear velocity.
     * @return Constrained units::Velocity limit.
     */
    units::Velocity max_velocity(
            const Pose2d& pose, units::Curvature curvature, units::Velocity velocity
    ) const override {
        return maxVelocity_;
    }

    /**
     * @brief Computes minimum and maximum allowed linear accelerations.
     * @param pose 2D position and orientation.
     * @param curvature Path curvature in rad/meter.
     * @param speed Current scalar speed.
     * @return MinMax acceleration bounds struct.
     */
    MinMax min_max_acceleration(
            const Pose2d& pose, units::Curvature curvature, units::Velocity speed
    ) const override {
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
    units::Velocity maxVelocity_;  ///< Maximum allowed linear velocity
};
