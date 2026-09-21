#pragma once

#include <algorithm>
#include <cmath>
#include <memory>

#include "core/utils/trajectory/constraints/trajectory_constraint.h"
#include "core/utils/units.h"

using namespace units::literals;

/**
 * @brief Trajectory constraint that limits velocity based on tank drive kinematics and maximum
 * wheel speed.
 */
class TankKinematicsConstraint : public TrajectoryConstraint {
   public:
    /**
     * @brief Constructs a TankKinematicsConstraint.
     * @param track_width Robot track width (distance between left and right wheels).
     * @param max_speed Maximum allowed speed of a single wheel.
     */
    TankKinematicsConstraint(units::Length track_width, units::Velocity max_speed)
        : track_width_(track_width), max_speed_(max_speed) {}

    /**
     * @brief Computes maximum allowed chassis velocity to keep wheel speeds within limits during
     * turns.
     * @param pose 2D position and orientation.
     * @param curvature Path curvature in rad/meter.
     * @param velocity Candidate linear velocity.
     * @return Constrained units::Velocity limit.
     */
    units::Velocity max_velocity(
            const Pose2d& pose, units::Curvature curvature, units::Velocity velocity
    ) const override {
        units::Velocity left_velocity =
                (velocity - (track_width_ / 2 * (velocity * curvature / 1_rad)));
        units::Velocity right_velocity =
                (velocity + (track_width_ / 2 * (velocity * curvature / 1_rad)));

        units::Velocity real_max_speed = units::max(units::abs(left_velocity), units::abs(right_velocity));

        if (real_max_speed > max_speed_) {
            left_velocity = left_velocity / real_max_speed * max_speed_;
            right_velocity = right_velocity / real_max_speed * max_speed_;
        }

        return (left_velocity + right_velocity) / 2.0;
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
     * @return Unique pointer to cloned TankKinematicsConstraint instance.
     */
    std::unique_ptr<TrajectoryConstraint> clone() const override {
        return std::unique_ptr<TrajectoryConstraint>(new TankKinematicsConstraint(*this));
    }

   private:
    units::Length track_width_;  ///< Robot track width
    units::Velocity max_speed_;  ///< Maximum allowed speed of a single wheel
};
