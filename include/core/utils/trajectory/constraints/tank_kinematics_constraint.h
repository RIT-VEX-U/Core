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
     * @param trackWidth Robot track width (distance between left and right wheels).
     * @param maxSpeed Maximum allowed speed of a single wheel.
     */
    TankKinematicsConstraint(units::Length trackWidth, units::Velocity maxSpeed)
        : trackWidth_(trackWidth), maxSpeed_(maxSpeed) {}

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
        units::Velocity leftVelocity =
                (velocity - (trackWidth_ / 2 * (velocity * curvature / 1_rad)));
        units::Velocity rightVelocity =
                (velocity + (trackWidth_ / 2 * (velocity * curvature / 1_rad)));

        units::Velocity realMaxSpeed = units::max(abs(leftVelocity), abs(rightVelocity));

        if (realMaxSpeed > maxSpeed_) {
            leftVelocity = leftVelocity / realMaxSpeed * maxSpeed_;
            rightVelocity = rightVelocity / realMaxSpeed * maxSpeed_;
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
    units::Length trackWidth_;  ///< Robot track width
    units::Velocity maxSpeed_;  ///< Maximum allowed speed of a single wheel
};
