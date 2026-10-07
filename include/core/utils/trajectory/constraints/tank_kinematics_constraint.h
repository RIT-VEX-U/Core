#pragma once

#include <algorithm>
#include <cmath>
#include <memory>

#include "core/utils/kinematics/differential_drive_kinematics.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"
#include "core/utils/units.h"

using namespace units::literals;

/**
 * @brief Trajectory constraint that limits velocity and acceleration based on tank drive kinematics
 * and maximum wheel constraints.
 */
class TankKinematicsConstraint : public TrajectoryConstraint {
   public:
    /**
     * @brief Constructs a TankKinematicsConstraint.
     * @param track_width Robot track width (distance between left and right wheels).
     * @param max_speed Maximum allowed speed of a single wheel.
     * @param max_acceleration Maximum allowed acceleration of a single wheel.
     */
    TankKinematicsConstraint(
            units::Length track_width,
            units::Velocity max_speed,
            units::Acceleration max_acceleration = 10000_inps2
    )
        : kinematics_(track_width), max_speed_(max_speed), max_acceleration_(max_acceleration) {}

    /**
     * @brief Constructs a TankKinematicsConstraint from DifferentialDriveKinematics.
     * @param kinematics Differential drive kinematics model.
     * @param max_speed Maximum allowed speed of a single wheel.
     * @param max_acceleration Maximum allowed acceleration of a single wheel.
     */
    TankKinematicsConstraint(
            const DifferentialDriveKinematics& kinematics,
            units::Velocity max_speed,
            units::Acceleration max_acceleration = 10000_inps2
    )
        : kinematics_(kinematics), max_speed_(max_speed), max_acceleration_(max_acceleration) {}

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
        units::AngularVelocity omega = velocity * curvature;
        ChassisVelocities chassis_speeds{velocity, 0_inps, omega};
        
        DifferentialDriveWheelVelocities wheel_speeds = kinematics_.to_wheel_velocities(chassis_speeds);

        units::Velocity real_max_speed = units::max(units::abs(wheel_speeds.left), units::abs(wheel_speeds.right));

        if (real_max_speed > max_speed_) {
            return velocity * (max_speed_ / real_max_speed);
        }

        return velocity;
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
        // Linear acceleration a mapping to wheel accelerations:
        // alpha_kinematic = a * curvature
        // wheel_accels = kinematics_.to_wheel_accelerations({a, 0, alpha_kinematic})
        // Since it's a linear mapping: a_left = a * left_factor, a_right = a * right_factor

        double left_factor = 1.0 - (curvature.to(units::radpm) * kinematics_.track_width.to(units::m)) / 2.0;
        double right_factor = 1.0 + (curvature.to(units::radpm) * kinematics_.track_width.to(units::m)) / 2.0;

        units::Acceleration max_a = max_acceleration_;
        units::Acceleration min_a = -max_acceleration_;

        double factors[2] = {left_factor, right_factor};
        for (int i = 0; i < 2; ++i) {
            double factor = factors[i];
            if (std::abs(factor) > 1e-6) {
                units::Acceleration limit = max_acceleration_ / std::abs(factor);
                max_a = units::min(max_a, limit);
                min_a = units::max(min_a, -limit);
            }
        }

        return {min_a, max_a};
    }

    /**
     * @brief Polymorphic deep-copy factory method.
     * @return Unique pointer to cloned TankKinematicsConstraint instance.
     */
    std::unique_ptr<TrajectoryConstraint> clone() const override {
        return std::unique_ptr<TrajectoryConstraint>(new TankKinematicsConstraint(*this));
    }

   private:
    DifferentialDriveKinematics kinematics_;  ///< Kinematics for tank drive
    units::Velocity max_speed_;               ///< Maximum allowed speed of a single wheel
    units::Acceleration max_acceleration_;    ///< Maximum allowed acceleration of a single wheel
};
