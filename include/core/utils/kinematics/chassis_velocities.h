#pragma once

#include "core/utils/math/geometry/pose2d.h"
#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/math/geometry/translation2d.h"
#include "core/utils/math/geometry/twist2d.h"
#include "core/utils/units.h"

using namespace units::literals;

/**
 * @brief Represents robot chassis velocities.
 */
struct ChassisVelocities {
    /** @brief Velocity along the x-axis. (Forward is positive) */
    units::Velocity vx = 0_inps;

    /** @brief Velocity along the y-axis. (Left is positive) */
    units::Velocity vy = 0_inps;

    /** @brief Angular velocity of the robot frame. (Counter-Clockwise is positive) */
    units::AngularVelocity omega = 0_radps;

    ChassisVelocities() = default;

    ChassisVelocities(units::Velocity vx, units::Velocity vy, units::AngularVelocity omega)
        : vx(vx), vy(vy), omega(omega) {}

    /**
     * @brief Converts this field-relative set of velocities into a robot-relative ChassisVelocities object.
     * @param robotAngle The CCW-positive angle of the robot relative to the field.
     * @return Robot-relative ChassisVelocities.
     */
    ChassisVelocities to_robot_relative(const Rotation2d& robotAngle) const {
        Translation2d rotated = Translation2d(vx.to(units::inps), vy.to(units::inps))
                                    .rotate_by(Rotation2d(-robotAngle.radians()));
        return ChassisVelocities(
            rotated.x() * units::inps,
            rotated.y() * units::inps,
            omega
        );
    }

    /**
     * @brief Converts this robot-relative set of velocities into a field-relative ChassisVelocities object.
     * @param robotAngle The CCW-positive angle of the robot relative to the field.
     * @return Field-relative ChassisVelocities.
     */
    ChassisVelocities to_field_relative(const Rotation2d& robotAngle) const {
        Translation2d rotated = Translation2d(vx.to(units::inps), vy.to(units::inps))
                                    .rotate_by(robotAngle);
        return ChassisVelocities(
            rotated.x() * units::inps,
            rotated.y() * units::inps,
            omega
        );
    }

    /**
     * @brief Discretizes continuous-time chassis velocities.
     *
     * Compensates for translational skew when rotating a holonomic drivetrain. Converts
     * the decoupled translation/rotation over a timestep into a constant-curvature
     * trajectory representation.
     *
     * @param dt The duration of the timestep.
     * @return Discretized ChassisVelocities.
     */
    ChassisVelocities discretize(units::Time dt) const {
        // Construct the desired pose after a timestep, relative to the current pose
        // with decoupled translation and rotation
        Pose2d desired_pose(
            vx.to(units::inps) * dt.to(units::s),
            vy.to(units::inps) * dt.to(units::s),
            omega.to(units::radps) * dt.to(units::s)
        );

        // Calculate the constant-curvature path (Twist2d) to get there
        Twist2d twist = Pose2d(0.0, 0.0, 0.0).log(desired_pose);

        // Turn the deltas back into average velocities over the timestep
        return ChassisVelocities(
            (twist.dx() / dt.to(units::s)) * units::inps,
            (twist.dy() / dt.to(units::s)) * units::inps,
            (twist.dtheta() / dt.to(units::s)) * units::radps
        );
    }

    // Operator overloads
    ChassisVelocities operator+(const ChassisVelocities& other) const {
        return ChassisVelocities(vx + other.vx, vy + other.vy, omega + other.omega);
    }

    ChassisVelocities operator-(const ChassisVelocities& other) const {
        return ChassisVelocities(vx - other.vx, vy - other.vy, omega - other.omega);
    }

    ChassisVelocities operator-() const {
        return ChassisVelocities(-vx, -vy, -omega);
    }

    ChassisVelocities operator*(double scalar) const {
        return ChassisVelocities(vx * scalar, vy * scalar, omega * scalar);
    }

    ChassisVelocities operator/(double scalar) const {
        return ChassisVelocities(vx / scalar, vy / scalar, omega / scalar);
    }

    bool operator==(const ChassisVelocities& other) const {
        return vx == other.vx && vy == other.vy && omega == other.omega;
    }

    bool operator!=(const ChassisVelocities& other) const {
        return !(*this == other);
    }
};