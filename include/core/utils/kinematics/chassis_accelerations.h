#pragma once

#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/math/geometry/translation2d.h"
#include "core/utils/units.h"

using namespace units::literals;

/**
 * @brief Represents robot chassis accelerations.
 */
struct ChassisAccelerations {
    /** @brief Acceleration along the x-axis. (Forward is positive) */
    units::Acceleration ax = 0_inps2;

    /** @brief Acceleration along the y-axis. (Left is positive) */
    units::Acceleration ay = 0_inps2;

    /** @brief Angular acceleration of the robot frame. (Counter-Clockwise is positive) */
    units::AngularAcceleration alpha = 0_radps2;

    ChassisAccelerations() = default;

    ChassisAccelerations(units::Acceleration ax, units::Acceleration ay, units::AngularAcceleration alpha)
        : ax(ax), ay(ay), alpha(alpha) {}

    /**
     * @brief Converts this field-relative set of accelerations into a robot-relative ChassisAccelerations object.
     * @param robotAngle The CCW-positive angle of the robot relative to the field.
     * @return Robot-relative ChassisAccelerations.
     */
    ChassisAccelerations to_robot_relative(const Rotation2d& robotAngle) const {
        Translation2d rotated = Translation2d(ax.to(units::inps2), ay.to(units::inps2))
                                    .rotate_by(Rotation2d(-robotAngle.radians()));
        return ChassisAccelerations(
            rotated.x() * units::inps2,
            rotated.y() * units::inps2,
            alpha
        );
    }

    /**
     * @brief Converts this robot-relative set of accelerations into a field-relative ChassisAccelerations object.
     * @param robotAngle The CCW-positive angle of the robot relative to the field.
     * @return Field-relative ChassisAccelerations.
     */
    ChassisAccelerations to_field_relative(const Rotation2d& robotAngle) const {
        Translation2d rotated = Translation2d(ax.to(units::inps2), ay.to(units::inps2))
                                    .rotate_by(robotAngle);
        return ChassisAccelerations(
            rotated.x() * units::inps2,
            rotated.y() * units::inps2,
            alpha
        );
    }

    // Operator overloads
    ChassisAccelerations operator+(const ChassisAccelerations& other) const {
        return ChassisAccelerations(ax + other.ax, ay + other.ay, alpha + other.alpha);
    }

    ChassisAccelerations operator-(const ChassisAccelerations& other) const {
        return ChassisAccelerations(ax - other.ax, ay - other.ay, alpha - other.alpha);
    }

    ChassisAccelerations operator-() const {
        return ChassisAccelerations(-ax, -ay, -alpha);
    }

    ChassisAccelerations operator*(double scalar) const {
        return ChassisAccelerations(ax * scalar, ay * scalar, alpha * scalar);
    }

    ChassisAccelerations operator/(double scalar) const {
        return ChassisAccelerations(ax / scalar, ay / scalar, alpha / scalar);
    }

    bool operator==(const ChassisAccelerations& other) const {
        return ax == other.ax && ay == other.ay && alpha == other.alpha;
    }

    bool operator!=(const ChassisAccelerations& other) const {
        return !(*this == other);
    }
};
