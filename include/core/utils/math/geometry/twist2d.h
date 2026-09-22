#pragma once

#include "core/utils/math/eigen_interface.h"
#include "cevalm.hpp"
#include "core/utils/units.h"

/**
 * Class representing a difference between two poses,
 * more specifically a distance along an arc from a pose.
 * The angular displacement is continuous and is never wrapped.
 * More specifically, dx and dy represent integrated motion in
 * the moving reference frame of the robot.
 *
 * Assumes conventional cartesian coordinate system:
 * Looking down at the coordinate plane,
 * +X is right
 * +Y is up
 * +Theta is counterclockwise
 */
struct Twist2d {
    /**
     * Default Constructor for Twist2d
     */
    constexpr Twist2d() : dx_(0), dy_(0), dtheta_(0) {}

    /**
     * Returns the x displacement.
     */
    constexpr units::Length dx() const { return dx_; }

    /**
     * Returns the y displacement.
     */
    constexpr units::Length dy() const { return dy_; }

    /**
     * Returns the angular displacement without wrapping.
     */
    constexpr units::Angle dtheta() const { return dtheta_; }

    /**
     * Returns the x displacement in the supplied unit.
     */
    constexpr double dx(units::Length unit) const { return dx_.to(unit); }

    /**
     * Returns the y displacement in the supplied unit.
     */
    constexpr double dy(units::Length unit) const { return dy_.to(unit); }

    /**
     * Returns the angular displacement in the supplied unit without wrapping.
     */
    constexpr double dtheta(units::Angle unit) const { return dtheta_.to(unit); }

    /**
     * Constructs a twist with given translation and angle deltas.
     * @param dx the linear dx component.
     * @param dy the linear dy component.
     * @param dtheta the angular dtheta component.
     */
    constexpr Twist2d(units::Length dx, units::Length dy, units::Angle dtheta) : dx_{dx}, dy_{dy}, dtheta_{dtheta} {}

    /**
     * Constructs a twist with given translation and angle deltas.
     * @param twist_vector vector of the form [dx, dy, dtheta]
     * @param length_unit unit for dx and dy.
     * @param angle_unit unit for dtheta, defaults to radians.
     */
    constexpr Twist2d(const Eigen::Vector3d &twist_vector, units::Length length_unit,
                      units::Angle angle_unit = units::radians)
        : dx_{twist_vector[0], length_unit}, dy_{twist_vector[1], length_unit},
          dtheta_{twist_vector[2], angle_unit} {}

    /**
     * Returns [dx, dy, dtheta] in the supplied units. Angles default to radians.
     */
    EVec<3> as_vector(units::Length length_unit,
                                       units::Angle angle_unit = units::radians) const {
        return EVec<3>{dx_.to(length_unit), dy_.to(length_unit), dtheta_.to(angle_unit)};
    }

    /**
     * Checks equality between this and another twist.
     * @param other the other twist to compare to.
     * @returns true if all displacements are within 1e-6 of each other (meters and radians).
     */
    constexpr bool operator==(const Twist2d &other) const {
        return cevalm::abs(dx_.internal() - other.dx_.internal()) < 1e-6 && cevalm::abs(dy_.internal() - other.dy_.internal()) < 1e-6 && cevalm::abs(dtheta_.internal() - other.dtheta_.internal()) < 1e-6;
    }

    /**
     * Multiplies this twist by a scalar.
     * @param scalar the scalar value to multiply by.
     */
    constexpr Twist2d operator*(double scalar) const {
        return Twist2d{dx_ * scalar, dy_ * scalar, dtheta_ * scalar};
    }

    /**
     * Divides this twist by a scalar.
     * @param scalar the scalar value to divide by.
     */
    constexpr Twist2d operator/(double scalar) const {
        return *this * (1.0 / scalar);
    }

    /**
     * Scales this twist without wrapping its angle.
     */
    constexpr Twist2d &operator*=(double scalar) {
        return *this = *this * scalar;
    }

    /**
     * Divides this twist without wrapping its angle.
     */
    constexpr Twist2d &operator/=(double scalar) {
        return *this = *this / scalar;
    }

    /**
     * Multiplies a scalar by this twist.
     */
    friend constexpr Twist2d operator*(double scalar, const Twist2d &twist) {
        return twist * scalar;
    }

    /**
     * Checks linear distance and unwrapped angle difference against tolerances.
     * Defaults to 1um and 1e-6 radians.
     */
    constexpr bool is_near(const Twist2d &other,
                           units::Length distance_tolerance = units::Length(1e-6),
                           units::Angle angle_tolerance = units::Angle(1e-6)) const {
        return units::hypot(dx_ - other.dx_, dy_ - other.dy_) <= distance_tolerance &&
               units::abs(dtheta_ - other.dtheta_) <= angle_tolerance;
    }

    units::Length dx_;
    units::Length dy_;
    units::Angle dtheta_;
};
