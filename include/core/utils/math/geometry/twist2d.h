#pragma once

#include "cevalm.hpp"
#include "core/utils/math/eigen_interface.h"
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
class Twist2d {
   private:
    units::Length dx_;
    units::Length dy_;
    units::Angle dtheta_;

   public:
    /// Default Constructor for Twist2d
    constexpr Twist2d() : dx_(0), dy_(0), dtheta_(0) {}

    /**
     * Constructs a twist with given translation and angle deltas.
     *
     * @param dx the linear dx component.
     * @param dy the linear dy component.
     * @param dtheta the angular dtheta component.
     */
    constexpr Twist2d(units::Length dx, units::Length dy, units::Angle dtheta)
        : dx_{dx}, dy_{dy}, dtheta_{dtheta} {}

    /**
     * Constructs a twist with given translation and angle deltas.
     *
     * @param twist_vector vector of the form [dx, dy, dtheta]
     * @param length_unit unit for dx and dy.
     * @param angle_unit unit for dtheta, defaults to radians.
     */
    constexpr Twist2d(
            const Eigen::Vector3d& twist_vector, units::Length length_unit,
            units::Angle angle_unit = units::radians
    )
        : dx_{twist_vector[0], length_unit},
          dy_{twist_vector[1], length_unit},
          dtheta_{twist_vector[2], angle_unit} {}

    /// @returns the x displacement.
    constexpr units::Length dx() const { return dx_; }

    /// @returns the x displacement in the supplied unit.
    constexpr double dx(units::Length unit) const { return dx_.to(unit); }

    /// Sets the x displacement.
    constexpr void set_dx(units::Length val) { dx_ = val; }

    /// @returns the y displacement.
    constexpr units::Length dy() const { return dy_; }

    /// @returns the y displacement in the supplied unit.
    constexpr double dy(units::Length unit) const { return dy_.to(unit); }

    /// Sets the y displacement.
    constexpr void set_dy(units::Length val) { dy_ = val; }

    /// @returns the Angle displacement
    constexpr units::Angle dtheta() const { return dtheta_; }

    /// @returns the angle displacement in the supplied unit.
    constexpr double dtheta(units::Angle unit) const { return dtheta_.to(unit); }

    /// Sets the angle displacement.
    constexpr void set_dtheta(units::Angle val) { dtheta_ = val; }

    /**
     * Returns [dx, dy, dtheta] in the supplied units. Angles default to radians.
     *
     * @param length_unit the unit of length to get the values as
     * @param angle_unit the unit of angle to get the rotation as, default radians
     * @return EVec<3> containing the values.
     */
    EVec<3> as_vector(units::Length length_unit, units::Angle angle_unit = units::radians) const {
        return EVec<3>{dx_.to(length_unit), dy_.to(length_unit), dtheta_.to(angle_unit)};
    }

    /// Multiplies this twist by a scalar.
    constexpr Twist2d operator*(double scalar) const {
        return Twist2d{dx_ * scalar, dy_ * scalar, dtheta_ * scalar};
    }

    /// Multiplies a scalar by this twist.
    friend constexpr Twist2d operator*(double scalar, const Twist2d& twist) {
        return twist * scalar;
    }

    /// Multiplies this twist by a scalar.
    constexpr Twist2d& operator*=(double scalar) { return *this = *this * scalar; }

    /// Divides this twist by a scalar.
    constexpr Twist2d operator/(double scalar) const { return *this * (1.0 / scalar); }

    /// Divides this twist by a scalar.
    constexpr Twist2d& operator/=(double scalar) { return *this = *this / scalar; }

    /// Checks exact equality between this and another twist.
    constexpr bool operator==(const Twist2d& other) const {
        return dx_ == other.dx_ && dy_ == other.dy_ && dtheta_ == other.dtheta_;
    }

    /**
     * Checks linear distance and unwrapped angle difference against tolerances.
     * Defaults to 1um and 1e-6 radians.
     */
    constexpr bool is_near(
            const Twist2d& other, units::Length distance_tolerance = units::Length(1e-6),
            units::Angle angle_tolerance = units::Angle(1e-6)
    ) const {
        return units::hypot(dx_ - other.dx_, dy_ - other.dy_) <= distance_tolerance &&
               units::abs(dtheta_ - other.dtheta_) <= angle_tolerance;
    }
};
