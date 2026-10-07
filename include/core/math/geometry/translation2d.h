#pragma once

#include <cmath>
#include <vector>

#include "cevalm.hpp"
#include "core/math/eigen_interface.h"
#include "core/math/geometry/rotation2d.h"
#include "core/utils/units.h"

class Rotation2d;

template <typename Q>
concept LinearKinematicQuantity =
        units::KinematicQuantity<Q> && std::ratio_equal_v<typename Q::length, std::ratio<1>> &&
        std::ratio_equal_v<typename Q::angle, std::ratio<0>>;

/**
 * Class representing a 2d vector. Depending on unit this could be
 * a Translation2d, Velocity2d, Acceleration2d, Jerk2d, or other
 *
 * Assumes conventional cartesian coordinate system:
 * Looking down at the coordinate plane,
 * +X is right
 * +Y is up
 * +Theta is counterclockwise
 */
template <LinearKinematicQuantity Q>
class LinearVector2d {
   private:
    Q x_;
    Q y_;

   public:
    /// Default Constructor for LinearVector2d valued (0, 0)
    constexpr LinearVector2d() : x_{0}, y_{0} {}

    /**
     * Constructs a vector with the given x and y values.
     *
     * @param x The x component of the vector.
     * @param y The y component of the vector.
     */
    constexpr LinearVector2d(Q x, Q y) : x_{x}, y_{y} {}

    /**
     * Constructs a vector given polar coordinates of the form (r, theta).
     *
     * @param r The magnitude of the vector.
     * @param theta The angle of the vector.
     */
    constexpr LinearVector2d(Q r, Rotation2d theta)
        : x_{r * theta.f_cos()}, y_{r * theta.f_sin()} {}

    /**
     * Constructs a vector with the values from the given Eigen::Vector.
     *
     * @param vector The vector whose values will be used.
     * @param unit The unit to use when assigning x and y (e.g. units::inches)
     */
    constexpr LinearVector2d(Eigen::Vector2d vector, Q unit)
        : x_{vector[0], unit}, y_{vector[1], unit} {}

    /// Gets the x component.
    constexpr Q x() const { return x_; }

    /// Gets x in the supplied unit.
    constexpr double x(Q unit) const { return x_.to(unit); }

    /// Sets the x component.
    constexpr void set_x(Q val) { x_ = val; }

    /// Gets the y component.
    constexpr Q y() const { return y_; }

    /// Gets y in the supplied unit.
    constexpr double y(Q unit) const { return y_.to(unit); }

    /// Sets the y component.
    constexpr void set_y(Q val) { y_ = val; }

    /// Gets the angle of the vector from the x axis.
    constexpr Rotation2d theta() const { return Rotation2d(x_.internal(), y_.internal()); }

    /**
     * Returns the vector as an Eigen::Vector2d.
     *
     * @param unit The unit to get the x and y components as (e.g. units::inches)
     * @returns Eigen::Vector2d with the same values as the vector.
     */
    constexpr Eigen::Vector2d as_vector(Q unit) const { return EVec<2>{x_.to(unit), y_.to(unit)}; }

    /// Gets the norm of the vector.
    constexpr Q norm() const { return units::hypot(x_, y_); }

    /**
     * Returns a vector of the same angle, but specific magnitude
     * (default 1 base unit)
     *
     * @param magnitude magnitude of the vector (default 1 base unit).
     * @returns The normalized vector.
     */
    constexpr LinearVector2d normalize(Q magnitude = Q(1.0)) const {
        return LinearVector2d(magnitude, theta());
    }

    /// Gets the distance between two vectors.
    constexpr Q distance(LinearVector2d other) const {
        return units::hypot(x_ - other.x_, y_ - other.y_);
    }

    /**
     * Interpolates along a straight line to another vector.
     * Fractions outside [0, 1] return the nearest endpoint.
     *
     * @param end endpoint
     * @param fraction percent of the way between the points
     * @return interpolated vector
     */
    constexpr LinearVector2d interpolate(LinearVector2d end, double fraction) const {
        if (fraction <= 0) {
            return *this;
        }
        if (fraction >= 1) {
            return end;
        }
        return *this + (end - *this) * fraction;
    }

    /**
     * Rotates this vector around the origin by the provided rotation.
     *
     * Equivalent to multiplying a vector by a rotation matrix:
     * x = [cos, -sin][x]
     * y = [sin,  cos][y]
     *
     * @param rotation the angle amount to rotate.
     * @returns The new vector that has been rotated around the origin.
     */
    constexpr LinearVector2d rotate_by(Rotation2d rotation) const {
        return {x_ * rotation.f_cos() - y_ * rotation.f_sin(),
                x_ * rotation.f_sin() + y_ * rotation.f_cos()};
    }

    /**
     * Applies a rotation to this vector around another given vector.
     *
     * [x] = [cos, -sin][x - otherx] + [otherx]
     * [y] = [sin,  cos][y - othery] + [othery]
     *
     * @param other the center of rotation.
     * @param rotation the angle amount the vector will be rotated.
     * @returns The vector that has been rotated.
     */
    constexpr LinearVector2d rotate_around(LinearVector2d other, Rotation2d rotation) const {
        LinearVector2d diff = *this - other;
        return diff.rotate_by(rotation) + other;
    }

    /**
     * Returns the inverse of this vector, or mirrors it across the origin.
     * [x] = [-x]
     * [y] = [-y]
     *
     * @returns The inverse of this vector.
     */
    constexpr LinearVector2d inverse() const { return LinearVector2d(-x_, -y_); }

    /**
     * Returns the dot product of two vectors.
     * The result has the product of the two component units.
     *
     * [scalar] = [x][otherx] + [y][othery]
     *
     * @param other the other vector to dot with.
     * @returns The dot product of this and other.
     */
    constexpr units::Multiplied<Q, Q> dot(LinearVector2d other) const {
        return (x_ * other.x_) + (y_ * other.y_);
    }

    /// Adds another vector to this vector component wise.
    constexpr LinearVector2d operator+(LinearVector2d other) const {
        return {x_ + other.x_, y_ + other.y_};
    }

    /// Adds another vector to this vector component wise.
    constexpr LinearVector2d& operator+=(LinearVector2d other) { return *this = *this + other; }

    /// Subtracts another vector from this vector component wise.
    constexpr LinearVector2d operator-(LinearVector2d other) const {
        return {x_ - other.x_, y_ - other.y_};
    }

    /// Subtracts another vector from this vector component wise.
    constexpr LinearVector2d& operator-=(LinearVector2d other) { return *this = *this - other; }

    /// Inverts this vector (flipped across origin).
    constexpr LinearVector2d operator-() const { return inverse(); }

    /// Multiplies this vector by a scalar.
    constexpr LinearVector2d operator*(double scalar) const { return {x_ * scalar, y_ * scalar}; }

    /// Multiplies a scalar by this vector.
    friend constexpr LinearVector2d operator*(double scalar, LinearVector2d vector) {
        return vector * scalar;
    }

    /**
     * Multiplies this vector by a scalar with a unit of time.
     * e.g. Velocity2d * Time = Translation2d
     */
    template <units::IsQuantity S>
        requires LinearKinematicQuantity<units::Multiplied<Q, S>>
    constexpr LinearVector2d<units::Multiplied<Q, S>> operator*(S scalar) const {
        return LinearVector2d<units::Multiplied<Q, S>>{x_ * scalar, y_ * scalar};
    }

    /**
     * Multiplies a scalar with a unit of time by this vector.
     * e.g. Time * Velocity2d = Translation2d
     */
    template <units::IsQuantity S>
        requires LinearKinematicQuantity<units::Multiplied<Q, S>>
    friend constexpr auto operator*(S scalar, LinearVector2d vector) {
        return vector * scalar;
    }

    /**
     * Returns the dot product of two vectors.
     * The result has the product of the two component units.
     *
     * [scalar] = [x][otherx] + [y][othery]
     *
     * @param other the other vector to dot with.
     * @returns The dot product of this and other.
     */
    constexpr units::Multiplied<Q, Q> operator*(LinearVector2d other) const { return dot(other); }

    /// Multiplies this vector by a scalar.
    constexpr LinearVector2d& operator*=(double scalar) { return *this = *this * scalar; }

    /// Divides this vector by a scalar.
    constexpr LinearVector2d operator/(double scalar) const { return {x_ / scalar, y_ / scalar}; }

    template <units::IsQuantity S>
        requires LinearKinematicQuantity<units::Divided<Q, S>>
    constexpr LinearVector2d<units::Divided<Q, S>> operator/(S scalar) const {
        return LinearVector2d<units::Divided<Q, S>>{x_ / scalar, y_ / scalar};
    }

    /// Divides this vector by a scalar.
    constexpr LinearVector2d& operator/=(double scalar) { return *this = *this / scalar; }

    /// Checks exact equality between this and another vector.
    constexpr bool operator==(LinearVector2d other) const {
        return cevalm::abs(x_.internal() - other.x_.internal()) < 1e-6 &&
               cevalm::abs(y_.internal() - other.y_.internal()) < 1e-6;
    }

    /**
     * Checks the distance between vectors against a tolerance.
     * Defaults to 1um for translations, 1um/s for velocities, etc.
     */
    constexpr bool is_near(LinearVector2d other, Q tolerance = Q(1e-6)) const {
        return distance(other) <= tolerance;
    }

    /**
     * Calculates the mean of a list of vectors.
     *
     * @param list std::vector containing a list of vectors.
     * @return the single vector mean of the list of vectors.
     */
    static constexpr LinearVector2d mean(const std::vector<LinearVector2d>& list) {
        if (list.size() == 0) {
            return LinearVector2d{};
        }

        Q sumx;
        Q sumy;

        for (LinearVector2d& vec : list) {
            sumx += vec.x_;
            sumy += vec.y_;
        }

        return LinearVector2d(sumx / list.size(), sumy / list.size());
    }
};

/// Translation2d is a 2d length vector
using Translation2d = LinearVector2d<units::Length>;
/// Velocity2d is a 2d velocity vector
using Velocity2d = LinearVector2d<units::Velocity>;
/// Acceleration2d is a 2d acceleration vector
using Acceleration2d = LinearVector2d<units::Acceleration>;
/// Jerk2d is a 2d jerk vector
using Jerk2d = LinearVector2d<units::Jerk>;
