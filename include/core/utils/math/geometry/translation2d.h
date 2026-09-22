#pragma once

#include <cmath>
#include <vector>

#include "cevalm.hpp"

#include "core/utils/math/eigen_interface.h"
#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/units.h"

class Rotation2d;

template <typename Q>
concept LinearKinematicQuantity =
    units::KinematicQuantity<Q> &&
    std::ratio_equal_v<typename Q::length, std::ratio<1>> &&
    std::ratio_equal_v<typename Q::angle, std::ratio<0>>;

/**
 * Class representing a 2d vector. Depending on unit this could be
 * a Translation2d, Velocity2d, Acceleration2d, or Jerk2d.
 *
 * Assumes conventional cartesian coordinate system:
 * Looking down at the coordinate plane,
 * +X is right
 * +Y is up
 * +Theta is counterclockwise
 */
template <LinearKinematicQuantity Q> struct LinearVector2d {
  /**
   * Default Constructor for LinearVector2d valued (0, 0)
   */
  constexpr LinearVector2d() : x_{0}, y_{0} {}

  /**
   * Constructs a vector with the given x and y values.
   * @param x The x component of the vector.
   * @param y The y component of the vector.
   */
  constexpr LinearVector2d(Q x, Q y) : x_{x}, y_{y} {}

  /**
   * Returns the x component.
   */
  constexpr Q x() const { return x_; }

  /**
   * Returns the y component.
   */
  constexpr Q y() const { return y_; }

  /**
   * Returns x in the supplied unit.
   */
  constexpr double x(Q unit) const { return x_.to(unit); }

  /**
   * Returns y in the supplied unit.
   */
  constexpr double y(Q unit) const { return y_.to(unit); }

  /**
   * Constructs a vector with the values from the given Eigen::Vector.
   * @param vector The vector whose values will be used.
   * @param unit The unit to use when assigning x and y (e.g. units::inches)
   */
  constexpr LinearVector2d(Eigen::Vector2d vector, Q unit)
      : x_{vector[0], unit}, y_{vector[1], unit} {}

  /**
   * Constructs a vector given polar coordinates of the form (r, theta).
   * @param r The magnitude of the vector.
   * @param theta The angle of the vector.
   */
  constexpr LinearVector2d(Q r, Rotation2d theta)
      : x_{r * theta.f_cos()}, y_{r * theta.f_sin()} {}

  /**
   * Returns the angle of the vector.
   * @returns The angle of the vector.
   */
  constexpr Rotation2d theta() const {
    return Rotation2d(x_.internal(), y_.internal());
  }

  /**
   * Returns the vector as an Eigen::Vector2d.
   *
   * @param unit The unit to get the x and y components as (e.g. units::inches)
   *
   * @returns Eigen::Vector2d with the same values as the vector.
   */
  constexpr Eigen::Vector2d as_vector(Q unit) const {
    return EVec<2>{x_.to(unit), y_.to(unit)};
  }

  /**
   * Returns the norm/magnitude of the vector.
   *
   * @returns the norm of the vector.
   */
  constexpr Q norm() const { return units::hypot(x_, y_); }

  /**
   * Returns a vector of the same angle, but specific magnitude
   * (default 1 base unit)
   *
   * @returns The normalized vector.
   */
  constexpr LinearVector2d normalize(Q magnitude = Q(1.0)) const {
    return LinearVector2d(magnitude, theta());
  }

  /**
   * Returns the distance between two vectors.
   * (mostly useful for Translation2d)
   *
   * @returns The distance between two vectors.
   */
  constexpr Q distance(LinearVector2d other) const {
    return units::hypot(x_ - other.x_, y_ - other.y_);
  }

  /**
   * Interpolates along a straight line to another vector.
   * Fractions outside [0, 1] return the nearest endpoint.
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
  constexpr LinearVector2d rotate_around(LinearVector2d other,
                                         Rotation2d rotation) const {
    LinearVector2d diff = *this - other;
    return diff.rotate_by(rotation) + other;
  }

  /**
   * Returns the sum of two vectors.
   *
   * [x] = [x] + [otherx];
   * [y] = [y] + [othery];
   *
   * @param other the other vector to add to this.
   * @returns The sum of the two vectors.
   */
  constexpr LinearVector2d operator+(LinearVector2d other) const {
    return {x_ + other.x_, y_ + other.y_};
  }

  /**
   * Returns the difference of two vectors.
   *
   * [x] = [x] - [otherx]
   * [y] = [y] - [othery]
   *
   * @param other the vector to subtract from this.
   * @returns The difference of the two vectors.
   */
  constexpr LinearVector2d operator-(LinearVector2d other) const {
    return {x_ - other.x_, y_ - other.y_};
  }

  /**
   * Adds another vector to this vector.
   */
  constexpr LinearVector2d &operator+=(LinearVector2d other) {
    return *this = *this + other;
  }

  /**
   * Subtracts another vector from this vector.
   */
  constexpr LinearVector2d &operator-=(LinearVector2d other) {
    return *this = *this - other;
  }

  /**
   * Scales this vector without changing its units.
   */
  constexpr LinearVector2d &operator*=(double scalar) {
    return *this = *this * scalar;
  }

  /**
   * Divides this vector without changing its units.
   */
  constexpr LinearVector2d &operator/=(double scalar) {
    return *this = *this / scalar;
  }

  /**
   * Multiplies a scalar by this vector.
   */
  friend constexpr LinearVector2d operator*(double scalar, LinearVector2d vector) {
    return vector * scalar;
  }

  /**
   * Multiplies a quantity by this vector, keeping the resulting units.
   */
  template <units::IsQuantity S>
    requires LinearKinematicQuantity<units::Multiplied<Q, S>>
  friend constexpr auto operator*(S scalar, LinearVector2d vector) {
    return vector * scalar;
  }

  /**
   * Returns the inverse of this vector.
   * Equivalent to flipping the vector across the origin.
   *
   * [x] = [-x]
   * [y] = [-y]
   *
   * @returns The inverse of this vector.
   */
  constexpr LinearVector2d operator-() const { return {-x_, -y_}; }

  /**
   * Returns this vector multiplied by a scalar.
   *
   * [x] = [x] * [scalar]
   * [y] = [y] * [scalar]
   *
   * @param scalar the scalar to multiply by.
   * @returns This vector multiplied by a scalar.
   */
  constexpr LinearVector2d operator*(double scalar) const {
    return {x_ * scalar, y_ * scalar};
  }

  /**
   * Returns this vector multiplied by a scalar. The unit result must be a LinearKinematicQuantity
   *
   *
   */
  template <units::IsQuantity S>
    requires LinearKinematicQuantity<units::Multiplied<Q, S>>
  constexpr LinearVector2d<units::Multiplied<Q, S>> operator*(S scalar) const {
    return LinearVector2d<units::Multiplied<Q, S>>{x_ * scalar, y_ * scalar};
  }

  /**
   * Returns this vector divided by a scalar.
   *
   * [x] = [x] / [scalar]
   * [y] = [y] / [scalar]
   *
   * @param scalar the scalar to divide by.
   * @returns This vector divided by a scalar.
   */
  constexpr LinearVector2d operator/(double scalar) const {
    return {x_ / scalar, y_ / scalar};
  }

  template <units::IsQuantity S>
    requires LinearKinematicQuantity<units::Divided<Q, S>>
  constexpr LinearVector2d<units::Divided<Q, S>> operator/(S scalar) const {
    return LinearVector2d<units::Divided<Q, S>>{x_ / scalar, y_ / scalar};
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
  template <LinearKinematicQuantity R>
  constexpr units::Multiplied<Q, R>
  operator*(LinearVector2d<R> other) const {
    return dot(other);
  }

  /**
   * Returns the dot product, keeping the resulting units.
   */
  template <LinearKinematicQuantity R>
  constexpr units::Multiplied<Q, R> dot(LinearVector2d<R> other) const {
    return (x_ * other.x_) + (y_ * other.y_);
  }

  /**
   * Checks the distance between vectors against a tolerance.
   * Defaults to 1um for translations, 1um/s for velocities, and so on.
   */
  constexpr bool is_near(LinearVector2d other, Q tolerance = Q(1e-6)) const {
    return distance(other) <= tolerance;
  }

  /**
   * Compares two vectors.
   * Returns true if their components are each within 1e-6, to account for
   * floating point error. This uses the internal base unit, which for length
   * based units is meters, so 1um epsilon, or 1um/s, etc.
   *
   * @param other the vector to compare to.
   * @returns Whether the two vectors are equal.
   */
  constexpr bool operator==(LinearVector2d other) const {
    return cevalm::abs(x_.internal() - other.x_.internal()) < 1e-6 &&
           cevalm::abs(y_.internal() - other.y_.internal()) < 1e-6;
  }

  /**
   * Calculates the mean of a list of vectors.
   *
   * @param list std::vector containing a list of vectors.
   *
   * @return the single vector mean of the list of vectors.
   */
  static constexpr LinearVector2d
  mean(const std::vector<LinearVector2d> &list) {
    if (list.size() == 0) {
      return LinearVector2d{};
    }

    Q sumx;
    Q sumy;

    for (int i = 0; i < list.size(); i++) {
      sumx += list.at(i).x_;
      sumy += list.at(i).y_;
    }

    return LinearVector2d(sumx / list.size(), sumy / list.size());
  }

  Q x_;
  Q y_;
};

using Translation2d = LinearVector2d<units::Length>;
using Velocity2d = LinearVector2d<units::Velocity>;
using Acceleration2d = LinearVector2d<units::Acceleration>;
using Jerk2d = LinearVector2d<units::Jerk>;
