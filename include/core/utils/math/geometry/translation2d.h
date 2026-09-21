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
   * Constructs a vector with the values from the given Eigen::Vector.
   * @param vector The vector whose values will be used.
   * @param unit The unit to use when assigning x and y (e.g. units::inches)
   */
  constexpr LinearVector2d(const Eigen::Vector2d &vector, Q unit)
      : x_{vector[0], unit}, y_{vector[1], unit} {}

  /**
   * Constructs a vector given polar coordinates of the form (r, theta).
   * @param r The magnitude of the vector.
   * @param theta The angle of the vector.
   */
  constexpr LinearVector2d(Q r, Rotation2d theta)
      : x_{r * theta.f_cos()}, y_{r * theta.f_sin()} {}

  /**
   * Returns the angle of the translation.
   * @returns The angle of the translation.
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
   * @returns The normalized translation.
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
  constexpr LinearVector2d operator+(const LinearVector2d &other) const {
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
   *
   * Returns this vector multiplied by a scalar. The unit result must be a LinearKinematicQuantity
   *
   *
   */
  template <units::IsQuantity S>
    requires LinearKinematicQuantity<units::Multiplied<Q, S>>
  constexpr units::Multiplied<Q, S> operator*(S scalar) const {
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
  constexpr LinearVector2d operator/(const double &scalar) const {
    return {x_ / scalar, y_ / scalar};
  }

  template <units::IsQuantity S>
    requires LinearKinematicQuantity<units::Divided<Q, S>>
  constexpr units::Divided<Q, S> operator/(S scalar) const {
    return LinearVector2d<units::Divided<Q, S>>{x_ / scalar, y_ / scalar};
  }

  /**
   * Returns the dot product of two vectors.
   * Note that the unit for this is Q*Q, not double.
   *
   * [scalar] = [x][otherx] + [y][othery]
   *
   * @param other the other translation dot with.
   * @returns The dot product of this and other.
   */
  constexpr units::Multiplied<Q, Q>
  operator*(const LinearVector2d &other) const {
    return (x_ * other.x_) + (y_ * other.y_);
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
  constexpr bool operator==(const LinearVector2d &other) const {
    return cevalm::abs(x_.internal() - other.x_.internal()) < 1e-6 &&
           cevalm::abs(y_.internal() - other.y_.internal()) < 1e-6;
  }

  /**
   * Calculates the mean of a list of vectors.
   *
   * @param list std::vector containing a list of vectors.
   *
   * @return the single vector mean of the list of translation.
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
