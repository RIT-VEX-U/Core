#pragma once

#include <cmath>

#include "cevalm.hpp"

#include "core/utils/math/eigen_interface.h"
#include "core/utils/units.h"

/**
 * Class representing a rotation in 2d space.
 * Stores cos and sin, and computes angles based on that.
 *
 * By nature the stored angle is wrapped, but there are functions
 * that return angles wrapped specifically between e.g. [-180, 180) and [0, 360)
 */
class Rotation2d {
public:
  /**
   * Default Constructor for Rotation2d
   */
  constexpr Rotation2d() = default;

  /**
   * Constructs a rotation with the given units::Angle.
   */
  constexpr Rotation2d(units::Angle value)
      : cos_(units::cos(value)), sin_(units::sin(value)) {}

  /**
   * Constructs a rotation with the given angle in radians.
   */
  constexpr Rotation2d(double value)
      : cos_(cevalm::cos(value)), sin_(cevalm::sin(value)) {}

  /**
   * Constructs a rotation given x and y values, as the angle from the x axis to
   * the point.
   *
   * [theta] = [atan2(y, x)]
   *
   * @param x The x value of the point.
   * @param y The y value of the point.
   */
  constexpr Rotation2d(double x, double y) {
    double mag = cevalm::hypot(x, y);
    if (x != 0 || y != 0) {
      cos_ = x / mag;
      sin_ = y / mag;
    } else {
      cos_ = 1;
      sin_ = 0;
    }
  }

  /**
   * Constructs a rotation given united x and y values, as the angle from the x
   * axis to the point.
   *
   * [theta] = [atan2(y, x)]
   *
   * @param x The x value of the point.
   * @param y The y value of the point.
   */
  constexpr Rotation2d(units::Length x, units::Length y)
      : Rotation2d(x.to(units::inches), y.to(units::inches)) {}

  /**
   * Adds another rotation to this using the rotation matrix.
   *
   * [new_cos] = [other.cos, -other.sin][cos]
   * [new_sin] = [other.sin,  other.cos][sin]
   *
   */
  constexpr Rotation2d operator+(Rotation2d other) const {
    return Rotation2d(cos_ * other.cos_ - sin_ * other.sin_,
                      cos_ * other.sin_ + sin_ * other.cos_);
  }

  /**
   * Adds another rotation to this rotation.
   */
  constexpr Rotation2d &operator+=(Rotation2d other) {
    return *this = *this + other;
  }

  /**
   * Subtracts another rotation from this. Also represents getting this rotation
   * relative to other.
   */
  constexpr Rotation2d operator-(Rotation2d other) const {
    return *this + -other;
  }

  /**
   * Subtracts another rotation from this rotation.
   */
  constexpr Rotation2d &operator-=(Rotation2d other) {
    return *this = *this - other;
  }

  /**
   * Takes the inverse (conjugate) of this rotation.
   */
  constexpr Rotation2d operator-() const { return Rotation2d(cos_, -sin_); }

  /**
   * Multiplies this rotation by a scalar.
   */
  constexpr Rotation2d operator*(double scalar) const {
    return Rotation2d(radians() * scalar);
  }

  /**
   * Multiplies a scalar by this rotation.
   */
  friend constexpr Rotation2d operator*(double scalar, Rotation2d rotation) {
    return rotation * scalar;
  }

  /**
   * Scales this rotation's principal angle.
   */
  constexpr Rotation2d &operator*=(double scalar) {
    return *this = *this * scalar;
  }

  /**
   * Divides this rotation by a scalar.
   */
  constexpr Rotation2d operator/(double scalar) const {
    return *this * (1.0 / scalar);
  }

  /**
   * Divides this rotation's principal angle.
   */
  constexpr Rotation2d &operator/=(double scalar) {
    return *this = *this / scalar;
  }

  /**
   * Checks whether this and another rotation are equal.
   *
   * @param other The other rotation to compare to.
   * @return true if the sin and cos values are both within 1e-6.
   */
  constexpr bool operator==(Rotation2d other) const {
    return cevalm::abs(cos_ - other.cos_) < 1e-6 &&
           cevalm::abs(sin_ - other.sin_) < 1e-6;
  }

  /**
   * Returns the rotation that undoes this rotation, the same as unary minus.
   */
  constexpr Rotation2d inverse() const { return -*this; }

  /**
   * Returns the heading facing the opposite direction, a half turn away.
   */
  constexpr Rotation2d opposite() const { return Rotation2d(-cos_, -sin_); }

  /**
   * Checks the smallest angle between rotations against a tolerance.
   * Defaults to 1e-6 radians.
   */
  constexpr bool is_near(Rotation2d other,
                         units::Angle tolerance = units::Angle(1e-6)) const {
    return units::abs((*this - other).angle()) <= tolerance;
  }

  /**
   * Returns a rotation matrix equivalent to this rotation.
   *
   * @return The 2x2 rotation matrix.
   */
  constexpr EMat<2, 2> rotation_matrix() const {
    return EMat<2, 2>{{cos_, -sin_}, {sin_, cos_}};
  }

  /**
   * Returns a united Angle corresponding to this rotation.
   *
   * @return The rotation as a units::Angle.
   */
  constexpr units::Angle angle() const {
    return units::Angle(cevalm::atan2(sin_, cos_), units::radians);
  }

  /**
   * Returns the angle in the supplied unit.
   */
  constexpr double angle(units::Angle unit) const { return angle().to(unit); }

  /**
   * Returns the rotation in radians.
   */
  constexpr double radians() const { return angle().to(units::radians); }

  /**
   * Returns the rotation in degrees.
   */
  constexpr double degrees() const { return angle().to(units::degrees); }

  /**
   * Returns the rotation in revolutions.
   */
  constexpr double revolutions() const {
    return angle().to(units::revolutions);
  }

  /**
   * Returns the rotation in gradians.
   */
  constexpr double gradians() const { return angle().to(units::gradians); }

  /**
   * Returns the cosine of the rotation.
   */
  constexpr double f_cos() const { return cos_; }

  /**
   * Returns the sine of the rotation.
   */
  constexpr double f_sin() const { return sin_; }

  /**
   * Returns the tangent of the rotation.
   */
  constexpr double f_tan() const { return sin_ / cos_; }

  /**
   * Helper function that converts degrees to radians.
   *
   * @param deg angle degrees
   * @return double angle radians.
   */
  static constexpr double deg2rad(double deg) {
    return deg * (std::numbers::pi / 180.0);
  }

  /**
   * Helper function that converts radians to degrees.
   *
   * @param deg angle in radians
   * @return double angle in degrees.
   */
  static constexpr double rad2deg(double rad) {
    return rad * (180.0 / std::numbers::pi);
  }

  /**
   * Helper function that wraps an angle in degrees from [-180, 180].
   *
   * @param angle The angle to wrap.
   * @return The wrapped angle.
   */
  static constexpr double wrap_degrees_180(double angle) {
    if (angle >= -180.0 && angle <= 180.0) {
      return angle;
    }
    double x = cevalm::fmod(angle, 360.0);
    if (x > 180.0) {
      x -= 360.0;
    } else if (x < -180.0) {
      x += 360.0;
    }
    return x;
  }

  /**
   * Helper function that wraps an angle in revolutions from [-0.5, 0.5].
   *
   * @param angle The angle to wrap.
   * @return The wrapped angle.
   */
  static constexpr double wrap_revolutions_180(double angle) {
    if (angle >= -0.5 && angle <= 0.5) {
      return angle;
    }
    double x = cevalm::fmod(angle, 1.0);
    if (x > 0.5) {
      x -= 1.0;
    } else if (x < -0.5) {
      x += 1.0;
    }
    return x;
  }

  /**
   * Helper function that wraps an angle in gradians from [-200, 200].
   *
   * @param angle The angle to wrap.
   * @return The wrapped angle.
   */
  static constexpr double wrap_gradians_180(double angle) {
    if (angle >= -200.0 && angle <= 200.0) {
      return angle;
    }
    double x = cevalm::fmod(angle, 400.0);
    if (x > 200.0) {
      x -= 400.0;
    } else if (x < -200.0) {
      x += 400.0;
    }
    return x;
  }

  /**
   * Helper function that wraps an angle in radians from [-pi, pi].
   *
   * @param angle The angle to wrap.
   * @return The wrapped angle.
   */
  static constexpr double wrap_radians_180(double angle) {
    if (angle >= -std::numbers::pi && angle <= std::numbers::pi) {
      return angle;
    }
    double x = cevalm::fmod(angle, 2 * std::numbers::pi);
    if (x > std::numbers::pi) {
      x -= 2 * std::numbers::pi;
    } else if (x < -std::numbers::pi) {
      x += 2 * std::numbers::pi;
    }
    return x;
  }

  /**
   * Helper function that wraps an angle in degrees from [0, 360).
   *
   * @param angle The angle to wrap.
   * @return The wrapped angle.
   */
  static constexpr double wrap_degrees_360(double angle) {
    if (angle >= 0.0 && angle < 360.0) {
      return angle;
    }
    double x = cevalm::fmod(angle, 360.0);
    if (x < 0.0) {
      x += 360.0;
    }
    return (x >= 360.0) ? 0.0 : x;
  }

  /**
   * Helper function that wraps an angle in revolutions from [0, 1).
   *
   * @param angle The angle to wrap.
   * @return The wrapped angle.
   */
  static constexpr double wrap_revolutions_360(double angle) {
    if (angle >= 0.0 && angle < 1.0) {
      return angle;
    }
    double x = cevalm::fmod(angle, 1.0);
    if (x < 0.0) {
      x += 1.0;
    }
    return (x >= 1.0) ? 0.0 : x;
  }

  /**
   * Helper function that wraps an angle in gradians from [0, 400).
   *
   * @param angle The angle to wrap.
   * @return The wrapped angle.
   */
  static constexpr double wrap_gradians_360(double angle) {
    if (angle >= 0.0 && angle < 400.0) {
      return angle;
    }
    double x = cevalm::fmod(angle, 400.0);
    if (x < 0.0) {
      x += 400.0;
    }
    return (x >= 400.0) ? 0.0 : x;
  }

  /**
   * Helper function that wraps an angle in radians from [0, 2pi).
   *
   * @param angle The angle to wrap.
   * @return The wrapped angle.
   */
  static constexpr double wrap_radians_360(double angle) {
    if (angle >= 0.0 && angle < 2 * std::numbers::pi) {
      return angle;
    }
    double x = cevalm::fmod(angle, 2 * std::numbers::pi);
    if (x < 0.0) {
      x += 2 * std::numbers::pi;
    }
    return (x >= 2 * std::numbers::pi) ? 0.0 : x;
  }

  /**
   * Returns the value of this rotation in radians from [-pi, pi].
   */
  constexpr double wrapped_radians_180() const {
    return wrap_radians_180(this->radians());
  }

  /**
   * Returns the value of this rotation in degrees from [-180, 180].
   */
  constexpr double wrapped_degrees_180() const {
    return wrap_degrees_180(this->degrees());
  }

  /**
   * Returns the value of this rotation in revolutions from [-0.5, 0.5].
   */
  constexpr double wrapped_revolutions_180() const {
    return wrap_revolutions_180(this->revolutions());
  }

  /**
   * Returns the value of this rotation in gradians from [-200, 200].
   */
  constexpr double wrapped_gradians_180() const {
    return wrap_gradians_180(this->gradians());
  }

  /**
   * Returns the value of this rotation in radians from [0, 2pi).
   */
  constexpr double wrapped_radians_360() const {
    return wrap_radians_360(this->radians());
  }

  /**
   * Returns the value of this rotation in degrees from [0, 360).
   */
  constexpr double wrapped_degrees_360() const {
    return wrap_degrees_360(this->degrees());
  }

  /**
   * Returns the value of this rotation in revolutions from [0, 1).
   */
  constexpr double wrapped_revolutions_360() const {
    return wrap_revolutions_360(this->revolutions());
  }

  /**
   * Returns the value of this rotation in gradians from [0, 400).
   */
  constexpr double wrapped_gradians_360() const {
    return wrap_gradians_360(this->gradians());
  }

private:
  double cos_ = 1;
  double sin_ = 0;
};
