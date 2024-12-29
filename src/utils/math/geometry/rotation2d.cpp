#include <cmath>
#include <iostream>
#include <vector>

#include "../core/include/utils/math/geometry/rotation2d.h"
#include "../core/include/utils/math/geometry/translation2d.h"

/**
 * Constructs a rotation with the given value in radians.
 *
 * @param radians the value of the rotation in radians.
 */
Rotation2d::Rotation2d(const double &radians)
    : m_radians{radians}, m_cos{std::cos(radians)}, m_sin{std::sin(radians)} {}

/**
 * Constructs a rotation given x and y values.
 * Does not have to be normalized.
 * The angle from the x axis to the point.
 *
 * [theta] = [atan2(y, x)]
 *
 * @param x the x value of the point
 * @param y the y value of the point
 */
Rotation2d::Rotation2d(const double &x, const double &y)
    : m_radians{std::atan2(y, x)}, m_cos{std::cos(m_radians)}, m_sin{std::sin(m_radians)} {}

/**
 * Constructs a rotation given x and y values in the form of a Translation2d.
 * Does not have to be normalized.
 * The angle from the x axis to the point.
 *
 * [theta] = [atan2(y, x)]
 *
 * @param translation
 */
Rotation2d::Rotation2d(const Translation2d &translation)
    : m_radians{std::atan2(translation.y(), translation.x())}, m_cos{std::cos(m_radians)}, m_sin{std::sin(m_radians)} {}

/**
 * Constructs a rotation given radian angle value.
 *
 * @param radians angle in radians.
 */
Rotation2d from_radians(const double &radians) { return Rotation2d(radians); }

/**
 * Constructs a rotation given degree angle value.
 *
 * @param degrees angle in degrees.
 */
Rotation2d from_degrees(const double &degrees) { return Rotation2d((degrees / 180) * M_PI); }

/**
 * Constructs a rotation given revolution angle value.
 *
 * @param revolutions angle in revolutions.
 */
Rotation2d from_revolutions(const double &revolutions) { return Rotation2d(revolutions * M_TWOPI); }

/**
 * Returns the radian angle value.
 *
 * @return the radian angle value.
 */
double Rotation2d::radians() const { return m_radians; }

/**
 * Returns the degree angle value.
 *
 * @return the degree angle value.
 */
double Rotation2d::degrees() const { return 360 * (m_radians / M_TWOPI); }

/**
 * Returns the revolution angle value.
 *
 * @return the revolution angle value.
 */
double Rotation2d::revolutions() const { return (m_radians / M_TWOPI); }

/**
 * Returns the cosine of the angle value.
 *
 * @return the cosine of the angle value
 */
double Rotation2d::f_cos() const { return m_cos; }

/**
 * Returns the sine of the angle value.
 *
 * @return the sine of the angle value.
 */
double Rotation2d::f_sin() const { return m_sin; }

/**
 * Returns the tangent of the angle value.
 *
 * @return the tangent of the angle value.
 */
double Rotation2d::f_tan() const { return (m_sin / m_cos); }

/**
 * Returns the rotation matrix equivalent to this rotation
 *     [cos, -sin]
 * R = [sin,  cos]
 *
 * @return the rotation matrix equivalent to this rotation
 */
Eigen::Matrix2d Rotation2d::rotation_matrix() const { return Eigen::Matrix2d{{m_cos, -m_sin}, {m_sin, m_cos}}; }

/**
 * Returns the radian angle value, wrapped from [-pi, pi).
 *
 * @return the radian angle value, wrapped from [-pi, pi)
 */
double Rotation2d::wrapped_radians_180() const { return wrap_radians_180(m_radians); }

/**
 * Returns the degree angle value, wrapped from [-180, 180).
 *
 * @return the degree angle value, wrapped from [-180, 180)
 */
double Rotation2d::wrapped_degrees_180() const { return wrap_radians_180(m_radians) * (180. / M_PI); }

/**
 * Returns the revolution angle value, wrapped from [-0.5, 0.5).
 *
 * @return the revolution angle value, wrapped from [-0.5, 0.5)
 */
double Rotation2d::wrapped_revolutions_180() const { return wrap_radians_180(m_radians) / M_TWOPI; }

/**
 * Returns the radian angle value, wrapped from [0, 2pi).
 *
 * @return the radian angle value, wrapped from [0, 2pi)
 */
double Rotation2d::wrapped_radians_360() const { return wrap_radians_360(m_radians); }

/**
 * Returns the degree angle value, wrapped from [0, 360).
 *
 * @return the degree angle value, wrapped from [0, 360)
 */
double Rotation2d::wrapped_degrees_360() const { return wrap_radians_360(m_radians) * (180. / M_PI); }

/**
 * Returns the revolution angle value, wrapped from [0, 1).
 *
 * @return the revolution angle value, wrapped from [0, 1)
 */
double Rotation2d::wrapped_revolutions_360() const { return wrap_radians_360(m_radians) / M_TWOPI; }

/**
 * Adds the values of two rotations using a rotation matrix.
 *
 * [new_cos] = [other.cos, -other.sin][cos]
 * [new_sin] = [other.sin,  other.cos][sin]
 * new_value = atan2(new_sin, new_cos)
 *
 * @param other the other rotation to add to this rotation.
 *
 * @return the sum of the two rotations.
 */
Rotation2d Rotation2d::operator+(const Rotation2d &other) const {
  return {f_cos() * other.f_cos() - f_sin() * other.f_sin(), f_cos() * other.f_sin() + f_sin() * other.f_cos()};
}

/**
 * Subtracts the values of two rotations.
 *
 * @param other the other rotation to subtract from this rotation.
 *
 * @return the difference between the two rotations.
 */
Rotation2d Rotation2d::operator-(const Rotation2d &other) const { return *this + -other; }

/**
 * Takes the inverse of this rotation by flipping it.
 *
 * @return this inverse of the rotation.
 */
Rotation2d Rotation2d::operator-() const { return Rotation2d(-m_radians); }

/**
 * Multiplies this rotation by a scalar.
 *
 * @param scalar the scalar value to multiply the rotation by.
 *
 * @return the rotation multiplied by the scalar.
 */
Rotation2d Rotation2d::operator*(const double &scalar) const { return Rotation2d(m_radians * scalar); }

/**
 * Divides this rotation by a scalar.
 *
 * @param scalar the scalar value to divide the rotation by.
 *
 * @return the rotation divided by the scalar.
 */
Rotation2d Rotation2d::operator/(const double &scalar) const { return *this * (1. / scalar); }

/**
 * Compares two rotations.
 * Returns true if their values are within 1e-9 radians of each other, to account for floating point error.
 *
 * @param other the other rotation to compare to
 *
 * @return whether the values of the rotations are within 1e-9 radians of each other
 */
bool Rotation2d::operator==(const Rotation2d &other) const {
  return std::abs(this->radians() - other.radians()) < 1e-9;
}

/**
 * Sends a rotation to an output stream.
 * Ex.
 * std::cout << rotation;
 *
 * prints "Rotation2d[rad: (radians), deg: (degrees)]"
 */
std::ostream &operator<<(std::ostream &os, const Rotation2d &rotation) {
  os << "Rotation2d[rad: " << rotation.radians() << ", deg: " << rotation.degrees() << "]";
  return os;
}

// functions that don't belong in the class because they're useful elsewhere

/**
 * Wraps a radian angle value from [-pi, pi).
 *
 * @param angle the radian angle value to wrap.
 *
 * @return the wrapped radian angle value from [-pi, pi).
 */
double wrap_radians_180(const double &angle) {
  double x = fmod(angle + M_PI, M_TWOPI);
  if (x < 0) {
    x += M_TWOPI;
  }
  return x - M_PI;
}

/**
 * Wraps a degree angle value from [-180, 180).
 *
 * @param angle the degree angle value to wrap.
 *
 * @return the wrapped degree angle value from [-180, 180).
 */
double wrap_degrees_180(const double &angle) {
  double x = fmod(angle + 180, 360);
  if (x < 0) {
    x += 360;
  }
  return x - 180;
}

/**
 * Wraps a revolution angle vlue from [-0.5, 0.5).
 *
 * @param angle the revolution angle value to wrap.
 *
 * @return the wrapped revolution angle vlue from [-0.5, 0.5).
 */
double wrap_revolutions_180(const double &angle) {
  double x = fmod(angle + 0.5, 1);
  if (x < 0) {
    x += 1;
  }
  return x - 0.5;
}

/**
 * Wraps a radian angle value from [0, 2pi).
 *
 * @param angle the radian angle value to wrap.
 *
 * @return the wrapped radian angle value from [0, 2pi).
 */
double wrap_radians_360(const double &angle) {
  double x = fmod(angle, M_TWOPI);
  if (x < 0) {
    x += M_TWOPI;
  }
  return x;
}

/**
 * Wraps a degree angle value from [0, 360).
 *
 * @param angle the degree angle value to wrap.
 *
 * @return the wrapped degree angle value from [0, 360).
 */
double wrap_degrees_360(const double &angle) {
  double x = fmod(angle, 360);
  if (x < 0) {
    x += 360;
  }
  return x;
}

/**
 * Wraps a revolution angle value from [0, 1).
 *
 * @param angle the revolution angle value to wrap.
 *
 * @return the wrapped revolution angle value from [0, 1).
 */
double wrap_revolutions_360(const double &angle) {
  double x = fmod(angle, 1);
  if (x < 0) {
    x += 1;
  }
  return x;
}

/**
 * Calculates the mean of a list of angle values directly.
 * !! DOES NOT WRAP INPUTS (probably not useful) !!
 *
 * @param list std::vector containing a list of rotations.
 *
 * @return the single rotation mean of the list of rotations.
 */
Rotation2d unwrapped_mean(const std::vector<Rotation2d> &list) {
  double sum_rads = 0;

  for (int i = 0; i < list.size(); i++) {
    sum_rads += list.at(i).radians();
  }

  return Rotation2d(sum_rads / list.size());
}

/**
 * Calculates the mean of a list of angle values directly.
 * !! WRAPS INPUTS !!
 *
 * @param list std::vector containing a list of rotations.
 *
 * @return the single rotation mean of the list of rotations.
 */
Rotation2d wrapped_mean(const std::vector<Rotation2d> &list) {
  double sum_sin = 0;
  double sum_cos = 0;

  for (int i = 0; i < list.size(); i++) {
    sum_sin += list.at(i).f_sin();
    sum_cos += list.at(i).f_cos();
  }

  return Rotation2d(sum_cos, sum_sin);
}
