#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <vector>

#include "../core/include/utils/math/geometry/rotation2d.h"
#include "../core/include/utils/math/geometry/translation2d.h"

/**
 * Constructs a Translation2d with the given x and y values.
 *
 * @param x The x component of the translation.
 * @param y The y component of the translation.
 */
Translation2d::Translation2d(const double &x, const double &y) : m_x{x}, m_y{y} {}

/**
 * Constructs a Translation2d with the values from the given vector.
 *
 * @param vector The vector whose values will be used.
 */
Translation2d::Translation2d(const Eigen::Vector2d &vector) : m_x{vector(0)}, m_y{vector(1)} {}

/**
 * Constructs a Translation2d given polar coordinates of the form (r, theta).
 *
 * @param r The radius (magnitude) of the vector.
 * @param theta The angle (direction) of the vector.
 */
Translation2d::Translation2d(const double &r, const Rotation2d &theta)
    : m_x{r * theta.f_cos()}, m_y{r * theta.f_sin()} {}

/**
 * Returns the x value of the translation.
 *
 * @return the x value of the translation.
 */
double Translation2d::x() const { return m_x; }

/**
 * Returns the y value of the translation.
 *
 * @return the y value of the translation.
 */
double Translation2d::y() const { return m_y; }

/**
 * Returns the angle of the translation.
 *
 * @return the angle of the translation.
 */
Rotation2d Translation2d::theta() const { return Rotation2d(m_x, m_y); }

/**
 * Returns the vector as an Eigen::Vector2d.
 *
 * @return Eigen::Vector2d with the same values as the translation.
 */
Eigen::Vector2d Translation2d::as_vector() const { return Eigen::Vector2d(m_x, m_y); }

/**
 * Returns the norm/radius/magnitude/distance from origin.
 *
 * @return the norm of the translation.
 */
double Translation2d::norm() const { return std::hypot(m_x, m_y); }

/**
 * Returns the distance between two translations.
 *
 * @return the distance between two translations.
 */
double Translation2d::distance(const Translation2d &other) const {
  return std::hypot(m_x - other.x(), m_y - other.y());
}

/**
 * Applies a rotation to this translation around the origin.
 *
 * Equivalent to multiplying a vector by a rotation matrix:
 * x = [cos, -sin][x]
 * y = [sin,  cos][y]
 *
 * @param rotation the angle amount the translation will be rotated.
 *
 * @return the new translation that has been rotated around the origin.
 */
Translation2d Translation2d::rotate_by(const Rotation2d &rotation) const {
  return {m_x * rotation.f_cos() - m_y * rotation.f_sin(), m_x * rotation.f_sin() + m_y * rotation.f_cos()};
}

/**
 * Applies a rotation to this translation around another given point.
 *
 * [x] = [cos, -sin][x - otherx] + [otherx]
 * [y] = [sin,  cos][y - othery] + [othery]
 *
 * @param other the center of rotation.
 * @param rotation the angle amount the translation will be rotated.
 *
 * @return the translation that has been rotated.
 */
Translation2d Translation2d::rotate_around(const Translation2d &other, const Rotation2d &rotation) const {
  Translation2d diff = *this - other;
  return diff.rotate_by(rotation) + other;
}

/**
 * Returns the sum of two translations.
 *
 * [x] = [x] + [otherx];
 * [y] = [y] + [othery];
 *
 * @param other the other translation to add to this translation.
 *
 * @return the sum of the two translations.
 */
Translation2d Translation2d::operator+(const Translation2d &other) const { return {m_x + other.x(), m_y + other.y()}; }

/**
 * Returns the difference of two translations.
 *
 * [x] = [x] - [otherx]
 * [y] = [y] - [othery]
 *
 * @param other the translation to subtract from this translation.
 *
 * @return the difference of the two translations.
 */
Translation2d Translation2d::operator-(const Translation2d &other) const { return {m_x - other.x(), m_y - other.y()}; }

/**
 * Returns the inverse of this translation.
 * Equivalent to flipping the vector across the origin.
 *
 * [x] = [-x]
 * [y] = [-y]
 *
 * @return the inverse of this translation.
 */
Translation2d Translation2d::operator-() const { return {-m_x, -m_y}; }

/**
 * Returns this translation multiplied by a scalar.
 *
 * [x] = [x] * [scalar]
 * [y] = [y] * [scalar]
 *
 * @param scalar the scalar to multiply by.
 *
 * @return this translation multiplied by a scalar.
 */
Translation2d Translation2d::operator*(const double &scalar) const { return {m_x * scalar, m_y * scalar}; }

/**
 * Returns this translation divided by a scalar.
 *
 * [x] = [x] / [scalar]
 * [y] = [y] / [scalar]
 *
 * @param scalar the scalar to divide by.
 *
 * @return this translation divided by a scalar.
 */
Translation2d Translation2d::operator/(const double &scalar) const { return {m_x / scalar, m_y / scalar}; }

/**
 * Returns the dot product of two translations.
 *
 * [scalar] = [x][otherx] + [y][othery]
 *
 * @param other the other translation to find the dot product with.
 *
 * @return the scalar valued dot product.
 */
double Translation2d::operator*(const Translation2d &other) const { return (m_x * other.m_x) + (m_y * other.m_y); }

/**
 * Compares two translations.
 * Returns true if their components are each within 1e-9, to account for floating point error.
 *
 * @param other the translation to compare to.
 *
 * @return whether the two translations are equal.
 */
bool Translation2d::operator==(const Translation2d &other) const {
  return std::abs(m_x - other.m_x) < 1e-9 && std::abs(m_y - other.m_y) < 1e-9;
}

/**
 * Sends a translation to an output stream.
 * Ex.
 * std::cout << translation;
 *
 * prints "Translation2d[x: (value), y: (value), rad: (radians), deg: (degrees)]"
 */
std::ostream &operator<<(std::ostream &os, const Translation2d &translation) {
  os << "Translation2d[x: " << translation.x() << ", y: " << translation.y() << "]";
  return os;
}

/**
 * Calculates the mean of the translations in the list.
 *
 * @param list std::vector containing a list of translations.
 *
 * @return the single translation mean of the list of translations.
 */
Translation2d mean(const std::vector<Translation2d> &list) {
  double sumx = 0;
  double sumy = 0;

  for (int i = 0; i < list.size(); i++) {
    sumx += list.at(i).x();
    sumy += list.at(i).y();
  }

  return Translation2d(sumx / list.size(), sumy / list.size());
}
