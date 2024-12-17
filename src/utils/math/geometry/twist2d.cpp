#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <vector>

#include "../core/include/utils/math/geometry/twist2d.h"

/**
 * Constructs a twist with given translation and angle deltas.
 *
 * @param dx the linear dx component.
 * @param dy the linear dy component.
 * @param dtheta the angular dtheta component.
 */
Twist2d::Twist2d(const double &dx, const double &dy, const double &dtheta) : m_dx{dx}, m_dy{dy}, m_dtheta{dtheta} {}

/**
 * Constructs a twist with given translation and angle deltas.
 *
 * @param twist_vector vector of the form [dx, dy, dtheta]
 */
Twist2d::Twist2d(const Eigen::Vector3d &twist_vector)
    : m_dx{twist_vector(0)}, m_dy{twist_vector(1)}, m_dtheta{twist_vector(2)} {}

/**
 * Returns the linear dx component.
 *
 * @return the linear dx component.
 */
double Twist2d::dx() const { return m_dx; }

/**
 * Returns the linear dy component.
 *
 * @return the linear dy component.
 */
double Twist2d::dy() const { return m_dy; }

/**
 * Returns the angular dtheta component.
 *
 * @return the angular dtheta component.
 */
double Twist2d::dtheta() const { return m_dtheta; }

/**
 * Compares this to another twist.
 *
 * @param other the other twist to compare to.
 *
 * @return true if each of the components are within 1e-9 of each other.
 */
bool Twist2d::operator==(const Twist2d &other) const {
  return std::abs(dx() - other.dx()) < 1e-9 && std::abs(dy() - other.dy()) < 1e-9 &&
         std::abs(dtheta() - other.dtheta()) < 1e-9;
}

/**
 * Multiplies this twist by a scalar.
 *
 * @param scalar the scalar value to multiply by.
 */
Twist2d Twist2d::operator*(const double &scalar) const {
  return Twist2d{dx() * scalar, dy() * scalar, dtheta() * scalar};
}

/**
 * Divides this twist by a scalar.
 *
 * @param scalar the scalar value to divide by.
 */
Twist2d Twist2d::operator/(const double &scalar) const { return *this * (1. / scalar); }

/**
 * Sends a twist to an output stream.
 * Ex.
 * std::cout << twist;
 *
 * prints "Twist2d[x: (value), y: (value), rad: (radians), deg: (degrees)]"
 */
std::ostream &operator<<(std::ostream &os, const Twist2d &twist) {
  os << "Twist2d[dx: " << twist.dx() << ", dy: " << twist.dy() << ", dtheta: " << twist.dtheta() << "]";
  return os;
}
