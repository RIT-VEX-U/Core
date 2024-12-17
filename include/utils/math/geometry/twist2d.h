#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <vector>

/**
 * Class representing a difference between two poses,
 * more specifically a distance along an arc from a pose.
 *
 * Assumes conventional cartesian coordinate system:
 * Looking down at the coordinate plane,
 * +X is right
 * +Y is up
 * +Theta is counterclockwise
 */
class Twist2d {
public:
  /**
   * Constructs a twist with given translation and angle deltas.
   *
   * @param dx the linear dx component.
   * @param dy the linear dy component.
   * @param dtheta the angular dtheta component.
   */
  Twist2d(const double &dx, const double &dy, const double &dtheta);

  /**
   * Constructs a twist with given translation and angle deltas.
   *
   * @param twist_vector vector of the form [dx, dy, dtheta]
   */
  Twist2d(const Eigen::Vector3d &twist_vector);

  /**
   * Returns the linear dx component.
   *
   * @return the linear dx component.
   */
  double dx() const;

  /**
   * Returns the linear dy component.
   *
   * @return the linear dy component.
   */
  double dy() const;

  /**
   * Returns the angular dtheta component.
   *
   * @return the angular dtheta component.
   */
  double dtheta() const;

  /**
   * Compares this to another twist.
   *
   * @param other the other twist to compare to.
   *
   * @return true if each of the components are within 1e-9 of each other.
   */
  bool operator==(const Twist2d &other) const;

  /**
   * Multiplies this twist by a scalar.
   *
   * @param scalar the scalar value to multiply by.
   */
  Twist2d operator*(const double &scalar) const;

  /**
   * Divides this twist by a scalar.
   *
   * @param scalar the scalar value to divide by.
   */
  Twist2d operator/(const double &scalar) const;

  /**
   * Sends a twist to an output stream.
   * Ex.
   * std::cout << twist;
   *
   * prints "Twist2d[dx: (value), dy: (value), drad: (radians)]"
   */
  friend std::ostream &operator<<(std::ostream &os, const Twist2d &twist);

private:
  double m_dx;
  double m_dy;
  double m_dtheta;
};
