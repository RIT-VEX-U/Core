#pragma once

// These are required for Eigen to compile
// https://www.vexforum.com/t/eigen-integration-issue/61474/5
#undef __ARM_NEON__
#undef __ARM_NEON
#include <Eigen/Dense>

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
  double dx();

  /**
   * Returns the linear dy component.
   *
   * @return the linear dy component.
   */
  double dy();

  /**
   * Returns the angular dtheta component.
   *
   * @return the angular dtheta component.
   */
  double dtheta();

  /**
   * Compares this to another twist.
   *
   * @param other the other twist to compare to.
   *
   * @return true if each of the components are within 1e-9 of each other.
   */
  bool operator==(const Twist2d &other);

  /**
   * Multiplies this twist by a scalar.
   *
   * @param scalar the scalar value to multiply by.
   */
  Twist2d operator*(const double &scalar);

  /**
   * Divides this twist by a scalar.
   *
   * @param scalar the scalar value to divide by.
   */
  Twist2d operator/(const double &scalar);

private:
  double m_dx;
  double m_dy;
  double m_dtheta;
};
