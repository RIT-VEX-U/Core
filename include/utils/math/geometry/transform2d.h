#pragma once

// These are required for Eigen to compile
// https://www.vexforum.com/t/eigen-integration-issue/61474/5
#undef __ARM_NEON__
#undef __ARM_NEON
#include <Eigen/Dense>

#include "../core/include/utils/math/geometry/pose2d.h"
#include "../core/include/utils/math/geometry/rotation2d.h"
#include "../core/include/utils/math/geometry/translation2d.h"

/**
 * Class representing a transformation of a pose2d, or a linear difference between the components of poses.
 *
 * Assumes conventional cartesian coordinate system:
 * Looking down at the coordinate plane,
 * +X is right
 * +Y is up
 * +Theta is counterclockwise
 */
class Transform2d {
public:
  /**
   * Constructs a transform given translation and rotation components.
   *
   * @param translation the translational component of the transform.
   * @param rotation the rotational component of the transform.
   */
  Transform2d(const Translation2d &translation, const Rotation2d &rotation);

  /**
   * Constructs a transform given translation and rotation components.
   *
   * @param x the x component of the transform.
   * @param y the y component of the transform.
   * @param rotation the rotational component of the transform.
   */
  Transform2d(const double &x, const double &y, const Rotation2d &rotation);

  /**
   * Constructs a transform given translation and rotation components.
   *
   * @param x the x component of the transform.
   * @param y the y component of the transform.
   * @param radians the rotational component of the transform in radians.
   */
  Transform2d(const double &x, const double &y, const double &radians);

  /**
   * Constructs a transform given translation and rotation components.
   *
   * @param translation the translational component of the transform.
   * @param radians the rotational component of the transform in radians.
   */
  Transform2d(const Translation2d &translation, const double &radians);

  /**
   * Constructs a transform given translation and rotation components given as a vector.
   *
   * @param transform_vector vector of the form [x, y, theta]
   */
  Transform2d(const Eigen::Vector3d &transform_vector);

  /**
   * Constructs a transform given translation and rotation components.
   *
   * @param translation the translational component of the transform.
   * @param rotation the rotational component of the transform.
   */
  Transform2d(const Pose2d &start, const Pose2d &end);

  /**
   * Returns the translational component of the transform.
   *
   * @return the translational component of the transform.
   */
  Translation2d translation();

  /**
   * Returns the x component of the transform.
   *
   * @return the x component of the transform.
   */
  double x();

  /**
   * Returns the y component of the transform.
   *
   * @return the y component of the transform.
   */
  double y();

  /**
   * Returns the rotational component of the transform.
   *
   * @return the rotational component of the transform.
   */
  Rotation2d rotation();

  /**
   * Inverts the transform.
   *
   * @return the inverted transform.
   */
  Transform2d inverse();

  /**
   * Composes two transforms, does this transform first then other after.
   *
   * @param other the other transform to add to this transform.
   */
  Transform2d operator+(const Transform2d &other);

  /**
   * Subtracts a transform from this transform, composes this transform and the inverse of the other transform.
   *
   * @param other the other transform to subtract from this transform.
   */
  Transform2d operator-(const Transform2d &other);

  /**
   * Multiplies this transform by a scalar.
   *
   * @param scalar the scalar to multiply this transform by.
   */
  Transform2d operator*(const double &scalar);

  /**
   * Divides this transform by a scalar.
   *
   * @param scalar the scalar to divide this transform by.
   */
  Transform2d operator/(const double &scalar);

  /**
   * Compares this to another transform.
   *
   * @param other the other transform to compare to.
   *
   * @return true if the components are within 1e-9 of each other.
   */
  bool operator==(const Transform2d &other);

private:
  Translation2d translation;
  Rotation2d rotation;
};
