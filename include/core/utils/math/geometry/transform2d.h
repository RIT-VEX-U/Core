#pragma once

#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/math/geometry/translation2d.h"
#include "core/utils/units.h"

/**
 * Class representing a transformation of a pose2d, by rotating a translation into the frame
 * of the pose2d, then adding the translation, then rotating by theta.
 *
 * Assumes conventional cartesian coordinate system:
 * Looking down at the coordinate plane,
 * +X is right
 * +Y is up
 * +Theta is counterclockwise
 */
struct Transform2d {
  /**
   * Default Constructor for Transform2d
   */
  constexpr Transform2d() = default;

  /**
   * Constructs a transform given translation and rotation components.
   *
   * @param translation the translational component of the transform.
   * @param rotation the rotational component of the transform.
   */
  constexpr Transform2d(Translation2d translation, Rotation2d rotation)
      : translation_(translation), rotation_(rotation) {}

  /**
   * Constructs a transform given translation and rotation components.
   *
   * @param x the x component of the transform.
   * @param y the y component of the transform.
   * @param rotation the rotational component of the transform.
   */
  constexpr Transform2d(units::Length x, units::Length y, Rotation2d rotation)
      : translation_(x, y), rotation_(rotation) {}

  /**
   * Constructs a transform given translation and rotation components.
   *
   * @param x the x component of the transform.
   * @param y the y component of the transform.
   * @param angle the rotational component of the transform.
   */
  constexpr Transform2d(units::Length x, units::Length y, units::Angle angle)
      : translation_(x, y), rotation_(angle) {}

  /**
   * Constructs a transform given translation and rotation components.
   *
   * @param translation the translational component of the transform.
   * @param angle the rotational component of the transform.
   */
  constexpr Transform2d(Translation2d translation, units::Angle angle)
      : translation_(translation), rotation_(angle) {}

  /**
   * Constructs a transform given translation and rotation components given as a
   * vector using the length unit specified, and radians.
   *
   * @param transform_vector vector of the form [x, y, theta]
   * @param unit The length unit to use when assigning the translation.
   */
  constexpr Transform2d(const Eigen::Vector3d &transform_vector,
                        units::Length unit)
      : translation_({transform_vector(0), transform_vector(1)}, unit),
        rotation_(transform_vector(2)) {}

  /**
   * Inverts the transform.
   *
   * @return the inverted transform.
   */
  constexpr Transform2d inverse() const {
    return Transform2d(-translation_.rotate_by(-rotation_), -rotation_);
  }

  /**
   * Multiplies this transform by a scalar.
   *
   * @param scalar the scalar to multiply this transform by.
   */
  constexpr Transform2d operator*(double scalar) const {
      return Transform2d(translation_ * scalar, rotation_ * scalar);
  }

  /**
   * Divides this transform by a scalar.
   *
   * @param scalar the scalar to divide this transform by.
   */
  constexpr Transform2d operator/(const double &scalar) const {
      return Transform2d(translation_ / scalar, rotation_ / scalar);
  }

  /**
   * Inverts the transform.
   *
   * @return the inverted transform.
   */
  constexpr Transform2d operator-() const {
      return inverse();
  }

  /**
   * Compares this to another transform.
   *
   * @param other the other transform to compare to.
   *
   * @return true if the components are equal.
   */
  constexpr bool operator==(const Transform2d &other) const {
      return (translation_ == other.translation_) && (rotation_ == other.rotation_);
  }

  Translation2d translation_;
  Rotation2d rotation_;
};
