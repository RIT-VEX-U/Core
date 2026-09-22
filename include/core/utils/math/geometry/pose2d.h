#pragma once

#include <vector>

#include "cevalm.hpp"

#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/math/geometry/transform2d.h"
#include "core/utils/math/geometry/translation2d.h"
#include "core/utils/math/geometry/twist2d.h"
#include "core/utils/units.h"

/**
 * Class representing a pose in 2d space with x, y, and rotational components
 *
 * Assumes conventional cartesian coordinate system:
 * Looking down at the coordinate plane,
 * +X is right
 * +Y is up
 * +Theta is counterclockwise
 */
struct Pose2d {
  /**
   * Default Constructor for Pose2d
   */
  constexpr Pose2d() : translation_{Translation2d()}, rotation_{Rotation2d()} {}

  /**
   * Constructs a pose with given translation and rotation components.
   *
   * @param translation translational component.
   * @param rotation rotational component.
   */
  constexpr Pose2d(Translation2d translation, Rotation2d rotation)
      : translation_{translation}, rotation_{rotation} {}

  /**
   * Constructs a pose with given translation and rotation components.
   *
   * @param x x component.
   * @param y y component.
   * @param rotation rotational component.
   */
  constexpr Pose2d(units::Length x, units::Length y, Rotation2d rotation)
      : translation_{x, y}, rotation_{rotation} {}

  /**
   * Constructs a pose with given translation and rotation components.
   *
   * @param x x component.
   * @param y y component.
   * @param radians rotational component in radians.
   */
  constexpr Pose2d(units::Length x, units::Length y, double radians)
      : translation_{x, y}, rotation_{radians} {}

  /**
   * Constructs a pose with given translation and rotation components in a vector,
   * using the supplied units.
   *
   * @param pose_vector vector of the form [x, y, theta].
   * @param length_unit unit of length to use.
   * @param angle_unit unit of angle to use, defaults to radians.
   */
  constexpr Pose2d(const Eigen::Vector3d &pose_vector,
                   units::Length length_unit, units::Angle angle_unit = units::radians)
      : translation_{units::Length(pose_vector[0], length_unit),
                     units::Length(pose_vector[1], length_unit)},
        rotation_{units::Angle(pose_vector[2], angle_unit)} {}

  /**
   * Returns [x, y, theta] in the supplied units. Angles default to radians.
   */
  EVec<3> as_vector(units::Length length_unit,
                    units::Angle angle_unit = units::radians) const {
    return {translation_.x_.to(length_unit), translation_.y_.to(length_unit),
            rotation_.angle().to(angle_unit)};
  }

  /**
   * Checks translation distance and the smallest angle against tolerances.
   * Defaults to 1um and 1e-6 radians.
   */
  constexpr bool is_near(const Pose2d &other,
                         units::Length distance_tolerance = units::Length(1e-6),
                         units::Angle angle_tolerance = units::Angle(1e-6)) const {
    return translation_.is_near(other.translation_, distance_tolerance) &&
           rotation_.is_near(other.rotation_, angle_tolerance);
  }

  /**
   * Scales this pose's translation and principal angle.
   */
  constexpr Pose2d &operator*=(double scalar) {
    return *this = *this * scalar;
  }

  /**
   * Divides this pose's translation and principal angle.
   */
  constexpr Pose2d &operator/=(double scalar) {
    return *this = *this / scalar;
  }

  /**
   * Multiplies a scalar by this pose.
   */
  friend constexpr Pose2d operator*(double scalar, const Pose2d &pose) {
    return pose * scalar;
  }

  /**
   * Applies a transform to this pose in its local frame.
   */
  constexpr Pose2d &operator+=(const Transform2d &transform) {
    return *this = *this + transform;
  }

  /**
   * Returns the x value of the translational component.
   * @return the x value of the translational component.
   */
  constexpr units::Length x() const { return translation_.x_; }

  /**
   * Returns the y value of the translational component.
   * @return the y value of the translational component.
   */
  constexpr units::Length y() const { return translation_.y_; }

  /**
   * Returns x in the supplied length unit.
   */
  constexpr double x(units::Length unit) const { return translation_.x(unit); }

  /**
   * Returns y in the supplied length unit.
   */
  constexpr double y(units::Length unit) const { return translation_.y(unit); }

  /**
   * Returns the position.
   */
  constexpr Translation2d translation() const { return translation_; }

  /**
   * Returns the orientation.
   */
  constexpr Rotation2d rotation() const { return rotation_; }

  /**
   * Returns the heading as an angle.
   */
  constexpr units::Angle angle() const { return rotation_.angle(); }

  /**
   * Returns the heading in the supplied angle unit.
   */
  constexpr double angle(units::Angle unit) const { return rotation_.angle(unit); }

  /**
   * Returns the straight-line distance to a point.
   */
  constexpr units::Length distance(Translation2d point) const {
    return translation_.distance(point);
  }

  /**
   * Returns the straight-line distance to another pose, ignoring its heading.
   */
  constexpr units::Length distance(const Pose2d &other) const {
    return distance(other.translation_);
  }

  /**
   * Returns the world direction toward a point.
   * Returns this pose's heading if the positions are identical.
   */
  constexpr Rotation2d bearing_to(Translation2d point) const {
    const auto delta = point - translation_;
    if (delta.x_.internal() == 0 && delta.y_.internal() == 0) {
      return rotation_;
    }
    return delta.theta();
  }

  /**
   * Returns the smallest signed turn toward a point.
   * Returns zero if the positions are identical.
   */
  constexpr units::Angle angle_to(Translation2d point) const {
    return (bearing_to(point) - rotation_).angle();
  }

  /**
   * Converts a point from this pose's local frame to the world frame.
   */
  constexpr Translation2d local_to_world(Translation2d point) const {
    return translation_ + point.rotate_by(rotation_);
  }

  /**
   * Converts a point from the world frame to this pose's local frame.
   */
  constexpr Translation2d world_to_local(Translation2d point) const {
    return (point - translation_).rotate_by(-rotation_);
  }

  /**
   * Returns a copy with a new position.
   */
  constexpr Pose2d with_translation(Translation2d translation) const {
    return {translation, rotation_};
  }

  /**
   * Returns a copy with a new orientation.
   */
  constexpr Pose2d with_rotation(Rotation2d rotation) const {
    return {translation_, rotation};
  }

  /**
   * Interpolates position along a straight line and heading along the shortest turn.
   * Fractions outside [0, 1] return the nearest endpoint.
   */
  constexpr Pose2d interpolate(const Pose2d &end, double fraction) const {
    if (fraction <= 0) {
      return *this;
    }
    if (fraction >= 1) {
      return end;
    }
    return {translation_ + (end.translation_ - translation_) * fraction,
            rotation_ + (end.rotation_ - rotation_) * fraction};
  }

  /**
   * Compares this to another pose.
   *
   * @param other the other pose to compare to.
   *
   * @return true if each of the components are within 1e-6 of each other (meters and radians).
   */
  constexpr bool operator==(const Pose2d &other) const {
    return (translation_ == other.translation_) &&
           (rotation_ == other.rotation_);
  }

  /**
   * Multiplies this pose by a scalar.
   * Simply multiplies each component.
   *
   * @param scalar the scalar value to multiply by.
   */
  constexpr Pose2d operator*(double scalar) const {
    return Pose2d{translation_ * scalar, rotation_ * scalar};
  }

  /**
   * Divides this pose by a scalar.
   * Simply divides each component.
   *
   * @param scalar the scalar value to divide by.
   */
  constexpr Pose2d operator/(double scalar) const {
    return *this * (1.0 / scalar);
  }

  /**
   * Adds a transform to this pose.
   * Transforms the pose in the pose's frame.
   *
   * @param transform the change in pose.
   */
  constexpr Pose2d operator+(const Transform2d &transform) const {
    return Pose2d{translation_ + (transform.translation_.rotate_by(rotation_)),
                  transform.rotation_ + rotation_};
  }

  /**
   * Subtracts one pose from another to find the transform between them.
   *
   * @param other the pose to subtract.
   */
  constexpr Transform2d operator-(const Pose2d &other) const {
    return Transform2d{
        (translation_ - other.translation_).rotate_by(-other.rotation_),
        rotation_ - other.rotation_};
  }

  /**
   * Finds the pose equivalent to this pose relative to another arbitrary pose
   * rather than the origin.
   *
   * @param other the pose representing the new origin.
   *
   * @return this pose relative to another pose.
   */
  constexpr Pose2d relative_to(const Pose2d &other) const {
    Transform2d transform = *this - other;
    return Pose2d{transform.translation_, transform.rotation_};
  }

  /**
   * Adds a transform to this pose.
   * Rotates the transform's translation into the pose's frame,
   * adds the translation component, then adds the rotation component.
   *
   * @param transform the change in pose.
   *
   * @return the pose after being transformed.
   */
  constexpr Pose2d transform_by(const Transform2d &transform) const {
    return Pose2d{translation_ + (transform.translation_.rotate_by(rotation_)),
                  rotation_ + transform.rotation_};
  }

  /**
   * Applies a twist (pose delta) to a pose by integrating constant velocity and angular velocity
   * in the robot's frame of motion.
   *
   * When applying a twist, imagine a constant angular velocity, the
   * translational components must be rotated into the global frame at every
   * point along the twist, simply adding the deltas does not do this, and using
   * euler integration results in some error. This is the analytic solution for integrating
   * along an arc.
   *
   * Can also be thought of more simply as following an arc rather than a straight line.
   *
   * See this document for more information on the pose exponential and its
   * derivation.
   * https://file.tavsys.net/control/controls-engineering-in-frc.pdf#section.10.2
   *
   * @param twist     The twist, represents a pose delta.
   * @return new pose that has been moved forward according to the twist.
   */
  constexpr Pose2d exp(const Twist2d &twist) const {
    const units::Length dx = twist.dx_;
    const units::Length dy = twist.dy_;
    const double dtheta = twist.dtheta_.to(units::radians);

    const double sin_theta = cevalm::sin(dtheta);
    const double cos_theta = cevalm::cos(dtheta);

    double s, c;
    if (cevalm::abs(dtheta) < 1e-9) {
      s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
      c = 0.5 * dtheta;
    } else {
      s = sin_theta / dtheta;
      c = (1 - cos_theta) / dtheta;
    }

    const Transform2d transform{Translation2d{dx * s - dy * c, dx * c + dy * s},
                                Rotation2d{cos_theta, sin_theta}};

    return *this + transform;
  }

  /**
   * The inverse of the pose exponential.
   *
   * Determines the twist required to go from this pose to the given end pose.
   *
   * Returns the principal angular displacement; full turns cannot be
   * recovered from pose orientations alone.
   *
   * @param end_pose the end pose to find the mapping to.
   * @return the twist required to go from this pose to the given end
   */
  constexpr Twist2d log(const Pose2d &end_pose) const {
    const Pose2d transform = end_pose.relative_to(*this);
    const double dtheta = transform.rotation_.radians();
    const double halfDtheta = dtheta / 2.0;

    const double cosMinusOne = transform.rotation_.f_cos() - 1;

    double halfThetaByTanOfHalfDtheta;

    if (cevalm::abs(cosMinusOne) < 1e-9) {
      halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halfThetaByTanOfHalfDtheta =
          -(halfDtheta * transform.rotation_.f_sin()) / cosMinusOne;
    }

    const Translation2d translationPart =
        transform.translation_.rotate_by(
            {halfThetaByTanOfHalfDtheta, -halfDtheta}) *
        cevalm::hypot(halfThetaByTanOfHalfDtheta, halfDtheta);

    return Twist2d{translationPart.x_, translationPart.y_,
                   units::Angle(dtheta, units::radians)};
  }

  /**
   * Calculates the mean of a list of poses.
   *
   * @param list std::vector containing a list of poses.
   * @return the single pose mean of the list of poses.
   */
  static Pose2d wrapped_mean(const std::vector<Pose2d> &list) {
    if (list.size() == 0) {
      return Pose2d{};
    }

    units::Length sumx;
    units::Length sumy;

    double sum_sin = 0;
    double sum_cos = 0;

    for (int i = 0; i < list.size(); i++) {
      sumx += list.at(i).x();
      sumy += list.at(i).y();

      sum_sin += list.at(i).rotation_.f_sin();
      sum_cos += list.at(i).rotation_.f_cos();
    }

    return Pose2d{Translation2d{sumx / list.size(), sumy / list.size()},
                  Rotation2d{sum_cos / list.size(), sum_sin / list.size()}};
  }

  Translation2d translation_;
  Rotation2d rotation_;
};
