#pragma once

#include "core/utils/math/geometry/pose2d.h"
#include "core/utils/math/geometry/rotation2d.h"
#include "core/units/units.h"
#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/math/geometry/pose2d.h"
#include "core/utils/math/geometry/translation2d.h"

/**
 * @brief Represents a boundary point along a Hermite spline.
 *
 * Each point stores position (2D), first derivative (tangent/velocity),
 * and optional second derivative (acceleration) information.
 */
struct HermitePoint {
    Translation2d point;             ///< 2D position coordinates (x, y)
    Translation2d tangent;           ///< First derivative vector (dx/du, dy/du)
    Translation2d second_derivative; ///< Second derivative vector (d^2x/du^2, d^2y/du^2)

    /**
     * @brief Default constructor initializing all vectors to zero.
     */
    HermitePoint() = default;

    /**
     * @brief Constructs a HermitePoint from Translation2d vectors.
     * @param point 2D position vector.
     * @param tangent Tangent (first derivative) vector.
     * @param second_derivative Second derivative vector (default zero).
     */
    HermitePoint(
      const Translation2d &point,
      const Translation2d &tangent,
      const Translation2d &second_derivative = Translation2d())
        : point(point),
          tangent(tangent),
          second_derivative(second_derivative) {}

    /**
     * @brief Constructs a HermitePoint from scalar position and tangent components.
     * @param x X coordinate.
     * @param y Y coordinate.
     * @param vx X tangent component.
     * @param vy Y tangent component.
     */
    HermitePoint(double x, double y, double vx, double vy)
        : point(x, y),
          tangent(vx, vy),
          second_derivative(0.0, 0.0) {}

    /**
     * @brief Constructs a HermitePoint from scalar position, tangent, and second derivative components.
     * @param x X coordinate.
     * @param y Y coordinate.
     * @param vx X tangent component.
     * @param vy Y tangent component.
     * @param ax X second derivative component.
     * @param ay Y second derivative component.
     */
    HermitePoint(double x, double y, double vx, double vy, double ax, double ay)
        : point(x, y),
          tangent(vx, vy),
          second_derivative(ax, ay) {}

    /**
     * @brief Factory method constructing HermitePoint from explicit derivative vectors.
     * @param point 2D position vector.
     * @param tangent Tangent vector.
     * @param second_derivative Second derivative vector.
     * @return Constructed HermitePoint instance.
     */
    static HermitePoint from_derivatives(
      const Translation2d &point,
      const Translation2d &tangent,
      const Translation2d &second_derivative = Translation2d()) {
        return HermitePoint(point, tangent, second_derivative);
    }

    /**
     * @brief Factory method constructing HermitePoint from heading angle and scalar speed.
     * @param x X coordinate in inches.
     * @param y Y coordinate in inches.
     * @param heading Heading angle.
     * @param speed Tangent magnitude (speed factor).
     * @param accel_heading Second derivative direction.
     * @param accel_mag Second derivative magnitude.
     * @return Constructed HermitePoint instance.
     */
    static HermitePoint from_heading(
      units::Length x,
      units::Length y,
      Rotation2d heading,
      double speed,
      Rotation2d accel_heading = Rotation2d(),
      double accel_mag = 0.0) {
        return HermitePoint(
          Translation2d(x.in(), y.in()),
          Translation2d(speed, heading),
          Translation2d(accel_mag, accel_heading));
    }

    /**
     * @brief Factory method constructing HermitePoint from position vector, heading angle, and speed.
     * @param point 2D position vector.
     * @param heading Heading angle.
     * @param speed Tangent magnitude.
     * @param accel_heading Second derivative direction.
     * @param accel_mag Second derivative magnitude.
     * @return Constructed HermitePoint instance.
     */
    static HermitePoint from_heading(
      const Translation2d &point,
      Rotation2d heading,
      double speed,
      Rotation2d accel_heading = Rotation2d(),
      double accel_mag = 0.0) {
        return HermitePoint(
          point,
          Translation2d(speed, heading),
          Translation2d(accel_mag, accel_heading));
    }

    /**
     * @brief Factory method constructing HermitePoint from Pose2d and speeds.
     * @param pose 2D position and orientation.
     * @param speed Tangent magnitude.
     * @param accel_mag Second derivative magnitude.
     * @return Constructed HermitePoint instance.
     */
    static HermitePoint from_pose(
      const Pose2d &pose,
      double speed,
      double accel_mag = 0.0) {
        return HermitePoint(
          pose.translation(),
          Translation2d(speed, pose.rotation()),
          Translation2d(accel_mag, pose.rotation()));
    }

    /** @return 2D position vector. */
    Translation2d get_point() const { return point; }
    /** @return Tangent vector. */
    Translation2d get_tangent() const { return tangent; }
    /** @return Second derivative vector. */
    Translation2d get_second_derivative() const { return second_derivative; }
};
