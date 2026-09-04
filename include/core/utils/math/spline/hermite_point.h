#pragma once

#include "core/utils/math/geometry/rotation2d.h"
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
    Translation2d second_derivative; ///< Second derivative vector (d²x/du², d²y/du²)

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
     * @param heading_rad Heading angle in radians.
     * @param speed Tangent magnitude (speed factor).
     * @param accel_heading_rad Second derivative direction in radians.
     * @param accel_mag Second derivative magnitude.
     * @return Constructed HermitePoint instance.
     */
    static HermitePoint from_heading(
      double x,
      double y,
      double heading_rad,
      double speed,
      double accel_heading_rad = 0.0,
      double accel_mag = 0.0) {
        return HermitePoint(
          Translation2d(x, y),
          Translation2d(speed, Rotation2d(heading_rad)),
          Translation2d(accel_mag, Rotation2d(accel_heading_rad)));
    }

    /**
     * @brief Factory method constructing HermitePoint from position vector, heading angle, and speed.
     * @param point 2D position vector.
     * @param heading_rad Heading angle in radians.
     * @param speed Tangent magnitude.
     * @param accel_heading_rad Second derivative direction in radians.
     * @param accel_mag Second derivative magnitude.
     * @return Constructed HermitePoint instance.
     */
    static HermitePoint from_heading(
      const Translation2d &point,
      double heading_rad,
      double speed,
      double accel_heading_rad = 0.0,
      double accel_mag = 0.0) {
        return HermitePoint(
          point,
          Translation2d(speed, Rotation2d(heading_rad)),
          Translation2d(accel_mag, Rotation2d(accel_heading_rad)));
    }

    /** @return 2D position vector. */
    Translation2d get_point() const { return point; }
    /** @return Tangent vector. */
    Translation2d get_tangent() const { return tangent; }
    /** @return Second derivative vector. */
    Translation2d get_second_derivative() const { return second_derivative; }
};
