#pragma once

#include "core/utils/geometry.h"
#include "core/utils/math/geometry/pose2d.h"
#include "core/utils/math/geometry/translation2d.h"
#include "vex.h"
#include <vector>

using namespace vex;

namespace PurePursuit {
/**
 * Wrapper for a vector of points, checking if any of the points are too close for pure pursuit
 */
class Path {
  public:
    /**
     * Create a Path
     * @param points the points that make up the path
     * @param radius the lookahead radius for pure pursuit
     */
    Path(std::vector<Translation2d> points, double radius);

    /**
     * Get the points associated with this Path
     */
    const std::vector<Translation2d> get_points();

    /**
     * Get the radius associated with this Path
     */
    double get_radius();

    /**
     * Get whether this path will behave as expected
     */
    bool is_valid();

  private:
    std::vector<Translation2d> points;
    double radius;
    bool valid;
};
/**
 * Represents a piece of a cubic spline with s(x) = a(x-xi)^3 + b(x-xi)^2 + c(x-xi) + d
 * The x_start and x_end shows where the equation is valid.
 */
struct spline {
    double a, b, c, d, x_start, x_end;

    double getY(double x) { return a * pow((x - x_start), 3) + b * pow((x - x_start), 2) + c * (x - x_start) + d; }
};
/**
 * a position along the hermite path
 * contains a position and orientation information that the robot would be at at this point
 */
struct hermite_point {
    double x;
    double y;
    double dir;
    double mag;

    Translation2d getPoint() const { return Translation2d(x, y); }

    Translation2d getTangent() const { return Translation2d(mag, Rotation2d(dir)); }
};

/**
 * Returns points of the intersections of a line segment and a circle. The line
 * segment is defined by two points, and the circle is defined by a center and radius.
 */
extern std::vector<Translation2d>
line_circle_intersections(Translation2d center, double r, Translation2d point1, Translation2d point2);
/**
 * Selects a look ahead from all the intersections in the path.
 */
extern Translation2d get_lookahead(const std::vector<Translation2d> &path, Pose2d robot_loc, double radius);

/**
 * Injects points in a path without changing the curvature with a certain spacing.
 */
extern std::vector<Translation2d> inject_path(const std::vector<Translation2d> &path, double spacing);

/**
 * Returns a smoothed path maintaining the start and end of the path.
 *
 * Weight data is how much weight to update the data (alpha)
 * Weight smooth is how much weight to smooth the coordinates (beta)
 * Tolerance is how much change per iteration is necessary to continue iterating.
 *
 * Honestly have no idea if/how this works.
 * https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4
 */

extern std::vector<Translation2d>
smooth_path(const std::vector<Translation2d> &path, double weight_data, double weight_smooth, double tolerance);

extern std::vector<Translation2d> smooth_path_cubic(const std::vector<Translation2d> &path, double res);

/**
 * Interpolates a smooth path given a list of waypoints using hermite splines.
 * For more information: https://www.youtube.com/watch?v=hG0p4XgePSA.
 *
 * @param path The path of hermite points to interpolate.
 * @param steps The number of points interpolated between points.
 * @return The smoothed path.
 */
extern std::vector<Translation2d> smooth_path_hermite(const std::vector<hermite_point> &path, double step);

/**
 * Estimates the remaining distance from the robot's position to the end,
 * by "searching" for the robot along the path and running a "connect the dots"
 * distance algoritm
 *
 * @param path The pure pursuit path the robot is following
 * @param robot_pose The robot's current position
 * @param radius Pure pursuit "radius", used to search for the robot along the path
 * @return A rough estimate of the remaining distance
 */
extern double estimate_remaining_dist(const std::vector<Translation2d> &path, Pose2d robot_pose, double radius);

} // namespace PurePursuit