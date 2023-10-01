#pragma once

#include <vector>
#include "../core/include/utils/geometry.h"
#include "../core/include/utils/vector2d.h"
#include "vex.h"

using namespace vex;

namespace PurePursuit {
  /**
   * Represents a piece of a cubic spline with s(x) = a(x-xi)^3 + b(x-xi)^2 + c(x-xi) + d
   * The x_start and x_end shows where the equation is valid.
   */
  struct spline
  {
    double a, b, c, d, x_start, x_end; 

    double getY(double x) {
      return a * pow((x - x_start), 3) + b * pow((x - x_start), 2) + c * (x - x_start) + d;
    }
  };
  /**
   * a position along the hermite path
   * contains a position and orientation information that the robot would be at at this point
   */
  struct hermite_point
  {
    double x;
    double y;
    double dir;
    double mag;

    point_t getPoint() const {
      return {x, y};
    }

    Vector2D getTangent() const {
      return Vector2D(dir, mag);
    }
  };

  /**
    * Returns points of the intersections of a line segment and a circle. The line 
    * segment is defined by two points, and the circle is defined by a center and radius.
    */
  extern std::vector<point_t> line_circle_intersections(point_t center, double r, point_t point1, point_t point2);
  /**
    * Selects a look ahead from all the intersections in the path.
    */
  extern point_t get_lookahead(const std::vector<point_t> &path, pose_t robot_loc, double radius);

  /**
    * Injects points in a path without changing the curvature with a certain spacing.
    */
  extern std::vector<point_t> inject_path(const std::vector<point_t> &path, double spacing);

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

  extern std::vector<point_t> smooth_path(const std::vector<point_t> &path, double weight_data, double weight_smooth, double tolerance);

  extern std::vector<point_t> smooth_path_cubic(const std::vector<point_t> &path, double res);

  /**
   * Interpolates a smooth path given a list of waypoints using hermite splines.
   * For more information: https://www.youtube.com/watch?v=hG0p4XgePSA.
   *
   * @param path The path of hermite points to interpolate.
   * @param steps The number of points interpolated between points.
   * @return The smoothed path.
   */
  extern std::vector<point_t> smooth_path_hermite(const std::vector<hermite_point> &path, double step);

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
  extern double estimate_remaining_dist(const std::vector<point_t> &path, pose_t robot_pose, double radius);

}