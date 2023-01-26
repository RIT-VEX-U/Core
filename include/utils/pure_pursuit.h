#pragma once

#include <vector>
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

    Vector2D::point_t getPoint() {
      return {x, y};
    }

    Vector2D getTangent() {
      return Vector2D(dir, mag);
    }
  };

  /**
    * Returns points of the intersections of a line segment and a circle. The line 
    * segment is defined by two points, and the circle is defined by a center and radius.
    */
  static std::vector<Vector2D::point_t> line_circle_intersections(Vector2D::point_t center, double r, Vector2D::point_t point1, Vector2D::point_t point2);
  /**
    * Selects a look ahead from all the intersections in the path.
    */
  static Vector2D::point_t get_lookahead(std::vector<Vector2D::point_t> path, Vector2D::point_t robot_loc, double radius);

  /**
    * Injects points in a path without changing the curvature with a certain spacing.
    */
  static std::vector<Vector2D::point_t> inject_path(std::vector<Vector2D::point_t> path, double spacing);

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

  static std::vector<Vector2D::point_t> smooth_path(std::vector<Vector2D::point_t> path, double weight_data, double weight_smooth, double tolerance);

  static std::vector<Vector2D::point_t> smooth_path_cubic(std::vector<Vector2D::point_t> path, double res);

  /**
   * Interpolates a smooth path given a list of waypoints using hermite splines.
   * For more information: https://www.youtube.com/watch?v=hG0p4XgePSA.
   *
   * @param path The path of hermite points to interpolate.
   * @param steps The number of points interpolated between points.
   * @return The smoothed path.
   */
  static std::vector<Vector2D::point_t> smooth_path_hermite(std::vector<hermite_point> path, double step);
}