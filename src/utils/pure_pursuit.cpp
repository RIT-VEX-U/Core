#include "core/utils/pure_pursuit.h"

/**
 * Create a Path
 * @param points the points that make up the path
 * @param radius the lookahead radius for pure pursuit
 */
PurePursuit::Path::Path(std::vector<Translation2d> points, double radius) {
    this->points = points;
    this->radius = radius;
    this->valid = true;

    for (int i = 0; i < points.size() - 1; i++) {
        for (int j = i + 2; j < points.size() - 1; j++) {
            // Iterate over points on the segments discretely and compare distances
            double segment_i_dist = points[i].distance(points[i + 1]);
            if (segment_i_dist == 0) {
                segment_i_dist = 0.1;
            }
            double segment_j_dist = points[j].distance(points[j + 1]);
            if (segment_j_dist == 0) {
                segment_j_dist = 0.1;
            }
            for (double t1 = 0; t1 <= 1; t1 += radius / segment_i_dist) {
                Translation2d p1(0, 0);
                p1 = Translation2d(
                  (points[i].x() + t1 * (points[i + 1].x() - points[i].x())),
                  (points[i].y() + t1 * (points[i + 1].y() - points[i].y()))
                );

                for (double t2 = 0; t2 <= 1; t2 += radius / segment_j_dist) {
                    Translation2d p2(0, 0);
                    p2 = Translation2d(
                      (points[j].x() + t2 * (points[j + 1].x() - points[j].x())),
                      (points[j].y() + t2 * (points[j + 1].y() - points[j].y()))
                    );

                    if (p1.distance(p2) < radius) {
                        this->valid = false;
                        return;
                    }
                }
            }
        }
    }
}

/**
 * Get the points associated with this Path
 */
const std::vector<Translation2d> PurePursuit::Path::get_points() { return this->points; }

/**
 * Get the radius associated with this Path
 */
double PurePursuit::Path::get_radius() { return this->radius; }

/**
 * Get whether this path will behave as expected
 */
bool PurePursuit::Path::is_valid() { return this->valid; }

/**
 * Returns points of the intersections of a line segment and a circle. The line
 * segment is defined by two points, and the circle is defined by a center and radius.
 */
std::vector<Translation2d>
PurePursuit::line_circle_intersections(Translation2d center, double r, Translation2d point1, Translation2d point2) {
    std::vector<Translation2d> intersections = {};

    // Do future calculations relative to the circle's center

    point1 = point1 - center;
    point2 = point2 - center;

    double x1, x2, y1, y2;
    // Handling an infinite slope using mx+b and x^2 + y^2 = r^2
    if (point1.x() - point2.x() == 0) {
        x1 = point1.x();
        y1 = sqrt(pow(r, 2) - pow(x1, 2));
        x2 = point1.x();
        y2 = -sqrt(pow(r, 2) - pow(x2, 2));
    }
    // Non-infinite slope using mx+b and x^2 + y^2 = r^2
    else {
        double m = (point1.y() - point2.y()) / (point1.x() - point2.x());
        double b = point1.y() - (m * point1.x());

        x1 = ((-m * b) + sqrt(pow(r, 2) + (pow(m, 2) * pow(r, 2)) - pow(b, 2))) / (1 + pow(m, 2));
        y1 = m * x1 + b;
        x2 = ((-m * b) - sqrt(pow(r, 2) + (pow(m, 2) * pow(r, 2)) - pow(b, 2))) / (1 + pow(m, 2));
        y2 = m * x2 + b;
    }

    // The equations used define an infinitely long line, so we check if the detected intersection falls on the line
    // segment.
    if (x1 >= fmin(point1.x(), point2.x()) && x1 <= fmax(point1.x(), point2.x()) &&
        y1 >= fmin(point1.y(), point2.y()) && y1 <= fmax(point1.y(), point2.y())) {
        intersections.push_back(Translation2d(x1 + center.x(), y1 + center.y()));
    }

    if (x2 >= fmin(point1.x(), point2.x()) && x2 <= fmax(point1.x(), point2.x()) &&
        y2 >= fmin(point1.y(), point2.y()) && y2 <= fmax(point1.y(), point2.y())) {
        intersections.push_back(Translation2d(x2 + center.x(), y2 + center.y()));
    }

    return intersections;
}

/**
 * Selects a look ahead from all the intersections in the path.
 */
[[maybe_unused]] Translation2d
PurePursuit::get_lookahead(const std::vector<Translation2d> &path, Pose2d robot_loc, double radius) {
    // Default: the end of the path
    Translation2d target = path.back();

    if (target.distance(robot_loc.translation()) <= radius) {
        return target;
    }

    // Check each line segment of the path for potential targets
    for (int i = 0; i < path.size() - 1; i++) {
        Translation2d start = path[i];
        Translation2d end = path[i + 1];

        std::vector<Translation2d> intersections =
          PurePursuit::line_circle_intersections(robot_loc.translation(), radius, start, end);
        // Choose the intersection that is closest to the end of the line segment
        // This prioritizes the closest intersection to the end of the path
        for (Translation2d intersection : intersections) {
            if (intersection.distance(end) < target.distance(end)) {
                target = intersection;
            }
        }
    }

    return target;
}

/**
 Injects points in a path without changing the curvature with a certain spacing.
*/
[[maybe_unused]] std::vector<Translation2d>
PurePursuit::inject_path(const std::vector<Translation2d> &path, double spacing) {
    std::vector<Translation2d> new_path;

    // Injecting points for each line segment
    for (int i = 0; i < path.size() - 1; i++) {
        Translation2d start = path[i];
        Translation2d end = path[i + 1];

        Translation2d diff = end - start;

        int num_points = ceil(diff.norm() / spacing);

        // This is the vector between each point
        diff = diff.normalize() * spacing;

        for (int j = 0; j < num_points; j++) {
            // We take the start point and add additional vectors
            Translation2d path_point = (Translation2d(start) + diff * j);
            new_path.push_back(path_point);
        }
    }
    // Adds the last point
    new_path.push_back(path.back());
    return new_path;
}

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
[[maybe_unused]] std::vector<Translation2d> PurePursuit::smooth_path(
  const std::vector<Translation2d> &path, double weight_data, double weight_smooth, double tolerance
) {
    std::vector<Translation2d> new_path = path;
    double change = tolerance;
    while (change >= tolerance) {
        change = 0;
        for (int i = 1; i < path.size() - 1; i++) {
            Translation2d x_i = path[i];
            Translation2d y_i = new_path[i];
            Translation2d y_prev = new_path[i - 1];
            Translation2d y_next = new_path[i + 1];

            Translation2d y_i_saved = y_i;

            y_i = Translation2d(
              (y_i.x() + weight_data * (x_i.x() - y_i.x()) + weight_smooth * (y_next.x() + y_prev.x() - (2 * y_i.x()))),
              (y_i.y() + weight_data * (x_i.y() - y_i.y()) + weight_smooth * (y_next.y() + y_prev.y() - (2 * y_i.y())))
            );
            new_path[i] = y_i;

            change += y_i.distance(y_i_saved);
        }
    }
    return new_path;
}

/**
 * Interpolates a smooth path given a list of waypoints using hermite splines.
 * For more information: https://www.youtube.com/watch?v=hG0p4XgePSA.
 *
 * @param path The path of hermite points to interpolate.
 * @param steps The number of points interpolated between points.
 * @return The smoothed path.
 */
[[maybe_unused]] std::vector<Translation2d>
PurePursuit::smooth_path_hermite(const std::vector<hermite_point> &path, double steps) {
    std::vector<Translation2d> new_path;
    for (int i = 0; i < path.size() - 1; i++) {
        for (int t = 0; t < steps; t++) {
            // Storing the start and end points and slopes at those points as Translation2ds.
            Translation2d tmp = path[i].getPoint();
            Translation2d p1 = Translation2d(tmp.x(), tmp.y());
            tmp = path[i + 1].getPoint();
            Translation2d p2 = Translation2d(tmp.x(), tmp.y());
            Translation2d t1 = path[i].getTangent();
            Translation2d t2 = path[i + 1].getTangent();

            // Scale s from 0.0 to 1.0
            double s = (double)t / (double)steps;

            // Hermite Blending functions
            double h1 = 2 * pow(s, 3) - 3 * pow(s, 2) + 1;
            double h2 = -2 * pow(s, 3) + 3 * pow(s, 2);
            double h3 = pow(s, 3) - 2 * pow(s, 2) + s;
            double h4 = pow(s, 3) - pow(s, 2);

            // Calculate the point
            Translation2d pv = p1 * h1 + p2 * h2 + t1 * h3 + t2 * h4;
            new_path.push_back(pv);
        }
    }
    // Adding last point
    new_path.push_back(path.back().getPoint());
    return new_path;
}

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
double PurePursuit::estimate_remaining_dist(const std::vector<Translation2d> &path, Pose2d robot_pose, double radius) {
    Translation2d lookahead_pt = PurePursuit::get_lookahead(path, robot_pose, radius);

    if (lookahead_pt == path[path.size() - 1]) {
        return robot_pose.translation().distance(lookahead_pt);
    }

    double dist = 0;

    // Run through the path backwards, adding distances
    for (int i = path.size() - 1; i >= 0; i--) {
        // Test if the robot is between the two points
        auto pts = PurePursuit::line_circle_intersections(robot_pose.translation(), radius, path[i - 1], path[i]);

        // There is an intersection? Robot is between the points so add the distance
        // from the bot to the next point and end.
        if (!pts.empty()) {
            dist += robot_pose.translation().distance(path[i]);
            return dist;
        }

        // No intersections? Add the distance between the two points and move backwards
        // in the path until we find the robot, or run out of points.
        dist += path[i - 1].distance(path[i]);
    }

    return dist;
}
