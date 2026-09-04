#pragma once

#include <functional>
#include <utility>
#include <vector>

#include "core/units/units.h"
#include "core/utils/math/spline/hermite_point.h"
#include "core/utils/trajectory/trajectory.h"
#include "core/utils/trajectory/trajectory_config.h"

/**
 * @brief Helper utility class for generating time-parameterized constrained trajectories from waypoints.
 */
class TrajectoryGenerator {
 public:
  /** @brief Pair representing 2D Pose and scalar path Curvature. */
  using PoseWithCurvature = std::pair<Pose2d, Curvature>;

  /**
   * @brief Generates a time-parameterized Trajectory from Hermite waypoints and a TrajectoryConfig.
   * @param waypoints Vector of Hermite boundary waypoints (positions, tangents, second derivatives).
   * @param config Trajectory configuration containing velocity limits, accelerations, direction, and constraints.
   * @return Time-parameterized Trajectory instance.
   */
  static Trajectory generate_trajectory(
      const std::vector<HermitePoint>& waypoints,
      const TrajectoryConfig& config);

  /**
   * @brief Sets custom error handler callback for trajectory generation failures.
   * @param func Error reporting function.
   */
  static void set_error_handler(std::function<void(const char*)> func);

 private:
  static void report_error(const char* error);

  static const Trajectory kDoNothingTrajectory;
  static std::function<void(const char*)> s_errorFunc;
};
