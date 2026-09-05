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

  /**
   * @brief Generates a time-parameterized Trajectory from Hermite waypoints and a TrajectoryConfig.
   * @param waypoints Vector of Hermite boundary waypoints (positions, tangents, second derivatives).
   * @param config Trajectory configuration containing velocity limits, accelerations, direction, and constraints.
   * @return Time-parameterized Trajectory instance.
   */
  static Trajectory generate_trajectory(
      const std::vector<HermitePoint>& waypoints,
      const TrajectoryConfig& config);

  static const Trajectory kDoNothingTrajectory;
};
