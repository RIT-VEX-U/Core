#pragma once

#include <functional>
#include <utility>
#include <vector>

#include "core/utils/math/spline/hermite_point.h"
#include "core/utils/trajectory/trajectory.h"
#include "core/utils/trajectory/trajectory_config.h"
#include "core/utils/units.h"

using namespace units::literals;

/**
 * @brief Helper utility class for generating time-parameterized constrained trajectories from
 * waypoints.
 */
class TrajectoryGenerator {
   public:
    /**
     * @brief Generates a time-parameterized Trajectory from Hermite waypoints and a
     * TrajectoryConfig.
     * @param waypoints Vector of Hermite boundary waypoints (positions, tangents, second
     * derivatives).
     * @param config Trajectory configuration containing velocity limits, accelerations, direction,
     * and constraints.
     * @return units::Time-parameterized Trajectory instance.
     */
    static Trajectory generate_trajectory(
            const std::vector<HermitePoint>& waypoints, const TrajectoryConfig& config
    );

    /**
     * @brief Generates a time-parameterized Trajectory from simple Pose2d waypoints.
     * Internal heuristics are used to compute optimal spline tangency (speed).
     * @param tangent_scale Scale factor applied to heuristic tangent magnitudes.
     */
    static Trajectory generate_trajectory(
            const std::vector<Pose2d>& waypoints,
            const TrajectoryConfig& config,
            double tangent_scale = 1.2
    );

    /**
     * @brief Splicing helper for On-The-Fly Replanning.
     * Generates a new path starting precisely at current_pose, maintaining the
     * units::momentum of current_velocity, and routing to the target_waypoints.
     * @param current_pose The robot's current instantaneous position and heading.
     * @param current_velocity The robot's current forward speed.
     * @param target_waypoints The remaining destinations.
     * @param config The base config. start_velocity will be overwritten to match current_velocity.
     * @param tangent_scale Fudge factor applied to tangent magnitudes.
     */
    static Trajectory generate_trajectory(
            const Pose2d& current_pose,
            units::Velocity current_velocity,
            const std::vector<Pose2d>& target_waypoints,
            TrajectoryConfig config,
            double tangent_scale = 1.2
    );

    static const Trajectory kDoNothingTrajectory;
};
