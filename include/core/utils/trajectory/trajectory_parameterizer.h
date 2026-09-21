#pragma once

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "core/utils/trajectory/constraints/trajectory_constraint.h"
#include "core/utils/trajectory/trajectory.h"
#include "core/utils/units.h"

using namespace units::literals;

/**
 * @brief Algorithms for time-parameterizing spatial path points into constrained trajectories.
 *
 * Implements forward-pass and backward-pass velocity profile integration under
 * physical acceleration and user-defined constraints.
 */
class TrajectoryParameterizer {
   public:
    /**
     * @brief units::Time-parameterizes a list of discrete path points into a Trajectory under
     * physical constraints.
     * @param points Vector of poses with curvature along path.
     * @param constraints Vector of physical trajectory constraints.
     * @param start_velocity Boundary velocity at path start.
     * @param end_velocity Boundary velocity at path end.
     * @param max_velocity Maximum chassis velocity limit.
     * @param max_acceleration Maximum chassis acceleration limit.
     * @param reversed Direction flag (true for driving backward).
     * @return units::Time-parameterized Trajectory object.
     */
    static Trajectory time_parameterize_trajectory(
            const std::vector<PoseWithCurvature>& points,
            const std::vector<std::unique_ptr<TrajectoryConstraint>>& constraints,
            units::Velocity start_velocity,
            units::Velocity end_velocity,
            units::Velocity max_velocity,
            units::Acceleration max_acceleration,
            bool reversed,
            std::function<void(const char*)> error_handler = nullptr
    );

    /**
     * @brief Applies an FIR moving average filter to the velocity profile to limit jerk (S-Curve).
     * @param base The base trapezoidal trajectory.
     * @param max_acceleration The maximum acceleration limit used in the base trajectory.
     * @param max_jerk The desired maximum jerk limit.
     * @return S-Curve filtered Trajectory.
     */
    static Trajectory jerk_limit_trajectory(
            const Trajectory& base, units::Acceleration max_acceleration, units::Jerk max_jerk
    );

   private:
    constexpr static double kEpsilon = 1E-6;

    struct ConstrainedState {
        PoseWithCurvature pose = {Pose2d{}, 0_radpm};
        units::Length distance = 0_in;
        units::Velocity max_velocity = 0_inps;
        units::Acceleration min_acceleration = 0_inps2;
        units::Acceleration max_acceleration = 0_inps2;
    };

    static bool enforce_acceleration_limits(
            bool reverse,
            const std::vector<std::unique_ptr<TrajectoryConstraint>>& constraints,
            ConstrainedState* state,
            const std::function<void(const char*)>& error_handler
    );
};
