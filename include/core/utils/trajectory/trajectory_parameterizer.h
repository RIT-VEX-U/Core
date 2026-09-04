#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "core/units/units.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"
#include "core/utils/trajectory/trajectory.h"

/**
 * @brief Algorithms for time-parameterizing spatial path points into constrained trajectories.
 *
 * Implements forward-pass and backward-pass velocity profile integration under
 * physical acceleration and user-defined constraints.
 */
class TrajectoryParameterizer {
 public:
  /** @brief Pair representing 2D Pose and scalar path Curvature. */
  using PoseWithCurvature = std::pair<Pose2d, Curvature>;

  /**
   * @brief Time-parameterizes a list of discrete path points into a Trajectory under physical constraints.
   * @param points Vector of poses with curvature along path.
   * @param constraints Vector of physical trajectory constraints.
   * @param start_velocity Boundary velocity at path start.
   * @param end_velocity Boundary velocity at path end.
   * @param max_velocity Maximum chassis velocity limit.
   * @param max_acceleration Maximum chassis acceleration limit.
   * @param reversed Direction flag (true for driving backward).
   * @return Time-parameterized Trajectory object.
   */
  static Trajectory time_parameterize_trajectory(
      const std::vector<PoseWithCurvature>& points,
      const std::vector<std::unique_ptr<TrajectoryConstraint>>& constraints,
      Velocity start_velocity,
      Velocity end_velocity,
      Velocity max_velocity,
      Acceleration max_acceleration,
      bool reversed);

 private:
  constexpr static double kEpsilon = 1E-6;

  struct ConstrainedState {
    PoseWithCurvature pose = {Pose2d{}, 0_radpm};
    Length distance = 0_in;
    Velocity maxVelocity = 0_inps;
    Acceleration minAcceleration = 0_inps2;
    Acceleration maxAcceleration = 0_inps2;
  };

  static bool enforce_acceleration_limits(
      bool reverse,
      const std::vector<std::unique_ptr<TrajectoryConstraint>>& constraints,
      ConstrainedState* state);
};
