#include "core/utils/trajectory/trajectory_parameterizer.h"

#include <cstdio>
#include <cmath>
#include <vector>

Trajectory TrajectoryParameterizer::time_parameterize_trajectory(
    const std::vector<PoseWithCurvature>& points,
    const std::vector<std::unique_ptr<TrajectoryConstraint>>& constraints,
    units::Velocity start_velocity,
    units::Velocity end_velocity,
    units::Velocity max_velocity,
    units::Acceleration max_acceleration,
    bool reversed,
    std::function<void(const char*)> error_handler) {
  if (points.empty()) {
    return Trajectory{};
  }

  const units::Length kEpsilonLength = kEpsilon * units::in;
  const units::Acceleration kAccelTolerance = 1E-6 * units::inps2;
  const units::Velocity kVelocityTolerance = 1E-6 * units::inps;

  std::vector<ConstrainedState> constrainedStates(points.size());

  ConstrainedState predecessor;
  predecessor.pose = points.front();
  predecessor.distance = 0_in;
  predecessor.max_velocity = start_velocity;
  predecessor.min_acceleration = -max_acceleration;
  predecessor.max_acceleration = max_acceleration;

  constrainedStates[0] = predecessor;

  for (size_t i = 0; i < points.size(); ++i) {
    auto& constrainedState = constrainedStates[i];
    constrainedState.pose = points[i];

    units::Length ds = units::in * (constrainedState.pose.pose.translation().distance(
        predecessor.pose.pose.translation()));
    constrainedState.distance = ds + predecessor.distance;

    while (true) {
      const double predecessor_velocity_sq = (
          predecessor.max_velocity * predecessor.max_velocity +
          predecessor.max_acceleration * ds * 2.0).internal();
      constrainedState.max_velocity = units::Velocity(
          std::min(max_velocity.internal(),
                   std::sqrt(std::max(0.0, predecessor_velocity_sq))));

      constrainedState.min_acceleration = -max_acceleration;
      constrainedState.max_acceleration = max_acceleration;

      for (const auto& constraint : constraints) {
        constrainedState.max_velocity = std::min(
            constrainedState.max_velocity,
            constraint->max_velocity(constrainedState.pose.pose,
                                    constrainedState.pose.curvature,
                                    constrainedState.max_velocity));
      }

      if (!enforce_acceleration_limits(reversed, constraints, &constrainedState, error_handler)) {
        return Trajectory{};
      }

      if (ds < kEpsilonLength) {
        break;
      }

      units::Acceleration actualAcceleration =
          (constrainedState.max_velocity * constrainedState.max_velocity -
           predecessor.max_velocity * predecessor.max_velocity) /
          (ds * 2.0);

      if (constrainedState.max_acceleration < actualAcceleration - kAccelTolerance) {
        predecessor.max_acceleration = constrainedState.max_acceleration;
      } else {
        if (actualAcceleration > predecessor.min_acceleration + kAccelTolerance) {
          predecessor.max_acceleration = actualAcceleration;
        }
        break;
      }
    }
    predecessor = constrainedState;
  }

  ConstrainedState successor;
  successor.pose = points.back();
  successor.distance = constrainedStates.back().distance;
  successor.max_velocity = end_velocity;
  successor.min_acceleration = -max_acceleration;
  successor.max_acceleration = max_acceleration;

  for (int i = static_cast<int>(points.size()) - 1; i >= 0; --i) {
    auto& constrainedState = constrainedStates[static_cast<size_t>(i)];
    units::Length ds = constrainedState.distance - successor.distance;

    while (true) {
      const double successor_velocity_sq = (
          successor.max_velocity * successor.max_velocity +
          successor.min_acceleration * ds * 2.0).internal();
      units::Velocity newMaxVelocity = units::Velocity(
          std::sqrt(std::max(0.0, successor_velocity_sq)));

      if (newMaxVelocity >= constrainedState.max_velocity) {
        break;
      }

      constrainedState.max_velocity = newMaxVelocity;

      if (!enforce_acceleration_limits(reversed, constraints, &constrainedState, error_handler)) {
        return Trajectory{};
      }

      if (ds > -kEpsilonLength) {
        break;
      }

      units::Acceleration actualAcceleration =
          (constrainedState.max_velocity * constrainedState.max_velocity -
           successor.max_velocity * successor.max_velocity) /
          (ds * 2.0);
      if (constrainedState.min_acceleration > actualAcceleration + kAccelTolerance) {
        successor.min_acceleration = constrainedState.min_acceleration;
      } else {
        successor.min_acceleration = actualAcceleration;
        break;
      }
    }
    successor = constrainedState;
  }

  std::vector<Trajectory::State> states(points.size());
  units::Time t = 0_s;
  units::Length s = 0_in;
  units::Velocity v = 0_inps;

  for (size_t i = 0; i < constrainedStates.size(); ++i) {
    auto state = constrainedStates[i];

    units::Length ds = state.distance - s;
    units::Acceleration accel = 0_inps2;
    units::Time dt = 0_s;

    if (i > 0) {
      if (abs(ds) > kEpsilonLength) {
        accel = (state.max_velocity * state.max_velocity - v * v) / (ds * 2.0);
      }
      states[i - 1].acceleration = reversed ? -accel : accel;
      if (abs(accel) > kAccelTolerance) {
        dt = (state.max_velocity - v) / accel;
      } else if (abs(v) > kVelocityTolerance) {
        dt = ds / v;
      } else {
        if (abs(ds) > kEpsilonLength) {
          if (error_handler) {
            error_handler("TrajectoryParameterizer: time parameterization failed.");
          } else {
            std::fprintf(stderr, "TrajectoryParameterizer: time parameterization failed.\n");
          }
          return Trajectory{};
        }
      }
    }

    v = state.max_velocity;
    s = state.distance;

    t += dt;

    states[i] = {t, reversed ? -v : v, 0_inps2,
                 state.pose.pose, state.pose.curvature, s};
  }

  if (states.size() >= 2) {
    states.back().acceleration = states[states.size() - 2].acceleration;
  }

  return Trajectory(states);
}

bool TrajectoryParameterizer::enforce_acceleration_limits(
    bool reverse,
    const std::vector<std::unique_ptr<TrajectoryConstraint>>& constraints,
    ConstrainedState* state,
    const std::function<void(const char*)>& error_handler) {
  for (auto&& constraint : constraints) {
    double factor = reverse ? -1.0 : 1.0;

    auto minMaxAccel = constraint->min_max_acceleration(
        state->pose.pose, state->pose.curvature, state->max_velocity * factor);

    if (minMaxAccel.min_acceleration > minMaxAccel.max_acceleration) {
      if (error_handler) {
        error_handler("TrajectoryParameterizer: infeasible trajectory constraint.");
      } else {
        std::fprintf(
            stderr,
            "TrajectoryParameterizer: infeasible trajectory constraint.\n");
      }
      return false;
    }

    state->min_acceleration = std::max(
        state->min_acceleration,
        reverse ? -minMaxAccel.max_acceleration : minMaxAccel.min_acceleration);

    state->max_acceleration = std::min(
        state->max_acceleration,
        reverse ? -minMaxAccel.min_acceleration : minMaxAccel.max_acceleration);
  }

  return true;
}


