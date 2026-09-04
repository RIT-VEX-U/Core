#include "core/utils/trajectory/trajectory_parameterizer.h"

#include <cstdio>
#include <cmath>
#include <vector>

Trajectory TrajectoryParameterizer::time_parameterize_trajectory(
    const std::vector<PoseWithCurvature>& points,
    const std::vector<std::unique_ptr<TrajectoryConstraint>>& constraints,
    Velocity start_velocity,
    Velocity end_velocity,
    Velocity max_velocity,
    Acceleration max_acceleration,
    bool reversed) {
  if (points.empty()) {
    return Trajectory{};
  }

  const Length kEpsilonLength = Length::from<inch_tag>(kEpsilon);
  const Acceleration kAccelTolerance = Acceleration::from<inches_per_second_squared_tag>(1E-6);
  const Velocity kVelocityTolerance = Velocity::from<inches_per_second_tag>(1E-6);

  std::vector<ConstrainedState> constrainedStates(points.size());

  ConstrainedState predecessor;
  predecessor.pose = points.front();
  predecessor.distance = 0_in;
  predecessor.maxVelocity = start_velocity;
  predecessor.minAcceleration = -max_acceleration;
  predecessor.maxAcceleration = max_acceleration;

  constrainedStates[0] = predecessor;

  for (size_t i = 0; i < points.size(); ++i) {
    auto& constrainedState = constrainedStates[i];
    constrainedState.pose = points[i];

    Length ds = Length::from<inch_tag>(constrainedState.pose.first.translation().distance(
        predecessor.pose.first.translation()));
    constrainedState.distance = ds + predecessor.distance;

    while (true) {
      const double predecessor_velocity_sq = (
          predecessor.maxVelocity * predecessor.maxVelocity +
          predecessor.maxAcceleration * ds * 2.0).canonical_value();
      constrainedState.maxVelocity = Velocity::from_canonical(
          std::min(max_velocity.canonical_value(),
                   std::sqrt(std::max(0.0, predecessor_velocity_sq))));

      constrainedState.minAcceleration = -max_acceleration;
      constrainedState.maxAcceleration = max_acceleration;

      for (const auto& constraint : constraints) {
        constrainedState.maxVelocity = std::min(
            constrainedState.maxVelocity,
            constraint->max_velocity(constrainedState.pose.first,
                                    constrainedState.pose.second,
                                    constrainedState.maxVelocity));
      }

      if (!enforce_acceleration_limits(reversed, constraints, &constrainedState)) {
        return Trajectory{};
      }

      if (ds < kEpsilonLength) {
        break;
      }

      Acceleration actualAcceleration =
          (constrainedState.maxVelocity * constrainedState.maxVelocity -
           predecessor.maxVelocity * predecessor.maxVelocity) /
          (ds * 2.0);

      if (constrainedState.maxAcceleration < actualAcceleration - kAccelTolerance) {
        predecessor.maxAcceleration = constrainedState.maxAcceleration;
      } else {
        if (actualAcceleration > predecessor.minAcceleration + kAccelTolerance) {
          predecessor.maxAcceleration = actualAcceleration;
        }
        break;
      }
    }
    predecessor = constrainedState;
  }

  ConstrainedState successor;
  successor.pose = points.back();
  successor.distance = constrainedStates.back().distance;
  successor.maxVelocity = end_velocity;
  successor.minAcceleration = -max_acceleration;
  successor.maxAcceleration = max_acceleration;

  for (int i = static_cast<int>(points.size()) - 1; i >= 0; --i) {
    auto& constrainedState = constrainedStates[static_cast<size_t>(i)];
    Length ds = constrainedState.distance - successor.distance;

    while (true) {
      const double successor_velocity_sq = (
          successor.maxVelocity * successor.maxVelocity +
          successor.minAcceleration * ds * 2.0).canonical_value();
      Velocity newMaxVelocity = Velocity::from_canonical(
          std::sqrt(std::max(0.0, successor_velocity_sq)));

      if (newMaxVelocity >= constrainedState.maxVelocity) {
        break;
      }

      constrainedState.maxVelocity = newMaxVelocity;

      if (!enforce_acceleration_limits(reversed, constraints, &constrainedState)) {
        return Trajectory{};
      }

      if (ds > -kEpsilonLength) {
        break;
      }

      Acceleration actualAcceleration =
          (constrainedState.maxVelocity * constrainedState.maxVelocity -
           successor.maxVelocity * successor.maxVelocity) /
          (ds * 2.0);
      if (constrainedState.minAcceleration > actualAcceleration + kAccelTolerance) {
        successor.minAcceleration = constrainedState.minAcceleration;
      } else {
        successor.minAcceleration = actualAcceleration;
        break;
      }
    }
    successor = constrainedState;
  }

  std::vector<Trajectory::State> states(points.size());
  Time t = 0_s;
  Length s = 0_in;
  Velocity v = 0_inps;

  for (size_t i = 0; i < constrainedStates.size(); ++i) {
    auto state = constrainedStates[i];

    Length ds = state.distance - s;
    Acceleration accel = 0_inps2;
    Time dt = 0_s;

    if (i > 0) {
      if (abs(ds) > kEpsilonLength) {
        accel = (state.maxVelocity * state.maxVelocity - v * v) / (ds * 2.0);
      }
      states[i - 1].acceleration = reversed ? -accel : accel;
      if (abs(accel) > kAccelTolerance) {
        dt = (state.maxVelocity - v) / accel;
      } else if (abs(v) > kVelocityTolerance) {
        dt = ds / v;
      } else {
        if (abs(ds) > kEpsilonLength) {
          std::fprintf(stderr, "TrajectoryParameterizer: time parameterization failed.\n");
          return Trajectory{};
        }
      }
    }

    v = state.maxVelocity;
    s = state.distance;

    t += dt;

    states[i] = {t, reversed ? -v : v, 0_inps2,
                 state.pose.first, state.pose.second};
  }

  if (states.size() >= 2) {
    states.back().acceleration = states[states.size() - 2].acceleration;
  }

  return Trajectory(states);
}

bool TrajectoryParameterizer::enforce_acceleration_limits(
    bool reverse,
    const std::vector<std::unique_ptr<TrajectoryConstraint>>& constraints,
    ConstrainedState* state) {
  for (auto&& constraint : constraints) {
    double factor = reverse ? -1.0 : 1.0;

    auto minMaxAccel = constraint->min_max_acceleration(
        state->pose.first, state->pose.second, state->maxVelocity * factor);

    if (minMaxAccel.minAcceleration > minMaxAccel.maxAcceleration) {
      std::fprintf(
          stderr,
          "TrajectoryParameterizer: infeasible trajectory constraint.\n");
      return false;
    }

    state->minAcceleration = std::max(
        state->minAcceleration,
        reverse ? -minMaxAccel.maxAcceleration : minMaxAccel.minAcceleration);

    state->maxAcceleration = std::min(
        state->maxAcceleration,
        reverse ? -minMaxAccel.minAcceleration : minMaxAccel.maxAcceleration);
  }

  return true;
}
