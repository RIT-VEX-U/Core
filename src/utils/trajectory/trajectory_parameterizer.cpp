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
    bool reversed,
    std::function<void(const char*)> error_handler) {
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

    Length ds = Length::from<inch_tag>(constrainedState.pose.pose.translation().distance(
        predecessor.pose.pose.translation()));
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
            constraint->max_velocity(constrainedState.pose.pose,
                                    constrainedState.pose.curvature,
                                    constrainedState.maxVelocity));
      }

      if (!enforce_acceleration_limits(reversed, constraints, &constrainedState, error_handler)) {
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

      if (!enforce_acceleration_limits(reversed, constraints, &constrainedState, error_handler)) {
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
          if (error_handler) {
            error_handler("TrajectoryParameterizer: time parameterization failed.");
          } else {
            std::fprintf(stderr, "TrajectoryParameterizer: time parameterization failed.\n");
          }
          return Trajectory{};
        }
      }
    }

    v = state.maxVelocity;
    s = state.distance;

    t += dt;

    states[i] = {t, reversed ? -v : v, 0_inps2,
                 state.pose.pose, state.pose.curvature};
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
        state->pose.pose, state->pose.curvature, state->maxVelocity * factor);

    if (minMaxAccel.minAcceleration > minMaxAccel.maxAcceleration) {
      if (error_handler) {
        error_handler("TrajectoryParameterizer: infeasible trajectory constraint.");
      } else {
        std::fprintf(
            stderr,
            "TrajectoryParameterizer: infeasible trajectory constraint.\n");
      }
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

Trajectory TrajectoryParameterizer::jerk_limit_trajectory(
    const Trajectory& base,
    Acceleration max_acceleration,
    Jerk max_jerk) {
  if (base.empty() || max_jerk <= 0_inps3) {
    return base;
  }

  Time t_window = max_acceleration / max_jerk;
  if (t_window < 10_ms) {
    return base;
  }

  Time dt = 10_ms;
  Time old_time = base.total_time();
  Time new_time = old_time + t_window;

  int num_samples = static_cast<int>(std::ceil((new_time / dt).value())) + 1;
  std::vector<Trajectory::State> new_states;
  new_states.reserve(num_samples);

  const auto& base_states = base.states();
  std::vector<Length> base_s(base_states.size());
  base_s[0] = 0_in;
  for (size_t i = 1; i < base_states.size(); ++i) {
    base_s[i] = base_s[i - 1] + Length::from<inch_tag>(
        base_states[i].pose.translation().distance(base_states[i - 1].pose.translation()));
  }
  Length total_s = base_s.back();

  Velocity current_v = 0_inps;
  Length current_s = 0_in;

  for (int i = 0; i < num_samples; ++i) {
    Time t = i * dt;
    Time t_start = t - t_window;
    
    const int sub_samples = 50;
    Time sub_dt = t_window / sub_samples;
    Velocity v_sum = 0_inps;
    
    for (int j = 0; j <= sub_samples; ++j) {
      Time sample_t = t_start + (j * sub_dt);
      if (sample_t >= 0_s && sample_t <= old_time) {
        v_sum += base.sample(sample_t).velocity;
      } else if (sample_t > old_time) {
        v_sum += base_states.back().velocity;
      }
    }
    
    Velocity v_avg = v_sum / (sub_samples + 1);

    if (i > 0) {
      current_s += abs((current_v + v_avg) / 2.0 * dt);
    }
    
    if (current_s > total_s) {
      current_s = total_s;
    }
    
    Time t_base = 0_s;
    auto upper = std::upper_bound(base_s.begin(), base_s.end(), current_s);
    if (upper == base_s.end()) {
      t_base = old_time;
    } else if (upper == base_s.begin()) {
      t_base = 0_s;
    } else {
      size_t idx = std::distance(base_s.begin(), upper);
      Length s0 = base_s[idx - 1];
      Length s1 = base_s[idx];
      Time t0 = base_states[idx - 1].t;
      Time t1 = base_states[idx].t;
      
      double alpha = (s1 == s0) ? 0.0 : ((current_s - s0) / (s1 - s0)).value();
      t_base = t0 + (t1 - t0) * alpha;
    }
    
    Trajectory::State mapped_state = base.sample(t_base);
    
    Acceleration a = 0_inps2;
    if (i > 0) {
      a = (v_avg - current_v) / dt;
      new_states.back().acceleration = a;
    }
    
    new_states.push_back({t, v_avg, a, mapped_state.pose, mapped_state.curvature});
    current_v = v_avg;
  }
  
  if (new_states.size() >= 2) {
    new_states.back().acceleration = new_states[new_states.size() - 2].acceleration;
  }
  
  return Trajectory(new_states);
}
