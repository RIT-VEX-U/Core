#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "core/units/units.h"
#include "core/utils/math/geometry/pose2d.h"
#include "core/utils/math_util.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"
#include "core/utils/trajectory/constraints/centripetal_acceleration_constraint.h"
#include "core/utils/trajectory/constraints/max_velocity_constraint.h"
#include "core/utils/trajectory/constraints/tank_kinematics_constraint.h"
#include "core/utils/trajectory/constraints/tank_voltage_constraint.h"

/**
 * @brief Represents a time-parameterized 2D motion trajectory.
 *
 * Stores discrete State samples (time, velocity, acceleration, pose, curvature)
 * along a parameterized path.
 */
class Trajectory {
 public:
  /**
   * @brief Represents a single discrete state sample along a trajectory.
   */
  struct State {
    Time t = 0_s;                        ///< Elapsed time from trajectory start
    Velocity velocity = 0_inps;          ///< Linear chassis velocity
    Acceleration acceleration = 0_inps2;  ///< Linear chassis acceleration
    Pose2d pose{0.0, 0.0, 0.0};           ///< 2D robot pose (x, y, theta)
    Curvature curvature = 0_radpm;       ///< Path curvature at pose

    /** @brief Default constructor. */
    State() = default;

    /**
     * @brief Constructs a Trajectory State with explicit values.
     * @param t Sample time.
     * @param velocity Linear velocity.
     * @param acceleration Linear acceleration.
     * @param pose 2D pose.
     * @param curvature Path curvature.
     */
    State(Time t, Velocity velocity, Acceleration acceleration, Pose2d pose, Curvature curvature)
        : t(t), velocity(velocity), acceleration(acceleration), pose(pose), curvature(curvature) {}

    /** @brief Checks equality between two States. */
    bool operator==(const State &other) const {
      return t == other.t && velocity == other.velocity && acceleration == other.acceleration && pose == other.pose &&
             curvature == other.curvature;
    }

    /**
     * @brief Linearly interpolates between this state and end_value at fraction i ∈ [0, 1].
     * @param end_value Target state for interpolation.
     * @param i Interpolation fraction ratio [0, 1].
     * @return Interpolated State.
     */
    State interpolate(State end_value, double i) const {
      const Time new_t = 1_s * (t.s() + (end_value.t.s() - t.s()) * i);
      const Time delta_t = new_t - t;

      if (delta_t < 0_s) {
        return end_value.interpolate(*this, 1.0 - i);
      }

      const bool reversing = velocity < 0_inps || (abs(velocity) < 1E-9_inps && acceleration < 0_inps2);
      const Velocity new_v = velocity + (acceleration * delta_t);
      const Length new_s =
        (velocity * delta_t + 0.5 * acceleration * delta_t * delta_t) * (reversing ? -1.0 : 1.0);

      const Length distance = Length(end_value.pose.translation().distance(pose.translation()));
      const double interpolation_frac = distance > 1E-9_in ? (new_s / distance).value() : i;

      Translation2d new_trans(
        pose.x() + (end_value.pose.x() - pose.x()) * interpolation_frac,
        pose.y() + (end_value.pose.y() - pose.y()) * interpolation_frac
      );
      double start_rot = pose.rotation().radians();
      double end_rot = end_value.pose.rotation().radians();
      Rotation2d new_rot(start_rot + (end_rot - start_rot) * interpolation_frac);
      Pose2d new_pose(new_trans, new_rot);

      Curvature new_curvature = 1_radpm * (
        curvature.radpm() + (end_value.curvature.radpm() - curvature.radpm()) * interpolation_frac
      );

      return State(new_t, new_v, acceleration, new_pose, new_curvature);
    }
  };

  /** @brief Default constructor. */
  Trajectory() = default;

  /**
   * @brief Constructs a Trajectory from a vector of State samples.
   * @param states Vector of time-parameterized states.
   */
  explicit Trajectory(std::vector<State> states) : m_states(std::move(states)) {
    if (!m_states.empty()) {
      m_total_time = m_states.back().t;
    }
  }

  /** @return True if the trajectory contains no state samples. */
  bool empty() const { return m_states.empty(); }

  /** @return Total duration of the trajectory. */
  Time total_time() const { return m_total_time; }

  /** @return Reference to the internal vector of State samples. */
  const std::vector<State> &states() const { return m_states; }

  /**
   * @brief Samples trajectory state at timestamp t using binary search interpolation.
   * @param t Target timestamp.
   * @return Interpolated State struct.
   */
  State sample(Time t) const {
    if (m_states.empty()) {
      return State{};
    }

    if (t <= m_states.front().t) {
      return m_states.front();
    }
    if (t >= m_total_time) {
      return m_states.back();
    }

    auto sample = std::lower_bound(
      m_states.cbegin() + 1,
      m_states.cend(),
      t,
      [](const State &a, const Time &b) { return a.t < b; });

    auto prev_sample = sample - 1;

    if (abs(sample->t - prev_sample->t) < 1E-9_s) {
      return *sample;
    }

    return prev_sample->interpolate(*sample, ((t - prev_sample->t) / (sample->t - prev_sample->t)).value());
  }

  /**
   * @brief Transforms all poses in the trajectory by a 2D affine transform.
   * @param transform Rigid 2D transformation.
   * @return Transformed Trajectory.
   */
  Trajectory transform_by(const Transform2d &transform) const {
    if (m_states.empty()) {
      return *this;
    }

    auto &first_state = m_states[0];
    auto &first_pose = first_state.pose;

    auto new_first_pose = first_pose + transform;
    auto new_states = m_states;
    new_states[0].pose = new_first_pose;

    for (size_t i = 1; i < new_states.size(); ++i) {
      auto &state = new_states[i];
      state.pose = new_first_pose + (state.pose - first_pose);
    }

    return Trajectory(new_states);
  }

  /**
   * @brief Expresses all poses in the trajectory relative to a reference pose.
   * @param pose Reference origin pose.
   * @return Relative Trajectory.
   */
  Trajectory relative_to(const Pose2d &pose) const {
    auto new_states = m_states;
    for (auto &state : new_states) {
      state.pose = state.pose.relative_to(pose);
    }
    return Trajectory(new_states);
  }

  /**
   * @brief Concatenates two trajectories end-to-end.
   * @param other Secondary trajectory to append.
   * @return Combined Trajectory.
   */
  Trajectory operator+(const Trajectory &other) const {
    if (m_states.empty()) {
      return other;
    }

    auto states = m_states;
    auto other_states = other.states();
    for (auto &other_state : other_states) {
      other_state.t += m_total_time;
    }

    states.insert(states.end(), other_states.begin() + 1, other_states.end());
    return Trajectory(states);
  }

  /** @return Initial pose at t = 0s. */
  Pose2d initial_pose() const { return sample(0_s).pose; }

  /** @brief Checks equality between two trajectories. */
  bool operator==(const Trajectory &other) const {
    return m_total_time == other.m_total_time && m_states == other.m_states;
  }

 private:
  std::vector<State> m_states;
  Time m_total_time = 0_s;
};

/**
 * @brief High-efficiency streaming sampler for real-time control loops.
 *
 * Caches the search index during sequential calls to sample(Time t),
 * achieving O(1) step lookup in 100Hz loop cycles.
 */
class TrajectorySampler {
 public:
  /**
   * @brief Constructs a TrajectorySampler bound to a target Trajectory.
   * @param trajectory Reference to trajectory to sample.
   */
  explicit TrajectorySampler(const Trajectory &trajectory)
      : m_trajectory(&trajectory), m_cached_index(0) {}

  /** @brief Resets cached index back to start. */
  void reset() { m_cached_index = 0; }

  /**
   * @brief Samples state at timestamp t using cached sequential lookup (O(1)).
   * @param t Target timestamp.
   * @return Interpolated State.
   */
  Trajectory::State sample(Time t) {
    if (!m_trajectory || m_trajectory->empty()) {
      return Trajectory::State{};
    }
    const auto &states = m_trajectory->states();
    if (t <= states.front().t) {
      m_cached_index = 0;
      return states.front();
    }
    if (t >= m_trajectory->total_time()) {
      m_cached_index = states.size() - 1;
      return states.back();
    }

    while (m_cached_index + 1 < states.size() && states[m_cached_index + 1].t <= t) {
      m_cached_index++;
    }
    while (m_cached_index > 0 && states[m_cached_index].t > t) {
      m_cached_index--;
    }

    if (m_cached_index + 1 >= states.size()) {
      return states.back();
    }

    const auto &prev = states[m_cached_index];
    const auto &next = states[m_cached_index + 1];
    if (abs(next.t - prev.t) < 1E-9_s) {
      return next;
    }
    return prev.interpolate(next, ((t - prev.t) / (next.t - prev.t)).value());
  }

 private:
  const Trajectory *m_trajectory = nullptr;
  size_t m_cached_index = 0;
};
