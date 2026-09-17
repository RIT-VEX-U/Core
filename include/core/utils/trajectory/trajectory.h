#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "core/utils/units.h"
#include "core/utils/math/geometry/pose2d.h"
#include "core/utils/math_util.h"

using namespace units::literals;

/**
 * @brief Pair representing 2D Pose and scalar path units::Curvature.
 */
struct PoseWithCurvature {
  Pose2d pose;
  units::Curvature curvature;
};

/**
 * @brief Represents an action event triggered at a specific time along a trajectory.
 */
struct TrajectoryEvent {
  units::Time time;
  std::string name;
};

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
    units::Time t = 0_s;                        ///< Elapsed time from trajectory start
    units::Length s = 0_in;                     ///< Distance along trajectory
    units::Velocity velocity = 0_inps;          ///< Linear chassis velocity
    units::Acceleration acceleration = 0_inps2;  ///< Linear chassis acceleration
    Pose2d pose{0.0, 0.0, 0.0};           ///< 2D robot pose (x, y, theta)
    units::Curvature curvature = 0_radpm;       ///< Path curvature at pose

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
    State(units::Time t, units::Velocity velocity, units::Acceleration acceleration, Pose2d pose, units::Curvature curvature, units::Length s = 0_in)
        : t(t), s(s), velocity(velocity), acceleration(acceleration), pose(pose), curvature(curvature) {}

    /** @brief Checks equality between two States. */
    bool operator==(const State &other) const {
      return t == other.t && velocity == other.velocity && acceleration == other.acceleration && pose == other.pose &&
             curvature == other.curvature;
    }

    /**
     * @brief Linearly interpolates between this state and end_value at fraction i in [0, 1].
     * @param end_value Target state for interpolation.
     * @param i Interpolation fraction ratio [0, 1].
     * @return Interpolated State.
     */
    State interpolate(State end_value, double i) const {
      const units::Time new_t = 1_s * (t.internal() + (end_value.t.internal() - t.internal()) * i);
      const units::Time delta_t = new_t - t;

      if (delta_t < 0_s) {
        return end_value.interpolate(*this, 1.0 - i);
      }

      const bool reversing = velocity < 0_inps || (abs(velocity) < 1E-9_inps && acceleration < 0_inps2);
      const units::Velocity new_v = velocity + (acceleration * delta_t);
      const units::Length new_s =
        (velocity * delta_t + 0.5 * acceleration * delta_t * delta_t) * (reversing ? -1.0 : 1.0);

      const units::Length distance = units::Length(end_value.pose.translation().distance(pose.translation()));
      const double interpolation_frac = distance > 1E-9_in ? (new_s / distance).internal() : i;

      Translation2d new_trans(
        pose.x() + (end_value.pose.x() - pose.x()) * interpolation_frac,
        pose.y() + (end_value.pose.y() - pose.y()) * interpolation_frac
      );
      double start_rot = pose.rotation().radians();
      double end_rot = end_value.pose.rotation().radians();
      Rotation2d new_rot(start_rot + (end_rot - start_rot) * interpolation_frac);
      Pose2d new_pose(new_trans, new_rot);

      units::Curvature new_curvature = 1_radpm * (
        curvature.internal() + (end_value.curvature.internal() - curvature.internal()) * interpolation_frac
      );

      return State(new_t, new_v, acceleration, new_pose, new_curvature, s + new_s);
    }
  };

  /** @brief Default constructor. */
  Trajectory() = default;

  /**
   * @brief Constructs a Trajectory from a vector of State samples.
   * @param states Vector of time-parameterized states.
   */
  explicit Trajectory(std::vector<State> states) : states_(std::move(states)) {
    if (!states_.empty()) {
      total_time_ = states_.back().t;
    }
  }

  /** @return True if the trajectory contains no state samples. */
  bool empty() const { return states_.empty(); }

  /** @return Total duration of the trajectory. */
  units::Time total_time() const { return total_time_; }

  /** @return Reference to the internal vector of State samples. */
  const std::vector<State> &states() const { return states_; }

  /** @return Vector of events attached to this trajectory. */
  const std::vector<TrajectoryEvent>& events() const { return events_; }

  /** @brief Attaches a vector of time-parameterized events to the trajectory. */
  void set_events(std::vector<TrajectoryEvent> events) { events_ = std::move(events); }

  /**
   * @brief Interpolates the time at which a given arc distance is reached.
   * @param s Arc distance along trajectory.
   * @return Interpolated time.
   */
  units::Time time_from_distance(units::Length s) const {
    if (states_.empty() || s <= 0_in) return 0_s;
    if (s >= states_.back().s) return total_time_;
    
    auto it = std::lower_bound(states_.begin() + 1, states_.end(), s, 
      [](const State& st, units::Length val) { return st.s < val; });
      
    const State& s0 = *(it - 1);
    const State& s1 = *it;
    
    if (s1.s == s0.s) return s1.t;
    double alpha = (s - s0.s).internal() / (s1.s - s0.s).internal();
    return s0.t + (s1.t - s0.t) * alpha;
  }

  /**
   * @brief Samples trajectory state at timestamp t using binary search interpolation.
   * @param t Target timestamp.
   * @return Interpolated State struct.
   */
  State sample(units::Time t) const {
    if (states_.empty()) {
      return State{};
    }

    if (t <= states_.front().t) {
      return states_.front();
    }
    if (t >= total_time_) {
      return states_.back();
    }

    auto sample = std::lower_bound(
      states_.cbegin() + 1,
      states_.cend(),
      t,
      [](const State &a, const units::Time &b) { return a.t < b; });

    auto prev_sample = sample - 1;

    if (abs(sample->t - prev_sample->t) < 1E-9_s) {
      return *sample;
    }

    return prev_sample->interpolate(*sample, ((t - prev_sample->t) / (sample->t - prev_sample->t)).internal());
  }

  /**
   * @brief Transforms all poses in the trajectory by a 2D affine transform.
   * @param transform Rigid 2D transformation.
   * @return Transformed Trajectory.
   */
  Trajectory transform_by(const Transform2d &transform) const {
    if (states_.empty()) {
      return *this;
    }

    auto &first_state = states_[0];
    auto &first_pose = first_state.pose;

    auto new_first_pose = first_pose + transform;
    auto new_states = states_;
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
    auto new_states = states_;
    for (auto &state : new_states) {
      state.pose = state.pose.relative_to(pose);
    }
    return Trajectory(new_states);
  }

  /**
   * @brief Returns a time-reversed version of this trajectory (traces the path from end to start).
   * @return Reversed Trajectory.
   */
  Trajectory reverse() const {
    std::vector<State> new_states;
    new_states.reserve(states_.size());
    for (auto it = states_.rbegin(); it != states_.rend(); ++it) {
      State st = *it;
      st.t = total_time_ - st.t;
      st.velocity = -st.velocity;
      st.curvature = -st.curvature;
      st.s = states_.back().s - st.s;
      new_states.push_back(st);
    }
    return Trajectory(new_states);
  }

  /**
   * @brief Concatenates two trajectories end-to-end.
   * @param other Secondary trajectory to append.
   * @return Combined Trajectory.
   */
  Trajectory operator+(const Trajectory &other) const {
    if (states_.empty()) {
      return other;
    }

    auto states = states_;
    auto other_states = other.states();
    for (auto &other_state : other_states) {
      other_state.t += total_time_;
    }

    states.insert(states.end(), other_states.begin() + 1, other_states.end());
    return Trajectory(states);
  }

  /** @return Initial pose at t = 0s. */
  Pose2d initial_pose() const { return sample(0_s).pose; }

  /** @brief Checks equality between two trajectories. */
  bool operator==(const Trajectory &other) const {
    return total_time_ == other.total_time_ && states_ == other.states_;
  }

 private:
  std::vector<State> states_;
  std::vector<TrajectoryEvent> events_;
  units::Time total_time_ = 0_s;
};

/**
 * @brief High-efficiency streaming sampler for real-time control loops.
 *
 * Caches the search index during sequential calls to sample(units::Time t),
 * achieving O(1) step lookup in 100Hz loop cycles.
 */
class TrajectorySampler {
 public:
  /**
   * @brief Constructs a TrajectorySampler bound to a target Trajectory.
   * 
   * @warning The TrajectorySampler holds a non-owning pointer to the trajectory. 
   * The provided Trajectory object MUST outlive the sampler. Do not pass temporary objects.
   * 
   * @param trajectory Reference to trajectory to sample.
   */
  explicit TrajectorySampler(const Trajectory &trajectory)
      : trajectory_(&trajectory), cached_index_(0) {}

  /** @brief Resets cached index back to start. */
  void reset() { cached_index_ = 0; }

  /**
   * @brief Samples state at timestamp t using cached sequential lookup (O(1)).
   * @param t Target timestamp.
   * @return Interpolated State.
   */
  Trajectory::State sample(units::Time t) {
    if (!trajectory_ || trajectory_->empty()) {
      return Trajectory::State{};
    }
    const auto &states = trajectory_->states();
    if (t <= states.front().t) {
      cached_index_ = 0;
      return states.front();
    }
    if (t >= trajectory_->total_time()) {
      cached_index_ = states.size() - 1;
      return states.back();
    }

    while (cached_index_ + 1 < states.size() && states[cached_index_ + 1].t <= t) {
      cached_index_++;
    }
    while (cached_index_ > 0 && states[cached_index_].t > t) {
      cached_index_--;
    }

    if (cached_index_ + 1 >= states.size()) {
      return states.back();
    }

    const auto &prev = states[cached_index_];
    const auto &next = states[cached_index_ + 1];
    if (abs(next.t - prev.t) < 1E-9_s) {
      return next;
    }
    return prev.interpolate(next, ((t - prev.t) / (next.t - prev.t)).internal());
  }

 private:
  const Trajectory *trajectory_ = nullptr;
  size_t cached_index_ = 0;
};
