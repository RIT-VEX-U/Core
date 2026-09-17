#pragma once

#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include <functional>
#include <string>

#include "core/utils/units.h"
#include "core/utils/math/spline/spline_path.h"
#include "core/utils/trajectory/constraints/tank_kinematics_constraint.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"

using namespace units::literals;

/**
 * @brief Configuration parameters and constraint container for generating trajectories.
 */
struct DistanceEvent {
  units::Length distance;
  std::string name;
};

class TrajectoryConfig {
 public:
  /**
   * @brief Constructs TrajectoryConfig with maximum velocity and acceleration.
   * @param max_velocity Maximum physical chassis velocity limit.
   * @param max_acceleration Maximum physical chassis acceleration limit.
   */
  TrajectoryConfig(units::Velocity max_velocity, units::Acceleration max_acceleration)
      : max_velocity_(max_velocity), max_acceleration_(max_acceleration) {}

  /**
   * @brief Constructs TrajectoryConfig with max velocity, max acceleration, and direction.
   * @param max_velocity Max velocity limit.
   * @param max_acceleration Max acceleration limit.
   * @param reversed True to generate path in reverse direction.
   */
  TrajectoryConfig(units::Velocity max_velocity, units::Acceleration max_acceleration, bool reversed)
      : TrajectoryConfig(max_velocity, max_acceleration) {
    set_reversed(reversed);
  }

  /**
   * @brief Constructs TrajectoryConfig with max velocity, max acceleration, and track width constraint.
   * @param max_velocity Max velocity limit.
   * @param max_acceleration Max acceleration limit.
   * @param track_width Drivetrain track width for kinematics constraints.
   */
  TrajectoryConfig(units::Velocity max_velocity, units::Acceleration max_acceleration, units::Length track_width)
      : TrajectoryConfig(max_velocity, max_acceleration) {
    set_track_width(track_width);
  }

  /**
   * @brief Constructs TrajectoryConfig with start/end boundary velocities.
   * @param max_velocity Max velocity limit.
   * @param max_acceleration Max acceleration limit.
   * @param start_velocity Initial trajectory velocity.
   * @param end_velocity Final trajectory velocity.
   * @param reversed Direction flag.
   */
  TrajectoryConfig(
      units::Velocity max_velocity,
      units::Acceleration max_acceleration,
      units::Velocity start_velocity,
      units::Velocity end_velocity,
      bool reversed)
      : TrajectoryConfig(max_velocity, max_acceleration, reversed) {
    set_start_velocity(start_velocity);
    set_end_velocity(end_velocity);
  }

  /**
   * @brief Constructs TrajectoryConfig with full parameters.
   */
  TrajectoryConfig(
      units::Velocity max_velocity,
      units::Acceleration max_acceleration,
      units::Velocity start_velocity,
      units::Velocity end_velocity,
      bool reversed,
      units::Length track_width)
      : TrajectoryConfig(max_velocity, max_acceleration, start_velocity, end_velocity, reversed) {
    set_track_width(track_width);
  }

  /** @brief Polymorphic deep-copy constructor. */
  TrajectoryConfig(const TrajectoryConfig &other)
      : start_velocity_(other.start_velocity_),
        end_velocity_(other.end_velocity_),
        max_velocity_(other.max_velocity_),
        max_acceleration_(other.max_acceleration_),
        sample_ds_(other.sample_ds_),
        events_(other.events_),
        reversed_(other.reversed_) {
    constraints_.reserve(other.constraints_.size());
    for (const auto &c : other.constraints_) {
      if (c) {
        constraints_.push_back(c->clone());
      }
    }
      events_ = other.events_;
  }

  /** @brief Polymorphic deep-copy assignment operator. */
  TrajectoryConfig &operator=(const TrajectoryConfig &other) {
    if (this != &other) {
      start_velocity_ = other.start_velocity_;
      end_velocity_ = other.end_velocity_;
      max_velocity_ = other.max_velocity_;
      max_acceleration_ = other.max_acceleration_;
      sample_ds_ = other.sample_ds_;
      reversed_ = other.reversed_;
      constraints_.clear();
      constraints_.reserve(other.constraints_.size());
      for (const auto &c : other.constraints_) {
        if (c) {
          constraints_.push_back(c->clone());
        }
      }
      events_ = other.events_;
    }
    return *this;
  }

  /** @brief Move constructor. */
  TrajectoryConfig(TrajectoryConfig &&) = default;
  /** @brief Move assignment. */
  TrajectoryConfig &operator=(TrajectoryConfig &&) = default;

  /** @brief Sets initial trajectory velocity. */
  void add_event(const std::string& name, units::Length distance) { events_.push_back({distance, name}); }
  void set_start_velocity(units::Velocity start_velocity) { start_velocity_ = start_velocity; }

  /** @brief Sets final trajectory velocity. */
  void set_end_velocity(units::Velocity end_velocity) { end_velocity_ = end_velocity; }

  /** @brief Sets direction flag (reversed = true for driving backward). */
  void set_reversed(bool reversed) { reversed_ = reversed; }

  /** @brief Sets spatial discretization step size for spline sampling. */
  void set_sample_ds(units::Length sample_ds) { sample_ds_ = sample_ds; }

  /** @brief Sets spline order. */
  void set_spline_order(SplinePath::Order order) { spline_order_ = order; }

  /** @brief Sets error handler for trajectory generation failures. */
  void set_error_handler(std::function<void(const char*)> handler) { error_handler_ = std::move(handler); }

  /**
   * @brief Adds a user-defined TrajectoryConstraint.
   * @tparam Constraint Constraint class inheriting from TrajectoryConstraint.
   * @param constraint Constraint instance to add.
   */
  template <typename Constraint>
  typename std::enable_if<std::is_base_of<TrajectoryConstraint, typename std::decay<Constraint>::type>::value, void>::type
  add_constraint(Constraint &&constraint) {
    typedef typename std::decay<Constraint>::type C;
    constraints_.emplace_back(std::unique_ptr<TrajectoryConstraint>(new C(std::forward<Constraint>(constraint))));
  }

  /** @brief Adds a TankKinematicsConstraint using track width and max velocity. */
  void set_track_width(units::Length track_width) { add_constraint(TankKinematicsConstraint(track_width, max_velocity_)); }

  /** @return Initial trajectory velocity. */
  units::Velocity start_velocity() const { return start_velocity_; }

  /** @return Final trajectory velocity. */
  units::Velocity end_velocity() const { return end_velocity_; }

  /** @return Maximum physical velocity limit. */
  units::Velocity max_velocity() const { return max_velocity_; }

  /** @return Maximum physical acceleration limit. */
  units::Acceleration max_acceleration() const { return max_acceleration_; }

  /** @return Spatial sampling step size for path generation. */
  units::Length sample_ds() const { return sample_ds_; }

  /** @return Vector of polymorphic trajectory constraint pointers. */
  const std::vector<std::unique_ptr<TrajectoryConstraint>> &constraints() const { return constraints_; }
  const std::vector<DistanceEvent>& events() const { return events_; }

  /** @return True if trajectory is driven in reverse. */
  bool is_reversed() const { return reversed_; }

  /** @return Spline order used for path generation. */
  SplinePath::Order spline_order() const { return spline_order_; }

  /** @return Maximum jerk limit for S-curve generation. Returns 0_inps3 if disabled. */
  units::Jerk max_jerk() const { return max_jerk_; }

  /** @brief Sets maximum jerk limit. Set to 0 to disable (pure trapezoidal). */
  void set_max_jerk(units::Jerk jerk) { max_jerk_ = jerk; }

  /** @return Error handler callback. */
  const std::function<void(const char*)>& error_handler() const { return error_handler_; }

 private:
  units::Velocity start_velocity_ = 0_inps;
  units::Velocity end_velocity_ = 0_inps;
  units::Velocity max_velocity_;
  units::Acceleration max_acceleration_;
  units::Jerk max_jerk_ = 0_inps3;
  units::Length sample_ds_ = 0.5_in;
  std::vector<std::unique_ptr<TrajectoryConstraint>> constraints_;
  std::vector<DistanceEvent> events_;
  bool reversed_ = false;
  SplinePath::Order spline_order_ = SplinePath::Order::Quintic;
  std::function<void(const char*)> error_handler_;
};

/**
 * @brief Fluent Builder API for constructing TrajectoryConfig instances cleanly.
 */
class TrajectoryConfigBuilder {
 public:
  /**
   * @brief Creates a builder with max velocity and max acceleration limits.
   */
  TrajectoryConfigBuilder(units::Velocity max_velocity, units::Acceleration max_acceleration)
      : config_(max_velocity, max_acceleration) {}

  /** @brief Configures initial trajectory velocity. */
  TrajectoryConfigBuilder &with_start_velocity(units::Velocity v) {
    config_.set_start_velocity(v);
    return *this;
  }

  /** @brief Configures final trajectory velocity. */
  TrajectoryConfigBuilder &with_end_velocity(units::Velocity v) {
    config_.set_end_velocity(v);
    return *this;
  }

  /** @brief Configures reverse driving flag. */
  TrajectoryConfigBuilder &with_reversed(bool reversed = true) {
    config_.set_reversed(reversed);
    return *this;
  }

  /** @brief Configures spatial spline discretization step size. */
  TrajectoryConfigBuilder &with_sample_ds(units::Length ds) {
    config_.set_sample_ds(ds);
    return *this;
  }

  /** @brief Configures drivetrain track width kinematics constraint. */
  TrajectoryConfigBuilder &with_track_width(units::Length track_width) {
    config_.set_track_width(track_width);
    return *this;
  }

  /** @brief Configures jerk limit for S-curve generation. */
  TrajectoryConfigBuilder &with_max_jerk(units::Jerk max_jerk) {
    config_.set_max_jerk(max_jerk);
    return *this;
  }

  /** @brief Configures spline order. */
  TrajectoryConfigBuilder &with_spline_order(SplinePath::Order order) {
    config_.set_spline_order(order);
    return *this;
  }

  /** @brief Configures custom error handler callback. */
  TrajectoryConfigBuilder &with_error_handler(std::function<void(const char*)> handler) {
    config_.set_error_handler(std::move(handler));
    return *this;
  }

  /** @brief Adds a custom physical trajectory constraint. */
  template <typename Constraint>
  TrajectoryConfigBuilder &with_constraint(Constraint &&constraint) {
    config_.add_constraint(std::forward<Constraint>(constraint));
    return *this;
  }

  /** @brief Builds and returns constructed TrajectoryConfig instance. */
  TrajectoryConfigBuilder &with_event(const std::string& name, units::Length distance) {
    config_.add_event(name, distance);
    return *this;
  }
  TrajectoryConfig build() { return std::move(config_); }

 private:
  TrajectoryConfig config_;
};
