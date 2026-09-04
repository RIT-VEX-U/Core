#pragma once

#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "core/units/units.h"
#include "core/utils/trajectory/constraints/tank_kinematics_constraint.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"


/**
 * @brief Configuration parameters and constraint container for generating trajectories.
 */
class TrajectoryConfig {
 public:
  /**
   * @brief Constructs TrajectoryConfig with maximum velocity and acceleration.
   * @param max_velocity Maximum physical chassis velocity limit.
   * @param max_acceleration Maximum physical chassis acceleration limit.
   */
  TrajectoryConfig(Velocity max_velocity, Acceleration max_acceleration)
      : m_max_velocity(max_velocity), m_max_acceleration(max_acceleration) {}

  /**
   * @brief Constructs TrajectoryConfig with max velocity, max acceleration, and direction.
   * @param max_velocity Max velocity limit.
   * @param max_acceleration Max acceleration limit.
   * @param reversed True to generate path in reverse direction.
   */
  TrajectoryConfig(Velocity max_velocity, Acceleration max_acceleration, bool reversed)
      : TrajectoryConfig(max_velocity, max_acceleration) {
    set_reversed(reversed);
  }

  /**
   * @brief Constructs TrajectoryConfig with max velocity, max acceleration, and track width constraint.
   * @param max_velocity Max velocity limit.
   * @param max_acceleration Max acceleration limit.
   * @param track_width Drivetrain track width for kinematics constraints.
   */
  TrajectoryConfig(Velocity max_velocity, Acceleration max_acceleration, Length track_width)
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
      Velocity max_velocity,
      Acceleration max_acceleration,
      Velocity start_velocity,
      Velocity end_velocity,
      bool reversed)
      : TrajectoryConfig(max_velocity, max_acceleration, reversed) {
    set_start_velocity(start_velocity);
    set_end_velocity(end_velocity);
  }

  /**
   * @brief Constructs TrajectoryConfig with full parameters.
   */
  TrajectoryConfig(
      Velocity max_velocity,
      Acceleration max_acceleration,
      Velocity start_velocity,
      Velocity end_velocity,
      bool reversed,
      Length track_width)
      : TrajectoryConfig(max_velocity, max_acceleration, start_velocity, end_velocity, reversed) {
    set_track_width(track_width);
  }

  /** @brief Polymorphic deep-copy constructor. */
  TrajectoryConfig(const TrajectoryConfig &other)
      : m_start_velocity(other.m_start_velocity),
        m_end_velocity(other.m_end_velocity),
        m_max_velocity(other.m_max_velocity),
        m_max_acceleration(other.m_max_acceleration),
        m_sample_ds(other.m_sample_ds),
        m_reversed(other.m_reversed) {
    m_constraints.reserve(other.m_constraints.size());
    for (const auto &c : other.m_constraints) {
      if (c) {
        m_constraints.push_back(c->clone());
      }
    }
  }

  /** @brief Polymorphic deep-copy assignment operator. */
  TrajectoryConfig &operator=(const TrajectoryConfig &other) {
    if (this != &other) {
      m_start_velocity = other.m_start_velocity;
      m_end_velocity = other.m_end_velocity;
      m_max_velocity = other.m_max_velocity;
      m_max_acceleration = other.m_max_acceleration;
      m_sample_ds = other.m_sample_ds;
      m_reversed = other.m_reversed;
      m_constraints.clear();
      m_constraints.reserve(other.m_constraints.size());
      for (const auto &c : other.m_constraints) {
        if (c) {
          m_constraints.push_back(c->clone());
        }
      }
    }
    return *this;
  }

  /** @brief Move constructor. */
  TrajectoryConfig(TrajectoryConfig &&) = default;
  /** @brief Move assignment. */
  TrajectoryConfig &operator=(TrajectoryConfig &&) = default;

  /** @brief Sets initial trajectory velocity. */
  void set_start_velocity(Velocity start_velocity) { m_start_velocity = start_velocity; }

  /** @brief Sets final trajectory velocity. */
  void set_end_velocity(Velocity end_velocity) { m_end_velocity = end_velocity; }

  /** @brief Sets direction flag (reversed = true for driving backward). */
  void set_reversed(bool reversed) { m_reversed = reversed; }

  /** @brief Sets spatial discretization step size for spline sampling. */
  void set_sample_ds(Length sample_ds) { m_sample_ds = sample_ds; }

  /**
   * @brief Adds a user-defined TrajectoryConstraint.
   * @tparam Constraint Constraint class inheriting from TrajectoryConstraint.
   * @param constraint Constraint instance to add.
   */
  template <typename Constraint>
  typename std::enable_if<std::is_base_of<TrajectoryConstraint, typename std::decay<Constraint>::type>::value, void>::type
  add_constraint(Constraint &&constraint) {
    typedef typename std::decay<Constraint>::type C;
    m_constraints.emplace_back(std::unique_ptr<TrajectoryConstraint>(new C(std::forward<Constraint>(constraint))));
  }

  /** @brief Adds a TankKinematicsConstraint using track width and max velocity. */
  void set_track_width(Length track_width) { add_constraint(TankKinematicsConstraint(track_width, m_max_velocity)); }

  /** @return Initial trajectory velocity. */
  Velocity start_velocity() const { return m_start_velocity; }

  /** @return Final trajectory velocity. */
  Velocity end_velocity() const { return m_end_velocity; }

  /** @return Maximum physical velocity limit. */
  Velocity max_velocity() const { return m_max_velocity; }

  /** @return Maximum physical acceleration limit. */
  Acceleration max_acceleration() const { return m_max_acceleration; }

  /** @return Spatial sampling step size for path generation. */
  Length sample_ds() const { return m_sample_ds; }

  /** @return Vector of polymorphic trajectory constraint pointers. */
  const std::vector<std::unique_ptr<TrajectoryConstraint>> &constraints() const { return m_constraints; }

  /** @return True if trajectory is driven in reverse. */
  bool is_reversed() const { return m_reversed; }

 private:
  Velocity m_start_velocity = 0_inps;
  Velocity m_end_velocity = 0_inps;
  Velocity m_max_velocity;
  Acceleration m_max_acceleration;
  Length m_sample_ds = 0.5_in;
  std::vector<std::unique_ptr<TrajectoryConstraint>> m_constraints;
  bool m_reversed = false;
};

/**
 * @brief Fluent Builder API for constructing TrajectoryConfig instances cleanly.
 */
class TrajectoryConfigBuilder {
 public:
  /**
   * @brief Creates a builder with max velocity and max acceleration limits.
   */
  TrajectoryConfigBuilder(Velocity max_velocity, Acceleration max_acceleration)
      : m_config(max_velocity, max_acceleration) {}

  /** @brief Configures initial trajectory velocity. */
  TrajectoryConfigBuilder &with_start_velocity(Velocity v) {
    m_config.set_start_velocity(v);
    return *this;
  }

  /** @brief Configures final trajectory velocity. */
  TrajectoryConfigBuilder &with_end_velocity(Velocity v) {
    m_config.set_end_velocity(v);
    return *this;
  }

  /** @brief Configures reverse driving flag. */
  TrajectoryConfigBuilder &with_reversed(bool reversed = true) {
    m_config.set_reversed(reversed);
    return *this;
  }

  /** @brief Configures spatial spline discretization step size. */
  TrajectoryConfigBuilder &with_sample_ds(Length ds) {
    m_config.set_sample_ds(ds);
    return *this;
  }

  /** @brief Configures drivetrain track width kinematics constraint. */
  TrajectoryConfigBuilder &with_track_width(Length track_width) {
    m_config.set_track_width(track_width);
    return *this;
  }

  /** @brief Adds a custom physical trajectory constraint. */
  template <typename Constraint>
  TrajectoryConfigBuilder &with_constraint(Constraint &&constraint) {
    m_config.add_constraint(std::forward<Constraint>(constraint));
    return *this;
  }

  /** @brief Builds and returns constructed TrajectoryConfig instance. */
  TrajectoryConfig build() { return std::move(m_config); }

 private:
  TrajectoryConfig m_config;
};
