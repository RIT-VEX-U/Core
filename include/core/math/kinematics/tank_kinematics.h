#pragma once

#include "core/math/kinematics/chassis_accelerations.h"
#include "core/math/kinematics/chassis_velocities.h"
#include "core/math/kinematics/tank_wheel_accelerations.h"
#include "core/math/kinematics/tank_wheel_velocities.h"
#include "core/math/units.h"

using namespace units::literals;

/**
 * @brief Helper class that converts a chassis velocity (vx and omega) to
 * left and right wheel velocities for a tank drive.
 *
 * Inverse kinematics converts a desired chassis velocity into left and right
 * velocity components whereas forward kinematics converts left and right
 * component velocities into a linear and angular chassis velocity.
 */
class TankKinematics {
 public:
  /** @brief Distance between the left and right wheels (often tuned empirically). */
  units::Length track_width;

  /**
   * @brief Constructs a tank drive kinematics object.
   * @param track_width The physical (or effective) distance between left and right wheels.
   */
  explicit TankKinematics(units::Length track_width) : track_width(track_width) {}

  /**
   * @brief Returns a chassis velocity from left and right component velocities using forward
   * kinematics.
   * @param wheel_velocities The left and right velocities.
   * @return The chassis velocity.
   */
  ChassisVelocities to_chassis_velocities(const TankWheelVelocities& wheel_velocities) const {
    units::Velocity vx = (wheel_velocities.left + wheel_velocities.right) / 2.0;

    // omega = (right - left) / track_width
    units::AngularVelocity omega =
        ((wheel_velocities.right.to(units::inps) - wheel_velocities.left.to(units::inps)) /
         track_width.to(units::in)) *
        units::radps;

    return ChassisVelocities(vx, 0_inps, omega);
  }

  /**
   * @brief Returns left and right component velocities from a chassis velocity using inverse
   * kinematics.
   * @param chassis_velocities The linear and angular components that represent the chassis'
   * velocity.
   * @return The left and right velocities.
   */
  TankWheelVelocities to_wheel_velocities(const ChassisVelocities& chassis_velocities) const {
    // v_L = vx - (track_width / 2) * omega
    // v_R = vx + (track_width / 2) * omega
    units::Velocity omega_linear =
        (chassis_velocities.omega.to(units::radps) * (track_width.to(units::in) / 2.0)) *
        units::inps;

    return TankWheelVelocities(chassis_velocities.vx - omega_linear,
                               chassis_velocities.vx + omega_linear);
  }

  /**
   * @brief Returns a chassis acceleration from left and right component accelerations using forward
   * kinematics.
   * @param wheel_accels The left and right accelerations.
   * @return The chassis acceleration.
   */
  ChassisAccelerations to_chassis_accelerations(const TankWheelAccelerations& wheel_accels) const {
    units::Acceleration ax = (wheel_accels.left + wheel_accels.right) / 2.0;

    // alpha = (right - left) / track_width
    units::AngularAcceleration alpha =
        ((wheel_accels.right.to(units::inps2) - wheel_accels.left.to(units::inps2)) /
         track_width.to(units::in)) *
        units::radps2;

    return ChassisAccelerations(ax, 0_inps2, alpha);
  }

  /**
   * @brief Returns left and right component accelerations from a chassis acceleration using inverse
   * kinematics.
   * @param chassis_accels The linear and angular components that represent the chassis'
   * acceleration.
   * @return The left and right accelerations.
   */
  TankWheelAccelerations to_wheel_accelerations(const ChassisAccelerations& chassis_accels) const {
    // a_L = ax - (track_width / 2) * alpha
    // a_R = ax + (track_width / 2) * alpha
    units::Acceleration alpha_linear =
        (chassis_accels.alpha.to(units::radps2) * (track_width.to(units::in) / 2.0)) * units::inps2;

    return TankWheelAccelerations(chassis_accels.ax - alpha_linear,
                                  chassis_accels.ax + alpha_linear);
  }
};
