#pragma once

#include <math.h>

// Struct representing a state along the motion profile containing position, velocity and acceleration.
typedef struct {
  double pos; // 1D position
  double vel; // 1D velocity
  double acc; // 1D acceleration
} motion_t;

/**
 * Class representing a trapezoidal motion profile. This consists of either two or three stages. First, an acceleration
 * stage where the velocity increases to a maximum. Then a cruise stage where the velocity is constant, then a
 * deceleration stage where the velocity decreases to zero.
 * 
 * This implementation allows for different acceleration and deceleration rates.
 * 
 * This is best used with LQR, as it tracks both velocity and position at the same time, however it can also be used
 * with PID and feedforward.
 * 
 * @author Jack Cammarata
 * @date 3/28/2025
 */
class TrapezoidProfile {
public:
/**
 * Constructs a TrapezoidProfile.
 * 
 * @param x_initial The initial position.
 * @param x_target The target position.
 * @param v_max The maximum velocity.
 * @param accel The acceleration.
 * @param decel The deceleration.
 */
  TrapezoidProfile(const double &x_initial, const double &x_target, const double &v_max, const double &accel, const double &decel);

  /**
   * Calculate the state along the motion profile at some given time.
   * 
   * @param t The time in seconds.
   * 
   * @return the state.
   */
  motion_t calculate(double t);

  /**
   * Returns the total time that the motion profile takes to complete.
   * 
   * @return the total time that the motion profile takes to complete.
   */
  double total_time();

private:
  double x_initial_;
  double x_target_;
  double v_max_;
  double v_peak_;
  double accel_;
  double decel_;
  double distance_;
  double dist_accel_;
  double dist_decel_;
  double dist_cruise_;
  double dist_full_;

  double time_accel_;
  double time_decel_;
  double time_cruise_;
  double time_total_;

  bool triangular_;

  // 1 if positive, -1 if negative
  int direction_;
};