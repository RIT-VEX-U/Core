#include "../core/include/utils/controls/asymmetric_trapezoid_profile.h"
#include "../core/include/utils/math_util.h"
#include <cmath>

/**
 * Asymmetric Trapezoid Profile
 *
 * This is a motion profile defined by an acceleration, deceleration, maximum velocity, start point and end point.
 * Using this information, a parametric function is generated, with a period of acceleration, constant
 * velocity, and deceleration. The velocity graph looks like a trapezoid, giving it it's name.
 *
 * If the maximum velocity is set high enough, this will become a S-curve profile, with only acceleration and
 * deceleration.
 *
 * This class is designed for use in properly modelling the motion of the robots to create a feedfoward
 * and target for PID. Acceleration and Maximum velocity should be measured on the robot and tuned down
 * slightly to account for battery drop.
 *
 * Here are the equations graphed for ease of understanding:
 * https://www.desmos.com/calculator/rkm3ivu1yk
 *
 * @author Jack Cammarata
 * @date 1/26/2025
 *
 */

/**
 * @brief Construct a new Asymmetric Trapezoid Profile object
 *
 * @param max_v Maximum velocity the robot can run at
 * @param accel Maximum acceleration of the robot
 * @param decel Maximum deceleration of the robot
 */
AsymmetricTrapezoidProfile::AsymmetricTrapezoidProfile(double max_v, double accel, double decel)
    : start(0), end(0), max_v(max_v), accel(accel), decel(decel) {}

// Kinematic equations as macros
#define CALC_POS(time_s, a, v, s) ((0.5 * (a) * (time_s) * (time_s)) + ((v) * (time_s)) + (s))
#define CALC_VEL(time_s, a, v) (((a) * (time_s)) + (v))

/**
 * @brief Run the trapezoidal profile based on the time that's ellapsed
 *
 * @param time_s Time since start of movement
 * @return motion_t Position, velocity and acceleration
 */
a_motion_t AsymmetricTrapezoidProfile::calculate(double time_s) {
  double delta_pos = end - start;

  // redefine accel and max_v in this scope for negative calcs
  double accel_local = this->accel;
  double decel_local = this->decel;
  double max_v_local = this->max_v;
  if (delta_pos < 0) {
    accel_local = -this->accel;
    decel_local = -this->decel;
    max_v_local = -this->max_v;
  }

  // Calculate the time spent during the acceleration / maximum velocity / deceleration stages
  double accel_time = max_v_local / accel_local;
  double decel_time = max_v_local / decel_local;
  double max_vel_time = (delta_pos - (0.5 * accel_local * accel_time * accel_time) - (0.5 * decel_local * decel_time * decel_time)) / max_v_local;
  this->time = accel_time + max_vel_time + decel_time;

  // If the time during the "max velocity" state is negative, use an S profile
  if (max_vel_time < 0) {
    accel_time = sqrt(fabs(delta_pos / accel));
    decel_time = sqrt(fabs(delta_pos / decel));
    max_vel_time = 0;
    this->time = accel_time + decel_time;
  }

  a_motion_t out;

  // Handle if a bad time is put in
  if (time_s < 0) {
    out.pos = start;
    out.vel = 0;
    out.accel = 0;
    return out;
  }

  // Handle after the setpoint is reached
  if (time_s > accel_local + max_vel_time + decel_local) {
    out.pos = end;
    out.vel = 0;
    out.accel = 0;
    return out;
  }

  // ======== KINEMATIC EQUATIONS ========

  // Displacement from initial acceleration
  if (time_s < accel_time) {
    out.pos = start + CALC_POS(time_s, accel_local, 0, 0);
    out.vel = CALC_VEL(time_s, accel_local, 0);
    out.accel = accel_local;
    return out;
  }

  double s_accel = CALC_POS(accel_time, accel_local, 0, 0);

  // Displacement during maximum velocity
  if (time_s < accel_time + max_vel_time) {
    out.pos = start + CALC_POS(time_s - accel_time, 0, max_v_local, s_accel);
    out.vel = sign(delta_pos) * max_v;
    out.accel = 0;
    return out;
  }

  double s_max_vel = CALC_POS(max_vel_time, 0, max_v_local, s_accel);

  // Displacement during deceleration
  out.pos = start + CALC_POS(time_s - accel_time - max_vel_time, -decel_local, 0, s_accel + s_max_vel);
  out.vel = CALC_VEL(time_s - accel_time - max_vel_time, -decel_local, max_v_local);
  out.accel = -decel_local;
  return out;
}

/**
 * set_endpts defines a start and end position
 * @param start the starting position of the path
 * @param end the ending position of the path
 */
void AsymmetricTrapezoidProfile::set_endpts(double start, double end) {
    this->start = start;
    this->end = end;
}

/**
 * set_accel sets the acceleration this profile will use (the left leg of the trapezoid)
 * @param accel the acceleration amount to use
 */
void AsymmetricTrapezoidProfile::set_accel(double accel) { this->accel = accel; }

/**
 * set_accel sets the deceleration this profile will use (right leg of the trapezoid)
 * @param decel the deceleration amount to use
 */
void AsymmetricTrapezoidProfile::set_decel(double decel) { this->decel = decel; }

/**
 * sets the maximum velocity for the profile
 * (the height of the top of the trapezoid)
 * @param max_v the maximum velocity the robot can travel at
 */
void AsymmetricTrapezoidProfile::set_max_v(double max_v) { this->max_v = max_v; }

/**
 * uses the kinematic equations to and specified accel and max_v to figure out how long moving along the profile
 * would take
 * @return the time the path will take to travel
 */
double AsymmetricTrapezoidProfile::get_movement_time() { return time; }
