#include "core/utils/controls/pid.h"

#include "core/subsystems/odometry/odometry_base.h"

/**
 * Create the PID object
 */
PID::PID(
    double kP, double kI, double kD, double deadband, double on_target_time, ERROR_TYPE error_method
)
    : kP(kP),
      kI(kI),
      kD(kD),
      deadband(deadband),
      on_target_time(on_target_time),
      error_method(error_method) {
  pid_timer.reset();
}

void PID::init(double start_pt, double set_pt) {
  set_target(set_pt);
  target_vel = 0;  // TODO change back when trapezoid profiles are fixed
  sensor_val = start_pt;
  reset();
}

/**
 * Update the PID loop by taking the time difference from last update,
 * and running the PID formula with the new sensor data
 * @param sensor_val the distance, angle, encoder position or whatever it is we
 * are measuring
 * @return the new output. What would be returned by PID::get()
 */
double PID::update(double sensor_val) { return update(sensor_val, 0); }

/**
 * Update the PID loop by taking the time difference from last update,
 * and running the PID formula with the new sensor data
 * @param sensor_val the distance, angle, encoder position or whatever it is we
 * are measuring
 * @param v_setpt Expected velocity setpoint, to subtract from the D term (for
 * velocity control)
 * @return the new output. What would be returned by PID::get()
 */
double PID::update(double sensor_val, double v_setpt) {
  this->sensor_val = sensor_val;
  // printf("Error: %.2f\n", get_error());

  double time_delta = (pid_timer.systemHighResolution() / 1000000.0) - last_time;

  // Avoid a divide by zero error
  double d_term = 0;
  if (time_delta != 0.0) {
    d_term = kD * (((get_error() - last_error) / time_delta) - v_setpt);
  } else if (last_time != 0.0) {
    printf("(pid.cpp): Warning - running PID without a delay is just a P loop!\n");
  }

  // P and D terms
  out = (kP * get_error()) + d_term;

  bool limits_exist = lower_limit != 0 || upper_limit != 0;

  // Only add to the accumulated error if the output is not saturated
  // aka "Integral Clamping" anti-windup technique
  if (!limits_exist || (limits_exist && (out < upper_limit && out > lower_limit))) {
    accum_error += time_delta * get_error();
  }

  // I term
  out += kI * accum_error;

  last_time = pid_timer.systemHighResolution() / 1000000.0;
  last_error = get_error();

  // Enable clamping if the limit is not 0
  if (limits_exist) {
    out = (out < lower_limit) ? lower_limit : (out > upper_limit) ? upper_limit : out;
  }
  return out;
}

double PID::get_sensor_val() const { return sensor_val; }

/**
 * Reset the PID loop by resetting time since 0 and accumulated error.
 */
void PID::reset() {
  pid_timer.reset();

  last_error = 0;
  last_time = 0;
  accum_error = 0;

  is_checking_on_target = false;
  on_target_last_time = 0;
}

/**
 * Gets the current PID out value, from when update() was last run
 */
double PID::get() { return out; }

/**
 * Get the delta between the current sensor data and the target
 */
double PID::get_error() {
  if (error_method == ERROR_TYPE::ANGULAR) {
    return OdometryBase::smallest_angle(target, sensor_val);
  }
  return target - sensor_val;
}

/**
 * Get the delta between the current sensor data and the target
 */
double PID::get_output() { return out; }

double PID::get_target() const { return target; }

/**
 * Set the target for the PID loop, where the robot is trying to end up
 */
void PID::set_target(double target) { this->target = target; }

/**
 * Set the limits on the PID out. The PID out will "clip" itself to be
 * between the limits.
 */
void PID::set_limits(double lower, double upper) {
  lower_limit = lower;
  upper_limit = upper;
}

/**
 * Returns true if the loop is within [deadband] for [on_target_time]
 * seconds
 */
bool PID::is_on_target() {
  if (fabs(get_error()) < deadband) {
    if (target_vel != 0) {
      return true;
    }
    if (is_checking_on_target == false) {
      on_target_last_time = pid_timer.value();
      is_checking_on_target = true;
    } else if (pid_timer.value() - on_target_last_time > on_target_time) {
      return true;
    }
  } else {
    is_checking_on_target = false;
  }

  return false;
}
