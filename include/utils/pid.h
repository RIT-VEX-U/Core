#pragma once

#include <cmath>
#include "vex.h"
#include "../core/include/utils/feedback_base.h"

using namespace vex;

/**
 * PID Class
 * 
 * Defines a standard feedback loop using the constants kP, kI, kD, deadband, and on_target_time.
 * The formula is:
 * 
 * out = kP*error + kI*integral(d Error) + kD*(dError/dt)
 * 
 * The PID object will determine it is "on target" when the error is within the deadband, for
 * a duration of on_target_time
 * 
 * @author Ryan McGee
 * @date 4/3/2020
 */
class PID : Feedback
{
public:
  /**
   *An enum to distinguish between a linear and angular caluclation of PID error.
   */
  enum ERROR_TYPE{
    LINEAR,
    ANGULAR
  };
  struct pid_config_t
  {
    double p, i, d;
    double deadband, on_target_time;
    ERROR_TYPE error_method;
  };

  

  /**
   * Create the PID object
   */
  PID(pid_config_t &config);


  /**
   * Inherited from Feedback for interoperability.
   * Update the setpoint and reset integral accumulation
   * 
   * start_pt can be safely ignored in this feedback controller
   */
  void init(double start_pt, double set_pt) override;

  /**
   * Update the PID loop by taking the time difference from last update,
   * and running the PID formula with the new sensor data
   */
  double update(double sensorVal) override;

  /**
   * Gets the current PID out value, from when update() was last run
   */
  double get() override;

  /**
   * Set the limits on the PID out. The PID out will "clip" itself to be 
   * between the limits.
   */
  void set_limits(double lower, double upper) override;

  /**
   * Returns true if the loop is within [deadband] for [on_target_time]
   * seconds
   */
  bool is_on_target() override;

  /**
   * Reset the PID loop by resetting time since 0 and accumulated error.
   */
  void reset();

  /**
   * Get the delta between the current sensor data and the target
   */
  double get_error();

  /**
   * Get the PID's target
   */
  double get_target();

  /**
   * Set the target for the PID loop, where the robot is trying to end up
   */
  void set_target(double target);

private:
  pid_config_t &config;


  double last_error = 0, accum_error = 0;
  double last_time = 0, on_target_last_time = 0;
  double lower_limit = 0, upper_limit = 0;

  double target = 0, sensor_val = 0, out = 0;
  bool is_checking_on_target = false;

  timer pid_timer;
};
