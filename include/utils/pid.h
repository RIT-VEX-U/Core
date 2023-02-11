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
class PID : public Feedback
{
public:
  /**
   * An enum to distinguish between a linear and angular caluclation of PID error.
   */
  enum ERROR_TYPE{
    LINEAR,
    ANGULAR // assumes degrees
  };
  /**
   * pid_config_t holds the configuration parameters for a pid controller
   * In addtion to the constant of proportional, integral and derivative, these parameters include: 
   * - deadband - 
   * - on_target_time - for how long do we have to be at the target to stop
   * As well, pid_config_t holds an error type which determines whether errors should be calculated as if the sensor position is a measure of distance or an angle
  */
  struct pid_config_t
  {
    double p; ///< proportional coeffecient p * error() 
    double i; ///< integral coeffecient i * integral(error)
    double d; ///< derivitave coeffecient d * derivative(error)
    double deadband; ///< at what threshold are we close enough to be finished
    double on_target_time; ///< the time in seconds that we have to be on target for to say we are officially at the target
    ERROR_TYPE error_method; ///< Linear or angular. wheter to do error as a simple subtraction or to wrap
  };

  

  /**
   * Create the PID object
   * @param config the configuration data for this controller
   */
  PID(pid_config_t &config);


  /**
   * Inherited from Feedback for interoperability.
   * Update the setpoint and reset integral accumulation
   * 
   * start_pt can be safely ignored in this feedback controller
   * @param start_pt commpletely ignored for PID. necessary to satisfy Feedback base
   * @param set_pt sets the target of the PID controller
   */
  void init(double start_pt, double set_pt) override;

  /**
   * Update the PID loop by taking the time difference from last update,
   * and running the PID formula with the new sensor data
   * @param sensor_val the distance, angle, encoder position or whatever it is we are measuring
   * @return the new output. What would be returned by PID::get()
   */
  double update(double sensor_val) override;

  /**
   * Gets the current PID out value, from when update() was last run
   * @return the Out value of the controller (voltage, RPM, whatever the PID controller is controlling)
   */
  double get() override;

  /**
   * Set the limits on the PID out. The PID out will "clip" itself to be 
   * between the limits.
   * @param lower the lower limit. the PID controller will never command the output go below `lower`
   * @param upper the upper limit. the PID controller will never command the output go higher than `upper`
   */
  void set_limits(double lower, double upper) override;

  /**
   * Checks if the PID controller is on target.  
   * @return true if the loop is within [deadband] for [on_target_time] seconds
   */
  bool is_on_target() override;

  /**
   * Reset the PID loop by resetting time since 0 and accumulated error.
   */
  void reset();

  /**
   * Get the delta between the current sensor data and the target
   * @return the error calculated. how it is calculated depends on error_method specified in pid_config_t
   */
  double get_error();

  /**
   * Get the PID's target
   * @return the target the PID controller is trying to achieve
   */
  double get_target();

  /**
   * Set the target for the PID loop, where the robot is trying to end up
   * @param target the sensor reading we would like to achieve
   */
  void set_target(double target);

  Feedback::FeedbackType get_type() override;

  pid_config_t &config; ///< configuration struct for this controller. see pid_config_t for information about what this contains

private:


  double last_error = 0;  ///< the error measured on the last iteration of update()
  double accum_error = 0; ///< the integral of error over time since we called init()
  
  double last_time = 0; ///< the time measured the last time update() was called
  double on_target_last_time = 0; ///< the time at which we started being on target
  
  double lower_limit = 0; ///< the PID controller will never set a target to go lower than this
  double upper_limit = 0; ///< the PID controller will never set a target to go higher than this

  double target = 0; ///< the target position of the PID controller (lower_limit <= target <= upper_limit)
  double sensor_val = 0; ///< the last recorded value of the sensor we use to feed the PID controller
  double out = 0; ///< the last calculated output value. we save it here so that we don't have to recalculate if we ask for it more than once between update() calls

  bool is_checking_on_target = false; ///< true if the sensor reading is within target +/- deadband

  timer pid_timer; ///< used for calculating integrals and derivatives in line with the real world times and checking the time we are on target 
};
