#pragma once

#include <cmath>

#include "core/utils/controls/feedback_base.h"
#include "vex.h"

/**
 * PID Class
 *
 * Defines a standard feedback loop using the constants kP, kI, kD, deadband,
 * and on_target_time. The formula is:
 *
 * out = kP*error + kI*integral(d Error) + kD*(dError/dt)
 *
 * The PID object will determine it is "on target" when the error is within the
 * deadband, for a duration of on_target_time
 *
 * @author Ryan McGee
 * @date 4/3/2020
 */
class PID : public Feedback {
 public:
  /**
   * An enum to distinguish between a linear and angular caluclation of PID
   * error.
   */
  enum ERROR_TYPE {
    LINEAR,
    ANGULAR  // assumes degrees
  };

  /**
   * Create the PID object
   * @param config the configuration data for this controller
   */
  PID(double kP = 0, double kI = 0, double kD = 0, double deadband = 0, double on_target_time = 0,
      ERROR_TYPE error_method = ERROR_TYPE::LINEAR);

  /**
   * Inherited from Feedback for interoperability.
   * Update the setpoint and reset integral accumulation
   *
   * start_pt can be safely ignored in this feedback controller
   * @param start_pt commpletely ignored for PID. necessary to satisfy Feedback
   * base
   * @param set_pt sets the target of the PID controller
   * @param start_vel completely ignored for PID. necessary to satisfy Feedback
   * base
   * @param end_vel sets the target end velocity of the PID controller
   */
  void init(double start_pt, double set_pt) override;

  /**
   * Update the PID loop by taking the time difference from last update,
   * and running the PID formula with the new sensor data
   * @param sensor_val the distance, angle, encoder position or whatever it is
   * we are measuring
   * @return the new output. What would be returned by PID::get()
   */
  double update(double sensor_val) override;

  /**
   * Update the PID loop by taking the time difference from last update,
   * and running the PID formula with the new sensor data
   * @param sensor_val the distance, angle, encoder position or whatever it is we
   * are measuring
   * @param v_setpt Expected velocity setpoint, to subtract from the D term (for
   * velocity control)
   * @return the new output. What would be returned by PID::get()
   */
  double update(double sensor_val, double v_setpt);

  /**
   * @brief gets the sensor value that we were last updated with
   * @return sensor_val
   */
  double get_sensor_val() const;

  /**
   * Gets the current PID out value, from when update() was last run
   * @return the Out value of the controller (voltage, RPM, whatever the PID
   * controller is controlling)
   */
  double get() override;

  /**
   * Set the limits on the PID out. The PID out will "clip" itself to be
   * between the limits.
   * @param lower the lower limit. the PID controller will never command the
   * output go below `lower`
   * @param upper the upper limit. the PID controller will never command the
   * output go higher than `upper`
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
   * @return the error calculated. how it is calculated depends on the error_method
   */
  double get_error();

  /**
   * Get the output calculated from the P, I, D and Error values
   * @return the output calculated from the pid controller
   */
  double get_output();

  /**
   * Get the PID's target
   * @return the target the PID controller is trying to achieve
   */
  double get_target() const;

  /**
   * Set the target for the PID loop, where the robot is trying to end up
   * @param target the sensor reading we would like to achieve
   */
  void set_target(double target);

  double kP;
  double kI;
  double kD;

  double deadband;
  double on_target_time;

  ERROR_TYPE error_method;

 private:
  double last_error = 0;   ///< the error measured on the last iteration of update()
  double accum_error = 0;  ///< the integral of error over time since we called init()

  double last_time = 0;            ///< the time measured the last time update() was called
  double on_target_last_time = 0;  ///< the time at which we started being on target

  double lower_limit = 0;  ///< the PID controller will never set a target to go lower than this
  double upper_limit = 0;  ///< the PID controller will never set a target to go higher than this

  double target = 0;      ///< the target position of the PID controller (lower_limit
                          ///< <= target <= upper_limit)
  double target_vel = 0;  ///< the target velocity of the PID controller (if !=
                          ///< 0, controller will not wait for stop)
  double sensor_val = 0;  ///< the last recorded value of the sensor we use to
                          ///< feed the PID controller
  double out = 0;         ///< the last calculated output value. we save it here so that
                          ///< we don't have to recalculate if we ask for it more than
                          ///< once between update() calls

  bool is_checking_on_target = false;  ///< true if the sensor reading is within target +/- deadband

  vex::timer pid_timer;  ///< used for calculating integrals and derivatives in line
                         ///< with the real world times and checking the time we are
                         ///< on target
};
