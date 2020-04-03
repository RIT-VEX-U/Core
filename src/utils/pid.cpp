#include "utils/pid.h"

/**
   * Create the PID object
   */
PID::PID(pid_config_t *config)
    : config(config)
{
  pid_timer.reset();
}

/**
   * Update the PID loop by taking the time difference from last update,
   * and running the PID formula with the new sensor data
   */
void PID::update(double sensor_val)
{
  this->sensor_val = sensor_val;

  double time_delta = pid_timer.value() - last_time;

  accum_error += time_delta * get_error();

  out = (config->f) + (config->p * get_error()) + (config->i * accum_error) + (config->d * (get_error() - last_error) / time_delta);

  last_time = pid_timer.value();
  last_error = get_error();

  if (lower_limit != 0 || upper_limit != 0)
    out = (out < lower_limit) ? lower_limit : (out > upper_limit) ? upper_limit : out;
}

/**
   * Reset the PID loop by resetting time since 0 and accumulated error.
   */
void PID::reset()
{
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
double PID::get()
{
  return out;
}

/**
   * Get the delta between the current sensor data and the target
   */
double PID::get_error()
{
  return target - sensor_val;
}

/**
   * Set the target for the PID loop, where the robot is trying to end up
   */
void PID::set_target(double target)
{
  this->target = target;
}

/**
   * Set the limits on the PID out. The PID out will "clip" itself to be 
   * between the limits.
   */
void PID::set_limits(double lower, double upper)
{
  lower_limit = lower;
  upper_limit = upper;
}

/**
   * Returns true if the loop is within [deadband] for [on_target_time]
   * seconds
   */
bool PID::is_on_target()
{
  if (fabs(get_error()) < config->deadband)
  {
    if (is_checking_on_target == false)
    {
      on_target_last_time = pid_timer.value();
      is_checking_on_target = true;
    }
    else if (pid_timer.value() - on_target_last_time > config->on_target_time)
    {
      return true;
    }
  }
  else
  {
    is_checking_on_target = false;
  }

  return false;
}