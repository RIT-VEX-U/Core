#include "../core/include/utils/pid.h"

/**
   * Create the PID object
   */
PID::PID(pid_config_t &config)
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

  double i_term = config.i * accum_error;

  // Avoid a divide by zero error
  double d_term = 0;
  if(time_delta != 0)
    d_term = config.d * (get_error() - last_error) / time_delta;
  else if(last_time != 0)
    printf("(pid.cpp): Warning - running PID without a delay is just a P loop!\n");

  double k_term = 0;
  if(get_error() < 0)
    k_term = config.k;
  else if(get_error() > 0)
    k_term = -config.k;


  out = (config.f * target) + (config.p * get_error()) + i_term + d_term + k_term;

  last_time = pid_timer.value();
  last_error = get_error();

  // Enable clamping if the limit is not 0
  if (lower_limit != 0 || upper_limit != 0)
  {
    // Integral back-calculation 
    // If the PID is oversaturated and the integrated term is not pushing it in the right direction,
    // then "turn off" the integral until it's actually helping.
    bool clamping_upper = (out > upper_limit) && ((out - i_term) > upper_limit);
    bool clamping_lower = (out < lower_limit) && ((out - i_term) < lower_limit);

    if(clamping_upper || clamping_lower)
      out -= i_term;

    // Actually clamp the value
    out = (out < lower_limit) ? lower_limit : (out > upper_limit) ? upper_limit : out;
  }
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

double PID::get_target()
{
  return target;
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
  if (fabs(get_error()) < config.deadband)
  {
    if (is_checking_on_target == false)
    {
      on_target_last_time = pid_timer.value();
      is_checking_on_target = true;
    }
    else if (pid_timer.value() - on_target_last_time > config.on_target_time)
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