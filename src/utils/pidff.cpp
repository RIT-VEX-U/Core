#include "../core/include/utils/pidff.h"
#include "../core/include/utils/math_util.h"

PIDFF::PIDFF(PID::pid_config_t &pid_cfg, FeedForward::ff_config_t &ff_cfg):  pid(pid_cfg), ff_cfg(ff_cfg),ff(ff_cfg)  
{
    out = 0;
    lower_lim = 0;
    upper_lim = 0;
}

/**
 * Initialize the feedback controller for a movement
 * 
 * @param start_pt the current sensor value
 * @param set_pt where the sensor value should be
 */
void PIDFF::init(double start_pt, double set_pt)
{
    pid.init(start_pt, set_pt);
}

void PIDFF::set_target(double set_pt)
{
    pid.set_target(set_pt);
}

/**
 * Iterate the feedback loop once with an updated sensor value.
 * Only kS for feedfoward will be applied.
 * 
 * @param val value from the sensor
 * @return feedback loop result
 */
double PIDFF::update(double val)
{
    double pid_out = pid.update(val);
    double ff_out = ff_cfg.kS * sign(pid_out);
    out = pid_out + ff_out;
    if (lower_lim != upper_lim)
        out = clamp(out, lower_lim, upper_lim);

    return out;
}

/**
 * Iterate the feedback loop once with an updated sensor value
 * 
 * @param val value from the sensor
 * @param vel_setpt Velocity for feedforward
 * @param a_setpt Acceleration for feedfoward
 * @return feedback loop result
*/
double PIDFF::update(double val, double vel_setpt, double a_setpt)
{

    double pid_out = pid.update(val);
    double ff_out = ff.calculate(vel_setpt, a_setpt);
    out = pid_out + ff_out;
    if (lower_lim != upper_lim)
        out = clamp(out, lower_lim, upper_lim);
    
    return out;
}

/**
 * @return the last saved result from the feedback controller
 */
double PIDFF::get()
{
    return out;
}

/**
 * Clamp the upper and lower limits of the output. If both are 0, no limits should be applied.
 * 
 * @param lower Upper limit
 * @param upper Lower limit
 */
void PIDFF::set_limits(double lower, double upper)
{
    upper_lim = upper;
    lower_lim = lower;
}

/** 
 * @return true if the feedback controller has reached it's setpoint
 */
bool PIDFF::is_on_target()
{
    return pid.is_on_target();
}