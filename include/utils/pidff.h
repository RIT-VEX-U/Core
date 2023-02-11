#pragma once
#include "../core/include/utils/feedback_base.h"
#include "../core/include/utils/pid.h"
#include "../core/include/utils/feedforward.h"

class PIDFF : public Feedback
{
    public:

    PIDFF(PID::pid_config_t &pid_cfg, FeedForward::ff_config_t &ff_cfg);

    /**
     * Initialize the feedback controller for a movement
     * 
     * @param start_pt the current sensor value
     * @param set_pt where the sensor value should be
     */
    void init(double start_pt, double set_pt) override;

    /**
     * Set the target of the PID loop
     * @param set_pt Setpoint / target value
    */
    void set_target(double set_pt);

    /**
     * Iterate the feedback loop once with an updated sensor value.
     * Only kS for feedfoward will be applied.
     * 
     * @param val value from the sensor
     * @return feedback loop result
     */
    double update(double val) override;

    /**
     * Iterate the feedback loop once with an updated sensor value
     * 
     * @param val value from the sensor
     * @param vel_setpt Velocity for feedforward
     * @param a_setpt Acceleration for feedfoward
     * @return feedback loop result
    */
    double update(double val, double vel_setpt, double a_setpt=0);

    /**
     * @return the last saved result from the feedback controller
     */
    double get() override;

    /**
     * Clamp the upper and lower limits of the output. If both are 0, no limits should be applied.
     * 
     * @param lower Upper limit
     * @param upper Lower limit
     */
    void set_limits(double lower, double upper) override;

    /** 
     * @return true if the feedback controller has reached it's setpoint
     */
    bool is_on_target() override;

    PID pid;


    private:

    FeedForward::ff_config_t &ff_cfg;

    FeedForward ff;

    double out;
    double lower_lim, upper_lim;

};