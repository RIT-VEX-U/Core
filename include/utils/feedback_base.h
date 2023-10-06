#pragma once

/**
 * Interface so that subsystems can easily switch between feedback loops
 *
 * @author Ryan McGee
 * @date 9/25/2022
 *
 */
class Feedback
{
public:
    enum FeedbackType
    {
        PIDType,
        FeedforwardType,
        OtherType,
    };

    /**
     * Initialize the feedback controller for a movement
     *
     * @param start_pt the current sensor value
     * @param set_pt where the sensor value should be
     * @param start_vel Movement starting velocity
     * @param end_vel Movement ending velocity
     */
    virtual void init(double start_pt, double set_pt, double start_vel, double end_vel) = 0;

    /**
     * Iterate the feedback loop once with an updated sensor value
     *
     * @param val value from the sensor
     * @return feedback loop result
     */
    virtual double update(double val) = 0;

    /**
     * @return the last saved result from the feedback controller
     */
    virtual double get() = 0;

    /**
     * Clamp the upper and lower limits of the output. If both are 0, no limits should be applied.
     *
     * @param lower Upper limit
     * @param upper Lower limit
     */
    virtual void set_limits(double lower, double upper) = 0;

    /**
     * @return true if the feedback controller has reached it's setpoint
     */
    virtual bool is_on_target() = 0;

    virtual Feedback::FeedbackType get_type()
    {
        return FeedbackType::OtherType;
    }
};