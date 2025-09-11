#pragma once
#include "core/utils/controls/feedback_base.h"

/// @brief A velocity controller
/// @warning If you try to use this as a position controller, it will fail.
class TakeBackHalf : public Feedback {

  public:
    TakeBackHalf(double TBH_gain, double first_cross_split, double on_target_threshold);
    /**
     * Initialize the feedback controller for a movement
     *
     * @param start_pt the current sensor value
     * @param set_pt where the sensor value should be
     * @param start_vel Movement starting velocity (IGNORED)
     * @param end_vel Movement ending velocity (IGNORED)
     */
    void init(double start_pt, double set_pt);
    /**
     * Iterate the feedback loop once with an updated sensor value
     *
     * @param val value from the sensor
     * @return feedback loop result
     */
    double update(double val) override;

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

    double TBH_gain; ///< tuned parameter
    double first_cross_split;

  private:
    double on_target_threshhold; ///< tuned parameter

    double target = 0.0;

    bool first_cross = true;
    double tbh = 0.0;
    double prev_error = 0.0;

    double output = 0.0;
    double lower = 0.0, upper = 0.0; ///< output limits
};