#include "core/utils/controls/feedback_base.h"

class BangBang : public Feedback {
 public:
  BangBang(double thresshold, double low, double high);
  /**
   * Initialize the feedback controller for a movement
   *
   * @param start_pt the current sensor value
   * @param set_pt where the sensor value should be
   * @param start_vel Movement starting velocity
   * @param end_vel Movement ending velocity
   */
  void init(double start_pt, double set_pt) override;

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

 private:
  double setpt;
  double sensor_val;
  double lower_bound, upper_bound;
  double last_output;
  double threshhold;
};
