#pragma once

#include <math.h>

#include <vector>

#include "core/utils/math_util.h"
#include "core/utils/moving_average.h"
#include "vex.h"

/**
 * FeedForward
 *
 * Stores the feedfoward constants, and allows for quick computation.
 * Feedfoward should be used in systems that require smooth precise movements
 * and have high inertia, such as drivetrains and lifts.
 *
 * This is best used alongside a PID loop, with the form:
 * output = pid.get() + feedforward.calculate(v, a);
 *
 * In this case, the feedforward does the majority of the heavy lifting, and the
 * pid loop only corrects for inconsistencies
 *
 * For information about tuning feedforward, I reccommend looking at this post:
 * https://www.chiefdelphi.com/t/paper-frc-drivetrain-characterization/160915
 * (yes I know it's for FRC but trust me, it's useful)
 *
 * @author Ryan McGee
 * @date 6/13/2022
 */
class FeedForward {
 public:
  /**
   * ff_config_t holds the parameters to make the theoretical model of a real world system
   * equation is of the form
   * kS if the system is not stopped, 0 otherwise
   * + kV * desired velocity
   * + kA * desired acceleration
   * + kG
   */

  /**
   * Creates a FeedForward object using the parameters to make the theoretical model of a real world
   * system equation is of the form
   * @param kS if the system is not stopped, 0 otherwise
   * @param + kV * desired velocity
   * @param + kA * desired acceleration
   * @param + kG
   */
  FeedForward(double kS = 0, double kV = 0, double kA = 0, double kG = 0);

  /**
   * @brief Perform the feedforward calculation
   *
   * This calculation is the equation:
   * F = kG + kS*sgn(v) + kV*v + kA*a
   *
   * @param v Requested velocity of system
   * @param a Requested acceleration of system
   * @return A feedforward that should closely represent the system if tuned correctly
   */
  double calculate(double v, double a, double pid_ref = 0.0) {
    double ks_sign = 0;
    if (v != 0)
      ks_sign = sign(v);
    else if (pid_ref != 0)
      ks_sign = sign(pid_ref);

    return (kS * ks_sign) + (kV * v) + (kA * a) + kG;
  }

  double kS;
  double kV;
  double kA;
  double kG;
};

/**
 * tune_feedforward takes a group of motors and finds the feedforward conifg parameters
 * automagically.
 *  @param motor the motor group to use
 *  @param pct Maximum velocity in percent (0->1.0)
 * @param duration Amount of time the motors spin for the test
 * @return A tuned feedforward object
 */
FeedForward tune_feedforward(vex::motor_group& motor, double pct, double duration);
