#pragma once

#include <math.h>
#include <vector>
#include "../core/include/utils/math_util.h"
#include "../core/include/utils/moving_average.h"
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
class FeedForward
{
    public:

    /**
     * kS - Coefficient to overcome static friction: the point at which the motor *starts* to move.
     * kV - Veclocity coefficient: the power required to keep the mechanism in motion. Multiplied by the requested velocity.
     * kA - Acceleration coefficient: the power required to change the mechanism's speed. Multiplied by the requested acceleration.
     * kG - Gravity coefficient: only needed for lifts. The power required to overcome gravity and stay at steady state.
     *      Should relate to acceleration due to gravity and mass of the lift.
     */
    typedef struct
    {
        double kS, kV, kA, kG;
    } ff_config_t;

    
    /**
     * Creates a FeedForward object.
     * @param cfg Configuration Struct for tuning
    */
    FeedForward(ff_config_t &cfg) : cfg(cfg) {}

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
    double calculate(double v, double a)
    {
        return (cfg.kS * (v > 0 ? 1 : v < 0 ? -1 : 0)) + (cfg.kV * v) + (cfg.kA * a) + cfg.kG;
    }

    private:

    ff_config_t &cfg;

};


/**
* tune_feedforward takes a group of motors and finds the feedforward conifg parameters automagically.
*  @param motor the motor group to use 
*  @param pct Maximum velocity in percent (0->1.0)
 * @param duration Amount of time the motors spin for the test
 * @return A tuned feedforward object
 */
FeedForward::ff_config_t tune_feedforward(vex::motor_group &motor, double pct, double duration);