#include "core/utils/controls/feedforward.h"

/**
 * tune_feedforward takes a group of motors and finds the feedforward conifg parameters automagically.
 * @param motor the motor group to use
 * @param pct Maximum velocity in percent (0->1.0)
 * @param duration Amount of time the motors spin for the test
 * @return A tuned feedforward object
 */
FeedForward tune_feedforward(vex::motor_group &motor, double pct, double duration);