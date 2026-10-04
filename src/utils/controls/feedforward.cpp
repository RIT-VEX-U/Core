#include "core/utils/controls/feedforward.h"


/**
 * Creates a FeedForward object.
 * @param Ks Coefficient to overcome static friction: the point at which the motor *starts* to move.
 * @param Kv Veclocity coefficient: the power required to keep the mechanism in motion.
 * @param Ka kA - Acceleration coefficient: the power required to change the mechanism's speed.
 * @param Kg kG - Gravity coefficient: only needed for lifts. The power required to overcome gravity and stay
 */
FeedForward::FeedForward(const double &kS, const double &kV, const double &kA, const double &kG) : kS(kS), kV(kV), kA(kA), kG(kG) {}

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
double FeedForward::calculate(double v, double a, double pid_ref) {
    double ks_sign = 0;
    if (v != 0)
        ks_sign = sign(v);
    else if (pid_ref != 0)
        ks_sign = sign(pid_ref);

    return (kS * ks_sign) + (kV * v) + (kA * a) + kG;
}