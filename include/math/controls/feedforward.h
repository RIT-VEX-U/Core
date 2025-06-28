#pragma once

#include <cmath>
#include "math/math_util.h"

namespace core {

/**
 * Stores feedforward constants and allows computation of voltage from reference vel/acc.
 * 
 * Feedforward should be used in systsems that require smooth precise movements and have
 * high inertia, such as drivetrains and lifts. It is also useful for flywheels since they're so simple.
 * 
 * The formula used is:
 * 
 * output = kS * sign(v) + kV * v + kA * a
 * 
 * where:
 * - kS is the static gain (voltage required to overcome static friction)
 * - kV is the velocity gain (voltage required to maintain a constant velocity)
 * - kA is the acceleration gain (voltage required to accelerate at a constant rate)
 * 
 * This is best used alongside a PID loop:
 * 
 * output = pid.calculate() + feedforward.calculate(vel, acc)
 * 
 * In this case the feedforward does the heavy lifting and the PID corrects for small errors.
 * 
 * For information about tuning feedforward, I reccommend looking at this post:
 * https://www.chiefdelphi.com/t/paper-frc-drivetrain-characterization/160915
 * (yes I know it's for FRC but trust me, it's useful)
 *
 * @author Ryan McGee, Jack Cammarata
 * @date 6/13/2022, 6/27/2025
 */
class Feedforward {
   public:
   typedef struct {
        double kS;  ///< static gain
        double kV;  ///< velocity gain
        double kA;  ///< acceleration gain
        double kG;  ///< gravity gain (optional, used in ArmFeedforward)
    } feedforward_config_t;
    /**
     * Constructs a Feedforward with the given parameters.
     *
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kA The acceleration gain.
     */
    Feedforward(double kS, double kV, double kA) : kS_(kS), kV_(kV), kA_(kA) {}
    Feedforward(Feedforward::feedforward_config_t config)
        : kS_(config.kS), kV_(config.kV), kA_(config.kA) {}

    Feedforward(const Feedforward&) = default;
    Feedforward(Feedforward&&) = default;
    Feedforward& operator=(const Feedforward&) = default;
    Feedforward& operator=(Feedforward&&) = default;

    /**
     * Calculates the output voltage based on the velocity and acceleration commands.
     * 
     * @param v The current velocity command.
     * @param a The current acceleration command.
     * @return The calculated output voltage.
     */
    double calculate(double v, double a) const {
        double ks_sign = sign(v);
        return kS_ * ks_sign + kV_ * v + kA_ * a;
    }

    /**
     * Calculates the maximum velocity that can be achieved with a given maximum voltage.
     * 
     * @param max_voltage The maximum voltage that can be applied.
     * @return The maximum velocity that can be achieved.
     */
    double max_vel(double max_voltage) const {
        if (kV_ == 0) {
            return 0;  // don't divide by zero
        }
        return max_voltage / kV_;
    }

    /**
     * Calculates the maximum acceleration that can be achieved with a given maximum voltage.
     * 
     * @param max_voltage The maximum voltage that can be applied.
     * @return The maximum acceleration that can be achieved.
     */
    double max_acc(double max_voltage) const {
        if (kA_ == 0) {
            return 0;  // don't divide by zero
        }
        return max_voltage / kA_;
    }

    double kS() const { return kS_; }
    double kV() const { return kV_; }
    double kA() const { return kA_; }

    void set_kS(double kS) { kS_ = kS; }
    void set_kV(double kV) { kV_ = kV; }
    void set_kA(double kA) { kA_ = kA; }

   private:
    double kS_;
    double kV_;
    double kA_;
};

}  // namespace core
