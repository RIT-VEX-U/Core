#pragma once

#include "math/controls/feedforward.h"
#include "math/geometry/rotation2d.h"

namespace core {

/**
 * ArmFeedforward is an extension of the Feedforward class that adds a gravity
 * compensation term for use in arms/lifts. It calculates the output required to
 * hold the mechanism in place against gravity.
 *
 * The formula used is:
 * 
 * output = kS * sign(v) + kV * v + kA * a + kG * cos(angle)
 *
 * where:
 * - kS is the static gain
 * - kV is the velocity gain
 * - kA is the acceleration gain
 * - kG is the gravity gain
 * - angle is the current angle of the arm (specifically the CG), for a lift this is always 0
 * 
 * @author Jack Cammarata
 * @date 6/27/2025
 */
class ArmFeedforward : public Feedforward {
   public:
    /**
     * Constructs an ArmFeedforward with the given parameters.
     *
     * @param kS The static gain.
     * @param kG The gravity gain.
     * @param kV The velocity gain.
     * @param kA The acceleration gain.
     */
    ArmFeedforward(double kS, double kV, double kA, double kG)
        : kG_(kG), Feedforward(kS, kV, kA) {}
    ArmFeedforward(Feedforward::feedforward_config_t config)
        : kG_(config.kG), Feedforward(config.kS, config.kV, config.kA) {}

    ArmFeedforward(const ArmFeedforward&) = default;
    ArmFeedforward(ArmFeedforward&&) = default;
    ArmFeedforward& operator=(const ArmFeedforward&) = default;
    ArmFeedforward& operator=(ArmFeedforward&&) = default;

    /**
     * Calculates the output voltage based on the velocity and acceleration commands.
     * 
     * @param v The current velocity command.
     * @param a The current acceleration command.
     * @param angle The current angle of the arm.
     * @return The calculated output voltage.
     */
    double calculate(double v, double a, Rotation2d angle) const {
        double out = Feedforward::calculate(v, a);
        return out + kG_ * angle.f_cos();
    }

    double kG() const { return kG_; }
    void set_kG(double kG) { kG_ = kG; }

   private:
    double kG_;
};

}  // namespace core
