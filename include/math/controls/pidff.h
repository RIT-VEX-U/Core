#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "math/controls/arm_feedforward.h"
#include "math/controls/pid.h"
#include "math/geometry/rotation2d.h"
#include "math/math_util.h"

namespace core {

/**
 * Creates a PIDFF controller that combines PID and Feedforward control.
 * 
 * This controller uses both feedback (PID) and feedforward control to improve tracking performance.
 * The feedforward component handles the known dynamics of the system, while the PID
 * component handles errors and disturbances.
 * 
 * output = pid_output + feedforward_output
 * 
 * @author Jack Cammarata
 * @date 6/27/2025
 */
class PIDFF {
   public:
    /**
     * Constructs a PIDFF controller with the given parameters.
     * 
     * @param kP The proportional gain.
     * @param kI The integral gain.
     * @param kD The derivative gain.
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kA The acceleration gain.
     * @param kG The gravity compensation gain.
     * @param tolerance The tolerance value for determining if at setpoint.
     * @param dt The time step in seconds.
     */
    PIDFF(double kP, double kI, double kD, double kS, double kV, double kA, double kG,
          double tolerance, double dt = 0.01)
        : pid_(kP, kI, kD, tolerance, dt),
          ff_(kS, kV, kA, kG) {}

    PIDFF(const PIDFF&) = default;
    PIDFF(PIDFF&&) = default;
    PIDFF& operator=(const PIDFF&) = default;
    PIDFF& operator=(PIDFF&&) = default;

    /**
     * Sets both the PID and Feedforward gains.
     * 
     * @param kP The proportional gain.
     * @param kI The integral gain.
     * @param kD The derivative gain.
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kA The acceleration gain.
     * @param kG The gravity compensation gain.
     */
    void set_gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
        pid_.set_pid(kP, kI, kD);
        ff_.set_kS(kS);
        ff_.set_kV(kV);
        ff_.set_kA(kA);
        ff_.set_kG(kG);
    }

    /**
     * Sets the PID gains.
     * 
     * @param kP The proportional gain.
     * @param kI The integral gain.
     * @param kD The derivative gain.
     */
    void set_pid(double kP, double kI, double kD) { pid_.set_pid(kP, kI, kD); }

    /**
     * Sets the Feedforward gains.
     * 
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kA The acceleration gain.
     * @param kG The gravity compensation gain.
     */
    void set_ff(double kS, double kV, double kA, double kG) {
        ff_.set_kS(kS);
        ff_.set_kV(kV);
        ff_.set_kA(kA);
        ff_.set_kG(kG);
    }

    // PID-specific methods
    void set_p(double kP) { pid_.set_p(kP); }
    void set_i(double kI) { pid_.set_i(kI); }
    void set_d(double kD) { pid_.set_d(kD); }
    void set_tolerance(double tolerance) { pid_.set_tolerance(tolerance); }
    void set_i_zone(double i_zone) { pid_.set_i_zone(i_zone); }
    void set_i_limits(double i_min, double i_max) { pid_.set_i_limits(i_min, i_max); }
    void set_dt(double dt) { pid_.set_dt(dt); }
    void enable_continuous(double minimum_input = -1,
                           double maximum_input = 1) {
        pid_.enable_continuous(minimum_input, maximum_input);
    }
    void disable_continuous() { pid_.disable_continuous(); }
    void reset() { pid_.reset(); }
    void reset_i() { pid_.reset_i(); }

    // Feedforward-specific methods
    void set_kS(double kS) { ff_.set_kS(kS); }
    void set_kV(double kV) { ff_.set_kV(kV); }
    void set_kA(double kA) { ff_.set_kA(kA); }
    void set_kG(double kG) { ff_.set_kG(kG); }

    // Getters
    double p() const { return pid_.p(); }
    double i() const { return pid_.i(); }
    double d() const { return pid_.d(); }
    double tolerance() const { return pid_.tolerance(); }
    double i_zone() const { return pid_.i_zone(); }
    double dt() const { return pid_.dt(); }
    double kS() const { return ff_.kS(); }
    double kV() const { return ff_.kV(); }
    double kA() const { return ff_.kA(); }
    double kG() const { return ff_.kG(); }
    bool is_continuous() const { return pid_.is_continuous(); }
    bool at_setpoint() const { return pid_.at_setpoint(); }

    /**
     * Sets the setpoint.
     * 
     * @param setpoint The setpoint value.
     */
    void set_setpoint(double setpoint) { pid_.set_setpoint(setpoint); }

    /**
     * Returns the current setpoint.
     * 
     * @return The setpoint value.
     */
    double setpoint() const { return pid_.setpoint(); }

    /**
     * Sets the output limits.
     * 
     * @param minimum_output The minimum output value.
     * @param maximum_output The maximum output value.
     */
    void set_output_limits(double minimum_output, double maximum_output) {
        pid_.set_output_limits(minimum_output, maximum_output);
    }

    /**
     * Calculates the PIDFF output based on the current measurement.
     * Uses only kS and kG from feedforward (no velocity or acceleration terms).
     * 
     * @param measurement The current measurement value.
     * @param angle The current angle (for gravity compensation).
     * @return The calculated PIDFF output.
     */
    double calculate(double measurement, Rotation2d angle = Rotation2d()) {
        double pid_output = pid_.calculate(measurement);
        double ff_output =
            ff_.kS() * sign(pid_output) + ff_.kG() * angle.f_cos();
        return pid_output + ff_output;
    }

    /**
     * Calculates the PIDFF output based on the current measurement and a new setpoint.
     * Uses only kS and kG from feedforward (no velocity or acceleration terms).
     * 
     * @param setpoint The desired setpoint value.
     * @param measurement The current measurement value.
     * @param angle The current angle (for gravity compensation).
     * @return The calculated PIDFF output.
     */
    double calculate(double setpoint, double measurement,
                     Rotation2d angle = Rotation2d()) {
        pid_.set_setpoint(setpoint);
        return calculate(measurement, angle);
    }

    /**
     * Calculates the PIDFF output based on the current measurement and velocity/acceleration commands.
     * 
     * @param measurement The current measurement value.
     * @param v The velocity command.
     * @param a The acceleration command.
     * @param angle The current angle (for gravity compensation).
     * @return The calculated PIDFF output.
     */
    double calculate_with_ff(double measurement, double v, double a,
                             Rotation2d angle = Rotation2d()) {
        double pid_output = pid_.calculate(measurement);
        double ff_output = ff_.calculate(v, a, angle);
        return pid_output + ff_output;
    }

    /**
     * Calculates the PIDFF output based on a new setpoint, measurement, and velocity/acceleration commands.
     * 
     * @param setpoint The new setpoint value.
     * @param measurement The current measurement value.
     * @param v The velocity command.
     * @param a The acceleration command.
     * @param angle The current angle (for gravity compensation).
     * @return The calculated PIDFF output.
     */
    double calculate_with_ff(double setpoint, double measurement, double v,
                             double a, Rotation2d angle = Rotation2d()) {
        pid_.set_setpoint(setpoint);
        return calculate_with_ff(measurement, v, a, angle);
    }

   private:
    PID pid_;
    ArmFeedforward ff_;
};

}  // namespace core
