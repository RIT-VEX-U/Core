#pragma once

#include <cmath>

namespace core {

/**
 * A BangBang controller is a simple feedback controller that outputs either
 * 1 or 0, 1 if the measurement is less than the setpoint, and 0 otherwise.
 * Multiply by the maximum voltage you're willing to use.
 * 
 * This should really only be used for velocity control of high inertia
 * systems like flywheels, where high acceleration is most important. For anything
 * else don't use this.
 * 
 * DO NOT use this with a motor that isn't in coast mode since it will oscillate
 * very fast and probably burn it out.
 * 
 * @author Jack Cammarata
 * @date 6/27/2025
 */
class BangBang {
   public:
    /**
     * Constructs a BangBang controller with the given tolerance and setpoint.
     * 
     * @param tolerance The tolerance for the setpoint.
     * @param setpoint The setpoint to compare against.
     */
    BangBang(double tolerance, double setpoint)
        : tolerance_(tolerance), setpoint_(setpoint) {}

    BangBang(const BangBang&) = default;
    BangBang(BangBang&&) = default;
    BangBang& operator=(const BangBang&) = default;
    BangBang& operator=(BangBang&&) = default;

    /**
     * Calculates the output based on the current measurement and setpoint.
     * 
     * @param measurement The current measurement value.
     * @param setpoint The setpoint to compare against.
     * @return 1 if the measurement is less than the setpoint, 0 otherwise.
     */
    double calculate(double measurement, double setpoint) {
        measurement_ = measurement;
        setpoint_ = setpoint;

        return measurement < setpoint ? 1 : 0;
    }

    /**
     * Calculates the output based on the current measurement and setpoint.
     * 
     * @param measurement The current measurement value.
     * @return 1 if the measurement is less than the setpoint, 0 otherwise.
     */
    double calculate(double measurement) {
        return calculate(measurement, setpoint_);
    }

    /**
     * Checks if the current measurement is at the setpoint within the tolerance.
     * 
     * @return True if the measurement is within the tolerance of the setpoint.
     */
    bool at_setpoint() {
        return std::abs(setpoint_ - measurement_) < tolerance_;
    }

    double tolerance() const { return tolerance_; }
    double setpoint() const { return setpoint_; }

    void set_tolerance(double tolerance) { tolerance_ = tolerance; }
    void set_setpoint(double setpoint) { setpoint_ = setpoint; }

   private:
    double tolerance_;
    double setpoint_;
    double measurement_;
};

}  // namespace core
