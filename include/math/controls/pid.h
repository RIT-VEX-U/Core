#pragma once

#include <cmath>
#include <algorithm>
#include <limits>

#include "math/math_util.h"

namespace core {

/**
 * Implements a PID controller with some additional features.
 * 
 * This class allows for setting PID gains, tolerance, integral zone,
 * integral limits, output limits, and continuous mode for angle wrapping.
 * 
 * The PID formula is:
 * output = kP * error + kI * integral(error) + kD * derivative(error)
 * 
 * @author Ryan McGee, Jack Cammarata
 * @date 4/3/2020, 6/27/2025
 */
class PID {
public:
    typedef struct {
        double kP = 0.0;
        double kI = 0.0;
        double kD = 0.0;

        double tolerance = 0.05; // default tolerance

        double dt = 0.01; // default period in seconds for derivative

        double i_zone = 0.0; // integral zone
        double i_min = -std::numeric_limits<double>::infinity(); // minimum integral limit
        double i_max = std::numeric_limits<double>::infinity(); // maximum integral limit
        
        double minimum_output = -12.0; // default minimum output limit
        double maximum_output = 12.0; // default maximum output limit

        bool continuous = false; // whether the PID is in continuous mode
        double minimum_input = -1.0; // minimum input for continuous mode
        double maximum_input = 1.0; // maximum input for continuous mode
    } pid_config_t;


    PID(double kP, double kI, double kD, double tolerance, double dt = 0.01) : kP_(kP), kI_(kI), kD_(kD), dt_(dt) {}
    PID(PID::pid_config_t config)
        : kP_(config.kP), kI_(config.kI), kD_(config.kD),
          tolerance_(config.tolerance), dt_(config.dt),
          i_zone_(config.i_zone), i_min_(config.i_min), i_max_(config.i_max),
          minimum_output_(config.minimum_output), maximum_output_(config.maximum_output),
          continuous_(config.continuous), minimum_input_(config.minimum_input),
          maximum_input_(config.maximum_input) {}
          
    PID(const core::PID&) = default;
    PID(core::PID&&) = default;
    PID& operator=(const core::PID&) = default;
    PID& operator=(core::PID&&) = default;

    /**
     * Sets the PID gains.
     * 
     * @param kP The proportional gain.
     * @param kI The integral gain.
     * @param kD The derivative gain.
     */
    void set_pid(double kP, double kI, double kD) {
        kP_ = kP;
        kI_ = kI;
        kD_ = kD;
    }

    /**
     * Sets the Proportional gain.
     * 
     * @param kP The proportional gain.
     */
    void set_p(double kP) {
        kP_ = kP;
    }

    /**
     * Sets the Integral gain.
     * 
     * @param kI The integral gain.
     */
    void set_i(double kI) {
        kI_ = kI;
    }

    /**
     * Sets the Derivative gain.
     * 
     * @param kD The derivative gain.
     */
    void set_d(double kD) {
        kD_ = kD;
    }

    /**
     * Sets the tolerance for whether the setpoint is reached.
     * 
     * @param tolerance The tolerance value. If absolute error < tol then it's at the setpoint.
     */
    void set_tolerance(double tolerance) {
        tolerance_ = tolerance;
    }

    /**
     * Sets the integral zone. If the absolute error is less than this value, the integral term will accumulate.
     * If the absolute error is greater than this value, the accumulated error will be reset to zero.
     * 
     * @param i_zone The integral zone value.
     */
    void set_i_zone(double i_zone) {
        i_zone_ = i_zone;
    }

    /**
     * Returns the proportional gain.
     * 
     * @return The proportional gain.
     */
    double p() const { return kP_; }

    /**
     * Returns the integral gain.
     * 
     * @return The integral gain.
     */
    double i() const { return kI_; }

    /**
     * Returns the derivative gain.
     * 
     * @return The derivative gain.
     */
    double d() const { return kD_; }

    /**
     * Returns the tolerance.
     * 
     * @return The tolerance value.
     */
    double tolerance() const { return tolerance_; }

    /**
     * Returns the integral zone.
     * 
     * @return The integral zone value.
     */
    double i_zone() const { return i_zone_; }

    /**
     * Sets limits on the effect of the integral term.
     * 
     * @param i_min The minimum value for the integral term.
     * @param i_max The maximum value for the integral term.
     */
    void set_i_limits(double i_min, double i_max) {
        i_min_ = i_min;
        i_max_ = i_max;
    }

    /**
     * Sets the output limits, usually -12V to 12V for a motor.
     * 
     * @param minimum_output The minimum output value.
     * @param maximum_output The maximum output value.
     */
    void set_output_limits(double minimum_output, double maximum_output) {
        minimum_output_ = minimum_output;
        maximum_output_ = maximum_output;
    }

    /**
     * Returns the time period in seconds.
     * 
     * @return The time period in seconds.
     */
    double dt() const { return dt_; }

    /**
     * Sets the update time period in seconds.
     * 
     * @param dt The time period in seconds.
     */
    void set_dt(double dt) {
        dt_ = dt;
    }

    /**
     * Sets the setpoint.
     * 
     * @param setpoint The setpoint value.
     */
    void set_setpoint(double setpoint) {
        setpoint_ = setpoint;
    }

    /**
     * Returns the current setpoint.
     * 
     * @return The setpoint value.
     */
    double setpoint() const {
        return setpoint_;
    }

    /**
     * Checks if the PID controller is in continuous mode.
     * 
     * @return True if in continuous mode.
     */
    bool is_continuous() const {
        return continuous_;
    }

    /**
     * Enables continuous mode and sets the input range.
     * This wraps the inputs, mostly useful for angles.
     * 
     * @param minimum_input The minimum input value (default -1).
     * @param maximum_input The maximum input value (default 1).
     */
    void enable_continuous(double minimum_input = -1, double maximum_input = 1) {
        continuous_ = true;
        minimum_input_ = minimum_input;
        maximum_input_ = maximum_input;
    }

    /**
     * Disables continuous mode.
     */
    void disable_continuous() {
        continuous_ = false;
    }

    /**
     * Checks if the measured value is at the setpoint within the tolerance.
     * 
     * @return True if the error is within the tolerance.
     */
    bool at_setpoint() const {
        return std::abs(error_) < tolerance_;
    }

    /**
     * Calculates the PID output based on the current measurement.
     * 
     * @param measurement The current measurement value.
     * @return The calculated PID output.
     */
    double calculate(double measurement) {
        measurement_ = measurement;
        prev_error_ = error_;

        if (continuous_) {
            
        } else {
            error_ = setpoint_ - measurement_;
        }

        error_derivative_ = (error_ - prev_error_) / dt_;

        if (std::abs(error_) < i_zone_) {
            total_error_ = core::clamp(total_error_ + error_ * dt_, i_min_, i_max_);
        } else {
            total_error_ = 0;
        }

        return core::clamp(
            kP_ * error_ + kI_ * total_error_ + kD_ * error_derivative_,
            minimum_output_,
            maximum_output_
        );
    }

    /**
     * Calculates the PID output based on a setpoint and current measurement.
     * 
     * @param setpoint The desired setpoint value.
     * @param measurement The current measurement value.
     * @return The calculated PID output.
     */
    double calculate(double setpoint, double measurement) {
        setpoint_ = setpoint;
        return calculate(measurement);
    }

    /**
     * Resets the PID controller.
     */
    void reset() {
        error_ = 0;
        prev_error_ = 0;
        error_derivative_ = 0;
        total_error_ = 0;
    }

    /**
     * Resets the integral term only.
     */
    void reset_i() {
        total_error_ = 0;
    }

private:
    double kP_;
    double kI_;
    double kD_;
    double dt_;
    double i_zone_ = std::numeric_limits<double>::infinity();
    double i_max_ = std::numeric_limits<double>::infinity();
    double i_min_ = -std::numeric_limits<double>::infinity();
    double tolerance_ = 0.05;

    bool continuous_ = false;
    double minimum_input_ = -1;
    double maximum_input_ = 1;

    double setpoint_;
    double measurement_;

    double error_;
    double error_derivative_;
    double prev_error_;
    double total_error_;

    double maximum_output_ = std::numeric_limits<double>::infinity();
    double minimum_output_ = -std::numeric_limits<double>::infinity();
};

} // namespace core