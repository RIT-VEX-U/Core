#pragma once

#include <cmath>
#include <memory>

#include "math/controls/pidff.h"
#include "math/geometry/rotation2d.h"
#include "math/math_util.h"
#include "math/trajectories/trapezoid_profile.h"

namespace core {

/**
 * A Motion Controller that combines trajectory generation using a trapezoidal profile
 * with feedback and feedforward control using PIDFF.
 * 
 * When given a target position, the controller will automatically generate a trajectory
 * and subsequent calls to calculate() will track along that trajectory.
 * 
 * This class does not handle timing that determines the current motion along the trajectory.
 * 
 * @author Ryan McGee, Jack Cammarata
 * @date 7/13/2022, 6/27/2025
 */
class MotionController {
   public:
    /**
     * Configuration for the complete motion controller.
     */
    typedef struct {
        PID::pid_config_t pid;
        Feedforward::feedforward_config_t ff;
        trapezoid_profile_config_t profile;
    } motion_controller_config_t;

    /**
     * Constructs a MotionController with the given configuration.
     * 
     * @param config The configuration parameters.
     */
    MotionController(motion_controller_config_t config)
        : config_(config),
          pidff_(config.pid.kP, config.pid.kI, config.pid.kD,
                 config.ff.kS, config.ff.kV, config.ff.kA, config.ff.kG,
                 config.pid.tolerance, config.pid.dt),
          current_position_(0.0),
          target_position_(0.0),
          profile_complete_(true),
          profile_start_time_(0.0),
          current_time_(0.0) {
        
        pidff_.set_i_zone(config.pid.i_zone);
        pidff_.set_i_limits(config.pid.i_min, config.pid.i_max);
        pidff_.set_output_limits(config.pid.minimum_output, config.pid.maximum_output);
        
        if (config.pid.continuous) {
            pidff_.enable_continuous(config.pid.minimum_input, config.pid.maximum_input);
        }
    }

    /**
     * Sets a new target position, which generates a new trajectory.
     * 
     * @param target_position The new target position.
     * @param current_position The current position (starting point).
     * @param current_time The current system time (used as the start time for the new trajectory).
     */
    void set_target(double target_position, double current_position, double current_time) {
        current_position_ = current_position;
        target_position_ = target_position;
        profile_start_time_ = current_time;
        profile_complete_ = false;
        
        // create a new trajectory from current position to target
        profile_ = std::unique_ptr<TrapezoidProfile>(new TrapezoidProfile(
            current_position, 
            target_position, 
            config_.profile.v_max, 
            config_.profile.accel,
            config_.profile.decel
        ));
    }

    /**
     * Calculates the motion controller output based on the current position and time.
     * 
     * @param current_position The current position measurement.
     * @param current_time The current system time.
     * @param angle The current angle (for gravity compensation).
     * @return The calculated control output.
     */
    double calculate(double current_position, double current_time, Rotation2d angle = Rotation2d()) {
        current_position_ = current_position;
        current_time_ = current_time;
        
        if (profile_complete_) {
            // if the profile is complete just hold position with pid
            return pidff_.calculate(target_position_, current_position, angle);
        }
        
        double profile_time = current_time - profile_start_time_;
        
        motion_t target_state = profile_->calculate(profile_time);
        
        // set flag if profile is complete (time wise, not position wise)
        if (profile_time >= profile_->total_time()) {
            profile_complete_ = true;
        }
        
        // use the profile's velocity and acceleration as feedforward inputs
        return pidff_.calculate_with_ff(
            target_state.pos,         // target position
            current_position,         // current position
            target_state.vel,         // velocity feedforward
            target_state.acc,         // acceleration feedforward
            angle                     // angle for gravity compensation
        );
    }

    /**
     * Checks if the motion profile is complete.
     * 
     * @return True if the profile has completed.
     */
    bool is_profile_complete() const {
        return profile_complete_;
    }

    /**
     * Checks if the controller is at the target position.
     * 
     * @return True if the current position is at the target position within tolerance.
     */
    bool at_target() const {
        return profile_complete_ && pidff_.at_setpoint();
    }

    /**
     * Gets the current target position.
     * 
     * @return The target position.
     */
    double target_position() const {
        return target_position_;
    }

    /**
     * Gets the most recent measured position.
     * 
     * @return The current position.
     */
    double current_position() const {
        return current_position_;
    }

    /**
     * Returns the PIDFF controller.
     * 
     * @return Reference to the internal PIDFF controller.
     */
    PIDFF& get_pidff() {
        return pidff_;
    }

    /**
     * Updates the motion profile configuration.
     * 
     * @param config The new trapezoid profile configuration.
     */
    void set_profile_config(trapezoid_profile_config_t config) {
        config_.profile = config;
    }

   private:
    motion_controller_config_t config_;
    PIDFF pidff_;
    std::unique_ptr<TrapezoidProfile> profile_;
    
    double current_position_;
    double target_position_;
    bool profile_complete_;
    double profile_start_time_;
    double current_time_;
};

}  // namespace core
