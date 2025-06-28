#pragma once

#include <cmath>

// Struct representing a state along the motion profile containing position, velocity and acceleration.
typedef struct {
    double pos;  // 1D position
    double vel;  // 1D velocity
    double acc;  // 1D acceleration
} motion_t;

/**
 * Configuration for a trapezoidal motion profile.
 */
typedef struct {
    double v_max = 1.0;  // Maximum velocity
    double accel = 1.0;  // Acceleration
    double decel = 1.0;  // Deceleration (typically equal to accel)
} trapezoid_profile_config_t;

/**
 * Class representing a trapezoidal motion profile. This consists of either two or three stages. First, an acceleration
 * stage where the velocity increases to a maximum. Then a cruise stage where the velocity is constant, then a
 * deceleration stage where the velocity decreases to zero.
 * 
 * This implementation allows for different acceleration and deceleration rates.
 * 
 * This is best used with LQR, since it allows full state feedback, but it can also be used with PID and Feedforward.
 * 
 * @author Jack Cammarata
 * @date 3/28/2025
 */
class TrapezoidProfile {
   public:
    /**
     * Constructs a TrapezoidProfile.
     * 
     * @param x_initial The initial position.
     * @param x_target The target position.
     * @param v_max The maximum velocity.
     * @param accel The acceleration.
     * @param decel The deceleration.
     */
    TrapezoidProfile::TrapezoidProfile(const double& x_initial,
                                       const double& x_target,
                                       const double& v_max, const double& accel,
                                       const double& decel)
        : x_initial_(x_initial),
          x_target_(x_target),
          v_max_(v_max),
          accel_(accel),
          decel_(decel),
          distance_(x_target - x_initial) {
        direction_ = distance_ > 0 ? 1 : -1;
        dist_accel_ = 0.5 * (v_max_ * v_max_) / accel_;
        dist_decel_ = 0.5 * (v_max_ * v_max_) / decel_;
        dist_full_ = dist_accel_ + dist_decel_;

        if (std::abs(distance_) > dist_full_) {
            // if the velocity graph is trapezoidal we use max velocity and set cruise time and distance
            triangular_ = false;
            time_accel_ = v_max_ / accel_;
            time_decel_ = v_max_ / decel_;
            dist_cruise_ = std::abs(distance_) - dist_full_;
            time_cruise_ = dist_cruise_ / v_max_;
            time_total_ = time_accel_ + time_cruise_ + time_decel_;
        } else {
            // if the velocity graph is triangular we compute the peak velocity, and don't use cruise time or distance
            triangular_ = true;
            v_peak_ = sqrt((2 * std::abs(distance_) * accel_ * decel_) /
                           (accel_ + decel_));
            time_accel_ = v_peak_ / accel_;
            time_decel_ = v_peak_ / decel_;
            time_cruise_ = 0.0;
            time_total_ = time_accel_ + time_decel_;
        }
    }

    /**
     * Calculate the state along the motion profile at some given time.
     * 
     * @param t The time in seconds.
     * 
     * @return the state.
     */
    motion_t TrapezoidProfile::calculate(double t) {
        // clamp the input time to within the timeframe of the motion profile
        if (t < 0) {
            t = 0;
        } else if (t > time_total_) {
            t = time_total_;
        }

        double pos_local;
        double vel_local;
        double acc_local;

        // acceleration phase
        if (t < time_accel_) {
            pos_local = 0.5 * accel_ * (t * t);
            vel_local = accel_ * t;
            acc_local = accel_;
            // cruise phase only if it's not triangular
        } else if (!triangular_ && (t < time_accel_ + time_cruise_)) {
            pos_local = dist_accel_ + v_max_ * (t - time_accel_);
            vel_local = v_max_;
            acc_local = 0.0;
            // deceleration phase
        } else {
            double time_deceled;
            if (triangular_) {
                time_deceled = t - time_accel_;
                pos_local = (0.5 * accel_ * (time_accel_ * time_accel_)) +
                            (v_peak_ * time_deceled) -
                            (0.5 * decel_ * (time_deceled * time_deceled));
                vel_local = v_peak_ - (decel_ * time_deceled);
                acc_local = -decel_;
            } else {
                time_deceled = t - (time_accel_ + time_cruise_);
                pos_local = dist_accel_ +
                            (v_max_ * (time_cruise_ + time_deceled)) -
                            (0.5 * decel_ * (time_deceled * time_deceled));
                vel_local = v_max_ - (decel_ * time_deceled);
                acc_local = -decel_;
            }
        }

        double pos = x_initial_ + (direction_ * pos_local);
        double vel = direction_ * vel_local;
        double acc = direction_ * acc_local;
        return motion_t{pos, vel, acc};
    }

    /**
     * Returns the total time that the motion profile takes to complete.
     * 
     * @return the total time that the motion profile takes to complete.
     */
    double TrapezoidProfile::total_time() { return time_total_; }

   private:
    double x_initial_;
    double x_target_;
    double v_max_;
    double v_peak_;
    double accel_;
    double decel_;
    double distance_;
    double dist_accel_;
    double dist_decel_;
    double dist_cruise_;
    double dist_full_;

    double time_accel_;
    double time_decel_;
    double time_cruise_;
    double time_total_;

    bool triangular_;

    // 1 if positive, -1 if negative
    int direction_;
};
