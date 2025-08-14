#include <cmath>

#include "core/utils/controls/trapezoid_profile.h"

/**
 * Constructs a TrapezoidProfile.
 *
 * @param x_initial The initial position.
 * @param x_target The target position.
 * @param v_max The maximum velocity.
 * @param accel The acceleration.
 * @param decel The deceleration.
 */
TrapezoidProfile::TrapezoidProfile(const double &x_initial, const double &x_target, const double &v_max, const double &accel, const double &decel)
    : x_initial_(x_initial), x_target_(x_target), v_max_(v_max), accel_(accel), decel_(decel),
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
    v_peak_ = sqrt((2 * std::abs(distance_) * accel_ * decel_) / (accel_ + decel_));
    time_accel_ = v_peak_ / accel_;
    time_decel_ = v_peak_ / decel_;
    time_cruise_ = 0.0;
    time_total_ = time_accel_ + time_decel_;
  }
}

/**
 * Calculate the state along the motion profile after some given time.
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
      pos_local = (0.5 * accel_ * (time_accel_ * time_accel_)) + (v_peak_ * time_deceled) -
                  (0.5 * decel_ * (time_deceled * time_deceled));
      vel_local = v_peak_ - (decel_ * time_deceled);
      acc_local = -decel_;
    } else {
      time_deceled = t - (time_accel_ + time_cruise_);
      pos_local =
        dist_accel_ + (v_max_ * (time_cruise_ + time_deceled)) - (0.5 * decel_ * (time_deceled * time_deceled));
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