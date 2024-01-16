#include "../core/include/utils/controls/trapezoid_profile.h"
#include "../core/include/utils/math_util.h"
#include <cmath>
#include <iostream>

const double EPSILON = 0.000005;

inline double calc_pos(double t, double a, double v, double si) {
  return (0.5 * (a) * (t) * (t)) + ((v) * (t)) + (si);
}

inline double calc_vel(double t, double a, double vi) {
  return ((a) * (t)) + (vi);
}

TrapezoidProfile::TrapezoidProfile(double max_v, double accel)
    : si(0), sf(0), vi(0), vf(0), max_v(max_v), accel(accel), segments(),
      num_acceleration_phases(0), precalculated(false) {}

void TrapezoidProfile::set_max_v(double max_v) {
  this->max_v = max_v;

  this->precalculated = false;
}

void TrapezoidProfile::set_accel(double accel) {
  this->accel = accel;

  this->precalculated = false;
}

void TrapezoidProfile::set_endpts(double start, double end) {
  this->si = start;
  this->sf = end;

  this->precalculated = false;
}

void TrapezoidProfile::set_vel_endpts(double start, double end) {
  this->vi = start;
  this->vf = end;

  this->precalculated = false;
}

motion_t TrapezoidProfile::calculate_time_based(double time_s) {
  if (!this->precalculated) {
    precalculate();
}

  int segment_i = 0;

  // position, velocity, acceleration, and time at the beginning of the segment
  // we're in
  double segment_s = this->si;
  double segment_v = this->vi;
  double segment_a = this->segments[0].accel;
  double segment_t = 0;

  // skip phases based on time
  while (segment_i < MAX_TRAPEZOID_PROFILE_SEGMENTS &&
         time_s > segment_t + this->segments[segment_i].duration) {
    segment_t += this->segments[segment_i].duration;
    segment_s = this->segments[segment_i].pos_after;
    segment_v = this->segments[segment_i].vel_after;
    segment_i++;
    segment_a = this->segments[segment_i].accel;
  }

  motion_t out;

  // if we are beyond the last phase, return the position/velocity at the last
  // phase
  if (segment_i == MAX_TRAPEZOID_PROFILE_SEGMENTS) {
    out.accel = 0;
    out.pos = this->segments[MAX_TRAPEZOID_PROFILE_SEGMENTS - 1].pos_after;
    out.vel = this->segments[MAX_TRAPEZOID_PROFILE_SEGMENTS - 1].vel_after;
    return out;
  }

  // calculate based on time
  out.accel = this->segments[segment_i].accel;
  out.vel = calc_vel(time_s - segment_t, segment_a, segment_v);
  out.pos = calc_pos(time_s - segment_t, segment_a, segment_v, segment_s);
  return out;
}

motion_t TrapezoidProfile::calculate(double time_s, double pos_s) {
  if (!this->precalculated) {
    precalculate();
}

  // printf("%f %f\n", time_s, pos_s);

  int segment_i = 0;

  // position, velocity, acceleration, and time at the beginning of the segment
  // we're in
  double segment_s = this->si;
  double segment_v = this->vi;
  double segment_a = this->segments[0].accel;
  double segment_t = 0;

  // skip acceleration phases based on time
  while (segment_i < MAX_TRAPEZOID_PROFILE_SEGMENTS &&
         segment_i < this->num_acceleration_phases &&
         time_s > segment_t + this->segments[segment_i].duration) {
    segment_t += this->segments[segment_i].duration;
    segment_s = this->segments[segment_i].pos_after;
    segment_v = this->segments[segment_i].vel_after;
    segment_i++;
    segment_a = this->segments[segment_i].accel;
  }

  // skip other segments based on distance, if we are past the time segments
  if (segment_i >= this->num_acceleration_phases) {
    while (
        segment_i < MAX_TRAPEZOID_PROFILE_SEGMENTS &&
        ((this->si < this->sf && pos_s > this->segments[segment_i].pos_after) ||
         (this->si > this->sf &&
          pos_s < this->segments[segment_i].pos_after))) {
      segment_t += this->segments[segment_i].duration;
      segment_s = this->segments[segment_i].pos_after;
      segment_v = this->segments[segment_i].vel_after;
      segment_i++;
      segment_a = this->segments[segment_i].accel;
    }
  }

  motion_t out;

  // if we are beyond the last phase, return the position/velocity at the last
  // phase
  if (segment_i == MAX_TRAPEZOID_PROFILE_SEGMENTS) {
    out.accel = 0;
    out.pos = this->segments[MAX_TRAPEZOID_PROFILE_SEGMENTS - 1].pos_after;
    out.vel = this->segments[MAX_TRAPEZOID_PROFILE_SEGMENTS - 1].vel_after;
    return out;
  }

  // if we are in an acceleration phase, calculate based on time
  if (segment_i < this->num_acceleration_phases) {
    out.accel = segment_a;
    out.vel = calc_vel(time_s - segment_t, segment_a, segment_v);
    out.pos = calc_pos(time_s - segment_t, segment_a, segment_v, segment_s);
    return out;
  }

  // otherwise calculate based on distance
  out.accel = segment_a;
  out.pos = pos_s;
  // Calculating the final velocity in uniformly accelerated motion:
  // v_f^2 = v_i^2 + 2a(x_f - x_i)
  // v_f = sqrt(v_i^2 + 2a(x_f - x_i))
  out.vel = sqrt(segment_v * segment_v + 2 * segment_a * (pos_s - segment_s));

  // match the sign of the starting velocity of the segment
  if (segment_v < 0) {
    out.vel *= -1;
}

  return out;
}

double TrapezoidProfile::get_movement_time() const { return duration; }

bool TrapezoidProfile::precalculate() {
  for (auto &segment : this->segments) {
    segment.pos_after = 0;
    segment.vel_after = 0;
    segment.accel = 0;
    segment.duration = 0;
  }
  this->num_acceleration_phases = 0;

  if (this->accel < EPSILON) {
    printf("WARNING: trapezoid motion profile acceleration was negative, or "
           "too small\n");
    this->accel = fmin(EPSILON, fabs(this->accel));
  }

  if (this->max_v < EPSILON) {
    printf("WARNING: trapezoid motion profile maximum velocity was negative, "
           "or too small\n");
    this->max_v = fmin(EPSILON, fabs(this->max_v));
  }

  // make sure vf is within v_max
  if (fabs(this->vf) > this->max_v) {
    printf("WARNING: trapezoid motion profile target velocity is greater than "
           "maximum velocity\n");
    if (this->vf > this->max_v) {
      this->vf = this->max_v;
    } else {
      this->vf = -this->max_v;
}
  }

  // if displacement is + but vf is -, or if displacement is - but vf is +
  if ((this->si < this->sf && this->vf < 0) ||
      (this->si > this->sf && this->vf > 0)) {
    printf("WARNING: trapezoid motion profile target velocity is in the wrong "
           "direction\n");
    this->vf = 0;
  }

  double s = this->si, v = this->vi;

  double total_time = 0;

  for (auto &segment : this->segments) {
    segment = calculate_next_segment(s, v);
    total_time += segment.duration;

    if (fabs(segment.pos_after - this->sf) < EPSILON) {
      return true;
}

    // Check if xf is between x and x_next (meaning we overshot the end
    // position)
    if ((((this->si < this->sf) && (this->sf < segment.pos_after)) ||
         ((segment.pos_after < this->sf) && (this->sf < this->si))) &&
        fabs(this->sf - segment.pos_after) > EPSILON) {
      // Solve for the exact time to reach self.xf using the quadratic formula
      // Given: x = x0 + v0 * t + 0.5 * a * t^2
      double a_coeff = 0.5 * segment.accel;
      if (fabs(a_coeff) < EPSILON) {
        a_coeff = EPSILON;
}
      double b_coeff = v;
      double c_coeff = s - this->sf;

      double discriminant = b_coeff * b_coeff - 4.0 * a_coeff * c_coeff;
      if (discriminant >= 0) {
        double t1 = (-b_coeff + sqrt(discriminant)) / (2 * a_coeff);
        double t2 = (-b_coeff - sqrt(discriminant)) / (2 * a_coeff);

        double t_corrected = fmin(t1, t2);
        if (t_corrected < 0) {
          t_corrected = fmax(t1, t2);
        }
        segment.duration = t_corrected;
        segment.vel_after = calc_vel(segment.duration, segment.accel, v);
        segment.pos_after = calc_pos(segment.duration, segment.accel, v, s);
        return true;
      } else {
        printf("ERROR: No real solution to reach sf.\n");
        return false;
      }
    }

    v = segment.vel_after;
    s = segment.pos_after;
    duration = total_time;
  }

  printf("WARNING: trapezoid motion profile did not reach end position in %d "
         "segments (the maximum)\n",
         MAX_TRAPEZOID_PROFILE_SEGMENTS);
  return false;
}

trapezoid_profile_segment_t
TrapezoidProfile::calculate_kinetic_motion(double si, double vi,
                                           double v_target) {
  trapezoid_profile_segment_t m;

  m.duration = fabs(v_target - vi) / this->accel;
  m.vel_after = v_target;
  if (vi < v_target) {
    m.accel = this->accel;
  } else {
    m.accel = -this->accel;
  }

  m.pos_after = calc_pos(m.duration, m.accel, vi, si);
  return m;
}

trapezoid_profile_segment_t TrapezoidProfile::calculate_next_segment(double s,
                                                                     double v) {
  // d represents the direction of travel, + or -
  int d = 1;
  if (this->si > this->sf) {
    d = -1;
  }

  // if going the wrong way, come to a stop
  if ((d == 1 && v < -EPSILON) || (d == -1 && v > EPSILON)) {
    this->num_acceleration_phases += 1;
    return calculate_kinetic_motion(s, v, 0);
  }

  // if |v| > max_v, slow down
  if (v > this->max_v || v < -this->max_v) {
    this->num_acceleration_phases += 1;
    return calculate_kinetic_motion(s, v, d * this->max_v);
  }

  // how much distance would it take to reach vf in one segment?
  trapezoid_profile_segment_t beeline_vf =
      calculate_kinetic_motion(s, v, this->vf);
  if ((d == 1 && beeline_vf.pos_after > this->sf - EPSILON) ||
      (d == -1 && beeline_vf.pos_after <= this->sf + EPSILON)) {
    // we can't make it to vf - get as close as possible
    return beeline_vf;
  }

  // we can reach vf in one segment - choose the fastest speed so that that is
  // still possible
  double v_middle =
      d * sqrt(((v * v + this->vf * this->vf) / 2.0) +
               this->accel * fabs(this->sf - s) + this->vf * this->vf);
  // cap this at our max speed
  v_middle = fmax(-this->max_v, fmin(v_middle, this->max_v));

  // if we aren't at this speed yet, let's get there
  if (fabs(v_middle - v) > EPSILON) {
    this->num_acceleration_phases += 1;
    return calculate_kinetic_motion(s, v, v_middle);
  }

  // we are at our max speed - hold it until we have to start slowing down
  trapezoid_profile_segment_t out;
  out.accel = 0;
  double distance = d * fabs(this->sf - beeline_vf.pos_after);
  out.pos_after = s + distance;
  out.duration = distance / v_middle;
  out.vel_after = v_middle;
  return out;
}

double TrapezoidProfile::get_max_v() const { return max_v; };

double TrapezoidProfile::get_accel() const { return accel; }