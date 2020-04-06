#ifndef _SPLINE_
#define _SPLINE_

#include "pathfinder.h"

#include "subsystems/tank_drive.h"


class SplinePath
{
public:
  struct motion_profile_t
  {
    // drive_pid controls the speed at which the robot drives forward,
    // turn_p is for turning the robot based on the desired heading
    float drive_p = .1, drive_i = 0, drive_d = 0;
    float turn_p = .05;

    // Maximum velocity, acceleration, and jerk the robot is allowed to achieve (Jerk doesn't matter THAT much...)
    double max_v = 10, max_a = 20, max_j = 100;

    // kv and ka are constants fed into the spline system, and are multiplied by the
    // velocity setpoint and acceleration setpoint inside the main pathfinder stuff.
    // Generally 1/max_v is sufficient for kv (does NOT work for ka)
    float kv = 1.0 / max_v, ka = 0;

    // Time delta between loops inside the pathfinder calls
    double dt = .01;

    // Misc robot info
    double wheel_diam = 4;
    int ticks_per_rev = 900; // 300 with blues, 900 with greens, 1800 with reds
    double wheelbase_width = 11.5;
  };

  SplinePath(TankDrive *drive_system, vex::inertial *imu, vex::motor *l_enc, vex::motor *r_enc, motion_profile_t *motion_profile);

  bool run_path(Waypoint *point_list, int list_length);

private:
  bool run_path_init = true;
  double reset_heading = 0;

  TrajectoryCandidate candidate;
  Segment *center_traj, *left_traj, *right_traj;

  EncoderFollower *left_follower, *right_follower;
  EncoderConfig enc_conf;

  TankDrive *drive_system;
  vex::motor *l_enc, *r_enc;
  vex::inertial *imu;

  motion_profile_t *motion_profile;
};

#endif