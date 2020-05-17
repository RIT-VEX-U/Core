#include "../Core/include/utils/spline_path.h"

SplinePath::SplinePath(TankDrive *drive_system, vex::inertial *imu, vex::motor *l_enc, vex::motor *r_enc, motion_profile_t *motion_profile)
: drive_system(drive_system), l_enc(l_enc), r_enc(r_enc), imu(imu), motion_profile(motion_profile)
{
}

/**
 * Makes the robot follow a predetermined path defined by point_list array,
 * with list_length number of waypoints. The first waypoint should be the
 * starting position.
 * 
 * Returns true when the path has finished
 */
bool SplinePath::run_path(Waypoint *point_list, int list_length)
{
  if (run_path_init)
  {
    // Set up Pathfinder's "EncoderConfig" struct that will be fed into the path generation
    enc_conf.initial_position = 0;
    enc_conf.ticks_per_revolution = motion_profile->ticks_per_rev;
    enc_conf.wheel_circumference = motion_profile->wheel_diam * PI;
    enc_conf.kp = motion_profile->drive_p;
    enc_conf.ki = motion_profile->drive_i;
    enc_conf.kd = motion_profile->drive_d;
    enc_conf.kv = motion_profile->kv;
    enc_conf.ka = motion_profile->ka;
    
    // Prepare the "trajectory candidate" with information about the curve (max velocities / accels, type of curve, etc)
    pathfinder_prepare(point_list, list_length, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_LOW, motion_profile->dt,
                       motion_profile->max_v, motion_profile->max_a, motion_profile->max_j, &candidate);

    // Allocate the memory for the center, left and right "trajectories", or paths.
    center_traj = (Segment *)malloc(sizeof(Segment) * candidate.length);
    left_traj = (Segment*) malloc(sizeof(Segment) * candidate.length);
    right_traj = (Segment*) malloc(sizeof(Segment) * candidate.length);

    // Generate the main center trajectory
    pathfinder_generate(&candidate, center_traj);

    // Generate the left wheel and right wheel paths from the center trajectory
    pathfinder_modify_tank(center_traj, candidate.length, left_traj, right_traj, motion_profile->wheelbase_width);
    
    // Create the "follower" structs, which contains info about the current state of each path when it is running
    // (e.g. current error for PID, whether it is finished)
    left_follower = (EncoderFollower *)malloc(sizeof(EncoderFollower));
    right_follower = (EncoderFollower *)malloc(sizeof(EncoderFollower));

    reset_heading = imu->rotation();

    // Make sure this only runs once per run
    run_path_init = false;
  }

  double lout = pathfinder_follow_encoder(enc_conf, left_follower, left_traj, candidate.length, l_enc->position(rotationUnits::raw));
  double rout = pathfinder_follow_encoder(enc_conf, right_follower, right_traj, candidate.length, r_enc->position(rotationUnits::raw));

  double in_heading = r2d(left_follower->heading);
  if(in_heading > 180)
    in_heading -= 360;
  double heading_error = in_heading + (imu->rotation() - reset_heading);

  lout -= motion_profile->turn_p * heading_error;
  rout += motion_profile->turn_p * heading_error;

  drive_system->drive_tank(lout, rout);

  if(left_follower->finished && right_follower->finished)
  {
    // Actively set all velocities of the wheels to 0
    drive_system->stop();

    // Free the memory allocated in the last run
    free(center_traj);
    free(left_traj);
    free(right_traj);
    free(left_follower);
    free(right_follower);

    // Re-run initialization for the next path
    run_path_init = true;
    return true;
  }

  return false;
}