#pragma once
#include "../core/include/utils/pid.h"
#include "../core/include/utils/feedback_base.h"

/**
 * Main robot characterization struct.
 * This will be passed to all the major subsystems 
 * that require info about the robot.
 * All distance measurements are in inches.
 */
typedef struct
{  
  double robot_radius; ///< if you were to draw a circle with this radius, the robot would be entirely contained within it

  double odom_wheel_diam; ///< the diameter of the wheels used for 
  double odom_gear_ratio; ///< the ratio of the odometry wheel to the encoder reading odometry data
  double dist_between_wheels; ///< the distance between centers of the central drive wheels 

  double drive_correction_cutoff; ///< the distance at which to stop trying to turn towards the target. If we are less than this value, we can continue driving forward to minimize our distance but will not try to spin around to point directly at the target

  Feedback *drive_feedback; ///< the default feedback for autonomous driving
  Feedback *turn_feedback; ///< the defualt feedback for autonomous turning
  PID::pid_config_t correction_pid; ///< the pid controller to keep the robot driving in as straight a line as possible

} robot_specs_t;