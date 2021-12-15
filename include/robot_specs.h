#pragma once
#include "../core/include/utils/pid.h"

/**
 * Main robot characterization struct.
 * This will be passed to all the major subsystems 
 * that require info about the robot.
 * All distance measurements are in inches.
 */
typedef struct 
{
  double robot_radius;

  double odom_wheel_diam;
  double odom_gear_ratio;
  double dist_between_wheels;

  double drive_correction_cutoff;

  PID::pid_config_t drive_pid;
  PID::pid_config_t turn_pid;
  PID::pid_config_t correction_pid;

} robot_specs_t;