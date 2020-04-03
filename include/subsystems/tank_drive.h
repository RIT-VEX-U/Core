#ifndef _TANKDRIVE_
#define _TANKDRIVE_

#define PI 3.141592654

#include "vex.h"
#include "utils/pid.h"

using namespace vex;

class TankDrive
{
public:

  struct tankdrive_config_t
  {
    PID::pid_config_t *drive_pid;
    PID::pid_config_t *turn_pid;

    double wheel_diam;
  };

  /**
   * Create the TankDrive object 
   */
  TankDrive(motor_group *left_motors, motor_group *right_motors, gyro *gyro_sensor, tankdrive_config_t *config);

  /**
   * Drive the robot using differential style controls. left_motors controls the left motors,
   * right_motors controls the right motors.
   * 
   * left_motors and right_motors are in "percent": -1.0 -> 1.0
   */
  void drive_tank(double left, double right);

  /**
   * Drive the robot using arcade style controls. forward_back controls the linear motion,
   * left_right controls the turning.
   * 
   * left_motors and right_motors are in "percent": -1.0 -> 1.0
   */
  void drive_arcade(double forward_back, double left_right);

  /**
   * Autonomously drive the robot X inches forward (Negative for backwards), with a maximum speed
   * of percent_speed (-1.0 -> 1.0).
   * 
   * Uses a PID loop for it's control.
   */
  bool drive_forward(double inches, double percent_speed);

  /**
   * Autonomously turn the robot X degrees to the right (negative for left), with a maximum motor speed
   * of percent_speed (-1.0 -> 1.0)
   * 
   * Uses a PID loop for it's control.
   */
  bool turn_degrees(double degrees, double percent_speed);

private:
  tankdrive_config_t *config;

  motor_group *left_motors;
  motor_group *right_motors;

  PID drive_pid;
  PID turn_pid;

  gyro *gyro_sensor;

  bool initialize_func = true;
};

#endif