#pragma once

#ifndef PI
#define PI 3.141592654
#endif

#include "vex.h"
#include "../core/include/utils/pid.h"
#include "../core/include/subsystems/odometry/odometry_tank.h"
#include <vector>

using namespace vex;

class TankDrive
{
public:

  struct tankdrive_config_t
  {
    PID::pid_config_t drive_pid;
    PID::pid_config_t turn_pid;
  };

  /**
   * Create the TankDrive object 
   */
  TankDrive(motor_group &left_motors, motor_group &right_motors, inertial &gyro_sensor, tankdrive_config_t &config, OdometryTank *odom=NULL);

  /**
   * Stops rotation of all the motors using their "brake mode"
   */
  void stop();

  /**
   * Drive the robot using differential style controls. left_motors controls the left motors,
   * right_motors controls the right motors.
   * 
   * left_motors and right_motors are in "percent": -1.0 -> 1.0
   */
  void drive_tank(double left, double right, int power=1);

  /**
   * Drive the robot using arcade style controls. forward_back controls the linear motion,
   * left_right controls the turning.
   * 
   * left_motors and right_motors are in "percent": -1.0 -> 1.0
   */
  void drive_arcade(double forward_back, double left_right, int power=1);

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

  /**
   * Use odometry to automatically drive the robot to a point on the field.
   * X, Y and Rotation are the final point and orientation we want the robot.
   *
   * Combing multiple drive_to_point's gives a pure pursuit system.
   */
  bool drive_to_point(double x, double y, double rot_deg, double speed);

  static double modify_inputs(double input, int power=2);

  /**
    * Returns points of the intersections of a line segment and a circle. The line 
    * segment is defined by two points, and the circle is defined by a center and radius.
    */
  std::vector<Vector::point_t> line_circle_intersections(Vector::point_t center, double r, Vector::point_t point1, Vector::point_t point2);

private:
  motor_group &left_motors;
  motor_group &right_motors;

  PID drive_pid;
  PID turn_pid;

  OdometryTank *odometry;

  position_t saved_pos;

  bool initialize_func = true;
};
