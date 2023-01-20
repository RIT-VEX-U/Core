#pragma once

#ifndef PI
#define PI 3.141592654
#endif

#include "vex.h"
#include "../core/include/subsystems/odometry/odometry_tank.h"
#include "../core/include/utils/pid.h"
#include "../core/include/utils/feedback_base.h"
#include "../core/include/robot_specs.h"
#include "../core/src/utils/pure_pursuit.cpp"
#include <vector>


using namespace vex;

class TankDrive
{
public:

  /**
   * Create the TankDrive object 
   */
  TankDrive(motor_group &left_motors, motor_group &right_motors, robot_specs_t &config, OdometryTank *odom=NULL);

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
  void drive_tank(double left, double right, int power=1, bool isdriver=false);

  /**
   * Drive the robot using arcade style controls. forward_back controls the linear motion,
   * left_right controls the turning.
   * 
   * left_motors and right_motors are in "percent": -1.0 -> 1.0
   */
  void drive_arcade(double forward_back, double left_right, int power=1);

  /**
   * Use odometry to automatically drive the robot to a point on the field.
   * X and Y is the final point we want the robot.
   *
   * Returns whether or not the robot has reached it's destination.
   * @param x          the x position of the target
   * @param y          the y position of the target
   * @param dir        the direction we want to travel forward and backward
   * @param feedback   the feedback controller we will use to travel. controls the rate at which we accelerate and drive.
   * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
   */
  bool drive_forward(double inches, directionType dir, Feedback &feedback, double max_speed=1);
  /**
   * Autonomously turn the robot X degrees to counterclockwise (negative for clockwise), with a maximum motor speed
   * of percent_speed (-1.0 -> 1.0)
   * 
   * Uses the specified feedback for it's control.
   * 
   * @param degrees     degrees by which we will turn relative to the robot (+) turns ccw, (-) turns cw
   * @param feedback    the feedback controller we will use to travel. controls the rate at which we accelerate and drive.
   * @param max_speed   the maximum percentage of robot speed at which the robot will travel. 1 = full power
   */

  bool drive_forward(double inches, directionType dir, double max_speed=1);

  /**
   * Autonomously turn the robot X degrees to the right (negative for left), with a maximum motor speed
   * of percent_speed (-1.0 -> 1.0)
   * 
   * Uses PID + Feedforward for it's control.
   * 
   * @param degrees     degrees by which we will turn relative to the robot (+) turns ccw, (-) turns cw
   * @param feedback    the feedback controller we will use to travel. controls the rate at which we accelerate and drive.
   * @param max_speed   the maximum percentage of robot speed at which the robot will travel. 1 = full power
   */
  bool turn_degrees(double degrees, Feedback &feedback, double max_speed=1);
  /**
   * Autonomously turn the robot X degrees to counterclockwise (negative for clockwise), with a maximum motor speed
   * of percent_speed (-1.0 -> 1.0)
   * 
   * Uses the defualt turning feedback of the drive system.
   * 
   * @param degrees     degrees by which we will turn relative to the robot (+) turns ccw, (-) turns cw
   * @param feedback    the feedback controller we will use to travel. controls the rate at which we accelerate and drive.
   * @param max_speed   the maximum percentage of robot speed at which the robot will travel. 1 = full power
   */  bool turn_degrees(double degrees, double max_speed=1);

  /**
   * Use odometry to automatically drive the robot to a point on the field.
   * X and Y is the final point we want the robot.
   *
   * Returns whether or not the robot has reached it's destination.
   * @param x          the x position of the target
   * @param y          the y position of the target
   * @param dir        the direction we want to travel forward and backward
   * @param feedback   the feedback controller we will use to travel. controls the rate at which we accelerate and drive.
   * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
   */
  bool drive_to_point(double x, double y, vex::directionType dir, Feedback &feedback, double max_speed=1);
  
  /**
   * Use odometry to automatically drive the robot to a point on the field.
   * X and Y is the final point we want the robot.
   * Here we use the default feedback controller from the drive_sys
   *
   * Returns whether or not the robot has reached it's destination.
   * @param x          the x position of the target
   * @param y          the y position of the target
   * @param dir        the direction we want to travel forward and backward
   * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
   */
  bool drive_to_point(double x, double y, vex::directionType dir, double max_speed=1);

  /**
   * Turn the robot in place to an exact heading relative to the field.
   * 0 is forward.
   * 
   * @param heading_deg the heading to which we will turn 
   * @param feedback    the feedback controller we will use to travel. controls the rate at which we accelerate and drive.
   * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
   */
  bool turn_to_heading(double heading_deg, Feedback &feedback, double max_speed=1);
  /**
   * Turn the robot in place to an exact heading relative to the field.
   * 0 is forward. Uses the defualt turn feedback of the drive system 
   * 
   * @param heading_deg the heading to which we will turn 
   * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
   */
  bool turn_to_heading(double heading_deg, double max_speed=1);

  /**
   * Reset the initialization for autonomous drive functions
   */
  void reset_auto();

  /**
   * Create a curve for the inputs, so that drivers have more control at lower speeds.
   * Curves are exponential, with the deault being squaring the inputs.
   */
  static double modify_inputs(double input, int power=2);

  /**
   * Follow a hermite curve using the pure pursuit algorithm.
   * 
   * @param path The hermite curve for the robot to take. Must have 2 or more points.
   * @param dir Whether the robot should move forward or backwards
   * @param radius How the pure pursuit radius, in inches, for finding the lookahead point
   * @param res The number of points to use along the path; the hermite curve is split up into "res" individual points.
   * @param feedback The feedback controller to use
   * @param max_speed Robot's maximum speed throughout the path, between 0 and 1.0
   */
  bool pure_pursuit(std::vector<PurePursuit::hermite_point> path, directionType dir, double radius, double res, Feedback &feedback, double max_speed=1);

private:
  motor_group &left_motors;
  motor_group &right_motors;

  PID correction_pid;
  Feedback *drive_default_feedback = NULL;
  Feedback *turn_default_feedback = NULL;

  OdometryTank *odometry;

  robot_specs_t &config;

  bool func_initialized = false;
  bool is_pure_pursuit = false;
};
