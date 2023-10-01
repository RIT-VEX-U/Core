#include "../core/include/utils/geometry.h"
#include "../core/include/subsystems/tank_drive.h"
#include "../core/include/utils/math_util.h"
#include "../core/include/utils/pidff.h"
#include "../core/include/utils/command_structure/drive_commands.h"

TankDrive::TankDrive(motor_group &left_motors, motor_group &right_motors, robot_specs_t &config, OdometryBase *odom)
    : left_motors(left_motors), right_motors(right_motors), correction_pid(config.correction_pid), odometry(odom), config(config)
{
  drive_default_feedback = config.drive_feedback;
  turn_default_feedback = config.turn_feedback;
}

AutoCommand *TankDrive::DriveToPointCmd(Feedback &fb, point_t pt, vex::directionType dir, double max_speed)
{
  return new DriveToPointCommand(*this, fb, pt, dir, max_speed);
}

AutoCommand *TankDrive::DriveToPointCmd(point_t pt, vex::directionType dir, double max_speed)
{
  return new DriveToPointCommand(*this, *drive_default_feedback, pt, dir, max_speed);
}

AutoCommand *TankDrive::DriveForwardCmd(double dist, vex::directionType dir, double max_speed)
{
  return new DriveForwardCommand(*this, *drive_default_feedback, dist, dir, max_speed);
}

AutoCommand *TankDrive::DriveForwardCmd(Feedback &fb, double dist, vex::directionType dir, double max_speed)
{
  return new DriveForwardCommand(*this, fb, dist, dir, max_speed);
}

AutoCommand *TankDrive::TurnToHeadingCmd(double heading, double max_speed)
{
  return new TurnToHeadingCommand(*this, *turn_default_feedback, heading, max_speed);
}
AutoCommand *TankDrive::TurnToHeadingCmd(Feedback &fb, double heading, double max_speed)
{
  return new TurnToHeadingCommand(*this, fb, heading, max_speed);
}

AutoCommand *TankDrive::TurnDegreesCmd(double degrees, double max_speed)
{
  return new TurnDegreesCommand(*this, *turn_default_feedback, degrees, max_speed);
}
AutoCommand *TankDrive::TurnDegreesCmd(Feedback &fb, double degrees, double max_speed)
{
  return new TurnDegreesCommand(*this, fb, degrees, max_speed);
}
AutoCommand *TankDrive::PurePursuitCmd(std::vector<point_t> path, directionType dir, double radius, double max_speed)
{
  return new PurePursuitCommand(*this, *drive_default_feedback, path, dir, radius, max_speed);
}
AutoCommand *TankDrive::PurePursuitCmd(Feedback &feedback, std::vector<point_t> path, directionType dir, double radius, double max_speed)
{
  return new PurePursuitCommand(*this, feedback, path, dir, radius, max_speed);
}

/**
 * Reset the initialization for autonomous drive functions
 */
void TankDrive::reset_auto()
{
  func_initialized = false;
}

/**
 * Stops rotation of all the motors using their "brake mode"
 */
void TankDrive::stop()
{
  left_motors.stop();
  right_motors.stop();
}

/**
 * Drive the robot using differential style controls. left_motors controls the left motors,
 * right_motors controls the right motors.
 *
 * left_motors and right_motors are in "percent": -1.0 -> 1.0
 */
void TankDrive::drive_tank(double left, double right, int power)
{
  left = modify_inputs(left, power);
  right = modify_inputs(right, power);

  left_motors.spin(directionType::fwd, left * 12, voltageUnits::volt);
  right_motors.spin(directionType::fwd, right * 12, voltageUnits::volt);
}

/**
 * Drive the robot using arcade style controls. forward_back controls the linear motion,
 * left_right controls the turning.
 *
 * left_motors and right_motors are in "percent": -1.0 -> 1.0
 */
void TankDrive::drive_arcade(double forward_back, double left_right, int power)
{
  forward_back = modify_inputs(forward_back, power);
  left_right = modify_inputs(left_right, power);

  double left = forward_back + left_right;
  double right = forward_back - left_right;

  left_motors.spin(directionType::fwd, left * 12, voltageUnits::volt);
  right_motors.spin(directionType::fwd, right * 12, voltageUnits::volt);
}

/**
 * Use odometry to drive forward a certain distance using a custom feedback controller
 *
 * Returns whether or not the robot has reached it's destination.
 * @param inches     the distance to drive forward
 * @param dir        the direction we want to travel forward and backward
 * @param feedback   the custom feedback controller we will use to travel. controls the rate at which we accelerate and drive.
 * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
 */
bool TankDrive::drive_forward(double inches, directionType dir, Feedback &feedback, double max_speed)
{
  static pose_t pos_setpt;

  // We can't run the auto drive function without odometry
  if (odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }

  // Generate a point X inches forward of the current position, on first startup
  if (!func_initialized)
  {
    pose_t cur_pos = odometry->get_position();

    // forwards is positive Y axis, backwards is negative
    if (dir == directionType::rev)
      inches = -fabs(inches);
    else
      inches = fabs(inches);

    // Use vector math to get an X and Y
    Vector2D cur_pos_vec({.x = cur_pos.x, .y = cur_pos.y});
    Vector2D delta_pos_vec(deg2rad(cur_pos.rot), inches);
    Vector2D setpt_vec = cur_pos_vec + delta_pos_vec;

    // Save the new X and Y values
    pos_setpt = {.x = setpt_vec.get_x(), .y = setpt_vec.get_y()};
  }

  // Call the drive_to_point with updated point values
  return drive_to_point(pos_setpt.x, pos_setpt.y, dir, feedback, max_speed);
}
/**
 * Autonomously drive the robot forward a certain distance
 *
 *
 * @param inches      degrees by which we will turn relative to the robot (+) turns ccw, (-) turns cw
 * @param dir        the direction we want to travel forward and backward
 * @param max_speed   the maximum percentage of robot speed at which the robot will travel. 1 = full power
 * @return true if we have finished driving to our point
 */
bool TankDrive::drive_forward(double inches, directionType dir, double max_speed)
{
  if (drive_default_feedback != NULL)
    return drive_forward(inches, dir, *drive_default_feedback, max_speed);

  printf("tank_drive.cpp: Cannot run drive_forward without a feedback controller!\n");
  fflush(stdout);
  return true;
}

/**
 * Autonomously turn the robot X degrees to counterclockwise (negative for clockwise), with a maximum motor speed
 * of percent_speed (-1.0 -> 1.0)
 *
 * Uses the specified feedback for it's control.
 *
 * @param degrees     degrees by which we will turn relative to the robot (+) turns ccw, (-) turns cw
 * @param feedback    the feedback controller we will use to travel. controls the rate at which we accelerate and drive.
 * @param max_speed   the maximum percentage of robot speed at which the robot will travel. 1 = full power
 * @return true if we have turned our target number of degrees
 */
bool TankDrive::turn_degrees(double degrees, Feedback &feedback, double max_speed)
{
  // We can't run the auto drive function without odometry
  if (odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }

  static double target_heading = 0;

  // On the first run of the funciton, reset the gyro position and PID
  if (!func_initialized)
  {
    double start_heading = odometry->get_position().rot;
    target_heading = start_heading + degrees;
  }

  return turn_to_heading(target_heading, feedback, max_speed);
}

/**
 * Autonomously turn the robot X degrees to counterclockwise (negative for clockwise), with a maximum motor speed
 * of percent_speed (-1.0 -> 1.0)
 *
 * Uses the defualt turning feedback of the drive system.
 *
 * @param degrees     degrees by which we will turn relative to the robot (+) turns ccw, (-) turns cw
 * @param max_speed   the maximum percentage of robot speed at which the robot will travel. 1 = full power
 * @return true if we turned te target number of degrees
 */
bool TankDrive::turn_degrees(double degrees, double max_speed)
{
  if (turn_default_feedback != NULL)
    return turn_degrees(degrees, *turn_default_feedback, max_speed);

  printf("tank_drive.cpp: Cannot run turn_degrees without a feedback controller!\n");
  fflush(stdout);
  return true;
}

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
 * @return true if we have reached our target point
 */
bool TankDrive::drive_to_point(double x, double y, vex::directionType dir, Feedback &feedback, double max_speed)
{
  // We can't run the auto drive function without odometry
  if (odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }

  if (!func_initialized)
  {

    double initial_dist = OdometryBase::pos_diff(odometry->get_position(), {.x = x, .y = y});

    // Reset the control loops
    correction_pid.init(0, 0);
    feedback.init(-initial_dist, 0);

    correction_pid.set_limits(-1, 1);
    feedback.set_limits(-1, 1);

    func_initialized = true;
  }

  // Store the initial position of the robot
  pose_t current_pos = odometry->get_position();
  pose_t end_pos = {.x = x, .y = y};

  // Create a point (and vector) to get the direction
  point_t pos_diff_pt =
      {
          .x = x - current_pos.x,
          .y = y - current_pos.y};

  Vector2D point_vec(pos_diff_pt);

  // Get the distance between 2 points
  double dist_left = OdometryBase::pos_diff(current_pos, end_pos);

  int sign = 1;

  // Make an imaginary perpendicualar line to that between the bot and the point. If the point is behind that line,
  // and the point is within the robot's radius, use negatives for feedback control.

  double angle_to_point = atan2(y - current_pos.y, x - current_pos.x) * 180.0 / PI;
  double angle = fmod(current_pos.rot - angle_to_point, 360.0);

  // Normalize the angle between 0 and 360
  if (angle > 360)
    angle -= 360;
  if (angle < 0)
    angle += 360;

  // If the angle is behind the robot, report negative.
  if (dir == directionType::fwd && angle > 90 && angle < 270)
    sign = -1;
  else if (dir == directionType::rev && (angle < 90 || angle > 270))
    sign = -1;

  if (fabs(dist_left) < config.drive_correction_cutoff)
  {
    // When inside the robot's cutoff radius, report the distance to the point along the robot's forward axis,
    // so we always "reach" the point without having to do a lateral translation
    dist_left *= fabs(cos(angle * PI / 180.0));
  }

  // Get the heading difference between where we are and where we want to be
  // Optimize that heading so we don't turn clockwise all the time
  double heading = rad2deg(point_vec.get_dir());
  double delta_heading = 0;

  // Going backwards "flips" the robot's current heading
  if (dir == directionType::fwd)
    delta_heading = OdometryBase::smallest_angle(current_pos.rot, heading);
  else
    delta_heading = OdometryBase::smallest_angle(current_pos.rot - 180, heading);

  // Update the PID controllers with new information
  correction_pid.update(delta_heading);
  feedback.update(sign * -1 * dist_left);

  // Disable correction when we're close enough to the point
  double correction = 0;
  if (is_pure_pursuit || fabs(dist_left) > config.drive_correction_cutoff)
    correction = correction_pid.get();

  // Reverse the drive_pid output if we're going backwards
  double drive_pid_rval;
  if (dir == directionType::rev)
    drive_pid_rval = feedback.get() * -1;
  else
    drive_pid_rval = feedback.get();

  // Combine the two pid outputs
  double lside = drive_pid_rval + correction;
  double rside = drive_pid_rval - correction;

  // limit the outputs between -1 and +1
  lside = clamp(lside, -max_speed, max_speed);
  rside = clamp(rside, -max_speed, max_speed);

  drive_tank(lside, rside);

  // Check if the robot has reached it's destination
  if (feedback.is_on_target())
  {
    stop();
    func_initialized = false;
    stop();
    return true;
  }

  return false;
}

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
 * @return true if we have reached our target point
 */
bool TankDrive::drive_to_point(double x, double y, vex::directionType dir, double max_speed)
{
  if (drive_default_feedback != NULL)
    return this->drive_to_point(x, y, dir, *drive_default_feedback, max_speed);

  printf("tank_drive.cpp: Cannot run drive_to_point without a feedback controller!\n");
  fflush(stdout);
  return true;
}

/**
 * Turn the robot in place to an exact heading relative to the field.
 * 0 is forward.
 *
 * @param heading_deg the heading to which we will turn
 * @param feedback    the feedback controller we will use to travel. controls the rate at which we accelerate and drive.
 * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
 * @return true if we have reached our target heading
 */
bool TankDrive::turn_to_heading(double heading_deg, Feedback &feedback, double max_speed)
{
  // We can't run the auto drive function without odometry
  if (odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }

  if (!func_initialized)
  {
    double initial_delta = OdometryBase::smallest_angle(odometry->get_position().rot, heading_deg);
    feedback.init(-initial_delta, 0);
    feedback.set_limits(-fabs(max_speed), fabs(max_speed));

    func_initialized = true;
  }

  // Get the difference between the new heading and the current, and decide whether to turn left or right.
  double delta_heading = OdometryBase::smallest_angle(odometry->get_position().rot, heading_deg);
  feedback.update(-delta_heading);

  fflush(stdout);

  drive_tank(-feedback.get(), feedback.get());

  // When the robot has reached it's angle, return true.
  if (feedback.is_on_target())
  {
    func_initialized = false;
    stop();
    return true;
  }

  return false;
}
/**
 * Turn the robot in place to an exact heading relative to the field.
 * 0 is forward. Uses the defualt turn feedback of the drive system
 *
 * @param heading_deg the heading to which we will turn
 * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
 * @return true if we have reached our target heading
 */
bool TankDrive::turn_to_heading(double heading_deg, double max_speed)
{
  if (turn_default_feedback != NULL)
    return turn_to_heading(heading_deg, *turn_default_feedback, max_speed);

  printf("tank_drive.cpp: Cannot run turn_to_heading without a feedback controller!\n");
  fflush(stdout);
  return true;
}

/**
 * Modify the inputs from the controller by squaring / cubing, etc
 * Allows for better control of the robot at slower speeds
 * @param input the input signal -1 -> 1
 * @param power the power to raise the signal to
 * @return input^power accounting for any sign issues that would arise with this naive solution
 */
double TankDrive::modify_inputs(double input, int power)
{
  return sign(input) * pow(std::abs(input), power);
}

/**
 * Drive the robot autonomously using a pure-pursuit algorithm - Input path with a set of 
 * waypoints - the robot will attempt to follow the points while cutting corners (radius)
 * to save time (compared to stop / turn / start)
 * 
 * @param path The list of coordinates to follow, in order
 * @param dir Run the bot forwards or backwards
 * @param radius How big the corner cutting should be - small values follow the path more closely
 * @param feedback The feedback controller determining speed
 * @param max_speed Limit the speed of the robot (for pid / pidff feedbacks)
 * @return True when the path is complete
*/
bool TankDrive::pure_pursuit(std::vector<point_t> path, directionType dir, double radius, Feedback &feedback, double max_speed)
{
  pose_t robot_pose = odometry->get_position();

  // On function initialization, send the path-length estimate to the feedback controller
  if(!func_initialized)
  {
    if(dir != directionType::rev)
      feedback.init(-estimate_path_length(path), 0);
    else
      feedback.init(estimate_path_length(path), 0);
    
    func_initialized = true;
  }
  
  point_t lookahead = PurePursuit::get_lookahead(path, odometry->get_position(), radius);
  point_t localized = lookahead - robot_pose.get_point();

  point_t last_point = path[path.size()-1];
  bool is_last_point = (lookahead == last_point);
  
  double correction = 0;
  double dist_remaining = PurePursuit::estimate_remaining_dist(path, robot_pose, radius);
  double angle_diff = 0;
  
  // Robot is facing forwards / backwards, change the bot's angle by 180
  if(dir != directionType::rev)
    angle_diff = OdometryBase::smallest_angle(robot_pose.rot, rad2deg(atan2(localized.y, localized.x)));
  else
    angle_diff = OdometryBase::smallest_angle(robot_pose.rot + 180, rad2deg(atan2(localized.y, localized.x)));

  // Correct the robot's heading until the last cut-off 
  if(!(is_last_point && robot_pose.get_point().dist(last_point) < config.drive_correction_cutoff))
  {
    correction_pid.update(angle_diff);
    correction = correction_pid.get();
  } else // Inside cut-off radius, ignore horizontal diffs
  {
    dist_remaining *= cos(angle_diff * (PI / 180.0));
  }

  if(dir != directionType::rev)
    feedback.update(-dist_remaining);
  else
    feedback.update(dist_remaining);

  max_speed = fabs(max_speed);

  double left = clamp(feedback.get(), -max_speed, max_speed);
  double right = clamp(feedback.get(), -max_speed, max_speed);

  left += correction;
  right -= correction;
  
  drive_tank(left, right);

  // When the robot has reached the end point and feedback reports on target, end pure pursuit
  if(is_last_point && feedback.is_on_target())
  {
    func_initialized = false;
    stop();
    return true;
  }
  return false;
}

/**
 * Drive the robot autonomously using a pure-pursuit algorithm - Input path with a set of 
 * waypoints - the robot will attempt to follow the points while cutting corners (radius)
 * to save time (compared to stop / turn / start)
 * 
 * Use the default drive feedback
 * 
 * @param path The list of coordinates to follow, in order
 * @param dir Run the bot forwards or backwards
 * @param radius How big the corner cutting should be - small values follow the path more closely
 * @param max_speed Limit the speed of the robot (for pid / pidff feedbacks)
 * @return True when the path is complete
*/
bool TankDrive::pure_pursuit(std::vector<point_t> path, directionType dir, double radius, double max_speed)
{
  return pure_pursuit(path, dir, radius, *config.drive_feedback, max_speed);
}