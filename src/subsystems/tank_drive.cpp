#include "../core/include/subsystems/tank_drive.h"

TankDrive::TankDrive(motor_group &left_motors, motor_group &right_motors, robot_specs_t &config, OdometryTank *odom)
    : left_motors(left_motors), right_motors(right_motors),
     drive_pid(config.drive_pid), turn_pid(config.turn_pid), correction_pid(config.correction_pid), odometry(odom), config(config)
{

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
void TankDrive::drive_tank(double left, double right, int power, bool isdriver)
{
  left = modify_inputs(left, power);
  right = modify_inputs(right, power);

  if(isdriver == false)
  {
    left_motors.spin(directionType::fwd, left * 12, voltageUnits::volt);
    right_motors.spin(directionType::fwd, right * 12, voltageUnits::volt);
  }else
  {
    left_motors.spin(directionType::fwd, left * 100.0, percentUnits::pct);
    right_motors.spin(directionType::fwd, right * 100.0, percentUnits::pct);
  }
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
 * Autonomously drive forward or backwards, X inches infront or behind the robot's current position.
 * This driving method is relative, so excessive use may cause the robot to get off course!
 *
 * @param inches Distance to drive in a straight line
 * @param speed How fast the robot should travel, 0 -> 1.0
 * @param correction How much the robot should correct for being off angle
 * @param dir Whether the robot is travelling forwards or backwards
 */
bool TankDrive::drive_forward(double inches, double speed, double correction, directionType dir)
{
  static position_t pos_setpt;

  // We can't run the auto drive function without odometry
  if(odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }

  // Generate a point X inches forward of the current position, on first startup
  if (!func_initialized)
  {
    saved_pos = odometry->get_position();
    drive_pid.reset();

    // Use vector math to get an X and Y
    Vector current_pos({saved_pos.x , saved_pos.y});
    Vector delta_pos(deg2rad(saved_pos.rot), inches);
    Vector setpt_vec = current_pos + delta_pos;

    // Save the new X and Y values
    pos_setpt = {.x=setpt_vec.get_x(), .y=setpt_vec.get_y()};

    func_initialized = true;
  }

  // Call the drive_to_point with updated point values
  return drive_to_point(pos_setpt.x, pos_setpt.y, speed, correction, dir);
}

/**
 * Autonomously turn the robot X degrees to the right (negative for left), with a maximum motor speed
 * of percent_speed (-1.0 -> 1.0)
 * 
 * Uses a PID loop for it's control.
 */
bool TankDrive::turn_degrees(double degrees, double percent_speed)
{
  // We can't run the auto drive function without odometry
  if(odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }

  // On the first run of the funciton, reset the gyro position and PID
  if (!func_initialized)
  {
    saved_pos = odometry->get_position();
    turn_pid.reset();

    turn_pid.set_limits(-fabs(percent_speed), fabs(percent_speed));
    turn_pid.set_target(0);

    func_initialized = true;
  }
  double heading = odometry->get_position().rot - saved_pos.rot;
  double delta_heading = OdometryBase::smallest_angle(heading, degrees);
  turn_pid.update(delta_heading);

  // printf("heading: %f, delta_heading: %f\n", heading, delta_heading);

  drive_tank(turn_pid.get(), -turn_pid.get());

  // If the robot is at it's target, return true
  if (turn_pid.is_on_target())
  {
    drive_tank(0, 0);
    func_initialized = false;
    return true;
  }

  return false;
}

/**
  * Use odometry to automatically drive the robot to a point on the field.
  * X and Y is the final point we want the robot.
  *
  * Returns whether or not the robot has reached it's destination.
  */
bool TankDrive::drive_to_point(double x, double y, double speed, double correction_speed, vex::directionType dir)
{
  // We can't run the auto drive function without odometry
  if(odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }
  
  if(!func_initialized)
  {
    // Reset the control loops
    drive_pid.reset();
    correction_pid.reset();

    drive_pid.set_limits(-fabs(speed), fabs(speed));
    correction_pid.set_limits(-fabs(correction_speed), fabs(correction_speed));

    // Set the targets to 0, because we update with the change in distance and angle between the current point
    // and the new point.
    drive_pid.set_target(0);
    correction_pid.set_target(0);

    // point_orientation_deg = atan2(y - odometry->get_position().y, x - odometry->get_position().x) * 180.0 / PI;

    func_initialized = true;
  }

  // Store the initial position of the robot
  position_t current_pos = odometry->get_position();
  position_t end_pos = {.x=x, .y=y};

  // Create a point (and vector) to get the direction
  Vector::point_t pos_diff_pt = 
  {
    .x = x - current_pos.x,
    .y = y - current_pos.y
  };

  Vector point_vec(pos_diff_pt);

  // Get the distance between 2 points
  double dist_left = OdometryBase::pos_diff(current_pos, end_pos);
  
  int sign = 1;

  // Make an imaginary perpendicualar line to that between the bot and the point. If the point is behind that line,
  // and the point is within the robot's radius, use negatives for feedback control.

  double angle_to_point = atan2(y - current_pos.y, x - current_pos.x) * 180.0 / PI;
  double angle = fmod(current_pos.rot - angle_to_point, 360.0);
  // Normalize the angle between 0 and 360
  if (angle > 360) angle -= 360;
  if (angle < 0) angle += 360; 
  // If the angle is behind the robot, report negative.
  if (dir == directionType::fwd && angle > 90 && angle < 270)
    sign = -1;
  else if(dir == directionType::rev && (angle < 90 || angle > 270))
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
  drive_pid.update(sign * -1 * dist_left);

  // Disable correction when we're close enough to the point
  double correction = 0;
  if(is_pure_pursuit || fabs(dist_left) > config.drive_correction_cutoff)
    correction = correction_pid.get();

  // Reverse the drive_pid output if we're going backwards
  double drive_pid_rval;
  if(dir == directionType::rev)
    drive_pid_rval = drive_pid.get() * -1;
  else
    drive_pid_rval = drive_pid.get();

  // Combine the two pid outputs
  double lside = drive_pid_rval + correction;
  double rside = drive_pid_rval - correction;

  // limit the outputs between -1 and +1
  lside = (lside > 1) ? 1 : (lside < -1) ? -1 : lside;
  rside = (rside > 1) ? 1 : (rside < -1) ? -1 : rside;

  drive_tank(lside, rside);

  printf("dist: %f\n", sign * -1 * dist_left);
  fflush(stdout);

  // Check if the robot has reached it's destination
  if(drive_pid.is_on_target())
  {
    stop();
    func_initialized = false;
    return true;
  }

  return false;
}

/**
 * Turn the robot in place to an exact heading relative to the field.
 * 0 is forward, and 0->360 is clockwise.
 */
bool TankDrive::turn_to_heading(double heading_deg, double speed)
{
  // We can't run the auto drive function without odometry
  if(odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }

  if(!func_initialized)
  {
    turn_pid.reset();
    turn_pid.set_limits(-fabs(speed), fabs(speed));

    // Set the target to zero, and the input will be a delta.
    turn_pid.set_target(0);

    func_initialized = true;
  }

  // Get the difference between the new heading and the current, and decide whether to turn left or right.
  double delta_heading = OdometryBase::smallest_angle(odometry->get_position().rot, heading_deg);
  turn_pid.update(delta_heading);

  printf("~TURN~ delta: %f\n", delta_heading);
  // printf("delta heading: %f, pid: %f\n", delta_heading, turn_pid.get());
  fflush(stdout);

  drive_tank(turn_pid.get(), -turn_pid.get());

  // When the robot has reached it's angle, return true.
  if(turn_pid.is_on_target())
  {
    func_initialized = false;
    stop();
    return true;
  }

  return false;
}

/**
 * Modify the inputs from the controller by squaring / cubing, etc
 * Allows for better control of the robot at slower speeds
 */
double TankDrive::modify_inputs(double input, int power)
{
  return (power % 2 == 0 ? (input < 0 ? -1 : 1) : 1) * pow(input, power);
}

bool TankDrive::pure_pursuit(std::vector<PurePursuit::hermite_point> path, double radius, double speed, double res, directionType dir) {
  is_pure_pursuit = true;
  std::vector<Vector::point_t> smoothed_path = PurePursuit::smooth_path_hermite(path, res);

  Vector::point_t lookahead = PurePursuit::get_lookahead(smoothed_path, {odometry->get_position().x, odometry->get_position().y}, radius);
  //printf("%f\t%f\n", odometry->get_position().x, odometry->get_position().y); 
  //printf("%f\t%f\n", lookahead.x, lookahead.y);
  bool is_last_point = (path.back().x == lookahead.x) && (path.back().y == lookahead.y);

  if(is_last_point)
    is_pure_pursuit = false;

  bool retval = drive_to_point(lookahead.x, lookahead.y, speed, 1, dir);

  if(is_last_point)
    return retval;

  return false;
}
