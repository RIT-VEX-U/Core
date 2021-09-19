#include "../core/include/subsystems/tank_drive.h"

TankDrive::TankDrive(motor_group &left_motors, motor_group &right_motors, inertial &gyro_sensor, TankDrive::tankdrive_config_t &config, OdometryTank *odom)
    : left_motors(left_motors), right_motors(right_motors),
     drive_pid(config.drive_pid), turn_pid(config.turn_pid), odometry(odom)
{

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
 * Autonomously drive the robot X inches forward (Negative for backwards), with a maximum speed
 * of percent_speed (-1.0 -> 1.0).
 * 
 * Uses a PID loop for it's control.
 * 
 * NOTE: uses relative positioning, so after a few drive_forward's, position may be lost!
 */
bool TankDrive::drive_forward(double inches, double percent_speed)
{
  // On the first run of the funciton, reset the motor position and PID
  if (initialize_func)
  {
    saved_pos = odometry->get_position();
    drive_pid.reset();

    drive_pid.set_limits(-fabs(percent_speed), fabs(percent_speed));
    drive_pid.set_target(inches);

    initialize_func = false;
  }

  double position_diff = odometry->get_position().y - saved_pos.y;

  // Update PID loop and drive the robot based on it's output
  drive_pid.update(position_diff);

  // Drive backwards if we input negative inches, forward for positive
  drive_tank(drive_pid.get(), drive_pid.get());

  // If the robot is at it's target, return true
  if (drive_pid.is_on_target())
  {
    drive_tank(0, 0);
    initialize_func = true;
    return true;
  }

  return false;
}

/**
 * Autonomously turn the robot X degrees to the right (negative for left), with a maximum motor speed
 * of percent_speed (-1.0 -> 1.0)
 * 
 * Uses a PID loop for it's control.
 */
bool TankDrive::turn_degrees(double degrees, double percent_speed)
{
  // On the first run of the funciton, reset the gyro position and PID
  if (initialize_func)
  {
    saved_pos = odometry->get_position();
    turn_pid.reset();

    turn_pid.set_limits(-fabs(percent_speed), fabs(percent_speed));
    turn_pid.set_target(degrees);

    initialize_func = false;
  }

  // Update PID loop and drive the robot based on it's output
  turn_pid.update(odometry->get_position().rot - saved_pos.rot);
  drive_tank(turn_pid.get(), -turn_pid.get());

  // If the robot is at it's target, return true
  if (turn_pid.is_on_target())
  {
    drive_tank(0, 0);
    initialize_func = true;
    return true;
  }

  return false;
}

bool TankDrive::drive_to_point(double x, double y, double rot_deg, double speed)
{
  static bool initialized = false;

  if(!initialized)
  {
    // Reset the control loops
    drive_pid.reset();
    turn_pid.reset();

    drive_pid.set_limits(-fabs(speed), fabs(speed));
    turn_pid.set_limits(-fabs(speed), fabs(speed));

    // Set the distance target to 0, because we update with the change in distance between the current point
    // and the new point.
    drive_pid.set_target(0);

    initialized = true;
  }

  position_t end_pos = 
  {
    .x = x,
    .y = y,
    .rot = rot_deg
  };

  // Store the initial position of the robot
  position_t current_pos = odometry->get_position();

  // Set the rotational target of the PID loop.
  // Every loop this gets updated, to make sure we are pointing in the right direction
  turn_pid.set_target(OdometryBase::rot_diff(end_pos, current_pos));

  // Update the PID controllers with new information
  turn_pid.update(current_pos.rot);
  drive_pid.update(OdometryBase::pos_diff(current_pos, end_pos));

  // Combine the two pid outputs
  double lside = drive_pid.get() + turn_pid.get();
  double rside = drive_pid.get() - turn_pid.get();

  // limit the outputs between -1 and +1
  lside = (lside > 1) ? 1 : (lside < -1) ? -1 : lside;
  rside = (rside > 1) ? 1 : (rside < -1) ? -1 : rside;

  drive_tank(lside, rside);


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