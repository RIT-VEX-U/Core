/**
 * File: drive_commands.h
 * Desc:
 *    Holds all the AutoCommand subclasses that wrap (currently) TankDrive functions
 *
 *    Currently includes:
 *      - drive_forward
 *      - turn_degrees
 *      - drive_to_point
 *      - turn_to_heading
 *      - stop
 *
 *    Also holds AutoCommand subclasses that wrap OdometryBase functions
 *
 *    Currently includes:
 *      - set_position
 */

#include "../core/include/utils/command_structure/drive_commands.h"

// ==== DRIVING ====

/**
 * Construct a DriveForward Command
 * @param drive_sys the drive system we are commanding
 * @param feedback the feedback controller we are using to execute the drive
 * @param inches how far forward to drive
 * @param dir the direction to drive
 * @param max_speed 0 -> 1 percentage of the drive systems speed to drive at
 */
DriveForwardCommand::DriveForwardCommand(TankDrive &drive_sys, Feedback &feedback, double inches, directionType dir,
                                         double max_speed, double end_speed)
    : drive_sys(drive_sys), feedback(feedback), inches(inches), dir(dir), max_speed(max_speed), end_speed(end_speed) {}

/**
 * Run drive_forward
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool DriveForwardCommand::run() { return drive_sys.drive_forward(inches, dir, feedback, max_speed, end_speed); }

/**
 * reset the drive system if we timeout
 */
void DriveForwardCommand::on_timeout() {
  drive_sys.stop();
  drive_sys.reset_auto();
}


/**
 * Construct a TurnDegreesCommand Command
 * @param drive_sys the drive system we are commanding
 * @param feedback the feedback controller we are using to execute the turn
 * @param degrees how many degrees to rotate
 * @param max_speed 0 -> 1 percentage of the drive systems speed to drive at
 */
TurnDegreesCommand::TurnDegreesCommand(TankDrive &drive_sys, Feedback &feedback, double degrees, double max_speed,
                                       double end_speed)
    : drive_sys(drive_sys), feedback(feedback), degrees(degrees), max_speed(max_speed), end_speed(end_speed) {}

/**
 * Run turn_degrees
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool TurnDegreesCommand::run() { return drive_sys.turn_degrees(degrees, max_speed, end_speed); }
/**
 * reset the drive system if we timeout
 */
void TurnDegreesCommand::on_timeout() {
  drive_sys.stop();
  drive_sys.reset_auto();
}

/**
 * Construct a DriveForward Command
 * @param drive_sys the drive system we are commanding
 * @param feedback the feedback controller we are using to execute the drive
 * @param x where to drive in the x dimension
 * @param y where to drive in the y dimension
 * @param dir the direction to drive
 * @param max_speed 0 -> 1 percentage of the drive systems speed to drive at
 */
DriveToPointCommand::DriveToPointCommand(TankDrive &drive_sys, Feedback &feedback, double x, double y,
                                         directionType dir, double max_speed, double end_speed)
    : drive_sys(drive_sys), feedback(feedback), x(x), y(y), dir(dir), max_speed(max_speed), end_speed(end_speed) {}

/**
 * Construct a DriveForward Command
 * @param drive_sys the drive system we are commanding
 * @param feedback the feedback controller we are using to execute the drive
 * @param point the point to drive to
 * @param dir the direction to drive
 * @param max_speed 0 -> 1 percentage of the drive systems speed to drive at
 */
DriveToPointCommand::DriveToPointCommand(TankDrive &drive_sys, Feedback &feedback, point_t point, directionType dir,
                                         double max_speed, double end_speed)
    : drive_sys(drive_sys), feedback(feedback), x(point.x), y(point.y), dir(dir), max_speed(max_speed),
      end_speed(end_speed) {}

/**
 * Run drive_to_point
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool DriveToPointCommand::run() { return drive_sys.drive_to_point(x, y, dir, feedback, max_speed, end_speed); }
/**
 * reset the drive system if we don't hit our target
 */
void DriveToPointCommand::on_timeout() {
  drive_sys.stop();
  drive_sys.reset_auto();
}

/**
 * Construct a TurnToHeadingCommand Command
 * @param drive_sys the drive system we are commanding
 * @param feedback the feedback controller we are using to execute the drive
 * @param heading_deg the heading to turn to in degrees
 * @param max_speed 0 -> 1 percentage of the drive systems speed to drive at
 */
TurnToHeadingCommand::TurnToHeadingCommand(TankDrive &drive_sys, Feedback &feedback, double heading_deg,
                                           double max_speed, double end_speed)
    : drive_sys(drive_sys), feedback(feedback), heading_deg(heading_deg), max_speed(max_speed), end_speed(end_speed) {}

/**
 * Run turn_to_heading
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool TurnToHeadingCommand::run() { return drive_sys.turn_to_heading(heading_deg, feedback, max_speed, end_speed); }
/**
 * reset the drive system if we don't hit our target
 */
void TurnToHeadingCommand::on_timeout() {
  drive_sys.stop();
  drive_sys.reset_auto();
}

/**
 * Construct a Pure Pursuit AutoCommand
 *
 * @param path The list of coordinates to follow, in order
 * @param dir Run the bot forwards or backwards
 * @param feedback The feedback controller determining speed
 * @param max_speed Limit the speed of the robot (for pid / pidff feedbacks)
 */
PurePursuitCommand::PurePursuitCommand(TankDrive &drive_sys, Feedback &feedback, PurePursuit::Path path,
                                       directionType dir, double max_speed, double end_speed)
    : drive_sys(drive_sys), path(path), dir(dir), feedback(feedback), max_speed(max_speed), end_speed(end_speed) {}

/**
 * Direct call to TankDrive::pure_pursuit
 */
bool PurePursuitCommand::run() { return drive_sys.pure_pursuit(path, dir, feedback, max_speed, end_speed); }

/**
 * Reset the drive system when it times out
 */
void PurePursuitCommand::on_timeout() {
  drive_sys.stop();
  drive_sys.reset_auto();
}

/**
 * Construct a DriveStop Command
 * @param drive_sys the drive system we are commanding
 */
DriveStopCommand::DriveStopCommand(TankDrive &drive_sys) : drive_sys(drive_sys) {}

void DriveStopCommand::on_timeout() { drive_sys.reset_auto(); }

/**
 * Stop the drive train
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool DriveStopCommand::run() {
  drive_sys.stop();
  return true;
}

// ==== ODOMETRY ====
/**
 * Construct an Odometry set pos
 * @param odom the odometry system we are setting
 * @param newpos the now position to set the odometry to
 */
OdomSetPosition::OdomSetPosition(OdometryBase &odom, const pose_t &newpos) : odom(odom), newpos(newpos) {}

bool OdomSetPosition::run() {
  odom.set_position(newpos);
  return true;
}
