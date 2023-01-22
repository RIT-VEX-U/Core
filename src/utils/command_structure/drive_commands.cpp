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


DriveForwardCommand::DriveForwardCommand(TankDrive &drive_sys, Feedback &feedback, double inches, directionType dir, double max_speed):
  drive_sys(drive_sys), feedback(feedback), inches(inches), dir(dir), max_speed(max_speed) {}

/**
 * Run drive_forward
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool DriveForwardCommand::run() {
  return drive_sys.drive_forward(inches, dir, feedback, max_speed);
}


// Turn Degrees

TurnDegreesCommand::TurnDegreesCommand(TankDrive &drive_sys, Feedback &feedback, double degrees, double max_speed):
  drive_sys(drive_sys), feedback(feedback), degrees(degrees), max_speed(max_speed) {}

/**
 * Run turn_degrees
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool TurnDegreesCommand::run() {
  return drive_sys.turn_degrees(degrees, max_speed);
}


// Drive to Point

DriveToPointCommand::DriveToPointCommand(TankDrive &drive_sys, Feedback &feedback, double x, double y, directionType dir, double max_speed):
  drive_sys(drive_sys), feedback(feedback), x(x), y(y), dir(dir), max_speed(max_speed) {}

/**
 * Run drive_to_point
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool DriveToPointCommand::run() {
  return drive_sys.drive_to_point(x, y, dir, max_speed);
}


// Turn to Heading

TurnToHeadingCommand::TurnToHeadingCommand(TankDrive &drive_sys, Feedback &feedback, double heading_deg, double max_speed):
  drive_sys(drive_sys), feedback(feedback), heading_deg(heading_deg), max_speed(max_speed) {}

/**
 * Run turn_to_heading
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool TurnToHeadingCommand::run() {
  return drive_sys.turn_to_heading(heading_deg, feedback, max_speed);
}


// Stop

DriveStopCommand::DriveStopCommand(TankDrive &drive_sys):
  drive_sys(drive_sys) {}

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

bool OdomSetPosition::run() {
  odom.set_position(newpos);
  return true;
}
