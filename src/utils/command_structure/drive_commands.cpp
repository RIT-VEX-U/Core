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
 */

#include "../core/include/utils/command_structure/drive_commands.h"


// ==== DRIVE FORWARD ====

DriveForwardCommand::DriveForwardCommand(TankDrive &drive_sys, double inches, double speed, double correction, directionType dir):
  drive_sys(drive_sys), inches(inches), speed(speed), correction(correction), dir(dir) {}

/**
 * Run drive_forward
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool DriveForwardCommand::run() {
  return drive_sys.drive_forward(inches, speed, correction, dir);
}


// ==== TURN DEGREES ====

TurnDegreesCommand::TurnDegreesCommand(TankDrive &drive_sys, double degrees, double percent_speed):
  drive_sys(drive_sys), degrees(degrees), percent_speed(percent_speed) {}

/**
 * Run turn_degrees
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool TurnDegreesCommand::run() {
  return drive_sys.turn_degrees(degrees, percent_speed);
}


// ==== DRIVE TO POINT ====

DriveToPointCommand::DriveToPointCommand(TankDrive &drive_sys, double x, double y, double speed, double correction_speed, directionType dir):
  drive_sys(drive_sys), x(x), y(y), speed(speed), correction_speed(correction_speed), dir(dir) {}

/**
 * Run drive_to_point
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool DriveToPointCommand::run() {
  return drive_sys.drive_to_point(x, y, speed, correction_speed);
}


// ==== TURN TO HEADING ====

TurnToHeadingCommand::TurnToHeadingCommand(TankDrive &drive_sys, double heading_deg, double speed):
  drive_sys(drive_sys), heading_deg(heading_deg), speed(speed) {}

/**
 * Run turn_to_heading
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool TurnToHeadingCommand::run() {
  return drive_sys.turn_to_heading(heading_deg, speed);
}


// ==== STOP ====

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
