/**
 * File: drive_commands.h
 * Desc:
 *    Holds all the AutoCommand subclasses that wrap TankDrive functions
 *    
 *    Currently includes:
 *      - drive_forward
 *      - turn_degrees
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
