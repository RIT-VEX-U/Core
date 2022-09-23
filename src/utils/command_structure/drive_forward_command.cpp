/**
 * File: drive_forward_command.cpp
 * Desc:
 *    Command wrapper class for the drive_forward function in the TankDrive class
 */

#include "vex.h"
#include "../core/include/utils/command_structure/drive_forward_command.h"

using namespace vex;

DriveForwardCommand::DriveForwardCommand(double inches, double speed, double correction, directionType dir):
  inches(inches), speed(speed), correction(correction), dir(dir) {}

bool DriveForwardCommand::run() {
  // while(!){statements
  // }
  return true;
}