/**
 * File: drive_forward_command.cpp
 * Desc:
 *    Command wrapper class for the drive_forward function in the TankDrive class
 */

#include "vex.h"
#include "../core/include/utils/command_structure/drive_forward_command.h"
#include <iostream>

using namespace vex;

DriveForwardCommand::DriveForwardCommand(TankDrive &drive_sys, double inches, double speed, double correction, directionType dir):
  drive_sys(drive_sys), inches(inches), speed(speed), correction(correction), dir(dir) {}

bool DriveForwardCommand::run() {
  std::cout << "DRIVE FORWARD COMMAND RUNNING\n";
  return drive_sys.drive_forward(inches, speed, correction, dir);
}