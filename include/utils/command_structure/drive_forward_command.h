/**
 * File: drive_forward_command.h
 * Desc:
 *    Command wrapper class for the drive_forward function in the TankDrive class
 */

#pragma once

#include "vex.h"
#include "../core/include/utils/command_structure/auto_command.h"

using namespace vex;

class DriveForwardCommand: public AutoCommand {
  public:
    DriveForwardCommand(double inches, double speed, double correction, directionType dir);
    bool run();

  private:
    double inches;
    double speed;
    double correction;
    directionType dir;
};