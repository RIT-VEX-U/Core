/**
 * File: drive_forward_command.h
 * Desc:
 *    Command wrapper class for the drive_forward function in the TankDrive class
 */

#pragma once

#include "vex.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/subsystems/tank_drive.h"

using namespace vex;

class DriveForwardCommand: public AutoCommand {
  public:
    DriveForwardCommand(TankDrive &drive_sys, double inches, double speed, double correction, directionType dir);
    bool run() override;

  private:
    TankDrive &drive_sys;
    double inches;
    double speed;
    double correction;
    directionType dir;
};