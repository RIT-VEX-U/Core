/**
 * File: drive_forward_command.h
 * Desc:
 *    AutoCommand wrapper class for the drive_forward function in the 
 *    TankDrive class
 */

#pragma once

#include "vex.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/subsystems/tank_drive.h"

using namespace vex;

class DriveForwardCommand: public AutoCommand {
  public:
    DriveForwardCommand(TankDrive &drive_sys, double inches, double speed, double correction, directionType dir);

    /**
     * Run drive_forward
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    // drive system to run the function on
    TankDrive &drive_sys;

    // parameters for drive_forward
    double inches;
    double speed;
    double correction;
    directionType dir;
};