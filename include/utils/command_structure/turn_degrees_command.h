/**
 * File: turn_degrees_command.h
 * Desc:
 *    AutoCommand wrapper class for the turn_degrees function in the 
 *    TankDrive class
 */

#pragma once

#include "vex.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/subsystems/tank_drive.h"

using namespace vex;

class TurnDegreesCommand: public AutoCommand {
  public:
    TurnDegreesCommand(TankDrive &drive_sys, double degrees, double percent_speed);

    /**
     * Run turn_degrees
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    // drive system to run the function on
    TankDrive &drive_sys;

    // parameters for turn_degrees
    double degrees;
    double percent_speed;
};