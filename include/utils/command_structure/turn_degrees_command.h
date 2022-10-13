/**
 * File: turn_degrees_command.h
 * Desc:
 *    Command wrapper class for the turn_degrees function in the TankDrive class
 */

#pragma once

#include "vex.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/subsystems/tank_drive.h"

using namespace vex;

class TurnDegreesCommand: public AutoCommand {
  public:
    TurnDegreesCommand(TankDrive &drive_sys, double degrees, double percent_speed);
    bool run() override;

  private:
    TankDrive &drive_sys;
    double degrees;
    double percent_speed;
};