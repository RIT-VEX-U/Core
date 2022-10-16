/**
 * File: drive_commands.h
 * Desc:
 *    Holds all the AutoCommand subclasses that wrap TankDrive functions
 *    
 *    Currently includes:
 *      - drive_forward
 *      - turn_degrees
 */

#pragma once

#include "vex.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/subsystems/tank_drive.h"

using namespace vex;

/**
 * AutoCommand wrapper class for the drive_forward function in the 
 * TankDrive class
 */
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

/**
 * AutoCommand wrapper class for the turn_degrees function in the 
 * TankDrive class
 */
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