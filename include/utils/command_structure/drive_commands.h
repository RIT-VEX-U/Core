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
 *
 *    Also holds AutoCommand subclasses that wrap OdometryBase functions
 *
 *    Currently includes:
 *      - set_position
 */

#pragma once

#include "vex.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/subsystems/tank_drive.h"

using namespace vex;


// ==== DRIVING ====

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

/**
 * AutoCommand wrapper class for the drive_to_point function in the
 * TankDrive class
 */
class DriveToPointCommand: public AutoCommand {
  public:
    DriveToPointCommand(TankDrive &drive_sys, double x, double y, double speed, double correction_speed, directionType dir=directionType::fwd);

    /**
     * Run drive_to_point
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    // drive system to run the function on
    TankDrive &drive_sys;

    // parameters for drive_to_point
    double x;
    double y;
    double speed;
    double correction_speed;
    directionType dir;
};


class TurnToHeadingCommand: public AutoCommand {
  public:
    TurnToHeadingCommand(TankDrive &drive_sys, double heading_deg, double speed);

    /**
     * Run turn_to_heading
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    // drive system to run the function on
    TankDrive &drive_sys;

    // parameters for turn_to_heading
    double heading_deg;
    double speed;
};

class DriveStopCommand: public AutoCommand {
  public:
    DriveStopCommand(TankDrive &drive_sys);

    /**
     * Stop the drive system
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    // drive system to run the function on
    TankDrive &drive_sys;
};


// ==== ODOMETRY ====

class OdomSetPosition: public AutoCommand {
  public:
    OdomSetPosition(OdometryBase &odom, const position_t &newpos=OdometryBase::zero_pos);

    /**
     * Run set_position
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    // drive system with an odometry config
    OdometryBase &odom;
    position_t newpos;
};
