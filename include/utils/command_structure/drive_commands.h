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
 *
 */
class DriveForwardCommand: public AutoCommand {
  public:
    DriveForwardCommand(TankDrive &drive_sys, Feedback &feedback, double inches, directionType dir, double max_speed=1);

    /**
     * Run drive_forward
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;
    /**
     * Cleans up drive system if we time out before finishing
    */
    void on_timeout() override;

  private:
    // drive system to run the function on
    TankDrive &drive_sys;

    // feedback controller to use
    Feedback &feedback;

    // parameters for drive_forward
    double inches;
    directionType dir;
    double max_speed;
};

/**
 * AutoCommand wrapper class for the turn_degrees function in the 
 * TankDrive class
 */
class TurnDegreesCommand: public AutoCommand {
  public:
    TurnDegreesCommand(TankDrive &drive_sys, Feedback &feedback, double degrees, double max_speed = 1);

    /**
     * Run turn_degrees
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;
    /**
     * Cleans up drive system if we time out before finishing
    */
    void on_timeout() override;


  private:
    // drive system to run the function on
    TankDrive &drive_sys;
    
    // feedback controller to use
    Feedback &feedback;

    // parameters for turn_degrees
    double degrees;
    double max_speed;
};

/**
 * AutoCommand wrapper class for the drive_to_point function in the
 * TankDrive class
 */
class DriveToPointCommand: public AutoCommand {
  public:
    DriveToPointCommand(TankDrive &drive_sys, Feedback &feedback, double x, double y, directionType dir, double max_speed = 1);
    DriveToPointCommand(TankDrive &drive_sys, Feedback &feedback, Vector2D::point_t point, directionType dir, double max_speed=1);

    /**
     * Run drive_to_point
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    // drive system to run the function on
    TankDrive &drive_sys;
    
    /**
     * Cleans up drive system if we time out before finishing
    */
    void on_timeout() override;


    // feedback controller to use
    Feedback &feedback;

    // parameters for drive_to_point
    double x;
    double y;
    directionType dir;
    double max_speed;
    
};

/**
 * AutoCommand wrapper class for the turn_to_heading() function in the 
 * TankDrive class
 *
 */
class TurnToHeadingCommand: public AutoCommand {
  public:
    TurnToHeadingCommand(TankDrive &drive_sys, Feedback &feedback, double heading_deg, double speed = 1);

    /**
     * Run turn_to_heading
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;
    /**
     * Cleans up drive system if we time out before finishing
    */
    void on_timeout() override;


  private:
    // drive system to run the function on
    TankDrive &drive_sys;

    // feedback controller to use
    Feedback &feedback;

    // parameters for turn_to_heading
    double heading_deg;
    double max_speed;
};

/**
 * AutoCommand wrapper class for the stop() function in the 
 * TankDrive class
 */
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

/**
 * AutoCommand wrapper class for the set_position function in the 
 * Odometry class
 */
class OdomSetPosition: public AutoCommand {
  public:
    /**
     * constructs a new OdomSetPosition command
     * @param odom the odometry system we are setting
     * @param newpos the position we are telling the odometry to take. defaults to (0, 0), angle = 90
    */
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
