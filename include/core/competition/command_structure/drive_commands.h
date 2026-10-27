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

#include "core/competition/command_structure/auto_command.h"
#include "core/subsystems/tank_drive.h"
#include "core/math/geometry/pose2d.h"
#include "vex/vex.h"

// ==== DRIVING ====

/**
 * AutoCommand wrapper class for the drive_forward function in the
 * TankDrive class
 *
 */
class DriveForwardCommand : public AutoCommand {
 public:
  DriveForwardCommand(
      TankDrive& drive_sys, Feedback& feedback, double inches, vex::directionType dir,
      double max_speed = 1, double end_speed = 0
  );

  /**
   * Run drive_forward
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;

  /// Returns a string describing the commands functionality
  std::string toString() override;

  /// Cleans up drive system if we time out before finishing
  void on_timeout() override;

 private:
  // drive system to run the function on
  TankDrive& drive_sys;

  // feedback controller to use
  Feedback& feedback;

  // parameters for drive_forward
  double inches;
  vex::directionType dir;
  double max_speed;
  double end_speed;
};

/**
 * AutoCommand wrapper class for the turn_degrees function in the
 * TankDrive class
 */
class TurnDegreesCommand : public AutoCommand {
 public:
  TurnDegreesCommand(
      TankDrive& drive_sys, Feedback& feedback, double degrees, double max_speed = 1,
      double end_speed = 0
  );

  /**
   * Run turn_degrees
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;

  /// Returns a string describing the commands functionality
  std::string toString() override;

  /// Cleans up drive system if we time out before finishing
  void on_timeout() override;

 private:
  // drive system to run the function on
  TankDrive& drive_sys;

  // feedback controller to use
  Feedback& feedback;

  // parameters for turn_degrees
  double degrees;
  double max_speed;
  double end_speed;
};

/**
 * AutoCommand wrapper class for the drive_to_point function in the
 * TankDrive class
 */
class DriveToPointCommand : public AutoCommand {
 public:
  DriveToPointCommand(
      TankDrive& drive_sys, Feedback& feedback, double x, double y, vex::directionType dir,
      double max_speed = 1, double end_speed = 0
  );
  DriveToPointCommand(
      TankDrive& drive_sys, Feedback& feedback, Translation2d translation, vex::directionType dir,
      double max_speed = 1, double end_speed = 0
  );

  /**
   * Run drive_to_point
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;

  /// Returns a string describing the commands functionality
  std::string toString() override;

 private:
  // drive system to run the function on
  TankDrive& drive_sys;

  /// Cleans up drive system if we time out before finishing
  void on_timeout() override;

  // feedback controller to use
  Feedback& feedback;

  // parameters for drive_to_point
  double x;
  double y;
  vex::directionType dir;
  double max_speed;
  double end_speed;
};

class TurnToPointCommand : public AutoCommand {
 public:
  TurnToPointCommand(
      TankDrive& drive_sys, double x, double y, vex::directionType dir, double max_speed = 1,
      double end_speed = 0
  );
  TurnToPointCommand(
      TankDrive& drive_sys, Translation2d translation, vex::directionType dir, double max_speed = 1,
      double end_speed = 0
  );

  bool run() override;

  /// Returns a string describing the commands functionality
  std::string toString() override;

 private:
  void on_timeout() override;
  TankDrive& drive_sys;
  double x, y;
  vex::directionType dir;
  double max_speed;
  double end_speed;
  bool func_initialized = false;
  double heading;
};

/**
 * AutoCommand wrapper class for the turn_to_heading() function in the
 * TankDrive class
 */
class TurnToHeadingCommand : public AutoCommand {
 public:
  TurnToHeadingCommand(
      TankDrive& drive_sys, Feedback& feedback, double heading_deg, double speed = 1,
      double end_speed = 0
  );

  /**
   * Run turn_to_heading
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;

  /// Returns a string describing the commands functionality
  std::string toString() override;

  /// Cleans up drive system if we time out before finishing
  void on_timeout() override;

 private:
  // drive system to run the function on
  TankDrive& drive_sys;

  // feedback controller to use
  Feedback& feedback;

  // parameters for turn_to_heading
  double heading_deg;
  double max_speed;
  double end_speed;
};

/**
 * AutoCommand wrapper class for the stop() function in the
 * TankDrive class
 */
class DriveStopCommand : public AutoCommand {
 public:
  DriveStopCommand(TankDrive& drive_sys);

  /**
   * Stop the drive system
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;

  /// Returns a string describing the commands functionality
  std::string toString() override;

  void on_timeout() override;

 private:
  // drive system to run the function on
  TankDrive& drive_sys;
};

// ==== ODOMETRY ====

/**
 * AutoCommand wrapper class for the set_position function in the
 * Odometry class
 */
class OdomSetPosition : public AutoCommand {
 public:
  /**
   * constructs a new OdomSetPosition command
   * @param odom the odometry system we are setting
   * @param newpos the position we are telling the odometry to take. defaults to (0, 0), angle =
   * 90
   */
  OdomSetPosition(OdometryBase& odom, const Pose2d& newpos = OdometryBase::zero_pos);

  /**
   * Run set_position
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;

  /// Returns a string describing the commands functionality
  std::string toString() override;

 private:
  // drive system with an odometry config
  OdometryBase& odom;
  Pose2d newpos;
};


/**
 * AutoCommand that prints the current odometry position.
 * The position is printed by toString(), which the AutoCommand framework logs when it runs.
 */
class PrintPositionCommand : public AutoCommand {
public:
  /**
   * constructs a new PrintPositionCommand
   * @param odom the odometry system whose position we are printing
   */
  PrintPositionCommand(OdometryBase &odom);

  /**
   * Does nothing; the position is printed through toString()
   * @returns true
   */
  bool run() override;
  /*
  * Returns a string describing the current position
  */
  std::string toString() override;
private:
  // odometry system to read the position from
  OdometryBase &odom;
};