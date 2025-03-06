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

#include "../core/include/subsystems/tank_drive.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/utils/geometry.h"
#include "vex.h"
#include "../core/include/utils/math/geometry/pose2d.h"
using namespace vex;

// ==== DRIVING ====

/**
 * AutoCommand wrapper class for the drive_forward function in the
 * TankDrive class
 *
 */
class DriveForwardCommand : public AutoCommand {
public:
  DriveForwardCommand(TankDrive &drive_sys, Feedback &feedback, double inches, directionType dir, double max_speed = 1,
                      double end_speed = 0);

  /**
   * Run drive_forward
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;
  std::string toString(){
    std::string returnStr = "Driving ";
    switch(dir){
      case directionType::fwd:
        returnStr = returnStr + " forwards ";
        break;
      case directionType::rev:
      returnStr = returnStr + " reverse ";
      default:
        break;
    }
    char inStr[21];
    sprintf(inStr, "%d", inches);
    char speedStr[21];
    sprintf(speedStr, "%d", (max_speed*100));
    returnStr = returnStr + inStr + " inches at " + speedStr + " percent speed";
    return returnStr;
  }
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
  double end_speed;
};

/**
 * AutoCommand wrapper class for the turn_degrees function in the
 * TankDrive class
 */
class TurnDegreesCommand : public AutoCommand {
public:
  TurnDegreesCommand(TankDrive &drive_sys, Feedback &feedback, double degrees, double max_speed = 1,
                     double end_speed = 0);

  /**
   * Run turn_degrees
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;
  std::string toString(){
    char degreeStr[32];
    sprintf(degreeStr, "5d", degrees);
    char max_speedStr[32];
    sprintf(max_speedStr, "5d", max_speed);
    std::string returnStr = "Turning ";
    returnStr.append(degreeStr);
    returnStr.append(" degrees at ");
    returnStr.append(max_speedStr);
    return returnStr;
  }
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
  double end_speed;
};

/**
 * AutoCommand wrapper class for the drive_to_point function in the
 * TankDrive class
 */
class DriveToPointCommand : public AutoCommand {
public:
  DriveToPointCommand(TankDrive &drive_sys, Feedback &feedback, double x, double y, directionType dir,
                      double max_speed = 1, double end_speed = 0);
  DriveToPointCommand(TankDrive &drive_sys, Feedback &feedback, Translation2d translation, directionType dir, double max_speed = 1,
                      double end_speed = 0);

  /**
   * Run drive_to_point
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;
  std::string toString(){
    std::string returnStr = "Driving ";
    switch(dir){
      case directionType::fwd:
        returnStr = returnStr + " forwards ";
        break;
      case directionType::rev:
      returnStr = returnStr + " reverse ";
      default:
        break;
    }
    char xStr[21];
    sprintf(xStr, "%d", x);
    char yStr[21];
    sprintf(yStr, "%d", y);
    
    returnStr = returnStr + " to (" + xStr + ", " + yStr + ") ";
    char speedStr[21];
    sprintf(speedStr, "%d", (max_speed*100));
    returnStr = returnStr + " at " + speedStr + " percent speed";
    return returnStr;
  }
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
  double end_speed;
};

class TurnToPointCommand: public AutoCommand{
  public:

    TurnToPointCommand(TankDrive &drive_sys, double x, double y, vex::directionType dir, double max_speed = 1, double end_speed = 0);
    TurnToPointCommand(TankDrive &drive_sys, Translation2d translation, vex::directionType dir, double max_speed = 1, double end_speed = 0);

    bool run() override;

    std::string toString(){
      std::string returnStr = "Turning ";
      switch(dir){
        case directionType::fwd:
          returnStr = returnStr + " forwards ";
          break;
        case directionType::rev:
        returnStr = returnStr + " reverse ";
        default:
          break;
      }
      char xStr[21];
      sprintf(xStr, "%d", x);
      char yStr[21];
      sprintf(yStr, "%d", y);
      
      returnStr = returnStr + " to (" + xStr + ", " + yStr + ") ";
      char speedStr[21];
      sprintf(speedStr, "%d", (max_speed*100));
      returnStr = returnStr + " at " + speedStr + " percent speed";
      return returnStr;
    }

  private:

    void on_timeout() override;
    TankDrive &drive_sys;
    double x, y;
    vex::directionType dir;
    double max_speed;
    double end_speed;
    bool func_initialized;
    double heading;
};

/**
 * AutoCommand wrapper class for the turn_to_heading() function in the
 * TankDrive class
 *
 */
class TurnToHeadingCommand : public AutoCommand {
public:
  TurnToHeadingCommand(TankDrive &drive_sys, Feedback &feedback, double heading_deg, double speed = 1,
                       double end_speed = 0);

  /**
   * Run turn_to_heading
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;
  /**
   * Cleans up drive system if we time out before finishing
   */
  std::string toString(){
    std::string returnStr = "Turning to ";
    char headStr[21];
    sprintf(headStr, "%d", heading_deg);
    char speedStr[21];
    sprintf(speedStr, "%d", (max_speed*100));
    returnStr = returnStr + headStr + " degrees at " + speedStr + " percent speed";
    return returnStr;
  }
  void on_timeout() override;

private:
  // drive system to run the function on
  TankDrive &drive_sys;

  // feedback controller to use
  Feedback &feedback;

  // parameters for turn_to_heading
  double heading_deg;
  double max_speed;
  double end_speed;
};

/**
 * Autocommand wrapper class for pure pursuit function in the TankDrive class
 */
class PurePursuitCommand : public AutoCommand {
public:
  /**
   * Construct a Pure Pursuit AutoCommand
   *
   * @param path The list of coordinates to follow, in order
   * @param dir Run the bot forwards or backwards
   * @param feedback The feedback controller determining speed
   * @param max_speed Limit the speed of the robot (for pid / pidff feedbacks)
   */
  PurePursuitCommand(TankDrive &drive_sys, Feedback &feedback, PurePursuit::Path path, directionType dir,
                     double max_speed = 1, double end_speed = 0);

  /**
   * Direct call to TankDrive::pure_pursuit
   */
  bool run() override;
  std::string toString(){
    std::string returnStr = "Driving through ";
    std::vector<Translation2d> thePoints = path.get_points();
    char xStr[21];
    char yStr[21];
    for (int i = 0; i < thePoints.size(); i++){
      sprintf(xStr, "%d", thePoints.at(i).x());
      sprintf(yStr, "%d", thePoints.at(i).y());
      returnStr = returnStr + "(" + xStr + ", " + yStr + ") \n";
    }
    char speedStr[21];
    sprintf(speedStr, "%d", (max_speed*100));
    returnStr = returnStr + " at " + speedStr + " percent speed";

  }
  /**
   * Reset the drive system when it times out
   */
  void on_timeout() override;

private:
  TankDrive &drive_sys;
  PurePursuit::Path path;
  directionType dir;
  Feedback &feedback;
  double max_speed;
  double end_speed;
};

/**
 * AutoCommand wrapper class for the stop() function in the
 * TankDrive class
 */
class DriveStopCommand : public AutoCommand {
public:
  DriveStopCommand(TankDrive &drive_sys);

  /**
   * Stop the drive system
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;
  void on_timeout() override;
  std::string toString(){
    return "Stopping the drive";
  }

private:
  // drive system to run the function on
  TankDrive &drive_sys;
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
   * @param newpos the position we are telling the odometry to take. defaults to (0, 0), angle = 90
   */
  OdomSetPosition(OdometryBase &odom, const Pose2d &newpos = zero_pos);

  /**
   * Run set_position
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override;
  std::string toString(){
    char headStr[21];
    sprintf(headStr, "%d", newpos.rotation().degrees());
    char speedStr[21];
    char xStr[21];
    sprintf(xStr, "%d", newpos.x());
    char yStr[21];
    sprintf(yStr, "%d", newpos.y());
    std::string returnStr = "Setting positon to X: ";
    returnStr = returnStr + xStr + ", Y: " + yStr + ", ROT: " + headStr;
    return returnStr;
  }

private:
  // drive system with an odometry config
  OdometryBase &odom;
  Pose2d newpos;
};
