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

#include "../core/include/utils/command_structure/drive_commands.h"

// ==== DRIVING ====

/**
 * Construct a DriveForward Command
 * @param drive_sys the drive system we are commanding
 * @param feedback the feedback controller we are using to execute the drive
 * @param inches how far forward to drive
 * @param dir the direction to drive
 * @param max_speed 0 -> 1 percentage of the drive systems speed to drive at
 */
DriveForwardCommand::DriveForwardCommand(TankDrive &drive_sys, Feedback &feedback, double inches, directionType dir,
                                         double max_speed, double end_speed)
    : drive_sys(drive_sys), feedback(feedback), inches(inches), dir(dir), max_speed(max_speed), end_speed(end_speed) {}

/**
 * Run drive_forward
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool DriveForwardCommand::run() { return drive_sys.drive_forward(inches, dir, feedback, max_speed, end_speed); }

/*
* returns a to string command describing the commands functionality
*/
std::string DriveForwardCommand::toString(){
  std::string returnStr = "Driving ";
  switch(dir){
    case directionType::fwd:
      returnStr.append(" forwards ");
      break;
    case directionType::rev:
    returnStr.append(" reverse ");
    default:
      break;
  }
  char inStr[21];
  sprintf(inStr, "%f", inches);
  returnStr.append(inStr);
  returnStr.append(" inches at ");
  char speedStr[21];
  sprintf(speedStr, "%f", (max_speed*100));
  returnStr.append(speedStr);
  returnStr.append("%% speed");
  return returnStr;
}

/**
 * reset the drive system if we timeout
 */
void DriveForwardCommand::on_timeout() {
  drive_sys.stop();
  drive_sys.reset_auto();
}

/**
 * Construct a TurnDegreesCommand Command
 * @param drive_sys the drive system we are commanding
 * @param feedback the feedback controller we are using to execute the turn
 * @param degrees how many degrees to rotate
 * @param max_speed 0 -> 1 percentage of the drive systems speed to drive at
 */
TurnDegreesCommand::TurnDegreesCommand(TankDrive &drive_sys, Feedback &feedback, double degrees, double max_speed,
                                       double end_speed)
    : drive_sys(drive_sys), feedback(feedback), degrees(degrees), max_speed(max_speed), end_speed(end_speed) {}
/**
 * Run turn_degrees
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool TurnDegreesCommand::run() { return drive_sys.turn_degrees(degrees, max_speed, end_speed); }

/*
* returns a to string command describing the commands functionality
*/
std::string TurnDegreesCommand::toString(){
  char degreeStr[32];
  sprintf(degreeStr, "%f", degrees);
  char max_speedStr[32];
  sprintf(max_speedStr, "%f", max_speed*100);
  std::string returnStr = "Turning ";
  returnStr.append(degreeStr);
  returnStr.append(" degrees at ");
  returnStr.append(max_speedStr);
  returnStr.append("%% speed");
  return returnStr;
}

/**
 * reset the drive system if we timeout
 */
void TurnDegreesCommand::on_timeout() {
  drive_sys.stop();
  drive_sys.reset_auto();
}

/**
 * Construct a DriveForward Command
 * @param drive_sys the drive system we are commanding
 * @param feedback the feedback controller we are using to execute the drive
 * @param x where to drive in the x dimension
 * @param y where to drive in the y dimension
 * @param dir the direction to drive
 * @param max_speed 0 -> 1 percentage of the drive systems speed to drive at
 */
DriveToPointCommand::DriveToPointCommand(TankDrive &drive_sys, Feedback &feedback, double x, double y,
                                         directionType dir, double max_speed, double end_speed)
    : drive_sys(drive_sys), feedback(feedback), x(x), y(y), dir(dir), max_speed(max_speed), end_speed(end_speed) {}

/**
 * Construct a DriveForward Command
 * @param drive_sys the drive system we are commanding
 * @param feedback the feedback controller we are using to execute the drive
 * @param translation the point to drive to
 * @param dir the direction to drive
 * @param max_speed 0 -> 1 percentage of the drive systems speed to drive at
 */
DriveToPointCommand::DriveToPointCommand(TankDrive &drive_sys, Feedback &feedback, Translation2d translation, directionType dir,
                                         double max_speed, double end_speed)
    : drive_sys(drive_sys), feedback(feedback), x(translation.x()), y(translation.y()), dir(dir), max_speed(max_speed),
      end_speed(end_speed) {x = translation.x(); y  = translation.y();}

/**
 * Run drive_to_point
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */

bool DriveToPointCommand::run() { return drive_sys.drive_to_point(x, y, dir, feedback, max_speed, end_speed); }

/*
* returns a to string command describing the commands functionality
*/
std::string DriveToPointCommand::toString(){
  std::string returnStr = "Driving ";
  switch(dir){
    case directionType::fwd:
      returnStr.append(" forwards ");
      break;
    case directionType::rev:
    returnStr.append(" reverse ");
    default:
      break;
  }
  returnStr.append(" to (");
  char xStr[21];
  sprintf(xStr, "%f", x);
  returnStr.append(xStr);

  returnStr.append(", ");
  char yStr[21];
  sprintf(yStr, "%f", y);
  returnStr.append(yStr);
  returnStr.append(") at ");
  
  returnStr = returnStr + " to (" + xStr + ", " + yStr + ") ";
  char speedStr[21];
  sprintf(speedStr, "%f", (max_speed*100));
  returnStr.append(speedStr);
  returnStr.append("%% speed");
  return returnStr;
}

/**
 * reset the drive system if we don't hit our target
 */
void DriveToPointCommand::on_timeout() {
  drive_sys.stop();
  drive_sys.reset_auto();
}

TurnToPointCommand::TurnToPointCommand(TankDrive &drive_sys, double x, double y, vex::directionType dir, double max_speed, double end_speed)
  : drive_sys(drive_sys), x(x), y(y), dir(dir), max_speed(max_speed), end_speed(end_speed){}

TurnToPointCommand::TurnToPointCommand(TankDrive &drive_sys, Translation2d translation, vex::directionType dir, double max_speed, double end_speed)
  : drive_sys(drive_sys), x(translation.x()), y(translation.y()), dir(dir), max_speed(max_speed), end_speed(end_speed){x = translation.x(); y = translation.y();}

bool TurnToPointCommand::run() {
  if (!func_initialized) {
    Pose2d pose = drive_sys.odometry->get_position();
    double dy = y - pose.y();
    double dx = x - pose.x();
    heading = rad2deg(atan2(dy, dx));
    if (dir != vex::directionType::fwd) {
      heading += 180.0;
    }
    func_initialized = true;
  }
  return drive_sys.turn_to_heading(heading, max_speed, end_speed);
}

/*
* returns a to string command describing the commands functionality
*/
std::string TurnToPointCommand::toString(){
  std::string returnStr = "Turning ";
  switch(dir){
    case directionType::fwd:
      returnStr.append(" forwards ");
      break;
    case directionType::rev:
    returnStr.append(" reverse ");
    default:
      break;
  }
  returnStr.append(" to (");
  char xStr[21];
  sprintf(xStr, "%f", x);
  returnStr.append(xStr);

  returnStr.append(", ");
  char yStr[21];
  sprintf(yStr, "%f", y);
  returnStr.append(yStr);
  returnStr.append(") at ");
  
  returnStr = returnStr + " to (" + xStr + ", " + yStr + ") ";
  char speedStr[21];
  sprintf(speedStr, "%f", (max_speed*100));
  returnStr.append(speedStr);
  returnStr.append("%% speed");
  return returnStr;
}

void TurnToPointCommand::on_timeout() { drive_sys.stop(); }

/**
 * Construct a TurnToHeadingCommand Command
 * @param drive_sys the drive system we are commanding
 * @param feedback the feedback controller we are using to execute the drive
 * @param heading_deg the heading to turn to in degrees
 * @param max_speed 0 -> 1 percentage of the drive systems speed to drive at
 */
TurnToHeadingCommand::TurnToHeadingCommand(TankDrive &drive_sys, Feedback &feedback, double heading_deg,
                                           double max_speed, double end_speed)
    : drive_sys(drive_sys), feedback(feedback), heading_deg(heading_deg), max_speed(max_speed), end_speed(end_speed) {}

/**
 * Run turn_to_heading
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool TurnToHeadingCommand::run() { return drive_sys.turn_to_heading(heading_deg, feedback, max_speed, end_speed); }

/*
* returns a to string command describing the commands functionality
*/
std::string TurnToHeadingCommand::toString(){
  std::string returnStr = "Turning to ";
  char headStr[21];
  sprintf(headStr, "%f", heading_deg);
  returnStr.append(headStr);
  returnStr.append(" degrees at ");

  char speedStr[21];
  sprintf(speedStr, "%f", (max_speed*100));
  returnStr.append(speedStr);
  returnStr.append("");
  returnStr = returnStr + headStr + " degrees at " + speedStr + "%% speed";
  return returnStr;
}

/**
 * reset the drive system if we don't hit our target
 */
void TurnToHeadingCommand::on_timeout() {
  drive_sys.stop();
  drive_sys.reset_auto();
}

/**
 * Construct a Pure Pursuit AutoCommand
 *
 * @param path The list of coordinates to follow, in order
 * @param dir Run the bot forwards or backwards
 * @param feedback The feedback controller determining speed
 * @param max_speed Limit the speed of the robot (for pid / pidff feedbacks)
 */
PurePursuitCommand::PurePursuitCommand(TankDrive &drive_sys, Feedback &feedback, PurePursuit::Path path,
                                       directionType dir, double max_speed, double end_speed)
    : drive_sys(drive_sys), path(path), dir(dir), feedback(feedback), max_speed(max_speed), end_speed(end_speed) {}

/**
 * Direct call to TankDrive::pure_pursuit
 */
bool PurePursuitCommand::run() { return drive_sys.pure_pursuit(path, dir, feedback, max_speed, end_speed); }

/*
* returns a to string command describing the commands functionality
*/
std::string PurePursuitCommand::toString(){
  std::string returnStr = "Driving through ";
  std::vector<Translation2d> thePoints = path.get_points();
  char xStr[21];
  char yStr[21];
  for (int i = 0; i < thePoints.size(); i++){
    sprintf(xStr, "%f", thePoints.at(i).x());
    returnStr.append("(");
    returnStr.append(xStr);
    returnStr.append(", ");
    sprintf(yStr, "%f", thePoints.at(i).y());
    returnStr.append(yStr);
    returnStr.append(") \n");

  }
  returnStr.append(" at ");

  char speedStr[21];
  sprintf(speedStr, "%f", (max_speed*100));
  returnStr.append(speedStr);
  returnStr.append("%% speed");
  return returnStr;  
}

/**
 * Reset the drive system when it times out
 */
void PurePursuitCommand::on_timeout() {
  drive_sys.stop();
  drive_sys.reset_auto();
}

/**
 * Construct a DriveStop Command
 * @param drive_sys the drive system we are commanding
 */
DriveStopCommand::DriveStopCommand(TankDrive &drive_sys) : drive_sys(drive_sys) {}

/*
* returns a to string command describing the commands functionality
*/
std::string DriveStopCommand::toString(){
  return "Stopping the drive";
}

void DriveStopCommand::on_timeout() { drive_sys.reset_auto(); }



/**
 * Stop the drive train
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool DriveStopCommand::run() {
  drive_sys.stop();
  return true;
}

// ==== ODOMETRY ====
/**
 * Construct an Odometry set pos
 * @param odom the odometry system we are setting
 * @param newpos the now position to set the odometry to
 */
OdomSetPosition::OdomSetPosition(OdometryBase &odom, const Pose2d &newpos) : odom(odom), newpos(newpos) {}

bool OdomSetPosition::run() {
  odom.set_position(newpos);
  return true;
}
