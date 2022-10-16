/**
 * File: turn_degrees_command.cpp
 * Desc:
 *    AutoCommand wrapper class for the turn_degrees function in the 
 *    TankDrive class
 */

#include "vex.h"
#include "../core/include/utils/command_structure/turn_degrees_command.h"

using namespace vex;

TurnDegreesCommand::TurnDegreesCommand(TankDrive &drive_sys, double degrees, double percent_speed):
  drive_sys(drive_sys), degrees(degrees), percent_speed(percent_speed) {}

/**
 * Run turn_degrees
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool TurnDegreesCommand::run() {
  return drive_sys.turn_degrees(degrees, percent_speed);
}