/**
 * File: flywheel_commands.cpp
 * Desc:
 *    [insert meaningful desc]
 */

#include "../core/include/utils/command_structure/flywheel_commands.h"

SpinRawCommand::SpinRawCommand(Flywheel &flywheel, double speed, directionType dir):
  flywheel(flywheel), speed(speed), dir(dir) {}

bool SpinRawCommand::run() {
  flywheel.spin_raw(speed, dir);
  return true;
}


SpinManualCommand::SpinManualCommand(Flywheel &flywheel, double speed, directionType dir):
  flywheel(flywheel), speed(speed), dir(dir) {}

bool SpinManualCommand::run() {
  flywheel.spin_manual(speed, dir);
  return true;
}


SpinRPMCommand::SpinRPMCommand(Flywheel &flywheel, int rpm):
  flywheel(flywheel), rpm(rpm) {}

bool SpinRPMCommand::run() {
  flywheel.spinRPM(rpm);
  return true;
}


FlywheelStopCommand::FlywheelStopCommand(Flywheel &flywheel):
  flywheel(flywheel) {}

bool FlywheelStopCommand::run() {
  flywheel.stop();
  return true;
}


FlywheelStopMotorsCommand::FlywheelStopMotorsCommand(Flywheel &flywheel):
  flywheel(flywheel) {}

bool FlywheelStopMotorsCommand::run() {
  flywheel.stopMotors();
  return true;
}


FlywheelStopNonTasksCommand::FlywheelStopNonTasksCommand(Flywheel &flywheel):
  flywheel(flywheel) {}

bool FlywheelStopNonTasksCommand::run() {
  flywheel.stopNonTasks();
  return true;
}
