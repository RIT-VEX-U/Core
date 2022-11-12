/**
 * File: flywheel_commands.cpp
 * Desc:
 *    [insert meaningful desc]
 */

#include "../core/include/utils/command_structure/flywheel_commands.h"


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
