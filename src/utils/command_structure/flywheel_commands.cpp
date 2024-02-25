/**
 * File: flywheel_commands.cpp
 * Desc:
 *    [insert meaningful desc]
 */

#include "../core/include/utils/command_structure/flywheel_commands.h"

SpinRPMCommand::SpinRPMCommand(Flywheel &flywheel, int rpm) : flywheel(flywheel), rpm(rpm) {}

bool SpinRPMCommand::run() {
  flywheel.spin_rpm(rpm);
  return true;
}

WaitUntilUpToSpeedCommand::WaitUntilUpToSpeedCommand(Flywheel &flywheel, int threshold_rpm)
    : flywheel(flywheel), threshold_rpm(threshold_rpm) {}

bool WaitUntilUpToSpeedCommand::run() {
  // If we're withing the specified threshold, we're ready to fire
  if (fabs(flywheel.get_target() - flywheel.getRPM()) < threshold_rpm) {
    return true;
  }
  // else, keep waiting
  return false;
}

FlywheelStopCommand::FlywheelStopCommand(Flywheel &flywheel) : flywheel(flywheel) {}

bool FlywheelStopCommand::run() {
  flywheel.stop();
  return true;
}

FlywheelStopMotorsCommand::FlywheelStopMotorsCommand(Flywheel &flywheel) : flywheel(flywheel) {}

bool FlywheelStopMotorsCommand::run() {
  flywheel.stop();
  return true;
}

FlywheelStopNonTasksCommand::FlywheelStopNonTasksCommand(Flywheel &flywheel) : flywheel(flywheel) {}

bool FlywheelStopNonTasksCommand::run() {
  flywheel.stop();
  return true;
}
