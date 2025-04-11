/**
 * File: flywheel_commands.cpp
 * Desc:
 *    [insert meaningful desc]
 */

#include "core/utils/command_structure/flywheel_commands.h"

SpinRPMCommand::SpinRPMCommand(Flywheel &flywheel, int rpm) : flywheel(flywheel), rpm(rpm) {}

bool SpinRPMCommand::run() {
    flywheel.spin_rpm(rpm);
    return true;
}

std::string SpinRPMCommand::toString() {
    std::string returnStr = "Spiinning at %d Rpm", rpm;
    return returnStr;
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

std::string WaitUntilUpToSpeedCommand::toString() {
    std::string returnStr = "Waiting until at %d Rpm", rpm;
    return returnStr;
}

FlywheelStopCommand::FlywheelStopCommand(Flywheel &flywheel) : flywheel(flywheel) {}

bool FlywheelStopCommand::run() {
    flywheel.stop();
    return true;
}

/*
 * Returns a string describing the commands functionality
 */
std::string FlywheelStopCommand::toString() { return "Stopping Flywheel"; }

FlywheelStopMotorsCommand::FlywheelStopMotorsCommand(Flywheel &flywheel) : flywheel(flywheel) {}

bool FlywheelStopMotorsCommand::run() {
    flywheel.stop();
    return true;
}

/*
 * Returns a string describing the commands functionality
 */
std::string FlywheelStopMotorsCommand::toString() { return "Stopping Flywheel Motors"; }

FlywheelStopNonTasksCommand::FlywheelStopNonTasksCommand(Flywheel &flywheel) : flywheel(flywheel) {}

bool FlywheelStopNonTasksCommand::run() {
    flywheel.stop();
    return true;
}

/*
 * Returns a string describing the commands functionality
 */
std::string FlywheelStopNonTasksCommand::toString() { return "Stopping Flywheel Non Tasks"; }
