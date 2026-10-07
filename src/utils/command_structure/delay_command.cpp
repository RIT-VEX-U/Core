/**
 * File: delay_command.h
 * Desc:
 *    A DelayCommand will make the robot wait the set amount of
 *    milliseconds before continuing execution of the autonomous route
 */

#pragma once

#include "core/utils/command_structure/auto_command.h"
#include "core/utils/command_structure/delay_command.h"


/**
 * Construct a delay command
 * @param ms the number of milliseconds to delay for
 */
DelayCommand::DelayCommand(int ms) : ms(ms) {}

/**
 * Delays for the amount of milliseconds stored in the command
 * Overrides run from AutoCommand
 * @returns true when complete
 */
bool DelayCommand::run() {
    vexDelay(ms);
    return true;
}

std::string DelayCommand::toString() { return "Delaying for " + std::to_string(ms) + "ms"; }

