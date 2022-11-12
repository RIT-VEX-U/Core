/**
 * File: flywheel_commands.h
 * Desc:
 *    [insert meaningful desc]
 */

#pragma once

#include "../core/include/subsystems/flywheel.h"
#include "../core/include/utils/command_structure/auto_command.h"

class SpinRPMCommand: AutoCommand {
  SpinRPMCommand(Flywheel &flywheel, int rpm);

    /**
     * Run spin_manual
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    // Flywheel instance to run the function on
    Flywheel &flywheel;

    // parameters for spinRPM
    int rpm;
};

class FlywheelStopCommand: public AutoCommand {
  FlywheelStopCommand(Flywheel &flywheel);

    /**
     * Run stop
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    // Flywheel instance to run the function on
    Flywheel &flywheel;
};

class FlywheelStopMotorsCommand: public AutoCommand {
  FlywheelStopMotorsCommand(Flywheel &flywheel);

    /**
     * Run stop
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    // Flywheel instance to run the function on
    Flywheel &flywheel;
};

class FlywheelStopNonTasksCommand: public AutoCommand {
  FlywheelStopNonTasksCommand(Flywheel &flywheel);

    /**
     * Run stop
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    // Flywheel instance to run the function on
    Flywheel &flywheel;
};
