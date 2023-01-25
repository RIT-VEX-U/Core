/**
 * File: flywheel_commands.h
 * Desc:
 *    [insert meaningful desc]
 */

#pragma once

#include "../core/include/subsystems/flywheel.h"
#include "../core/include/utils/command_structure/auto_command.h"

/**
 * AutoCommand wrapper class for the spinRPM function
 * in the Flywheel class
 *
 */
class SpinRPMCommand: public AutoCommand {
  public:
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

/**
 * AutoCommand that listens to the Flywheel and waits until it is at its target speed +/- the specified threshold
 *
 */
class WaitUntilUpToSpeedCommand: public AutoCommand {
  public:
    WaitUntilUpToSpeedCommand(Flywheel &flywheel, int threshold_rpm);

    /**
     * Run spin_manual
     * Overrides run from AutoCommand
     * @returns true when execution is complete, false otherwise
     */
    bool run() override;

  private:
    // Flywheel instance to run the function on
    Flywheel &flywheel;

    // if the actual speed is equal to the desired speed +/- this value, we are ready to fire
    int threshhold_rpm;
};

/**
 * AutoCommand wrapper class for the stop function
 * in the Flywheel class
 *
 */
class FlywheelStopCommand: public AutoCommand {
  public:
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

/**
 * AutoCommand wrapper class for the stopMotors function
 * in the Flywheel class
 *
 */
class FlywheelStopMotorsCommand: public AutoCommand {
  public:
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

/**
 * AutoCommand wrapper class for the stopNonTasks function
 * in the Flywheel class
 *
 */
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
