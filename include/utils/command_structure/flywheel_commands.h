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
  /**
   * Construct a SpinRPM Command
   * @param flywheel the flywheel sys to command
   * @param rpm the rpm that we should spin at
  */
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
    /** 
     * Creat a WaitUntilUpToSpeedCommand
     * @param flywheel the flywheel system we are commanding
     * @param threshold_rpm the threshold over and under the flywheel target RPM that we define to be acceptable
    */
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
    int threshold_rpm;
};

/**
 * AutoCommand wrapper class for the stop function
 * in the Flywheel class
 *
 */
class FlywheelStopCommand: public AutoCommand {
  public:
  /**
   * Construct a FlywheelStopCommand
   * @param flywheel the flywheel system we are commanding
  */
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
  /**
   * Construct a FlywheeStopMotors Command
   * @param flywheel the flywheel system we are commanding
  */
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
