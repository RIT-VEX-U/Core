/**
 * @file basic_commands.h
 * @author Ethan Kishimori GITHUB(LilacEthan)
 *
 * @brief Basic Commands for Motors and Solenoids able to throw into the Command Controller
 *  Commands Included
 *      - BasicSpinCommand
 *      - BasicStopCommand
 *      - BasicSolenoidSet
 *
 * @version 0.1
 *
 */
#pragma once

#include "core/utils/command_structure/auto_command.h"

// Basic Motor Classes-----------------------------------------------

/**
 * AutoCommand wrapper class for BasicSpinCommand
 * using the vex hardware functions
 */
class BasicSpinCommand : public AutoCommand {
  public:
    // Enumurator for the type of power setting in the motor
    enum type { percent, voltage, velocity };

    /**
     * @brief Construct a new BasicSpinCommand
     *
     * @param motor Motor to spin
     * @param direc Direction of motor spin
     * @param setting Power setting in volts,percentage,velocity
     * @param power Value of desired power
     */
    BasicSpinCommand(vex::motor &motor, vex::directionType dir, BasicSpinCommand::type setting, double power);

    /**
     * @brief Runs the BasicSpinCommand
     * Overrides run from Auto Command
     *
     * @return True Async running command
     */
    bool run() override;

    /*
     * Returns a string describing the commands functionality
     */
    std::string toString() override;

  private:
    vex::motor &motor;

    type setting;

    vex::directionType dir;

    double power;
};
/**
 * AutoCommand wrapper class for BasicStopCommand
 * Using the Vex hardware functions
 */
class BasicStopCommand : public AutoCommand {
  public:
    /**
     * @brief Construct a new BasicMotorStop Command
     *
     * @param motor The motor to stop
     * @param setting The brake setting for the motor
     */
    BasicStopCommand(vex::motor &motor, vex::brakeType setting);

    /**
     * @brief Runs the BasicMotorStop Command
     * Overrides run command from AutoCommand
     *
     * @return True Command runs once
     */
    bool run() override;

    /*
     * Returns a string describing the commands functionality
     */
    std::string toString() override;

  private:
    vex::motor &motor;

    vex::brakeType setting;
};

// Basic Solenoid Commands----------------------------------

/**
 * AutoCommand wrapper class for BasicSolenoidSet
 * Using the Vex hardware functions
 */
class BasicSolenoidSet : public AutoCommand {
  public:
    /**
     * @brief Construct a new BasicSolenoidSet Command
     *
     * @param solenoid Solenoid being set
     * @param setting Setting of the solenoid in boolean (true,false)
     */
    BasicSolenoidSet(vex::pneumatics &solenoid, bool setting);

    /**
     * @brief Runs the BasicSolenoidSet
     * Overrides run command from AutoCommand
     *
     * @return True Command runs once
     */
    bool run() override;
    /*
     * Returns a string describing the commands functionality
     */
    std::string toString() override;

  private:
    vex::pneumatics &solenoid;

    bool setting;
};