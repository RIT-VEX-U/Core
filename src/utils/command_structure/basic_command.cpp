/**
 * @file basic_commands.cpp
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
#include "core/utils/command_structure/basic_command.h"

// Basic Motor Commands--------------------------------------
/**
 * @brief a BasicMotorSpin Command
 *
 * @param motor Motor port to spin
 * @param dir Direction for spining
 * @param setting Power setting in volts,percentage,velocity
 * @param power Value of desired power
 */
BasicSpinCommand::BasicSpinCommand(vex::motor& motor, vex::directionType dir, BasicSpinCommand::type setting,
                                   double power)
    : motor(motor), setting(setting), dir(dir), power(power) {}

/**
 * @brief Run the BasicSpinCommand
 * Overrides run from Auto Command
 *
 * @return True Command runs once
 */
bool BasicSpinCommand::run() {
  switch (setting) {  // Switch Statement taking the setting Enum
    case voltage:     // Voltage Setting
      motor.spin(dir, power, vex::volt);
      break;
    case percent:  // Percentage Setting
      motor.spin(dir, power, vex::percent);
      break;
    case velocity:  // Velocity Setting
      motor.spin(dir, power, vex::velocityUnits::rpm);
      break;
  }
  return true;  // Always return True to send next on CommandController
}

/*
 * Returns a string describing the commands functionality
 */
std::string BasicSpinCommand::toString() {
  std::string str = "Spinnning motors ";
  str.append((dir == vex::directionType::fwd) ? "forwards at " : "reverse at ");
  switch (setting) {  // Switch Statement taking the setting Enum
    case voltage:     // Voltage Setting
      str.append(double_to_string(power) + "V");
      break;
    case percent:  // Percentage Setting
      str.append(double_to_string(power * 100) + "%");
      break;
    case velocity:  // Velocity Setting
      str.append(double_to_string(power) + "Dps");
      break;
  }
  return str;
};

/**
 * @brief Construct a BasicMotorStop Command
 *
 * @param motor Motor to stop
 * @param setting Braketype setting brake,coast,hold
 */
BasicStopCommand::BasicStopCommand(vex::motor& motor, vex::brakeType setting) : motor(motor), setting(setting) {}

/**
 * @brief Runs the BasicMotorStop command
 * Ovverides run command from AutoCommand
 *
 * @return True Command runs once
 */
bool BasicStopCommand::run() {
  motor.stop(setting);
  return true;
}

/*
 * Returns a string describing the commands functionality
 */
std::string BasicStopCommand::toString() {
  switch (setting) {
    case vex::brakeType::brake:
      return "Braking motors";
    case vex::brakeType::coast:
      return "Coasting motors";
    case vex::brakeType::hold:
      return "Holding motors";
    default:
      return "UNKNOWN BRAKE TYPE";
  }
}

// Basic Solenoid Commands-----------------------------------
/**
 * @brief Construct a new BasicSolenoidSet Command
 *
 * @param solenoid Solenoid being set
 * @param setting Setting of the solenoid in boolean (true,false)
 */
BasicSolenoidSet::BasicSolenoidSet(vex::pneumatics& solenoid, bool setting) : solenoid(solenoid), setting(setting) {}

/**
 * @brief Runs the BasicSolenoidSet
 * Overrides run command from AutoCommand
 *
 * @return True Command runs once
 */
bool BasicSolenoidSet::run() {
  solenoid.set(setting);
  return true;
}

/*
 * Returns a string describing the commands functionality
 */
std::string BasicSolenoidSet::toString() {
  if (setting) {
    return "Activating solonoid";
  } else {
    return "Deactivating solonoid";
  }
};