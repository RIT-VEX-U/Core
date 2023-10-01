#pragma once
/*********************************************************
 *
 *     File:     Flywheel.h
 *     Purpose:  Generalized flywheel class for Core.
 *     Author:   Chris Nokes
 *
 **********************************************************
 * EDIT HISTORY
 **********************************************************
 * 09/23/2022  <CRN> Reorganized, added documentation.
 * 09/23/2022  <CRN> Added functions elaborated on in .cpp.
 *********************************************************/
#include "../core/include/utils/feedforward.h"
#include "vex.h"
#include "../core/include/robot_specs.h"
#include "../core/include/utils/pid.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include <atomic>

using namespace vex;

/**
 * a Flywheel class that handles all control of a high inertia spinning disk
 * It gives multiple options for what control system to use in order to control wheel velocity and functions alerting the user when the flywheel is up to speed.
 * Flywheel is a set and forget class.
 * Once you create it you can call spinRPM or stop on it at any time and it will take all necessary steps to accomplish this
 *
 */
class Flywheel
{
  enum FlywheelControlStyle
  {
    PID_Feedforward,
    Feedforward,
    Take_Back_Half,
    Bang_Bang,
  };

public:
  // CONSTRUCTORS, GETTERS, AND SETTERS
  /**
   * Create the Flywheel object using PID + feedforward for control.
   * @param motors      pointer to the motors on the fly wheel
   * @param pid_config  pointer the pid config to use
   * @param ff_config   the feedforward config to use
   * @param ratio       ratio of the whatever just multiplies the velocity
   */
  Flywheel(motor_group &motors, PID::pid_config_t &pid_config, FeedForward::ff_config_t &ff_config, const double ratio);

  /**
   * Create the Flywheel object using only feedforward for control
   * @param motors    the motors on the fly wheel
   * @param ff_config the feedforward config to use
   * @param ratio     ratio of the whatever just multiplies the velocity
   */
  Flywheel(motor_group &motors, FeedForward::ff_config_t &ff_config, const double ratio);

  /**
   * Create the Flywheel object using Take Back Half for control
   * @param motors   the motors on the fly wheel
   * @param tbh_gain the TBH control paramater
   * @param ratio    ratio of the whatever just multiplies the velocity
   */
  Flywheel(motor_group &motors, double tbh_gain, const double ratio);

  /**
   * Create the Flywheel object using Bang Bang for control
   * @param motors the motors on the fly wheel
   * @param ratio  ratio of the whatever just multiplies the velocity
   */
  Flywheel(motor_group &motors, const double ratio);

  /**
   * Return the RPM that the flywheel is currently trying to achieve
   * @return RPM  the target rpm
   */
  double getDesiredRPM();

  /**
   * Checks if the background RPM controlling task is running
   * @return true if the task is running
   */
  bool isTaskRunning();

  /**
   * Returns a POINTER to the motors
   */
  motor_group *getMotors();

  /**
   * make a measurement of the current RPM of the flywheel motor and return a smoothed version
   */
  double measureRPM();

  /**
   * return the current smoothed velocity of the flywheel motors, in RPM
   */
  double getRPM();
  /**
   * Returns a POINTER to the PID.
   */
  PID *getPID();

  /**
   * returns the current OUT value of the PID - the value that the PID would set the motors to
   */
  double getPIDValue();

  /**
   * returns the current OUT value of the PID - the value that the PID would set the motors to
   */
  double getFeedforwardValue();

  /**
   * get the gain used for TBH control
   */
  double getTBHGain();

  /**
   * Sets the value of the PID target
   * @param value - desired value of the PID
   */
  void setPIDTarget(double value);

  /**
   * updates the value of the PID
   * @param value - value to update the PID with
   */
  void updatePID(double value);

  // SPINNERS AND STOPPERS

  /**
   * Spin motors using voltage; defaults forward at 12 volts
   * FOR USE BY TASKS ONLY
   * @param speed - speed (between -1 and 1) to set the motor
   * @param dir - direction that the motor moves in; defaults to forward
   */
  void spin_raw(double speed, directionType dir = fwd);

  /**
   * Spin motors using voltage; defaults forward at 12 volts
   * FOR USE BY OPCONTROL AND AUTONOMOUS - this only applies if the RPM thread is not running
   * @param speed - speed (between -1 and 1) to set the motor
   * @param dir - direction that the motor moves in; defaults to forward
   */
  void spin_manual(double speed, directionType dir = fwd);

  /**
   * starts or sets the RPM thread at new value
   * what control scheme is dependent on control_style
   * @param rpm - the RPM we want to spin at
   */
  void spinRPM(int rpm);

  /**
   * stop the RPM thread and the wheel
   */
  void stop();

  /**
   * stop only the motors; exclusively for BANG BANG use
   */
  void stopMotors();

  /**
   * Stop the motors if the task isn't running - stop manual control
   */
  void stopNonTasks();

  AutoCommand *SpinRpmCmd(int rpm)
  {

    return new FunctionCommand([this]()
                               {spinRPM(1000); return true; });
  }

  AutoCommand *WaitUntilUpToSpeedCmd()
  {
    return new WaitUntilCondition(
        new FunctionCondition([this]()
                              { return RPM == smoothedRPM; }));
  }

private:
  motor_group &motors;                // motors that make up the flywheel
  bool taskRunning = false;           // is the task (thread but not) currently running?
  PID pid;                            // PID on the flywheel
  FeedForward ff;                     // FF constants for the flywheel
  double TBH_gain;                    // TBH gain parameter for the flywheel
  double ratio;                       // multiplies the velocity by this value
  std::atomic<double> RPM;            // Desired RPM of the flywheel.
  task rpmTask;                       // task (thread but not) that handles spinning the wheel at a given RPM
  FlywheelControlStyle control_style; // how the flywheel should be controlled
  double smoothedRPM;
  MovingAverage RPM_avger;
};