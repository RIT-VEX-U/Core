#pragma once

#include "core/robot_specs.h"
#include "core/subsystems/screen.h"
#include "core/utils/command_structure/auto_command.h"
#include "core/utils/controls/feedforward.h"
#include "core/utils/controls/pid.h"
#include "vex.h"
#include <atomic>

/**
 * a Flywheel class that handles all control of a high inertia spinning disk
 * It gives multiple options for what control system to use in order to control wheel velocity and functions alerting
 * the user when the flywheel is up to speed. Flywheel is a set and forget class. Once you create it you can call
 * spin_rpm or stop on it at any time and it will take all necessary steps to accomplish this
 *
 */
class Flywheel {

public:
  // CONSTRUCTORS, GETTERS, AND SETTERS
  /**
   * Create the Flywheel object using PID + feedforward for control.
   * @param motors      pointer to the motors on the fly wheel
   * @param feedback    a feedback controleller
   * @param helper      a feedforward config (only kV is used) to help the feedback controller along
   * @param ratio       ratio of the gears from the motor to the flywheel just multiplies the velocity
   * @param filter      the filter to use to smooth noisy motor readings
   */
  Flywheel(vex::motor_group &motors, Feedback &feedback, FeedForward &helper, const double ratio, Filter &filt);

  /**
   * Return the target_rpm that the flywheel is currently trying to achieve
   * @return target_rpm  the target rpm
   */
  double get_target() const;

  /**
   * return the velocity of the flywheel
   */
  double getRPM() const;

  /**
   * Returns the motors
   */
  vex::motor_group &get_motors() const;

  /**
   * Spin motors using voltage; defaults forward at 12 volts
   * FOR USE BY OPCONTROL AND AUTONOMOUS - this only applies if the target_rpm thread is not running
   * @param speed - speed (between -1 and 1) to set the motor
   * @param dir - direction that the motor moves in; defaults to forward
   */
  void spin_manual(double speed, directionType dir = fwd);

  /**
   * starts or sets the target_rpm thread at new value
   * what control scheme is dependent on control_style
   * @param rpm - the target_rpm we want to spin at
   */
  void spin_rpm(double rpm);

  /**
   * Stops the motors. If manually spinning, this will do nothing just call spin_mainual(0.0) to send 0 volts
   */
  void stop();

  /**
   * @brief check if the feedback controller thinks the flywheel is on target
   * @return true if on target
   */
  bool is_on_target() { return fb.is_on_target(); }

  /**
   *  @brief Creates a page displaying info about the flywheel
   *  @return the page should be used for `screen::start_screen(screen, {fw.Page()});
   */
  screen::Page *Page() const;

  /**
   * @brief Creates a new auto command to spin the flywheel at the desired velocity
   * @param rpm the rpm to spin at
   * @return an auto command to add to a command controller
   */
  AutoCommand *SpinRpmCmd(int rpm) {

    return new FunctionCommand([this, rpm]() {
      spin_rpm(rpm);
      return true;
    });
  }

  /**
   * @brief Creates a new auto command that will hold until the flywheel has its target as defined by its feedback
   * controller
   * @return an auto command to add to a command controller
   */
  AutoCommand *WaitUntilUpToSpeedCmd() {
    return new WaitUntilCondition(new FunctionCondition([this]() { return is_on_target(); }));
  }

private:
  friend class FlywheelPage;
  friend int spinRPMTask(void *wheelPointer);

  vex::motor_group &motors;       ///< motors that make up the flywheel
  bool task_running = false;      ///< is the task currently running?
  Feedback &fb;                   ///< Main Feeback controller
  FeedForward &ff;                ///< Helper Feedforward Controller
  vex::mutex fb_mut;              ///< guard for talking to the runner thread
  double ratio;                   ///< ratio between motor and flywheel. For accurate RPM calcualation
  std::atomic<double> target_rpm; ///< Desired RPM of the flywheel.
  task rpm_task;                  ///< task that handles spinning the wheel at a given target_rpm
  Filter &avger;                  ///< Moving average to smooth out noise from

  // Functions for internal use only
  /**
   * Sets the target rpm of the flywheel
   * @param value - desired RPM
   */
  void set_target(double value);
  /**
   * make a measurement of the current target_rpm of the flywheel motor and return a smoothed version
   */
  double measure_RPM();

  /**
   * Spin motors using voltage; defaults forward at 12 volts
   * FOR USE BY TASKS ONLY
   * @param speed - speed (between -1 and 1) to set the motor
   * @param dir - direction that the motor moves in; defaults to forward
   */
  void spin_raw(double speed, directionType dir = fwd);
};