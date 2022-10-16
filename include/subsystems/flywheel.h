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
#include <atomic>

using namespace vex;

class Flywheel{
  enum FlywheelControlStyle{
    PID_Feedforward,
    Feedforward,
    Take_Back_Half,
    Bang_Bang,
  };
  public:

  // CONSTRUCTORS, GETTERS, AND SETTERS
  /**
  * Create the Flywheel object using PID + feedforward for control.
  * @param motors     - pointer to the motors on the fly wheel
  * @param pid_config - pointer the pid config
  * @param ff         - pointer to the feedforward config
  * @param ratio      - ratio of the whatever just multiplies the velocity
  */
  Flywheel(motor_group &motors, PID::pid_config_t &pid_config, FeedForward::ff_config_t &ff_config, const double ratio);

  /**
  * Create the Flywheel object using only feedforward for control
  * @param motors - pointer to the motors on the fly wheel
  * @param ff     - pointer to the feedforward config
  * @param ratio  - ratio of the whatever just multiplies the velocity
  */
  Flywheel(motor_group &motors, FeedForward::ff_config_t &ff_config, const double ratio);

  /**
  * Create the Flywheel object using Take Back Half for control
  * @param motors   - pointer to the motors on the fly wheel
  * @param TBH_gain - the TBH control paramater
  * @param ratio    - ratio of the whatever just multiplies the velocity
  */
  Flywheel(motor_group &motors, double tbh_gain, const double ratio);

  /**
  * Create the Flywheel object using Bang Bang for control
  * @param motors - pointer to the motors on the fly wheel
  * @param ratio  - ratio of the whatever just multiplies the velocity
  */
  Flywheel(motor_group &motors, const double ratio);

  /**
  * Return the current value that the RPM should be set to
  * @return RPM = the target rpm
  */
  double getDesiredRPM();

  /**
  * Checks if the background RPM controlling task is running
  * @return taskRunning - If the task is running
  */
  bool isTaskRunning();

  /**
  * Returns a POINTER to the motors
  */
  motor_group* getMotors();

  /**
  * return the current velocity of the flywheel motors, in RPM
  */
  double getRPM();

  /**
  * Returns a POINTER to the PID.
  */
  PID* getPID();

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
  void spin_raw(double speed, directionType dir=fwd);
  
  /**
  * Spin motors using voltage; defaults forward at 12 volts
  * FOR USE BY OPCONTROL AND AUTONOMOUS - this only applies if the RPM thread is not running
  */
  void spin_manual(double speed, directionType dir=fwd);
  
  /**
  * starts or sets the RPM thread at new value
  * what control scheme is dependent on control_style
  * @param inputRPM - set the current RPM
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

};