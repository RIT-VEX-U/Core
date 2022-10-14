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
  Flywheel(motor_group &motors, PID::pid_config_t &pid_config, FeedForward::ff_config_t &ff_config, const double ratio);  // constructor for feedforward + pid
  Flywheel(motor_group &motors, FeedForward::ff_config_t &ff_config, const double ratio);  // constructor for only feed forward
  Flywheel(motor_group &motors, double tbh_gain, const double ratio);  // constructor for take back half
  Flywheel(motor_group &motors, const double ratio);  // constructor for bang-bang control


  double getDesiredRPM();                                        // returns the desired RPM
  bool isTaskRunning();                                         // returns if a task is running
  motor_group* getMotors();                                     // returns a pointer to the motors
  double getRPM();                                     // get the current velocity of the motors in RPM
  PID* getPID();                                                // returns a pointer to the PID
  double getPIDValue();                                         // get the current OUT value of the PID
  double getFeedforwardValue();                                 // get the current OUT value of the feedforward
  double getTBHGain();                                          // get the gain used for TBH control
  
  void setPIDTarget(double value);                              // set the PID target
  void updatePID(double value);                                 // update the PID with the current value it's tracking

  // SPINNERS AND STOPPERS
  void spin_raw(double speed, directionType dir=fwd);           // Spins at a given speed between -1 and 1
  void spin_manual(double speed, directionType dir=fwd);        // Same as spin_raw, but check to make sure a task isn't running before doing it.
  void spinRPM(int rpm);                                        // spins the turret at a target RPM
  void stop();                                                  // stops the motors and the thread
  void stopMotors();                                            // stops ONLY the motors
  void stopNonTasks();                                          // stops motors IFF a task isn't running and a manual setter isn't being pressed

  private:

  motor_group &motors;                // motors that make up the flywheel
  bool taskRunning = false;           // is the task (thread but not) currently running?
  PID pid;                            // PID on the flywheel
  FeedForward ff;                     // FF constants for the flywheel
  double TBH_gain;                    // TBH gain parameter for the flywheel
  double ratio;                       // multiplies the velocity by this value
  double RPM = -1.0;                  // Desired RPM of the flywheel. 
  task rpmTask;                       // task (thread but not) that handles spinning the wheel at a given RPM
  FlywheelControlStyle control_style; // how the flywheel should be controlled

};