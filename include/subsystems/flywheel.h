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
*********************************************************/

#include "vex.h"
#include "../core/include/robot_specs.h"
#include "../core/include/utils/pid.h"

using namespace vex;

class Flywheel{
  public:

  // CONSTRUCTORS, GETTERS, AND SETTERS
  Flywheel(motor_group &motors, PID::pid_config_t &pid_config); // constructor
  double getRPM();                                              // returns the desired RPM
  motor_group* getMotors();                                     // returns a pointer to the motors
  double getVelocity_RPM();                                     // get the current velocity of the motors in RPM
  PID* getPID();                                                // returns a pointer to the PID
  double getPIDValue();                                         // get the current OUT value of the PID
  void setPIDTarget(double value);                              // set the PID target
  void updatePID(double value);                                 // update the PID with the current value it's tracking

  // SPINNERS AND STOPPERS
  void spin_raw(double speed, directionType dir=fwd);           // Spins at a given speed between -1 and 1
  void spinRPM(int rpm);                                        // spins the turret at a target RPM
  void stop();                                                  // stops the motors and the thread
  void stopMotors();                                            // stops ONLY the motors
  void stopThread();                                            // stops ONLY the thread

  private:

  motor_group &motors;      // motors that make up the flywheel
  bool taskRunning = false; // is the task (thread but not) currently running?
  PID pid;                  // PID on the flywheel
  double RPM = -1.0;        // Desired RPM of the flywheel. 
  task rpmTask;             // task (thread but not) that handles spinning the wheel at a given RPM
};