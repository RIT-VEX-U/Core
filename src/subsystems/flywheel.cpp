/*********************************************************
*
*     File:     Flywheel.cpp
*     Purpose:  Generalized flywheel class for Core.
*     Author:   Chris Nokes
*     
**********************************************************
* EDIT HISTORY
**********************************************************
* 09/16/2022  <CRN> Created file, added constructor, spins, RPM setting, stop.
* 09/18/2022  <CRN> Added async functionality.
* 09/22/2022  <CRN> Documentation improvements, fixed error if RPM is set but motor is stopped.
* 09/23/2022  <CRN> Neatened up program, added getters and setters, fixed documentation and bang bang.
* 09/29/2022  <CRN> Bug fixes, RPM handling. Multiplied the motor by 18.
*********************************************************/

#include "../core/include/subsystems/flywheel.h"
#include "../core/include/utils/pid.h"
#include "vex.h"

using namespace vex;

/*********************************************************
*         CONSTRUCTOR, GETTERS, SETTERS
*********************************************************/

/*
* Create the Flywheel object.
* @param motors - pointer to the motors on the fly wheel
* @param pid - pointer to the PID
* @param ratio - ratio of the whatever just multiplies the velocity
*/
Flywheel::Flywheel(motor_group &motors, PID::pid_config_t &pid_config, const double ratio)
    :motors(motors), pid(pid_config), ratio(ratio) { }

// return the current value that the RPM should be set to;
double Flywheel::getRPM() { return RPM; }

// returns whether or not the wheel is running a task currently
bool Flywheel::isTaskRunning() { return taskRunning; }
  
// Returns a POINTER TO the motors; not currently used.
motor_group* Flywheel::getMotors() { return &motors; } // TODO -- Remove?

// return the current velocity of the flywheel motors, in RPM
double Flywheel::getVelocity_RPM() { return ratio * motors.velocity(velocityUnits::rpm); }

// Returns a POINTER TO the PID; not currently used.
PID* Flywheel::getPID() { return &pid; } // TODO -- Remove?

// returns the current OUT value of the PID
double Flywheel::getPIDValue() { return pid.get(); }

/* 
* Sets the value of the PID target
* @param value - desired value of the PID
*/
void Flywheel::setPIDTarget(double value) { pid.set_target(value); }

/* 
* updates the value of the PID
* @param value - value to update the PID with
*/
void Flywheel::updatePID(double value) { pid.update(value); }

/*********************************************************
*         RPM SETTING THREADS
* ALL OF THE FOLLOWING PROGRAMS HAVE THE SAME PARAMETERS AND RESULTS:
* spin this flywheel at a given RPM, async; runs until stop(), stopThread(), or a new spinRPM() is called. 
* @param wheelPointer - points to the current wheel object
*********************************************************/

// Runs a PID loop to get to the set RPM.
int spinRPMTask_PID(void* wheelPointer) {
  Flywheel* wheel = (Flywheel*) wheelPointer;
  // get the pid from the wheel and set its target to the RPM stored in the wheel.
  while(true) {
    wheel->updatePID(wheel->getVelocity_RPM());   // check the current velocity and update the PID with it.
    wheel->spin_raw(wheel->getPIDValue(), fwd);   // set the motors to whatever PID tells them to do
    vexDelay(1);
  }
  return 0; // only here to make the compiler SHUT UP
}

//  Runs a BANG BANG variant. I have NO IDEA if it will work properly but I have NOTHING to test this on.
int spinRPMTask_BangBang(void* wheelPointer) {
  Flywheel* wheel = (Flywheel*) wheelPointer; 
  while(true) {
    // if it below the RPM, go, otherwise don't
    if(wheel->getVelocity_RPM() < wheel->getRPM()) { 
      wheel->spin_raw(1, fwd);
    }   
    else { wheel->stopMotors(); }
    vexDelay(1);
  }
  return 0; // only here to make the compiler SHUT UP
}

// Runs a Feedforward variant; FUNCTION STUB
int spinRPMTask_Feedforward(void* wheelPointer) { return 0; }

// Runs a PID + Feedforward variant; FUNCTION STUB
int spinRPMTask_PID_Feedforward(void* wheelPointer) { return 0; }

// Runs a Take Back Half variant; FUNCTION STUB
int spinRPMTask_TBH(void* wheelPointer) { return 0; }

// Runs a 'Moving average filter with above closed loop systems' variant, whatever that means; FUNCTION STUB
int spinRPMTask_ClosedLoop(void* wheelPointer) { return 0; }

/*********************************************************
*         SPINNERS AND STOPPERS
*********************************************************/

/* Spin motors using voltage; defaults forward at 12 volts
* FOR USE BY TASKS ONLY
* @param speed - speed (between -1 and 1) to set the motor
* @param dir - direction that the motor moves in; defaults to forward
*/
void Flywheel::spin_raw(double speed, directionType dir){
  motors.spin(dir, speed * 12, voltageUnits::volt);
}

// Spin motors using voltage; defaults forward at 12 volts
// FOR USE BY OPCONTROL AND AUTONOMOUS, same as spin_raw otherwise
void Flywheel::spin_manual(double speed, directionType dir){
  if(!taskRunning) motors.spin(dir, speed * 12, voltageUnits::volt);
}

/* starts / restarts RPM thread at new value
* @param inputRPM - set the current RPM
*/
void Flywheel::spinRPM(int inputRPM) {
  // only run if the RPM is different or it isn't already running
  if(!taskRunning) {
    rpmTask = task(spinRPMTask_PID, this);
    taskRunning = true;
  }
  RPM = inputRPM;
  setPIDTarget(RPM);
}

// stop the RPM thread and the wheel
void Flywheel::stop() {
  rpmTask.stop();
  taskRunning = false;
  motors.stop();
}

// stop only the motors; exclusively for BANG BANG use
void Flywheel::stopMotors() { motors.stop(); }

// stop the motors iff the task isn't running
void Flywheel::stopNonTasks() { if(!taskRunning) { motors.stop(); }}