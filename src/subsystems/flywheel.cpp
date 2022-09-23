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
*********************************************************/

#include "../core/include/subsystems/flywheel.h"
#include "../core/include/utils/pid.h"
#include "vex.h"

using namespace vex;

/*
* Create the Flywheel object.
* @param motors - pointer to the motors on the fly wheel
* @param pid - pointer to the PID
*/
Flywheel::Flywheel(motor_group &motors, PID &pid)
    :motors(motors), pid(pid){ }

/* Spin motors using voltage; defaults forward at 12 volts
* @param speed - speed (between -1 and 1) to set the motor
* @param dir - direction that the motor moves in; defaults to forward
*/
void Flywheel::spin_raw(double speed, directionType dir){
  motors.spin(dir, speed * 12, voltageUnits::volt);
}

/* spin this flywheel at a given RPM, async; runs until stop(), stopThread(), or a new spinRPM() is called. 
* @param wheelPointer - points to the current wheel object
*/
int spinRPMThread(void* wheelPointer) {
  Flywheel* wheel = (Flywheel*) wheelPointer;
  wheel->getPID()->set_target((double) wheel->getRPM());  // get the pid from the wheel and set its target to the RPM stored in the wheel.
  while(true) {
    wheel->getPID()->update(wheel->getMotors()->velocity(velocityUnits::rpm));        // check the current velocity and update the PID with it.
    wheel->getMotors()->spin(fwd, wheel->getPID()->get() * 12, voltageUnits::volt);   // set the motors to whatever PID tells them to do
    vexDelay(20);
  }
  return 0; // only here to make the compiler SHUT UP
}

/* starts / restarts RPM thread at new value
* @param inputRPM - set the current RPM
*/
void Flywheel::spinRPM(int inputRPM) {
  if(inputRPM != RPM || !taskRunning) {
    RPM = inputRPM;
    rpmTask.stop();
    rpmTask = task(spinRPMThread, this);
    taskRunning = true;
  }
}

// return the current value that the RPM should be set to;
double Flywheel::getRPM() { return RPM; }

// Returns a POINTER TO the PID; only for use in the spinRPMThread function.
PID* Flywheel::getPID() { return &pid; }

// Returns a POINTER TO the motors; only for use in the spinRPMThread function.
motor_group* Flywheel::getMotors() { return &motors; }

// stop the RPM thread and the wheel
void Flywheel::stop() {
  rpmTask.stop();
  taskRunning = false;
  motors.stop();
}


// end the thread but keep spinning the wheel; not sure why anyone would use this but here it is anyway.
void Flywheel::stopThread() {
  taskRunning = false;
  rpmTask.stop();
}