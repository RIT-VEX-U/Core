/*********************************************************
*
*     File:     Flywheel.cpp
*     Purpose:  Generalized flywheel class for Core.
*     Author:   Chris Nokes, Richie Sommers
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
#include "../core/include/utils/feedforward.h"
#include "../core/include/utils/pid.h"
#include "../core/include/utils/math_util.h"
#include "vex.h"

using namespace vex;

const int FlywheelWindowSize = 20;

/*********************************************************
*         CONSTRUCTOR, GETTERS, SETTERS
*********************************************************/
//used when using bang bang or TBH control
PID::pid_config_t empty_pid = PID::pid_config_t{};
FeedForward::ff_config_t empty_ff = FeedForward::ff_config_t{};

/**
* Create the Flywheel object using PID + feedforward for control.
*/
Flywheel::Flywheel(motor_group &motors, PID::pid_config_t &pid_config, FeedForward::ff_config_t &ff_config, const double ratio)
    :motors(motors), pid(pid_config), ff(ff_config), ratio(ratio), control_style(PID_Feedforward), smoothedRPM(0), RPM_avger(MovingAverage(FlywheelWindowSize)) {
    }

/**
* Create the Flywheel object using only feedforward for control
*/
Flywheel::Flywheel(motor_group &motors, FeedForward::ff_config_t &ff_config, const double ratio)
    :motors(motors), pid(empty_pid), ff(ff_config), ratio(ratio), control_style(Feedforward), smoothedRPM(0), RPM_avger(MovingAverage(FlywheelWindowSize)) {}

/**
* Create the Flywheel object using Take Back Half for control
*/
Flywheel::Flywheel(motor_group &motors, const double TBH_gain, const double ratio)
    :motors(motors), pid(empty_pid), ff(empty_ff), TBH_gain(TBH_gain), ratio(ratio), control_style(Take_Back_Half), smoothedRPM(0), RPM_avger(MovingAverage(FlywheelWindowSize)) {}

/**
* Create the Flywheel object using Bang Bang for control
*/
Flywheel::Flywheel(motor_group &motors, const double ratio)
    :motors(motors), pid(empty_pid), ff(empty_ff), ratio(ratio), control_style(Bang_Bang), smoothedRPM(0), RPM_avger(MovingAverage(FlywheelWindowSize)) {}

/**
* Return the current value that the RPM should be set to
*/
double Flywheel::getDesiredRPM() { return RPM; }

/**
* Checks if the background RPM controlling task is running
* @return taskRunning - If the task is running
*/
bool Flywheel::isTaskRunning() { return taskRunning; }

/**
* Returns a POINTER TO the motors; not currently used.
* @return motorPointer -pointer to the motors
*/
motor_group* Flywheel::getMotors() { return &motors; } // TODO -- Remove?

/**
* return the current velocity of the flywheel motors, in RPM
* @return the measured velocity of the flywheel
*/
double Flywheel::measureRPM() { 
  double rawRPM = ratio * motors.velocity(velocityUnits::rpm); 
  RPM_avger.add_entry(rawRPM);
  smoothedRPM = RPM_avger.get_average();
  return smoothedRPM; //TODO Change back
}

double Flywheel::getRPM(){
  return smoothedRPM;
}
/**
* Returns a POINTER TO the PID; not currently used.
* @return pidPointer -pointer to the PID
*/
PID* Flywheel::getPID() { return &pid; } // TODO -- Remove?

/**
* returns the current OUT value of the PID - the value that the PID would set the motors to
* @return the voltage that PID wants the motors at to achieve the target RPM
*/
double Flywheel::getPIDValue() { return pid.get(); }

/**
* returns the current OUT value of the Feedforward - the value that the Feedforward would set the motors to
* @return the voltage that feedforward wants the motors at to achieve the target RPM
*/
double Flywheel::getFeedforwardValue() { 
  double v = getDesiredRPM();
  return ff.calculate(v, 0); 
}

/**
* get the gain used for TBH control
* @return the gain used in TBH control
*/
double Flywheel::getTBHGain(){
  return TBH_gain;
}

/**
* Sets the value of the PID target
* @param value - desired value of the PID
*/
void Flywheel::setPIDTarget(double value) { pid.set_target(value); }

/**
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

/**
* Runs a Feedforward variant to control rpm
*/
int spinRPMTask_BangBang(void* wheelPointer) {
  Flywheel* wheel = (Flywheel*) wheelPointer; 
  while(true) {
    // if it below the RPM, go, otherwise don't
    wheel->measureRPM();

    if(wheel->getRPM() < wheel->getDesiredRPM()) { 
      wheel->spin_raw(1, fwd);
    }   
    else { wheel->stopMotors(); }
    vexDelay(10);
  }
  return 0;
}

/**
* Runs a Feedforward variant to control rpm
*/
int spinRPMTask_Feedforward(void* wheelPointer) { 
  Flywheel* wheel = (Flywheel*) wheelPointer;
  // get the pid from the wheel and set its target to the RPM stored in the wheel.
  while(true) {
    wheel->measureRPM();
    wheel->updatePID(wheel->getRPM());   // check the current velocity and update the PID with it.
    double output = wheel->getFeedforwardValue();
    wheel->spin_raw(output, fwd);   // set the motors to whatever feedforward tells them to do
    vexDelay(1);
  }
  return 0; 
}
/**
* Runs a PID + Feedforward variant to control rpm
*/
int spinRPMTask_PID_Feedforward(void* wheelPointer) {
  Flywheel* wheel = (Flywheel*) wheelPointer;
  // get the pid from the wheel and set its target to the RPM stored in the wheel.
  while(true) {
    wheel->measureRPM();
    wheel->updatePID(wheel->getRPM());   // check the current velocity and update the PID with it.
    double output = wheel->getPIDValue() + wheel->getFeedforwardValue();
    wheel->spin_raw(output, fwd);   // set the motors to whatever PID tells them to do
    vexDelay(1);
  }
  return 0; 
}

/**
* Runs a Take Back Half variant to control RPM
* https://www.vexwiki.org/programming/controls_algorithms/tbh
*/
int spinRPMTask_TBH(void* wheelPointer) {
  Flywheel* wheel = (Flywheel*) wheelPointer;

  double tbh = 0.0;
  double output = 0.0;
  double previous_error = 0.0;

  while (true){
    wheel->measureRPM();

    //reset if set to 0, this keeps the tbh val from screwing us up when we start up again
    if (wheel->getDesiredRPM()==0){
      output = 0;
      tbh = 0;
    }
   
    double error = wheel->getDesiredRPM() - wheel->getRPM();
    output += wheel->getTBHGain() * error;
    wheel->spin_raw(clamp(output, 0, 1), fwd);

    if (sign(error)!=sign(previous_error)){
      output = .5 * (output + tbh);
      tbh = output;
      previous_error = error;
    }

    vexDelay(1);
  }
  
   return 0; 
}

// Runs a 'Moving average filter with above closed loop systems' variant, whatever that means; FUNCTION STUB
int spinRPMTask_ClosedLoop(void* wheelPointer) { return 0; }

/*********************************************************
*         SPINNERS AND STOPPERS
*********************************************************/

/** 
* Spin motors using voltage; defaults forward at 12 volts
* FOR USE BY TASKS ONLY
* @param speed - speed (between -1 and 1) to set the motor
* @param dir - direction that the motor moves in; defaults to forward
*/
void Flywheel::spin_raw(double speed, directionType dir){
  motors.spin(dir, speed * 12, voltageUnits::volt);
}

/**
* Spin motors using voltage; defaults forward at 12 volts
* FOR USE BY OPCONTROL AND AUTONOMOUS - this only applies if the RPM thread is not running
* @param speed - speed (between -1 and 1) to set the motor
* @param dir - direction that the motor moves in; defaults to forward
*/
void Flywheel::spin_manual(double speed, directionType dir){
  if(!taskRunning) motors.spin(dir, speed * 12, voltageUnits::volt);
}

/**
* starts or sets the RPM thread at new value
* what control scheme is dependent on control_style
* @param inputRPM - set the current RPM
*/
void Flywheel::spinRPM(int inputRPM) {
  // setting to 0 is equivelent to stopping
  if (inputRPM==0){
    stop();
  }
  // only run if the RPM is different or it isn't already running
  if(!taskRunning) {

    int (*rpm_control_task)(void *);  // this just means a function that returns int and takes a void pointer as an argument aka a spinRPMTask function
    // choose which version to use based on how  the flywheel was constructed
    switch(control_style){
      case Bang_Bang:
        rpm_control_task = spinRPMTask_BangBang;
        break;
      case Take_Back_Half:
        rpm_control_task = spinRPMTask_TBH;
        break;
      case Feedforward:
        rpm_control_task = spinRPMTask_Feedforward;
        break;
      case PID_Feedforward:
        rpm_control_task = spinRPMTask_PID_Feedforward;
        break;
    }


    rpmTask = task(rpm_control_task, this);
    taskRunning = true;
  }
  RPM = inputRPM;
  setPIDTarget(RPM);
}

/**
* stop the RPM thread and the wheel
*/
void Flywheel::stop() {
  rpmTask.stop();
  taskRunning = false;
  RPM = 0.0;
  smoothedRPM = 0.0;
  motors.stop();
}

/**
* stop only the motors; exclusively for BANG BANG use
*/
void Flywheel::stopMotors() { motors.stop(); }

/**
* Stop the motors if the task isn't running - stop manual control
*/
void Flywheel::stopNonTasks() { if(!taskRunning) { motors.stop(); }}