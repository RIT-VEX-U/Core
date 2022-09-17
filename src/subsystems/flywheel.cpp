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
*********************************************************/

#include "../core/include/subsystems/flywheel.h"
#include "../core/include/utils/pid.h"
#include "vex.h"

using namespace vex;

/*
* Create the Flywheel object.
*/
Flywheel::Flywheel(motor_group &motors, robot_specs_t &config, PID &pid)
         :motors(motors), config(config), pid(pid){ }

// Spin motors using voltage; defaults forward at 12 volts
// Speed is between -1 and 1.
void Flywheel::spin(double speed, directionType dir){
  motors.spin(dir, speed * VOLTAGECONSTANT, voltageUnits::volt);
}


// Gets to RPM; continue calling at some delay
bool Flywheel::spinRPM(int RPM) {
  // run if this is the first time calling
  if(!inRPMLoop){
    pid.set_target(RPM);
    inRPMLoop = true;
  }

  pid.update(motors.velocity(velocityUnits::rpm));

  if(pid.is_on_target()) {
    inRPMLoop = false;
    return true;
  } return false;
}

// stop
void Flywheel::stop() {
  motors.stop();
}

// Sets RPM based on distance to lubber's hole
void Flywheel::spinToDistance(double distance){
  
}