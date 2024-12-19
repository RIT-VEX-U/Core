/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       unknown                                                   */
/*    Created:      12/1/2024, 11:02:31 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "robot-config.h"
#include "competition/autonomous.h"
#include "competition/opcontrol.h"

using namespace vex;

competition comp;


/**
 * Entry point to the program. No code should be placed here;
 * instead use competition/opcontrol.cpp and
 * competition/autonomous.cpp
*/
int main() {
    comp.autonomous(autonomous);
    comp.drivercontrol(opcontrol);
    robot_init();
}