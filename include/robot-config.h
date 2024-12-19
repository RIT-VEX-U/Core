#pragma once
#include "vex.h"
#include "core.h"

extern vex::brain brain;
extern vex::controller con;

// ================ INPUTS ================
// Digital sensors

// Analog sensors


// ================ OUTPUTS ================
// Motors
extern vex::motor left_back_bottom;
extern vex::motor left_center_bottom;
extern vex::motor left_front_top;
extern vex::motor left_back_top;

extern vex::motor right_back_bottom;
extern vex::motor right_center_bottom;
extern vex::motor right_front_top;
extern vex::motor right_back_top;

extern vex::motor_group left_drive_motors;
extern vex::motor_group right_drive_motors;
// Pneumatics

// ================ SUBSYSTEMS ================
extern PID drive_pid;
extern PID turn_pid;
extern OdometryNWheel<3> odom;

extern robot_specs_t robot_cfg;
extern TankDrive drive_sys;

// ================ UTILS ================

void robot_init();