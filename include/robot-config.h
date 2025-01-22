#pragma once
#include "vex.h"
#include "core.h"

extern vex::brain brain;
extern vex::controller con;

// ================ INPUTS ================
// Digital sensors

// Analog sensors
extern CustomEncoder Left_enc;
extern CustomEncoder right_enc;
extern CustomEncoder front_enc;

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

extern vex::motor conveyor;
extern vex::motor intake;

void conveyor_intake(double volts);
void intake_spin(double volts);

extern vex::motor_group left_drive_motors;
extern vex::motor_group right_drive_motors;

// Pneumatics
extern vex::digital_out goal_grabber_sol;

// ================ SUBSYSTEMS ================
extern PID drive_pid;
extern PID turn_pid;
extern OdometrySerial odom;

extern robot_specs_t robot_cfg;
extern TankDrive drive_sys;

// ================ UTILS ================

void robot_init();
