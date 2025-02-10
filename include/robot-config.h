#pragma once
#include "vex.h"
#include "core.h"
#include "../core/include/subsystems/odometry/odometry_serial.h"
#include "TempSubSystems/TempSubSystems.h"


#define WALLSTAKE_POT_OFFSET 

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
extern vex::motor intake_motor;

extern vex::motor wallstake_left;
extern vex::motor wallstake_right;
extern vex::motor_group wallstake_motors;

extern Rotation2d initial;
extern Rotation2d tolerance;
extern double pot_offset;
extern vex::pot wall_pot;
extern WallStakeMech wallstakemech_sys;
extern vex::optical color_sensor;
extern vex::digital_out mcglight_board;

extern vex::motor_group left_drive_motors;
extern vex::motor_group right_drive_motors;

// Pneumatics
extern vex::digital_out goal_grabber_sol;
extern vex::inertial imu;

extern vex::distance goal_sensor;

extern vex::pot wall_pot;
//Button Definitions
extern const controller::button &goal_grabber;
extern const controller::button &conveyor_button;
extern const controller::button &conveyor_button_rev;

extern const controller::button &wallstake_toggler;
extern const controller::button &wallstake_stow;
extern const controller::button &wallstake_alliancestake;

extern const controller::button &ColorSortToggle;


// ================ SUBSYSTEMS ================
extern ClamperSys clamper_sys;
extern IntakeSys intake_sys;
extern PID drive_pid;
extern PID turn_pid;
extern PID turn_pidBigI;
extern MotionController::m_profile_cfg_t drive_motioncontroller_cfg;
extern MotionController drive_motioncontroller;

extern PID::pid_config_t correction_pid_cfg;
extern OdometrySerial odom;
extern OdometryTank tankodom;

extern robot_specs_t robot_cfg;
extern TankDrive drive_sys;

// ================ UTILS ================
enum MatchPaths{
	BLUE_SAFE_AUTO,
	RED_SAFE_AUTO,
	BASIC_SKILLS,
};
extern MatchPaths matchpath;
extern bool blue_alliance();
void robot_init();