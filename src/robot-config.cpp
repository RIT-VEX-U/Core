#include "robot-config.h"

vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors

// Analog sensors
CustomEncoder Left_enc(Brain.ThreeWirePort.C, 2048);
CustomEncoder right_enc(Brain.ThreeWirePort.E, 2048);
CustomEncoder front_enc(Brain.ThreeWirePort.G, 2048);
// ================ OUTPUTS ================
// Motors
vex::motor left_back_bottom(vex::PORT11, vex::gearSetting::ratio6_1, false);
vex::motor left_center_bottom(vex::PORT12, vex::gearSetting::ratio6_1, false);
vex::motor left_front_top(vex::PORT13, vex::gearSetting::ratio6_1, false);
vex::motor left_back_top(vex::PORT14, vex::gearSetting::ratio6_1, false);
vex::motor_group left_drive_motors({left_back_bottom, left_center_bottom, left_back_top, left_front_top});

vex::motor right_back_bottom(vex::PORT1, vex::gearSetting::ratio6_1, true);
vex::motor right_center_bottom(vex::PORT2, vex::gearSetting::ratio6_1, true);
vex::motor right_front_top(vex::PORT3, vex::gearSetting::ratio6_1, true);
vex::motor right_back_top(vex::PORT4, vex::gearSetting::ratio6_1, true);
vex::motor_group right_drive_motors({right_back_bottom, right_center_bottom, right_back_top, right_front_top});

vex::motor conveyor(vex::PORT19, vex::gearSetting::ratio6_1,false);
vex::motor intake(vex::PORT20, vex::gearSetting::ratio6_1,true);

//pnematices
vex::digital_out goal_grabber_sol{Brain.ThreeWirePort.A};

// ================ SUBSYSTEMS ================
PID::pid_config_t drive_pid_cfg{};
PID drive_pid(drive_pid_cfg);

PID::pid_config_t turn_pid_cfg{};
PID turn_pid(turn_pid_cfg);
// ======== SUBSYSTEMS ========

robot_specs_t robot_cfg = {
    .robot_radius = 12,
    .odom_wheel_diam = 2.125,
    .odom_gear_ratio = 1.0,

    .drive_feedback = &drive_pid,
    .turn_feedback = &turn_pid,
};

OdometrySerial odom(true, true, pose_t{0, 0, 0}, pose_t{-3.83, 0.2647, 180}, 9, 115200);
TankDrive drive_sys(left_drive_motors, right_drive_motors, robot_cfg);

// ================ UTILS ================

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{
    //imu.startCalibration();
}

void conveyor_intake(double volts) {
    conveyor.spin(vex::directionType::fwd, volts, vex::volt);

}

void intake_spin(double volts) {
    intake.spin(vex::directionType::fwd, volts, vex::volt);

}