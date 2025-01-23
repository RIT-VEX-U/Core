#include "robot-config.h"
#include "inttypes.h"
#include "wallstake_mech.h"



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

vex::motor wallstake_left(vex::PORT15, vex::gearSetting::ratio18_1, false);
vex::motor wallstake_right(vex::PORT16, vex::gearSetting::ratio18_1, true);
vex::motor_group wallstake_motors({wallstake_left, wallstake_right});

Rotation2d initial(from_degrees(1));
Rotation2d tolerance(from_degrees(1));
double offset(41.8);
vex::pot wall_pot(Brain.ThreeWirePort.H);

PID::pid_config_t wallstake_pid_config {.p = 0.2, .d = 0.01};
PID wallstake_pid(wallstake_pid_config);
WallStakeMech wallstake_mech(wallstake_motors, wall_pot, tolerance, initial, offset, wallstake_pid);

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
    .odom_wheel_diam = 2.75,
    .odom_gear_ratio = 0.75,

    .drive_feedback = &drive_pid,
    .turn_feedback = &turn_pid,
};

TankDrive drive_sys(left_drive_motors,  right_drive_motors, robot_cfg);

OdometrySerial odom(true, true, pose_t{0, 0, 0}, pose_t{-3.83, 0.2647, 180}, 9, 115200);
OdometryTank tankodom{left_drive_motors, right_drive_motors, robot_cfg, &imu};
vex::inertial imu(vex::PORT18);


// ================ UTILS ================

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{
    vexDelay(50);
    // wallstake_mech.set_voltage(5);
    
    

    // while (true) {
        // Pose2d pose = odom.get_pose2d();
        // pose_t posetank = tankodom.get_position();
        // printf("%" PRIu64 ", %f, %f, %f, %f, %f, %f\n", vexSystemHighResTimeGet(), pose.translation().x(), pose.translation().y(), pose.rotation().wrapped_degrees_360(), posetank.x, posetank.y, posetank.rot);
        // wallstake_mech.update();
        // printf("%f\n", wallstake_mech.get_angle().degrees());
        // wallstake_mech.set_setpoint(from_degrees(0));
        // vexDelay(5000);
        // wallstake_mech.set_setpoint(from_degrees(180));
        // vexDelay(5000);
    // }
}

void conveyor_intake(double volts) {
    conveyor.spin(vex::directionType::fwd, volts, vex::volt);

}

void intake_spin(double volts) {
    intake.spin(vex::directionType::fwd, volts, vex::volt);
    vex::this_thread::yield();

}