#include "robot-config.h"
#include "inttypes.h"
#include "wallstake_mech.h"
#include "core.h"



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

vex::motor conveyor(vex::PORT20, vex::gearSetting::ratio6_1,true);
vex::motor intake_motor(vex::PORT19, vex::gearSetting::ratio6_1,false);

vex::motor wallstake_left(vex::PORT15, vex::gearSetting::ratio18_1, false);
vex::motor wallstake_right(vex::PORT16, vex::gearSetting::ratio18_1, true);
vex::motor_group wallstake_motors({wallstake_left, wallstake_right});

Rotation2d initial(from_degrees(1));
Rotation2d tolerance(from_degrees(1));
double offset(41.8);
vex::pot wall_pot(Brain.ThreeWirePort.H);

PID::pid_config_t wallstake_pid_config {.p = 0.2, .d = 0.01};
PID wallstake_pid(wallstake_pid_config);

vex::distance goal_sensor(vex::PORT8);
// WallStakeMech wallstake_mech(wallstake_motors, wall_pot, tolerance, initial, offset, wallstake_pid);

vex::optical color_sensor(vex::PORT17);

//pnematices
vex::digital_out goal_grabber_sol{Brain.ThreeWirePort.A};

// ================ SUBSYSTEMS ================
PID::pid_config_t drive_pid_cfg{
  .p = 0.2,
  .i = 0.0,
  .d = 0.02,
  .deadband = 0.5,
  .on_target_time = 0.1,
};

PID drive_pid{drive_pid_cfg};

PID::pid_config_t turn_pid_cfg{
  .p = 0.01,
  .i = 0.00,
  .d = 0.00,
  .deadband = 1.5,
  .on_target_time = 0.2,
};

PID::pid_config_t correction_pid_cfg{
    .p = 0.01,
    .i = 0.0001,
    .d = 0.0025,
};

FeedForward::ff_config_t drive_ff_cfg{
    .kS = 0.08,
    .kV = 0.05,
    .kA = 0.0005,
    .kG = 0
};

MotionController::m_profile_cfg_t drive_motioncontroller_cfg{
    .max_v = 50,
    .accel = 150,
    .pid_cfg = drive_pid_cfg,
    .ff_cfg = drive_ff_cfg
};

AsymmetricMotionController::a_m_profile_cfg_t drive_motioncontroller_slow_decel_cfg {
    .max_v = 50,
    .accel = 200,
    .decel = 70,
    .pid_cfg = drive_pid_cfg,
    .ff_cfg = drive_ff_cfg
};

AsymmetricMotionController drive_motioncontroller_slow_decel{drive_motioncontroller_slow_decel_cfg};
MotionController drive_motioncontroller{drive_motioncontroller_cfg};


PID turn_pid{turn_pid_cfg};
// ======== SUBSYSTEMS ========

robot_specs_t robot_cfg = {
    .robot_radius = 12,
    .odom_wheel_diam = 2.75,
    .odom_gear_ratio = 0.75,

    .drive_correction_cutoff = 10,

    .drive_feedback = &drive_pid,
    .turn_feedback = &turn_pid,
    .correction_pid = correction_pid_cfg,
};
pose_t skills_start{19.25, 96, 0};
pose_t auto_start{121.73, 54.77, 30};
pose_t zero{0, 0, 0};

OdometrySerial odom(true, true, auto_start, pose_t{-3.83, 0.2647, 180}, 9, 115200);
OdometryBase* base = &odom;

TankDrive drive_sys(left_drive_motors, right_drive_motors, robot_cfg, &odom);
// OdometryTank tankodom{left_drive_motors, right_drive_motors, robot_cfg, &imu};
vex::inertial imu(vex::PORT18);


// ================ UTILS ================

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{
    screen::start_screen(Brain.Screen, {new screen::PIDPage(drive_pid, "drivepid")});
    vexDelay(50);
    printf("started!\n");
    color_sensor.setLight(vex::ledState::on);
    color_sensor.setLightPower(100, vex::pct);
    // wallstake_mech.set_voltage(5);
    
    

    // while (true) {
    //     pose_t pose = base->get_position();
    //     // pose_t posetank = tankodom.get_position();
    //     printf("%" PRIu64 ", %f, %f, %f\n", vexSystemHighResTimeGet(), pose.x, pose.y, pose.rot);
    //     // wallstake_mech.update();
    //     // printf("%f\n", wallstake_mech.get_angle().degrees());
    //     // wallstake_mech.set_setpoint(from_degrees(0));
    //     // vexDelay(5000);
    //     // wallstake_mech.set_setpoint(from_degrees(180));
    //     vexDelay(20);
    // }
}

const double intake_volts = 12.0;

void intake(double volts) {
    intake_motor.spin(vex::directionType::fwd, volts, vex::volt);
}

void intake() {
    intake_motor.spin(vex::directionType::fwd, intake_volts, vex::volt);
}

void outtake(double volts) {
    intake_motor.spin(vex::directionType::rev, volts, vex::volt);
}

void outtake() {
    intake_motor.spin(vex::directionType::rev, intake_volts, vex::volt);
}

void conveyor_intake() {
    conveyor.spin(vex::directionType::fwd, intake_volts, vex::volt);
    intake_motor.spin(vex::directionType::fwd, intake_volts, vex::volt);
}

void conveyor_intake(double volts) {
    conveyor.spin(vex::directionType::fwd, volts, vex::volt);
    intake_motor.spin(vex::directionType::fwd, volts, vex::volt);
}

void intake_spin(double volts) {
    intake_motor.spin(vex::directionType::fwd, volts, vex::volt);
    vex::this_thread::yield();

}