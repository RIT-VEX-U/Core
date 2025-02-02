#include "robot-config.h"
#include "inttypes.h"
#include "wallstake_mech.h"
#include "core.h"



vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors

// ================ OUTPUTS ================
// Motors
vex::motor left_back_bottom(vex::PORT4, vex::gearSetting::ratio6_1, true);
vex::motor left_center_bottom(vex::PORT9, vex::gearSetting::ratio6_1, true);
vex::motor left_front_top(vex::PORT20, vex::gearSetting::ratio6_1, true);
vex::motor left_back_top(vex::PORT19, vex::gearSetting::ratio6_1, true);
vex::motor_group left_drive_motors({left_back_bottom, left_center_bottom, left_back_top, left_front_top});

vex::motor right_back_bottom(vex::PORT8, vex::gearSetting::ratio6_1, false);
vex::motor right_center_bottom(vex::PORT7, vex::gearSetting::ratio6_1, false);
vex::motor right_front_top(vex::PORT18, vex::gearSetting::ratio6_1, false);
vex::motor right_back_top(vex::PORT17, vex::gearSetting::ratio6_1, false);
vex::motor_group right_drive_motors({right_back_bottom, right_center_bottom, right_back_top, right_front_top});

vex::motor conveyor(vex::PORT15, vex::gearSetting::ratio6_1,true);
vex::motor intake_motor(vex::PORT16, vex::gearSetting::ratio6_1,false);

vex::optical color_sensor(vex::PORT5);

vex::motor wallstake_left(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor wallstake_right(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor_group wallstake_motors({wallstake_left, wallstake_right});

Rotation2d initial(from_degrees(1));
Rotation2d tolerance(from_degrees(1));
double offset(40);
vex::pot wall_pot(Brain.ThreeWirePort.B);

PID::pid_config_t wallstake_pid_config {.p = 0.2, .d = 0.01};
PID wallstake_pid(wallstake_pid_config);

vex::distance goal_sensor(vex::PORT6);
WallStakeMech wallstake_mech(wallstake_motors, wall_pot, tolerance, initial, offset, wallstake_pid);

//pnematices
vex::digital_out mcglight_board(Brain.ThreeWirePort.C);
vex::digital_out goal_grabber_sol{Brain.ThreeWirePort.A};

// ================ SUBSYSTEMS ================
PID::pid_config_t drive_pid_cfg{
  .p = 0.25,
  .i = 0.0,
  .d = 0.03,
  .deadband = 0.5,
  .on_target_time = 0.1,
};

PID drive_pid{drive_pid_cfg};

PID::pid_config_t turn_pid_cfg{
  .p = 0.036,
  .i = 0.0001,
  .d = 0.0036,
  .deadband = 2,
  .on_target_time = 0.1,
  .error_method = PID::ERROR_TYPE::ANGULAR,
  
};

PID::pid_config_t turn_pid_cfg_bigI{
  .p = 0.036,
  .i = 0.001,
  .d = 0.0036,
  .deadband = 2,
  .on_target_time = 0.1,
  .error_method = PID::ERROR_TYPE::ANGULAR,
  
};

PID::pid_config_t correction_pid_cfg{
    .p = 0.036,
  .i = 0.0001,
  .d = 0.0036,
  .deadband = 2,
};

FeedForward::ff_config_t drive_ff_cfg{
    .kS = 0.01,
    .kV = 0.015,
    .kA = 0.002,
    .kG = 0
};

MotionController::m_profile_cfg_t drive_motioncontroller_cfg{
    .max_v = 50,
    .accel = 150,
    .pid_cfg = drive_pid_cfg,
    .ff_cfg = drive_ff_cfg
};
MotionController drive_motioncontroller{drive_motioncontroller_cfg};


PID turn_pid{turn_pid_cfg};
PID turn_pidBigI{turn_pid_cfg_bigI};
// ======== SUBSYSTEMS ========

robot_specs_t robot_cfg = {
    .robot_radius = 12,
    .odom_wheel_diam = 2.75,
    .odom_gear_ratio = 0.75,

    .drive_correction_cutoff = 10,

    .drive_feedback = &drive_pid,
    .turn_feedback = &turn_pid,
    // .correction_pid = correction_pid_cfg,
};

bool conveyor_started;
int color_sensor_counter = 0;
bool color_sort_on = false;

MatchPaths matchpath = BLUE_SAFE_AUTO;
bool blue_alliance(){
    switch(matchpath){
        case BLUE_SAFE_AUTO:
            return true;
        default:
            return false;
    }
}

pose_t skills_start{19.25, 48, 0};
pose_t test{24, 96, 0};
pose_t auto_start_red{16.75, 89.25, 180};
pose_t auto_start_blue{127.25, 89.25, 0};
pose_t zero{0, 0, 0};



OdometrySerial odom(true, true, zero, pose_t{-3.83, 0.2647, 270}, vex::PORT1, 115200);

OdometryBase* base = &odom;

TankDrive drive_sys(left_drive_motors, right_drive_motors, robot_cfg, &odom);

// ================ UTILS ================

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{

    screen::start_screen(Brain.Screen, {new screen::PIDPage(drive_pid, "drivepid")});
    vexDelay(100);
    switch(matchpath){
        case RED_SAFE_AUTO:
            odom.send_config(auto_start_red, pose_t{-3.83, 0.2647, 270}, false);
        case BLUE_SAFE_AUTO:
            odom.send_config(auto_start_blue, pose_t{-3.83, 0.2647, 270}, false);
        case BASIC_SKILLS:
            odom.send_config(skills_start, pose_t{-3.83, 0.2647, 270}, false);
        default:
            printf("ERROR: NO PATH GIVEN");
    }
    odom.send_config(auto_start_blue, pose_t{-3.83, 0.2647, 270}, false);
    printf("started!\n");
    printf("%d, %d\n", competition::bStopTasksBetweenModes, competition::bStopAllTasksBetweenModes);
    competition::bStopAllTasksBetweenModes = true;
    competition::bStopTasksBetweenModes = true;
    color_sensor.setLight(vex::ledState::on);
    color_sensor.setLightPower(100, vex::pct);
    turn_pid.set_limits(0.5, 1);
    // mcglight_board.set(true);
    // wallstake_mech.set_voltage(5);
    
    

    while (true) {
        pose_t pose = base->get_position();
        // pose_t posetank = tankodom.get_position();
        // printf("%" PRIu64 ", %f, %f, %f\n", vexSystemHighResTimeGet(), pose.x, pose.y, pose.rot);
        // wallstake_mech.update();
        // printf("%f\n", color_sensor.hue());
        // wallstake_mech.set_setpoint(from_degrees(0));
        // vexDelay(5000);
        // wallstake_mech.set_setpoint(from_degrees(180));
        vexDelay(100);
    }
}

const double intake_volts = 10.0;

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
}

void conveyor_outtake() {
    conveyor.spin(vex::directionType::rev, intake_volts, vex::volt);
}
void conveyor_outtake(double volts) {
    conveyor.spin(vex::directionType::rev, intake_volts, vex::volt);
}

void conveyor_intake(double volts) {
    conveyor.spin(vex::directionType::fwd, volts, vex::volt);

}

void intake_spin(double volts) {
    intake_motor.spin(vex::directionType::fwd, volts, vex::volt);
    vex::this_thread::yield();

}