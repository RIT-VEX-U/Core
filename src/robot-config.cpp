#include "robot-config.h"
#include "core.h"
#include "inttypes.h"
// #include "autopathing/auto-red-safe.cpp"

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

vex::motor conveyor(vex::PORT15, vex::gearSetting::ratio6_1, true);
vex::motor intake_motor(vex::PORT16, vex::gearSetting::ratio6_1, false);

vex::optical color_sensor(vex::PORT5);

vex::motor wallstake_left(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor wallstake_right(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor_group wallstake_motors({wallstake_left, wallstake_right});

Rotation2d initial(from_degrees(210));
Rotation2d tolerance(from_degrees(1));
double offset(0);

vex::rotation wall_rot(vex::PORT11);
PID::pid_config_t wallstake_pid_config{.p = 0.3, .d = 0.005, .error_method = PID::ANGULAR};
PID wallstake_pid(wallstake_pid_config);

vex::distance goal_sensor(vex::PORT6);
WallStakeMech wallstakemech_sys(wallstake_motors, wall_rot, tolerance, initial, offset, wallstake_pid);

// pnematices
vex::digital_out mcglight_board(Brain.ThreeWirePort.C);
vex::digital_out goal_grabber_sol{Brain.ThreeWirePort.A};

// Button Definitions
const vex::controller::button &goal_grabber = con.ButtonB;
const vex::controller::button &conveyor_button = con.ButtonR1;
const vex::controller::button &conveyor_button_rev = con.ButtonR2;

const vex::controller::button &wallstake_toggler = con.ButtonL1;
const vex::controller::button &wallstake_stow = con.ButtonL2;
const vex::controller::button &wallstake_alliancestake = con.ButtonDown;

const vex::controller::button &ColorSortToggle = con.ButtonLeft;

// ================ SUBSYSTEMS ================
PID::pid_config_t drive_pid_cfg{
  .p = 0.08,
  .i = 0.002,
  .d = 0.008,
  .deadband = 0.5,
  .on_target_time = 0.1,
};

PID drive_pid{drive_pid_cfg};

PID::pid_config_t turn_pid_cfg{
  .p = 0.04,
  .i = 0.0042,
  .d = 0.004,
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
  .p = 0.01,
  .i = 0.0001,
  .d = 0.0025,
  .deadband = 2,
};

FeedForward::ff_config_t drive_ff_cfg{.kS = 0.01, .kV = 0.015, .kA = 0.002, .kG = 0};

MotionController::m_profile_cfg_t drive_motioncontroller_cfg{
  .max_v = 50, .accel = 150, .pid_cfg = drive_pid_cfg, .ff_cfg = drive_ff_cfg
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
MatchPaths matchpath = MatchPaths::BASIC_SKILLS;

bool blue_alliance() {
    if (matchpath == MatchPaths::BLUE_SAFE_AUTO) {
        return true;
        printf("BLUEA\n");
    } else {
        return false;
        printf("REDA\n");
    }
}
ClamperSys clamper_sys{};
IntakeSys intake_sys{};

pose_t skills_start{18.5, 96, 0};
pose_t test{24, 96, 0};
pose_t auto_start_red{16.25, 88.75, 180};
pose_t auto_start_blue{127.75, 88.75, 0};
pose_t zero{0, 0, 0};

OdometrySerial odom(true, true, skills_start, pose_t{-3.83, 0.2647, 270}, vex::PORT1, 115200);

OdometryBase *base = &odom;

TankDrive drive_sys(left_drive_motors, right_drive_motors, robot_cfg, &odom);

// ================ UTILS ================

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init() {

    screen::start_screen(Brain.Screen, {new screen::PIDPage(turn_pid, "turnpid")});
    // matchpath = MatchPaths::RED_SAFE_AUTO;
    //  odom.send_config(auto_start_red, pose_t{-3.83, 0.2647, 270}, false);
    vexDelay(1000);
    if (matchpath == MatchPaths::RED_SAFE_AUTO) {
        printf("RED\n");
        odom.send_config(auto_start_red, pose_t{-3.83, 0.2647, 270}, false);
    } else if (matchpath == MatchPaths::BLUE_SAFE_AUTO) {
        printf("BLUE\n");
        odom.send_config(auto_start_blue, pose_t{-3.83, 0.2647, 270}, false);
    } else if (matchpath == MatchPaths::BASIC_SKILLS) {
        printf("SKILLS\n");
        odom.send_config(skills_start, pose_t{-3.83, 0.2647, 270}, false);
    } else {
        printf("ERROR: NO PATH GIVEN\n");
    }
    printf("started!\n");
    // printf("%d, %d\n", competition::bStopTasksBetweenModes, competition::bStopAllTasksBetweenModes);
    // competition::bStopTasksBetweenModes = true;
    // competition::bStopAllTasksBetweenModes = true;
    color_sensor.setLight(vex::ledState::on);
    color_sensor.setLightPower(100, vex::pct);
    turn_pid.set_limits(0.5, 1);
    // mcglight_board.set(true);
    // wallstake_mech.set_voltage(5);
    wall_rot.setReversed(true);

    while (true) {
        // pose_t pose = base->get_position();
        // pose_t posetank = tankodom.get_position();
        // printf("%" PRIu64 ", %f, %f, %f\n", vexSystemHighResTimeGet(), pose.x, pose.y, pose.rot);
        // printf("%" PRIu64 ", %f, %f, %f\n", vexSystemHighResTimeGet(), pose.x, pose.y, pose.rot);
        // wallstakemech_sys.hold = false;
        // printf("%f\n", color_sensor.hue());
        // printf("Wallstake Angle: %f\n", wallstakemech_sys.get_angle().degrees());
        // wallstake_mech.set_setpoint(from_degrees(0));
        // vexDelay(5000);
        // wallstake_mech.set_setpoint(from_degrees(180));
        vexDelay(100);
    }
}