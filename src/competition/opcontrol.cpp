#include "competition/opcontrol.h"
#include "robot-config.h"
#include "vex.h"

#include "competition/autonomous.h"
const vex::controller::button &goal_grabber = con.ButtonRight;
const vex::controller::button &conveyor_button = con.ButtonR2;
const vex::controller::button &conveyor_button_rev = con.ButtonR1;

const vex::controller::button &wallstake_toggler = con.ButtonL1;
const vex::controller::button &wallstake_stow = con.ButtonL2;
const vex::controller::button &wallstake_alliancestake = con.ButtonDown;
const vex::controller::button &toggle_colorsort = con.ButtonLeft;

void testing();

void auto__();

void opcontrol() {
    vexDelay(1000);

    autonomous();
    return;
    conveyor.stop();

    wallstakemech_sys.set_setpoint(from_degrees(200));
    wallstakemech_sys.hold = true;
    clamper_sys.unclamp();

    toggle_colorsort.pressed([]() { intake_sys.set_color_sort_bool(!intake_sys.get_color_sort_bool()); });

    goal_grabber.pressed([]() { clamper_sys.toggle_clamp(); });

    conveyor_button.pressed([]() {
        intake_sys.intake();
        intake_sys.conveyor_in();
    });
    conveyor_button_rev.pressed([]() {
        intake_sys.outtake();
        intake_sys.conveyor_out();
    });

    wallstake_toggler.pressed([]() {
        wallstakemech_sys.hold = true;
        if (wallstakemech_sys.get_angle().degrees() > 180 || wallstake_motors.velocity(vex::velocityUnits::dps) > 5) {
            wallstakemech_sys.set_setpoint(from_degrees(170));
        } else if (wallstakemech_sys.get_angle().degrees() < 180) {
            wallstakemech_sys.set_setpoint(from_degrees(45));
        }
    });

    wallstake_stow.pressed([]() {
        wallstakemech_sys.hold = true;
        wallstakemech_sys.set_setpoint(from_degrees(200));
    });

    // ================ INIT ================
    while (true) {
        if (!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
            intake_sys.intake_stop();
            intake_sys.conveyor_stop();
        }

        double straight = (double)con.Axis3.position() / 100;
        double turn = (double)con.Axis1.position() / 100;

        drive_sys.drive_arcade(straight, turn * 1.75, 1, TankDrive::BrakeType::None);

        pose_t pos = odom.get_position();

        vexDelay(20);
    }

    // ================ PERIODIC ================
}
