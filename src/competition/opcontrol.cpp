#include "competition/opcontrol.h"
#include "robot-config.h"
#include "vex.h"

void testing();

void auto__();

void opcontrol() {

    conveyor.stop();

    wallstakemech_sys.set_state(WallStakeMech::STOW);
    wallstakemech_sys.hold = true;
    clamper_sys.unclamp();
    intake_sys.opcontrol_init();

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
        if (wallstakemech_sys.get_angle().degrees() > 180) {
            wallstakemech_sys.set_setpoint(from_degrees(170));
        } else if (wallstakemech_sys.get_angle().degrees() < 180) {
            wallstakemech_sys.set_setpoint(from_degrees(50));
        }
    });

    // wallstake_stow.pressed([]() {
    //     wallstakemech_sys.hold = true;
    //     wallstakemech_sys.set_setpoint(from_degrees(200));
    // });

    // wallstake_alliancestake.pressed([]() {
    //     wallstake_mech.hold = true;
    //     wallstake_mech.set_setpoint(from_degrees(0));
    // });

    // ================ INIT ================
    color_sensor.setLight(vex::ledState::on);
    color_sensor.setLightPower(100, vex::pct);

    conveyor_intake(0);

    while (true) {
        if (!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
            intake_sys.intake_stop();
            intake_sys.conveyor_stop();
            // if (self.color_sort_state == ColorSortState::ON) {
            // mcglight_board.set(true);
            // self.colorSort();
            // } else {
            // mcglight_board.set(false);
            // }
        }

        // if (!wallstake_down.pressing() || !wallstake_up.pressing()) {
        //     wallstake_mech.hold = true;
        // } else {
        //     wallstake_mech.hold = false;
        // }

        // if (!intake_button.pressing() && !intake_button_rev.pressing()) {
        //     intake(0);
        // }
        double straight = (double)con.Axis3.position() / 100;
        double turn = (double)con.Axis1.position() / 100;

        drive_sys.drive_arcade(straight, turn * 1.75, 1, TankDrive::BrakeType::None);

        pose_t pos = odom.get_position();
        // printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
        // printf("%f\n", color_sensor.hue());

        // if (goal_sensor.objectDistance(vex::mm) < 25 && goal_counterd == 0) {
        //     goal_grabber_sol.set(true);
        // }

        // if (goal_counterd > 0) {
        //     goal_counterd--;
        // }

        vexDelay(20);
    }

    // ================ PERIODIC ================
}
