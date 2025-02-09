#include "competition/opcontrol.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "vex.h"

const vex::controller::button &goal_grabber = con.ButtonB;
const vex::controller::button &conveyor_button = con.ButtonR1;
const vex::controller::button &conveyor_button_rev = con.ButtonR2;

const vex::controller::button &wallstake_toggler = con.ButtonL1;
const vex::controller::button &wallstake_stow = con.ButtonL2;
const vex::controller::button &wallstake_alliancestake = con.ButtonDown;

const vex::controller::button &ColorSortToggle = con.ButtonLeft;

void testing();

void auto__();

// int goal_counter = 0;


/**
 * Main entrypoint for the driver control period
 */
void opcontrol() {
    // testing();
    conveyor.stop();

    wallstake_mech.set_state(STOW);
    wallstake_mech.hold = true;

    ColorSortToggle.pressed([]() {
        intake_sys.setColorSortOn(!intake_sys.getColorSortOn());
    });

    goal_grabber.pressed([]() {
        clamper_sys.clamp();
    });

    conveyor_button.pressed([]() {
        intake_sys.intake();
        intake_sys.conveyor_in();
    });
    conveyor_button_rev.pressed([]() {
        intake_sys.outtake();
        intake_sys.conveyor_out();
    });

    wallstake_toggler.pressed([]() {
        wallstake_mech.hold = true;
        if (wallstake_mech.get_angle().degrees() > 180) {
            wallstake_mech.set_setpoint(from_degrees(173));
        } else if (wallstake_mech.get_angle().degrees() < 180) {
            wallstake_mech.set_setpoint(from_degrees(50));
        }
    });

    wallstake_stow.pressed([]() {
        wallstake_mech.hold = true;
        wallstake_mech.set_setpoint(from_degrees(198.5));
    });

    wallstake_alliancestake.pressed([]() {
        wallstake_mech.hold = true;
        wallstake_mech.set_setpoint(from_degrees(0));
    });

    // ================ INIT ================

    while (true) {
        if (!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
            intake_sys.conveyor_stop();
            intake_sys.intake_stop();
        }
        double left = (double)con.Axis3.position() / 100;
        double right = (double)con.Axis2.position() / 100;

        drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);

        vexDelay(20);
    }

    // ================ PERIODIC ================
}


void testing() {

    class DebugCommand : public AutoCommand {
      public:
        bool run() override {
            drive_sys.stop();
            pose_t pos = odom.get_position();
            // printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
            while (true) {
                double left = (double)con.Axis3.position() / 100;
                double right = (double)con.Axis2.position() / 100;

                // drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);

                vexDelay(100);
            }
            return true;
        }
    };

    

    con.ButtonA.pressed([]() {
    printf("running test");
	mcglight_board.set(true);
	CommandController cc {
		// odom.SetPositionCmd({.x = 9.5, .y = 72, .rot = 0}),

		new Async(new FunctionCommand([]() {
			while(true) {
				vexDelay(20);
            }
			return true;
		}))
        };
        cc.run();
    });
}

