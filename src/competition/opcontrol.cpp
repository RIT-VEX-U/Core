#include "competition/opcontrol.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "vex.h"

void testing();

void auto__();

/**
 * Main entrypoint for the driver control period
 */
void opcontrol() {
    // testing();
    conveyor.stop();

    wallstakemech_sys.set_state(WallStakeMech::STOW);
    wallstakemech_sys.hold = true;
    intake_sys.opcontrol_init();

    ColorSortToggle.pressed([]() {
        intake_sys.set_color_sort_bool(!intake_sys.get_color_sort_bool());
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
        wallstakemech_sys.hold = true;
        if (wallstakemech_sys.get_angle().degrees() > 180) {
            wallstakemech_sys.set_setpoint(from_degrees(170));
        } else if (wallstakemech_sys.get_angle().degrees() < 180) {
            wallstakemech_sys.set_setpoint(from_degrees(50));
        }
    });

    wallstake_stow.pressed([]() {
        wallstakemech_sys.hold = true;
        wallstakemech_sys.set_setpoint(from_degrees(200));
    });

    // ================ INIT ================

    while (true) {
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

