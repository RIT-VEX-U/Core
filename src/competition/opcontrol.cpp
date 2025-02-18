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
    intake_sys.conveyor_stop();

    // wallstakemech_sys.set_state(WallStakeMech::STOW);
    // wallstakemech_sys.hold = true;

    ColorSortToggle.pressed([]() { intake_sys.set_color_sort_bool(!intake_sys.get_color_sort_bool()); });

    goal_grabber.pressed([]() { clamper_sys.toggle_clamp(); });

    conveyor_button.pressed([]() {
        intake_sys.intake();
        intake_sys.conveyor_in();
    });
    conveyor_button_rev.pressed([]() {
        intake_sys.outtake();
        intake_sys.conveyor_out();
    });

    conveyor_button.released([]() {
        intake_sys.intake_stop();
        intake_sys.conveyor_stop();
    });

    conveyor_button_rev.released([]() {
        intake_sys.intake_stop();
        intake_sys.conveyor_stop();
    });

    wallstake_toggler.pressed([]() {
        wallstakemech_sys.hold = true;
        if (wallstakemech_sys.get_angle().degrees() > 180 || wallstake_motors.velocity(vex::velocityUnits::dps) > 5) {
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
        if (!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
            intake_sys.intake_stop();
            intake_sys.conveyor_stop();
        }

        double left = (double)con.Axis3.position() / 100;
        double right = (double)con.Axis2.position() / 100;
        
        // drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);

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
    con.ButtonX.pressed([]() {
        printf("running test");
        CommandController cc{
            new Async(new FunctionCommand([]() {
                while (true) {
                    printf("ODO X: %f ODO Y: %f, ODO ROT: %f TURNPID ERROR: %f\n", odom.get_position().x, odom.get_position().y, odom.get_position().rot, turn_pid.get_error());
                    vexDelay(20);
                }
                return true;
            })),
            // intake_sys.IntakeCmd(10),
            // new DelayCommand(1000),
            // intake_sys.OuttakeCmd(10),
            // new DelayCommand(1000),
            // intake_sys.IntakeStopCmd(),
            drive_sys.TurnDegreesCmd(15, 1),
            drive_sys.TurnDegreesCmd(30, 1),
            drive_sys.TurnDegreesCmd(45, 1),
            drive_sys.TurnDegreesCmd(90, 1),
            drive_sys.TurnDegreesCmd(180, 1),
            drive_sys.TurnDegreesCmd(360, 1),
        };
        cc.run();
    });
}
