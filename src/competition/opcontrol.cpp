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
        color_sort_on = !color_sort_on;
        printf("switching color sort mode");
    });

    goal_grabber.pressed([]() {
        goal_grabber_sol.set(!goal_grabber_sol);
        // goal_counter = 50;
    });

    conveyor_button.pressed([]() {
        conveyor.spin(vex::directionType::fwd, 10, vex::volt);
        intake();
        mcglight_board.set(color_sort_on);
    });
    conveyor_button_rev.pressed([]() {
        conveyor.spin(vex::directionType::rev, 10, vex::volt);
        outtake();
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
            conveyor.stop();
            intake(0);
            mcglight_board.set(false);
        }
        if(!color_sort_on){
            mcglight_board.set(false);
        }

        double left = (double)con.Axis3.position() / 100;
        double right = (double)con.Axis2.position() / 100;

        drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);

        pose_t pos = odom.get_position();
        // printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);

        if (blue_alliance() && color_sort_on) {
            if (color_sensor.hue() > 0 && color_sensor.hue() < 30 && color_sensor_counter == 0) {

                color_sensor_counter = 30;
				conveyor_intake(12);
            }
        } else {
            if ((color_sensor.hue() > 160 && color_sensor.hue() < 240 && color_sensor_counter == 0) && color_sort_on) {
                color_sensor_counter = 30;
				conveyor_intake(12);
            }
        }

        if (color_sensor_counter == 25) {
            color_sensor_counter--;
            conveyor.stop();
            // conveyor_intake(12);
        }

        if (color_sensor_counter > 0) {
            color_sensor_counter--;
            
        }

		 if (conveyor_button.pressing() && color_sensor_counter == 0) {
                      conveyor_intake();
                  }

        // if (goal_sensor.objectDistance(vex::mm) < 25 && goal_counter == 0) {
        //     goal_grabber_sol.set(true);
        // }

        // if (goal_counter > 0) {
        //     goal_counter--;
        // }

        if (color_sensor_counter == 0 && conveyor_button.pressing()) {
          conveyor_intake();
        }

        vexDelay(20);
    }

    // ================ PERIODIC ================
}

AutoCommand *intake_command_auto(double amt = 10.0) {
	return new FunctionCommand([=]() {
        
		intake(amt);
		return true;
	});
}

AutoCommand *conveyor_intake_command_auto(double amt = 10.0) {
	return new FunctionCommand([=]() {
		conveyor_intake(amt);
		conveyor_started = true;
		return true;
	});
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
				OdometryBase *odombase = &odom;
                pose_t pos = odombase->get_position();
            	// printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
				vexDelay(100);

                // if (goal_sensor.objectDistance(vex::mm) < 25 && goal_counter == 0) {
                // goal_grabber_sol.set(true);
                // }

                // if (goal_counter > 0) {
                //     goal_counter--;
                // }

		// 		if (goal_sensor.objectDistance(vex::mm) < 25 && goal_counter == 0) {
        //     goal_grabber_sol.set(true);
        // }

        // if (goal_counter > 0) {
        //     goal_counter--;
        // }

		if (blue_alliance()) {
            if (color_sensor.hue() > 0 && color_sensor.hue() < 30 && color_sensor_counter == 0) {
                printf("wrong color detected\n");
				printf("wrong color detected\n");
				printf("wrong color detected\n");
				printf("wrong color detected\n");
				printf("wrong color detected\n");
				printf("wrong color detected\n");

                color_sensor_counter = 30;
				conveyor_intake(12);
            }
        } else {
            if (color_sensor.hue() > 160 && color_sensor.hue() < 240 && color_sensor_counter == 0) {
                printf("wrong color detected\n");
				printf("wrong color detected\n");
				printf("wrong color detected\n");
				printf("wrong color detected\n");
				printf("wrong color detected\n");
				printf("wrong color detected\n");
				printf("wrong color detected\n");
				printf("wrong color detected\n");
				printf("wrong color detected\n");
                color_sensor_counter = 30;
				conveyor_intake(12);
            }
        }

        if (color_sensor_counter == 25) {
            color_sensor_counter--;
            conveyor.stop();
            // conveyor_intake(12);
        }

        if (color_sensor_counter > 0) {
            color_sensor_counter--;
            
        }

		 if (conveyor_started && color_sensor_counter == 0) {
                      conveyor_intake();
                  }
			}
			return true;
		})),
        intake_command_auto(),
        conveyor_intake_command_auto(),
        new DelayCommand(10000),
        
        };
        cc.run();
    });
}

