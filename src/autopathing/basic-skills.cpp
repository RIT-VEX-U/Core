#include "competition/autonomous.h"
#include "robot-config.h"
#include "autopathing/basic-skills.h"

AutoCommand *intake_command(double amt = 10.0) {
	return new FunctionCommand([=]() {
		intake(amt);
		return true;
	});
}

AutoCommand *outtake_command(double amt = 10.0) {
	return new FunctionCommand([=]() {
		intake(-amt);
		return true;
	});
}

AutoCommand *stop_intake() {
	return new FunctionCommand([=]() {
		intake(0);
		return true;
	});
}

AutoCommand *conveyor_intake_command(double amt = 10.0) {
	return new FunctionCommand([=]() {
		conveyor_intake(amt);
		conveyor_started = true;
		return true;
	});
}

AutoCommand *conveyor_stop_command() {
	return new FunctionCommand([=]() {
		conveyor_intake(0);
		conveyor_started = false;
		return true;
	});
}

AutoCommand *goal_grabber_command(bool value) {
	return new FunctionCommand([=]() {
		goal_grabber_sol.set(value);
		return true;
	});
}

AutoCommand *alliance_score_command(bool hold = true) {
	return new FunctionCommand([=]() {
		wallstake_mech.hold = hold;
        wallstake_mech.set_setpoint(from_degrees(0));
		return true;
	});
}

AutoCommand *stow_command() {
	return new FunctionCommand([=]() {
		wallstake_mech.hold = true;
        wallstake_mech.set_setpoint(from_degrees(198.5));
		return true;
	});
}

AutoCommand *handoff_command() {
	return new FunctionCommand([=]() {
		wallstake_mech.hold = true;
        wallstake_mech.set_state(HANDOFF);
		return true;
	});
}

AutoCommand *wallstake_command() {
	return new FunctionCommand([=]() {
		wallstake_mech.hold = true;
        wallstake_mech.set_setpoint(from_degrees(80));
		return true;
	});
}

void skills_basic() {
	mcglight_board.set(true);
	CommandController cc {
		// odom.SetPositionCmd({.x = 9.5, .y = 72, .rot = 0}),

		new Async(new FunctionCommand([]() {
			while(true) {
				OdometryBase *odombase = &odom;
                pose_t pos = odombase->get_position();
            	printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
				vexDelay(100);

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


      // goal 1 ring 1
	  intake_command(),
      drive_sys.DriveToPointCmd({48, 48}, vex::forward, 0.5, 0)->withTimeout(2),

      // goal 1 grab
      drive_sys.TurnToHeadingCmd(90, 0.5, 0)->withTimeout(0.8),
      drive_sys.DriveToPointCmd({48, 24}, vex::reverse, 0.5, 0)->withTimeout(1), 
	  conveyor_intake_command(),
	  goal_grabber_command(true),
	  

      // goal 1 ring 2
      drive_sys.TurnToHeadingCmd(0, 0.5, 0)->withTimeout(0.8),
      drive_sys.DriveToPointCmd({72, 24}, vex::forward, 0.5, 0)->withTimeout(1), new DelayCommand(350),

      // goal 1 ring 3
      drive_sys.TurnToHeadingCmd(-90, 0.6, 0)->withTimeout(0.5),
      drive_sys.DriveForwardCmd(6, vex::forward, 0.8, 0)->withTimeout(0.5), new DelayCommand(100),
      drive_sys.DriveForwardCmd(6, vex::reverse, 0.8, 0)->withTimeout(0.5),

      // goal 1 ring 4
      drive_sys.TurnToHeadingCmd(180, 1, 0)->withTimeout(0.8),
      drive_sys.DriveToPointCmd({24, 24}, vex::forward, 0.6, 0)->withTimeout(4),

      // goal 1 ring 5
      drive_sys.TurnToHeadingCmd(-135, 0.8, 0)->withTimeout(1),
      drive_sys.DriveForwardCmd(17, vex::forward, 0.3, 0)->withTimeout(1), new DelayCommand(500),
      drive_sys.DriveForwardCmd(10, vex::reverse, 0.4, 0)->withTimeout(1),

      // goal 1 drop
      drive_sys.TurnToHeadingCmd(45, 0.8, 0)->withTimeout(1), drive_sys.DriveTankCmd(-0.3, -0.3)->withTimeout(1),
      new DelayCommand(1000), goal_grabber_command(false),

      // exit corner
      drive_sys.DriveToPointCmd({24, 24}, vex::forward, 0.5, 0)->withTimeout(2), conveyor_stop_command(),

      // goal 2 ring 1
      drive_sys.TurnToPointCmd(96, 24, vex::fwd, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({96, 24}, vex::fwd, 0.8, 0)->withTimeout(3),

      // goal 2 grab
      drive_sys.TurnToHeadingCmd(-90, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({96, 48}, vex::reverse, 0.4, 0)->withTimeout(1), conveyor_intake_command(),
	  goal_grabber_command(true),
      new DelayCommand(1000),

      // goal 2 ring 2
      drive_sys.TurnToHeadingCmd(0, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({120, 48}, vex::forward, 0.6, 0)->withTimeout(1),

      // goal 2 ring 3
      drive_sys.TurnToHeadingCmd(-90, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({120, 24}, vex::forward, 0.6, 0)->withTimeout(1),

      // goal 2 ring 4
      drive_sys.TurnToHeadingCmd(-45, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveForwardCmd(17, vex::forward, 0.3, 0)->withTimeout(1), new DelayCommand(500),
      drive_sys.DriveForwardCmd(10, vex::forward, 0.4, 0)->withTimeout(1),
      drive_sys.TurnToHeadingCmd(135, 0.8, 0)->withTimeout(1), new DelayCommand(200), goal_grabber_command(false),
      drive_sys.DriveTankCmd(-0.3, -0.3)->withTimeout(1),
        };
	cc.run();
}