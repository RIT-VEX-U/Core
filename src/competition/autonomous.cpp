#include "competition/autonomous.h"
#include "robot-config.h"

/**
 * Main entrypoint for the autonomous period
 */

void skills();

int goal_countera = 0;

void autonomous()
{
	vexDelay(700);

	auto_();
}

AutoCommand *intake_command(double amt = 12.0) {
  return new FunctionCommand([=]() {
    smart_intake.conveyor_start();
    return true;
  });
}

AutoCommand *outtake_command(double amt = 12.0) {
  return new FunctionCommand([=]() {
    smart_intake.intake_out();
    return true;
  });
}

AutoCommand *stop_intake() {
  return new FunctionCommand([=]() {
    // intake(0);
    smart_intake.intake_in();
    return true;
  });
}

AutoCommand *conveyor_intake_command(double amt = 12.0) {
  return new FunctionCommand([=]() {
    // conveyor_intake(amt);
    smart_intake.conveyor_start();
    return true;
  });
}

AutoCommand *conveyor_stop_command() {
  return new FunctionCommand([=]() {
    smart_intake.conveyor_stop();
    return true;
  });
}

AutoCommand *goal_grabber_command(bool value) {
  return new FunctionCommand([=]() {
    goal_grabber_sol.set(value);
    return true;
  });
}

class DebugCommand : public AutoCommand {
public:
	bool run() override {
		drive_sys.stop();
		pose_t pos = odom.get_position();
		printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
		while (true) {
			double f = con.Axis3.position() / 200.0;
			double s = con.Axis1.position() / 200.0;
			drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
			pose_t pos = odom.get_position();
			printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
			vexDelay(100);
		}
		return false;
	}
};

void auto_() {
	CommandController cc {
		// odom.SetPositionCmd({.x = 9.5, .y = 72, .rot = 0}),

		new Async(new FunctionCommand([]() {
			while(true) {
				OdometryBase *odombase = &odom;
                pose_t pos = odombase->get_position();
            	printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
				vexDelay(100);

				if((conveyor.current() > 2) && conveyor.velocity(rpm) < 0.5){
					printf("Conveyor Stalling");
					conveyor_intake(-12);
					vexDelay(500);
					conveyor_intake(12);
				}

                if (goal_sensor.objectDistance(vex::mm) < 25 && goal_countera == 0) {
                goal_grabber_sol.set(true);
                }

                if (goal_countera > 0) {
                    goal_countera--;
                }
			}
			return true;
		})),

        };
	cc.run();
}

void skills() {
	CommandController cc {
		// odom.SetPositionCmd({.x = 9.5, .y = 72, .rot = 0}),

		new Async(new FunctionCommand([]() {
			while(true) {
				OdometryBase *odombase = &odom;
                pose_t pos = odombase->get_position();
            	printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
				vexDelay(100);

				if((conveyor.current() > 2) && conveyor.velocity(rpm) < 0.5){
					printf("Conveyor Stalling");
					conveyor_intake(-12);
					vexDelay(500);
					conveyor_intake(12);
				}

                if (goal_sensor.objectDistance(vex::mm) < 25 && goal_countera == 0) {
                goal_grabber_sol.set(true);
                }

                if (goal_countera > 0) {
                    goal_countera--;
                }
			}
			return true;
		})),


        };
	cc.run();
}