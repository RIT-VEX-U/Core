#include "competition/autonomous.h"
#include "robot-config.h"
/**
 * Main entrypoint for the autonomous period
 */

void skills();

void autonomous()
{
	switch(matchpath){
		case BLUE_SAFE_AUTO:
			auto_blue_safe();
			break;
		case RED_SAFE_AUTO:
			auto_red_safe();
			break;
		case BASIC_SKILLS:
			skills_basic();

	}
}

AutoCommand *intake_command(double amt) {
    return new FunctionCommand([=]() {
        intake(amt);
        return true;
    });
}

AutoCommand *outtake_command(double amt) {
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

AutoCommand *conveyor_intake_command(double amt) {
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

AutoCommand *alliance_score_command(bool hold) {
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

//functions to use across all paths

void colorSort(){
	if (blue_alliance()) {
            if (color_sensor.hue() > 0 && color_sensor.hue() < 30 && color_sensor_counter == 0) {
                color_sensor_counter = 30;
				conveyor_intake(12);
            }
        } else {
            if (color_sensor.hue() > 160 && color_sensor.hue() < 240 && color_sensor_counter == 0) {
                color_sensor_counter = 30;
				conveyor_intake(12);
            }
        }

        if (color_sensor_counter == 25) {
            color_sensor_counter--;
            conveyor.stop();
        }

        if (color_sensor_counter > 0) {
            color_sensor_counter--;
            
        }

		if (conveyor_started && color_sensor_counter == 0) {
            conveyor_intake();
        }
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
			// double left_enc_start_pos = left_enc.position(vex::rotationUnits::rev);
			drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
			pose_t pos = odom.get_position();
			printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
			// printf("ENC LEFT REV: %.2f, ENC RIGHT POS: %.2f, ENC BACK POS: %.2f\n", left_enc.position(vex::rotationUnits::deg), right_enc.position(vex::rotationUnits::deg), front_enc.position(vex::rotationUnits::deg));
			// if (left_enc.position(vex::rotationUnits::rev) >= 1.0) {
			//     break;
			// }
			vexDelay(100);
		}
		return false;
	}
};

