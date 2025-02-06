#include "autopathing/auto-blue-safe.h"
#include "robot-config.h"
#include "competition/autonomous.h"

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

void auto_blue_safe() {
	mcglight_board.set(true);
	CommandController cc {
		// odom.SetPositionCmd({.x = 9.5, .y = 72, .rot = 0}),

		(new Async((new FunctionCommand([]() {
			while(true) {
				OdometryBase *odombase = &odom;
                pose_t pos = odombase->get_position();
            	printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
				vexDelay(100);
				colorSort();
			}
			return true;
		})))),
		// drive_sys.DriveToPointCmd({72, 120}, vex::directionType::fwd, 1, 0),
		// drive_sys.DriveForwardCmd(34, vex::reverse, 1, 1),
		// drive_sys.DriveForwardCmd(15, vex::reverse, 0.3 , 0),
		// goal_grabber_command(true),
		// intake_command(10),
		// conveyor_intake_command(10),
		// new DelayCommand(100),
		// drive_sys.TurnToPointCmd(48, 120,vex::forward, 1, 0)->withTimeout(0.5),
		// drive_sys.DriveToPointCmd({48, 120}, vex::forward, 0.6 , 0.6),
		// drive_sys.TurnToPointCmd(24, 120,vex::forward, 0.6, 0)->withTimeout(0.5),
		// drive_sys.DriveToPointCmd({24, 120}, vex::forward, 0.3 , 0.3),
		// drive_sys.TurnToPointCmd(0, 144, vex::forward, 1)->withTimeout(0.5),
		// drive_sys.DriveToPointCmd({12, 132}, vex::forward, 1)->withTimeout(1),
		// drive_sys.DriveForwardCmd(12, vex::reverse, 0.3)->withTimeout(1),
		// drive_sys.DriveForwardCmd(14, vex::forward, 0.3)->withTimeout(1.5),
		// drive_sys.DriveForwardCmd(12, vex::reverse, 0.3)->withTimeout(1),
		// drive_sys.DriveForwardCmd(14, vex::forward, 0.3)->withTimeout(1.5),
		// drive_sys.DriveForwardCmd(12, vex::reverse, 0.3)->withTimeout(1),
		// drive_sys.DriveForwardCmd(14, vex::forward, 0.3)->withTimeout(1.5),
		// drive_sys.DriveForwardCmd(12, vex::reverse, 0.3)->withTimeout(1),
		// drive_sys.DriveForwardCmd(14, vex::forward, 0.3)->withTimeout(1.5),
		// goal_grabber_command(true),
		// new DelayCommand(100),
		// drive_sys.TurnDegreesCmd(180),
		//set up alliance stake
		handoff_command(),
		drive_sys.DriveToPointCmd({120, 89.25}, vex::reverse, 1),
		drive_sys.TurnToPointCmd(120, 72, vex::reverse, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({120, 72}, vex::reverse, 0.5),
		goal_grabber_command(true),
		new DelayCommand(50),
		// new TurnToHeadingCommand(drive_sys, turn_pidBigI, 180, 1),
		drive_sys.TurnToHeadingCmd(0, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({124.5, 72}, vex::reverse, 1)->withTimeout(1),
		alliance_score_command(),
		new DelayCommand(1000),
		intake_command(10),
		conveyor_intake_command(10),
		drive_sys.DriveForwardCmd(6, vex::reverse, 1)->withTimeout(0.3),
		stow_command(),
		new DelayCommand(1000),
		drive_sys.DriveForwardCmd(16, vex::forward, 0.5)->withTimeout(1),
		drive_sys.DriveForwardCmd(16, vex::reverse, 0.5)->withTimeout(1),
		//knock out blue ring
		drive_sys.TurnToPointCmd(96, 96, vex::forward, 1)->withTimeout(1),
		outtake_command(10),
		drive_sys.DriveToPointCmd({96, 96}, vex::forward, 1),
		stop_intake(),
		intake_command(10),
		conveyor_stop_command(),
		//move blue ring out of way
		drive_sys.TurnToPointCmd(96, 120, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({96, 120}, vex::forward, 0.5),
		// drive_sys.DriveForwardCmd(12, vex::reverse, 0.2)->withTimeout(1),
		drive_sys.TurnToHeadingCmd(225, 1)->withTimeout(1),
		outtake_command(10),
		drive_sys.DriveForwardCmd(6, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveForwardCmd(6, vex::reverse, 1)->withTimeout(1),
		new DelayCommand(100),
		intake_command(10),
		conveyor_intake_command(10),
		drive_sys.TurnToHeadingCmd(90, 1)->withTimeout(1),
		drive_sys.DriveForwardCmd(10, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveForwardCmd(10, vex::reverse, 1)->withTimeout(1),
		new DelayCommand(100),
		//turn to last duo ring and get rid of blue ring
		// conveyor_stop_command(),
		drive_sys.TurnToPointCmd(120, 120, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({120, 120}, vex::forward, 0.2)->withTimeout(1),
		// outtake_command(10),
		// drive_sys.TurnToHeadingCmd(315, 1)->withTimeout(1),
		// drive_sys.DriveForwardCmd(6, vex::forward, 1)->withTimeout(1),
		// drive_sys.DriveForwardCmd(6, vex::reverse, 1)->withTimeout(1),
		// conveyor_intake_command(10),
		// intake_command(10),
		//goes to corner
		drive_sys.TurnToPointCmd(144, 144, vex::forward, 1)->withTimeout(1),
		// drive_sys.TurnToHeadingCmd(45)->withTimeout(1),
		drive_sys.DriveTankCmd(0.2, 0.2)->withTimeout(2),
		drive_sys.DriveTankCmd(-0.4, -0.4)->withTimeout(0.6),
		new DelayCommand(1000),
		//drop off goal
		outtake_command(),
		drive_sys.TurnToPointCmd(108,36, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({108,42}, vex::forward, 1)->withTimeout(5),
		drive_sys.TurnDegreesCmd(180, 1)->withTimeout(1),
		goal_grabber_command(false),
		//get to last position
		drive_sys.TurnToPointCmd(120, 96, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({120, 96}, vex::forward, 1)->withTimeout(4),
		drive_sys.TurnToPointCmd(90, 96, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({90, 96}, vex::forward, 1)->withTimeout(1),
		drive_sys.TurnToHeadingCmd(260, 1)->withTimeout(1),
		alliance_score_command(),
		// new DelayCommand(100),
		// alliance_score_command(false),




		
        };
	cc.run();

}