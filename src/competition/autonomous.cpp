#include "competition/autonomous.h"
#include "robot-config.h"
#include "auton-pathing.h"
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