#include "autopathing/auto-red-safe.h"
#include "robot-config.h"
#include "competition/autonomous.h"

void auto_red_safe() {
	mcglight_board.set(true);
	CommandController cc {
		(new Async((new FunctionCommand([]() {
			while(true) {
				OdometryBase *odombase = &odom;
                pose_t pos = odombase->get_position();
            	printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
				vexDelay(100);
			}
			return true;
		})))),
		//set up alliance stake
		wallstakemech_sys.SetSetPointCmd(from_degrees(140)),
		drive_sys.DriveToPointCmd({24, 88.75}, vex::reverse, 1)->withTimeout(1),
		drive_sys.TurnToPointCmd(24, 72, vex::reverse, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({24, 72}, vex::reverse, 0.5)->withTimeout(1),
		clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
		new DelayCommand(50),
		drive_sys.TurnToHeadingCmd(180, 1)->withTimeout(1),
		intake_sys.IntakeCmd(10),
		drive_sys.DriveToPointCmd({11, 72}, vex::reverse, 1)->withTimeout(1),
		wallstakemech_sys.SetSetPointCmd(from_degrees(40)),
		new DelayCommand(1000),
		intake_sys.ConveyorInCmd(10),
		drive_sys.DriveForwardCmd(6, vex::reverse, 1)->withTimeout(0.3),
		new DelayCommand(500),
		wallstakemech_sys.SetStateCmd(WallStakeMech::STOW),
		new DelayCommand(1000),
		//knock out blue ring
		intake_sys.ConveyorStopCmd(),
		intake_sys.OuttakeCmd(),
		drive_sys.TurnToPointCmd(48, 96, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({48, 96}, vex::forward, 1)->withTimeout(1),

		drive_sys.TurnToPointCmd(48, 120, vex::forward, 1)->withTimeout(1),
		// drive_sys.TurnToHeadingCmd(90, 1)->withTimeout(1),
		intake_sys.IntakeCmd(10),
		intake_sys.ConveyorInCmd(10),
		// drive_sys.DriveToPointCmd({24, 144}, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveForwardCmd(48, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveForwardCmd(48, vex::reverse, 1)->withTimeout(1),
		//turn to last duo ring
		drive_sys.TurnToPointCmd(24, 120, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({24, 120}, vex::forward, 0.4)->withTimeout(2),
		// //goes to corner
		drive_sys.TurnToPointCmd(0, 144, vex::forward, 1)->withTimeout(1),
		intake_sys.IntakeCmd(),
		drive_sys.DriveForwardCmd(35, vex::forward, 1)->withTimeout(1.5),
		drive_sys.DriveForwardCmd(30, vex::reverse, 1)->withTimeout(1),
		drive_sys.DriveForwardCmd(35, vex::forward, 1)->withTimeout(1.5),
		drive_sys.DriveForwardCmd(30, vex::reverse, 1)->withTimeout(1),
		drive_sys.DriveForwardCmd(35, vex::forward, 1)->withTimeout(1.5),
		drive_sys.DriveForwardCmd(30, vex::reverse, 1)->withTimeout(1),
		drive_sys.DriveForwardCmd(35, vex::forward, 1)->withTimeout(1.5),
		drive_sys.DriveForwardCmd(30, vex::reverse, 1)->withTimeout(1),
		new DelayCommand(1000),
		//drop off goal
		intake_sys.OuttakeCmd(),
		drive_sys.TurnToPointCmd(36,36, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({36,42}, vex::forward, 1)->withTimeout(2),
		clamper_sys.ClampCmd(ClamperSys::UNCLAMPED),
		//get to last position
		drive_sys.TurnToPointCmd(60, 60, vex::reverse, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({60, 60}, vex::reverse, 1)



		
        };
	cc.run();

}