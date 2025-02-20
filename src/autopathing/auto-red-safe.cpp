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
		drive_sys.TurnToHeadingCmd(180, 1)->withTimeout(0.6),
		intake_sys.IntakeCmd(10),
		drive_sys.DriveToPointCmd({11, 72}, vex::reverse, 1)->withTimeout(1),
		wallstakemech_sys.SetSetPointCmd(from_degrees(40)),
		new DelayCommand(500),
		intake_sys.ConveyorInCmd(10),
		drive_sys.DriveForwardCmd(6, vex::reverse, 1)->withTimeout(1),
		wallstakemech_sys.SetSetPointCmd(from_degrees(210)),
		new DelayCommand(300),
		//knock out blue ring
		intake_sys.ConveyorStopCmd(),
		intake_sys.OuttakeCmd(),
		drive_sys.TurnToPointCmd(48, 96, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({48, 96}, vex::forward, 1)->withTimeout(1),
		//get first duo ring
		drive_sys.TurnToPointCmd(48, 120, vex::forward, 1)->withTimeout(1),
		drive_sys.TurnToHeadingCmd(90, 1)->withTimeout(0.01),
		intake_sys.IntakeCmd(12),
		intake_sys.ConveyorInCmd(12),
		drive_sys.DriveForwardCmd(38, vex::forward, 0.6)->withTimeout(1.5),
		drive_sys.TurnToHeadingCmd(90, 1)->withTimeout(0.01),
		drive_sys.DriveForwardCmd(18, vex::reverse, 1)->withTimeout(1),
		// drive_sys.DriveForwardCmd(12, vex::reverse, 0.4)->withTimeout(1),
		//turn to last duo ring
		drive_sys.TurnToPointCmd(24, 120, vex::forward, 1)->withTimeout(0.8),
		drive_sys.TurnToHeadingCmd(180, 1)->withTimeout(0.01),
		drive_sys.DriveToPointCmd({24, 120}, vex::forward, 0.4)->withTimeout(1),
		// //goes to corner
		drive_sys.TurnToPointCmd(0, 144, vex::forward, 1)->withTimeout(0.6),
		drive_sys.TurnToHeadingCmd(135, 1)->withTimeout(0.01),
		drive_sys.DriveForwardCmd(34, vex::forward, 0.5)->withTimeout(1.5),
		drive_sys.DriveForwardCmd(4, vex::reverse, 1)->withTimeout(0.8),
		drive_sys.TurnToPointCmd(0, 144, vex::forward, 1)->withTimeout(0.6),
		drive_sys.TurnToHeadingCmd(135, 1)->withTimeout(0.01),
		drive_sys.DriveForwardCmd(6, vex::forward, 0.5)->withTimeout(0.8),
		drive_sys.DriveForwardCmd(4, vex::reverse, 1)->withTimeout(0.8),
		drive_sys.TurnToPointCmd(0, 144, vex::forward, 1)->withTimeout(0.6),
		drive_sys.TurnToHeadingCmd(135, 1)->withTimeout(0.01),
		drive_sys.DriveForwardCmd(6, vex::forward, 0.5)->withTimeout(0.8),
		drive_sys.DriveForwardCmd(4, vex::reverse, 1)->withTimeout(0.8),
		drive_sys.TurnToPointCmd(0, 144, vex::forward, 1)->withTimeout(0.6),
		drive_sys.TurnToHeadingCmd(135, 1)->withTimeout(0.01),
		drive_sys.DriveForwardCmd(6, vex::forward, 0.5)->withTimeout(0.8),
		drive_sys.DriveForwardCmd(4, vex::reverse, 1)->withTimeout(0.8),
		new DelayCommand(500),
		//drop off goal
		intake_sys.OuttakeCmd(),
		drive_sys.TurnToPointCmd(36,36, vex::forward, 1)->withTimeout(1),
		(new Parallel{
			drive_sys.DriveToPointCmd({36, 36}, vex::reverse, 1)->withTimeout(1.5),
			new InOrder{
				new DelayCommand(1200),
				clamper_sys.ClampCmd(ClamperSys::UNCLAMPED),
			},
		})->withTimeout(1.5),
		new FunctionCommand([]() {
			printf("Parallel Finished");
			return true;
		}),
		//get to last position
		drive_sys.TurnToPointCmd(59, 59, vex::reverse, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({59, 59}, vex::reverse, 1)->withTimeout(1),
		wallstakemech_sys.SetSetPointCmd(from_degrees(169)),



		
        };
	cc.run();

}