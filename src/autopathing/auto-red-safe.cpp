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
		drive_sys.DriveToPointCmd({24, 89.25}, vex::reverse, 1)->withTimeout(1),
		drive_sys.TurnToPointCmd(24, 72, vex::reverse, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({24, 72}, vex::reverse, 0.5)->withTimeout(1),
		clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
		new DelayCommand(50),
		// new TurnToHeadingCommand(drive_sys, turn_pidBigI, 180, 1),
		drive_sys.TurnToHeadingCmd(180, 1)->withTimeout(1),
		intake_sys.IntakeCmd(10),
		drive_sys.DriveToPointCmd({11, 72}, vex::reverse, 1)->withTimeout(1),
		intake_sys.ConveyorInCmd(10),
		new DelayCommand(200),
		intake_sys.ConveyorStopCmd(),
		wallstakemech_sys.SetSetPointCmd(from_degrees(45)),
		new DelayCommand(1000),
		drive_sys.DriveForwardCmd(6, vex::reverse, 1)->withTimeout(0.3),
		wallstakemech_sys.SetStateCmd(WallStakeMech::STOW),
		new DelayCommand(500),
		intake_sys.ConveyorInCmd(10),
		new DelayCommand(1000),
		// drive_sys.DriveForwardCmd(16, vex::forward, 0.5)->withTimeout(1),
		// drive_sys.DriveForwardCmd(16, vex::reverse, 0.5)->withTimeout(1),
		//knock out blue ring
		intake_sys.ConveyorStopCmd(),
		drive_sys.TurnToPointCmd(48, 96, vex::forward, 1)->withTimeout(1),
		intake_sys.OuttakeCmd(10),
		drive_sys.DriveToPointCmd({48, 96}, vex::forward, 1)->withTimeout(1),
		intake_sys.IntakeStopCmd(),
		//move blue ring out of way
		drive_sys.TurnToPointCmd(48, 120, vex::forward, 1)->withTimeout(1),
		intake_sys.IntakeCmd(10),
		drive_sys.DriveToPointCmd({48, 120}, vex::forward, 0.5)->withTimeout(1),
		// drive_sys.DriveForwardCmd(12, vex::reverse, 0.2)->withTimeout(1),
		drive_sys.TurnToHeadingCmd(-45, 1)->withTimeout(0.5),
		intake_sys.OuttakeCmd(10),
		drive_sys.DriveForwardCmd(6, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveForwardCmd(6, vex::reverse, 1)->withTimeout(1),
		new DelayCommand(100),
		intake_sys.IntakeCmd(10),
		intake_sys.ConveyorInCmd(10),
		drive_sys.TurnToHeadingCmd(90, 1)->withTimeout(0.5),
		drive_sys.DriveForwardCmd(10, vex::forward, 0.6)->withTimeout(1),
		drive_sys.DriveForwardCmd(10, vex::reverse, 0.6)->withTimeout(1),
		new DelayCommand(100),
		//turn to last duo ring
		intake_sys.IntakeCmd(),
		intake_sys.ConveyorInCmd(),
		drive_sys.TurnToPointCmd(24, 120, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({24, 120}, vex::forward, 0.6)->withTimeout(1),
		// //goes to corner
		// drive_sys.TurnToPointCmd(144, 144, vex::forward, 1)->withTimeout(1),
		// drive_sys.TurnToHeadingCmd(45)->withTimeout(1),
		// drive_sys.DriveTankCmd(0.4, 0.4)->withTimeout(1.5),
		// drive_sys.DriveTankCmd(-0.4, -0.4)->withTimeout(1),
		// drive_sys.DriveTankCmd(0.4, 0.4)->withTimeout(11.5),
		// drive_sys.DriveTankCmd(-0.4, -0.4)->withTimeout(1),
		// drive_sys.DriveTankCmd(0.4, 0.4)->withTimeout(1.5),
		// drive_sys.DriveTankCmd(-0.4, -0.4)->withTimeout(1),
		new DelayCommand(1000),
		// //drop off goal
		intake_sys.OuttakeCmd(),
		drive_sys.TurnToPointCmd(36,36, vex::forward, 1)->withTimeout(0.5),
		drive_sys.DriveToPointCmd({36,42}, vex::forward, 1)->withTimeout(2),
		drive_sys.TurnDegreesCmd(180, 1)->withTimeout(1),
		clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
		//get to last position
		drive_sys.TurnToPointCmd(24, 96, vex::forward, 1)->withTimeout(0.5),
		drive_sys.DriveToPointCmd({24, 96}, vex::forward, 1)->withTimeout(2),
		drive_sys.TurnToPointCmd(54, 100, vex::forward, 1)->withTimeout(0.5),
		drive_sys.DriveToPointCmd({54, 100}, vex::forward, 1)->withTimeout(1),
		drive_sys.TurnToHeadingCmd(-80, 1)->withTimeout(0.5),
		wallstakemech_sys.SetSetPointCmd(from_degrees(50))
		// new DelayCommand(100),
		// alliance_score_command(false),




		
        };
	cc.run();

}