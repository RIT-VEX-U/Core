#include "autopathing/auto-blue-safe.h"
#include "robot-config.h"
#include "competition/autonomous.h"

void auto_blue_safe() {
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
		wallstakemech_sys.set_state_command(WallStakeMech::HANDOFF),
		drive_sys.DriveToPointCmd({120, 89.25}, vex::reverse, 1),
		drive_sys.TurnToPointCmd(120, 72, vex::reverse, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({120, 72}, vex::reverse, 0.5),
		clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
		new DelayCommand(50),
		// new TurnToHeadingCommand(drive_sys, turn_pidBigI, 180, 1),
		drive_sys.TurnToHeadingCmd(0, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({124.5, 72}, vex::reverse, 1)->withTimeout(1),
		// wallstakemech_sys.set_state_command(ON_ALLIANCE),
		new DelayCommand(1000),
		intake_sys.IntakeCmd(10),
		intake_sys.ConveyorInCmd(10),
		drive_sys.DriveForwardCmd(6, vex::reverse, 1)->withTimeout(0.3),
		wallstakemech_sys.set_state_command(WallStakeMech::STOW),
		new DelayCommand(1000),
		drive_sys.DriveForwardCmd(16, vex::forward, 0.5)->withTimeout(1),
		drive_sys.DriveForwardCmd(16, vex::reverse, 0.5)->withTimeout(1),
		//knock out blue ring
		drive_sys.TurnToPointCmd(96, 96, vex::forward, 1)->withTimeout(1),
		intake_sys.OuttakeCmd(10),
		drive_sys.DriveToPointCmd({96, 96}, vex::forward, 1),
		intake_sys.IntakeStopCmd(),
		intake_sys.IntakeCmd(10),
		intake_sys.ConveyorStopCmd(),
		//move blue ring out of way
		drive_sys.TurnToPointCmd(96, 120, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({96, 120}, vex::forward, 0.5),
		// drive_sys.DriveForwardCmd(12, vex::reverse, 0.2)->withTimeout(1),
		drive_sys.TurnToHeadingCmd(225, 1)->withTimeout(1),
		intake_sys.OuttakeCmd(10),
		drive_sys.DriveForwardCmd(6, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveForwardCmd(6, vex::reverse, 1)->withTimeout(1),
		new DelayCommand(100),
		intake_sys.IntakeCmd(10),
		intake_sys.ConveyorInCmd(10),
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
		intake_sys.OuttakeCmd(),
		drive_sys.TurnToPointCmd(108,36, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({108,42}, vex::forward, 1)->withTimeout(5),
		drive_sys.TurnDegreesCmd(180, 1)->withTimeout(1),
		clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
		//get to last position
		drive_sys.TurnToPointCmd(120, 96, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({120, 96}, vex::forward, 1)->withTimeout(4),
		drive_sys.TurnToPointCmd(90, 96, vex::forward, 1)->withTimeout(1),
		drive_sys.DriveToPointCmd({90, 96}, vex::forward, 1)->withTimeout(1),
		drive_sys.TurnToHeadingCmd(260, 1)->withTimeout(1),
		// wallstakemech_sys.set_state_command(ON_ALLIANCE),
		// new DelayCommand(100),
		// alliance_score_command(false),




		
        };
	cc.run();

}