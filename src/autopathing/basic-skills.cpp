#include "competition/autonomous.h"
#include "robot-config.h"
#include "autopathing/basic-skills.h"

void skills_basic() {
	mcglight_board.set(true);
	CommandController cc {
		// odom.SetPositionCmd({.x = 9.5, .y = 72, .rot = 0}),

		new Async(new FunctionCommand([]() {
			
		OdometryBase *odombase = &odom;
        pose_t pos = odombase->get_position();
        printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
		vexDelay(100);
        return false;
		})),


      // goal 1 ring 1
	  intake_sys.IntakeCmd(),
      drive_sys.DriveToPointCmd({48, 48}, vex::forward, 0.5, 0)->withTimeout(2),

      // goal 1 grab
      drive_sys.TurnToHeadingCmd(90, 0.5, 0)->withTimeout(0.8),
      drive_sys.DriveToPointCmd({48, 24}, vex::reverse, 0.5, 0)->withTimeout(1), 
	  intake_sys.ConveyorInCmd(),
	  clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
	  

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
      new DelayCommand(1000), clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),

      // exit corner
      drive_sys.DriveToPointCmd({24, 24}, vex::forward, 0.5, 0)->withTimeout(2), intake_sys.ConveyorStopCmd(),

      // goal 2 ring 1
      drive_sys.TurnToPointCmd(96, 24, vex::fwd, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({96, 24}, vex::fwd, 0.8, 0)->withTimeout(3),

      // goal 2 grab
      drive_sys.TurnToHeadingCmd(-90, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({96, 48}, vex::reverse, 0.4, 0)->withTimeout(1), intake_sys.ConveyorInCmd(),
	  clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED),
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
      drive_sys.TurnToHeadingCmd(135, 0.8, 0)->withTimeout(1), new DelayCommand(200), clamper_sys.ClampCmd(ClamperSys::ClamperState::UNCLAMPED),
      drive_sys.DriveTankCmd(-0.3, -0.3)->withTimeout(1),
        };
	cc.run();
}