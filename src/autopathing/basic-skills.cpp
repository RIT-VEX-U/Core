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
      wallstakemech_sys.SetSetPointCmd(from_degrees(200)),
	  intake_sys.IntakeCmd(),
      drive_sys.DriveToPointCmd({48, 96}, vex::forward, 0.5, 0)->withTimeout(2),

      // goal 1 grab
      drive_sys.TurnToHeadingCmd(270, 0.5, 0)->withTimeout(0.8),
      drive_sys.DriveToPointCmd({48, 120}, vex::reverse, 0.5, 0)->withTimeout(1), 
	  intake_sys.ConveyorInCmd(),
	  clamper_sys.ClampCmd(ClamperSys::ClamperState::CLAMPED), new DelayCommand(350),
	  

      // goal 1 ring 2
      drive_sys.TurnToHeadingCmd(0, 0.5, 0)->withTimeout(0.8),
      drive_sys.DriveToPointCmd({72, 120}, vex::forward, 0.5, 0)->withTimeout(1), new DelayCommand(350),

    // goal 1 ring 3 (center)
    drive_sys.DriveToPointCmd({48, 96}, vex::forward, 0.5, 0)->withTimeout(1),
    drive_sys.DriveToPointCmd({72, 72}, vex::forward, 0.5, 0)->withTimeout(1),

        };
	cc.run();
}