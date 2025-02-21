#include "autopathing/basic-skills.h"
#include "competition/autonomous.h"
#include "robot-config.h"

AutoCommand *get_into_wallstake() {
    return new InOrder({
      intake_sys.ConveyorInCmd(), new DelayCommand(100), intake_sys.ConveyorStopCmd(), new DelayCommand(100),
      intake_sys.ConveyorInCmd(), new DelayCommand(100), intake_sys.ConveyorStopCmd(), new DelayCommand(100),
      intake_sys.ConveyorInCmd(), new DelayCommand(100), intake_sys.ConveyorStopCmd(), new DelayCommand(100),
      intake_sys.ConveyorInCmd(), new DelayCommand(100), intake_sys.ConveyorStopCmd(), new DelayCommand(100),
      intake_sys.ConveyorInCmd(), new DelayCommand(100), intake_sys.ConveyorStopCmd(), new DelayCommand(100),
      intake_sys.ConveyorInCmd(), new DelayCommand(100), intake_sys.ConveyorStopCmd(),
    });
}

void skills_basic() {
    mcglight_board.set(true);
    // clang-format off
	CommandController cc {
		new Async(new FunctionCommand([]() {
			
		OdometryBase *odombase = &odom;
        pose_t pos = odombase->get_position();
        printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
		vexDelay(100);
        return false;
		})),


    //   new DelayCommand(5000),
      wallstakemech_sys.SetSetPointCmd(from_degrees(200)),
	  intake_sys.IntakeCmd(),

      // First Ring
      drive_sys.DriveToPointCmd({ 48.41,  96}, vex::fwd, .5)->withTimeout(2.0),

      // Goal
      drive_sys.TurnToHeadingCmd(-90.0, 0.4)->withTimeout(1.0),
      drive_sys.DriveToPointCmd({48, 120}, vex::reverse, 0.4)->withTimeout(1.75),
      clamper_sys.ClampCmd(ClamperSys::CLAMPED),
      new DelayCommand(200),
      intake_sys.ConveyorInCmd(),

      // midfield
      drive_sys.TurnToPointCmd(72, 120, vex::fwd, 0.5)->withTimeout(1.4),
      drive_sys.DriveToPointCmd({72, 120}, vex::fwd, 0.7)->withTimeout(1.8),
      intake_sys.ConveyorInCmd(12.0),
      new DelayCommand(400),
      wallstakemech_sys.SetSetPointCmd(from_degrees(170)),

      drive_sys.TurnToPointCmd(72, 144, vex::fwd, 0.8)->withTimeout(2),
      drive_sys.DriveToPointCmd({72, 136}, vex::fwd, 0.3)->withTimeout(1),
      drive_sys.DriveForwardCmd(6, vex::reverse, 0.5)->withTimeout(0.8),
      drive_sys.DriveForwardCmd(8, vex::fwd, 0.5)->withTimeout(0.8),

      get_into_wallstake(),        
      wallstakemech_sys.SetSetPointCmd(from_degrees(45)),
      new DelayCommand(800),
      drive_sys.DriveForwardCmd(2, vex::fwd, 0.9)->withTimeout(1.2),

      drive_sys.DriveForwardCmd(6, vex::reverse, 0.4)->withTimeout(1.2),
      wallstakemech_sys.SetSetPointCmd(from_degrees(200)),
      intake_sys.ConveyorInCmd(),

      // corner of tile
      drive_sys.TurnToPointCmd(22, 117, vex::fwd, 0.5)->withTimeout(2.0),
      drive_sys.DriveToPointCmd({22, 117}, vex::fwd, 0.5)->withTimeout(2.0),

      drive_sys.TurnToPointCmd(7, 137,vex::fwd, 0.9)->withTimeout(1.5),
      drive_sys.DriveToPointCmd({7,137}, vex::fwd, 0.3)->withTimeout(1.5),
      drive_sys.DriveToPointCmd({16,128}, vex::reverse, 0.8)->withTimeout(1.0),


      // Go to under tower
      drive_sys.TurnToPointCmd(75, 70, vex::fwd, 0.5)->withTimeout(2.0),
      drive_sys.DriveToPointCmd({50, 91}, vex::fwd, 0.85)->withTimeout(1.0),
      drive_sys.DriveToPointCmd({75, 70}, vex::fwd, 0.4)->withTimeout(2.0),
      drive_sys.TurnDegreesCmd(-15, 0.4)->withTimeout(0.6),
      drive_sys.TurnDegreesCmd(15, 0.4)->withTimeout(0.6),
      drive_sys.DriveForwardCmd(3, vex::fwd, 0.4)->withTimeout(0.4),

     // Pull back out from under tower
     drive_sys.DriveForwardCmd(70, vex::reverse, 0.4)->withTimeout(4),
    //   drive_sys.DriveToPointCmd({24, 120}, vex::reverse, 0.4)->withTimeout(4.0),

    // drive_sys.TurnToHeadingCmd(-45)->withTimeout(0.6),
    drive_sys.TurnToPointCmd(7, 137, vex::reverse, 0.5)->withTimeout(0.6),
    drive_sys.DriveToPointCmd({7,137}, vex::reverse, 0.3)->withTimeout(2.5),
    intake_sys.ConveyorOutCmd(),
    clamper_sys.ClampCmd(ClamperSys::UNCLAMPED),
    new DelayCommand(300),
    intake_sys.ConveyorStopCmd(),
    drive_sys.DriveForwardCmd(24, vex::fwd, 1)->withTimeout(1),
    drive_sys.TurnToPointCmd(96, 120, vex::fwd, 1)->withTimeout(1),
    drive_sys.DriveToPointCmd({96,120}, vex::reverse, 1)->withTimeout(3),
    drive_sys.TurnToPointCmd(96, 96, vex::reverse, 1)->withTimeout(1),
    drive_sys.DriveToPointCmd({96,96}, vex::reverse, 1)->withTimeout(1),
    clamper_sys.ClampCmd(ClamperSys::CLAMPED),
    intake_sys.ConveyorInCmd(12),
    drive_sys.TurnToPointCmd(120, 96, vex::fwd, 1)->withTimeout(1),
    drive_sys.DriveToPointCmd({120,96}, vex::reverse, 1)->withTimeout(1),
    drive_sys.TurnToPointCmd(120, 120, vex::fwd, 1)->withTimeout(1),
    drive_sys.DriveToPointCmd({120,120}, vex::reverse, 1)->withTimeout(1),

    drive_sys.TurnToPointCmd(144, 144,vex::fwd, 0.9)->withTimeout(1.5),
    drive_sys.DriveToPointCmd({137,137}, vex::fwd, 0.3)->withTimeout(1.5),
    drive_sys.DriveForwardCmd(24, vex::reverse, 1)->withTimeout(1),
    drive_sys.TurnToPointCmd(144, 144,vex::reverse, 0.9)->withTimeout(1.5),
    drive_sys.DriveForwardCmd(30, vex::reverse, 1)->withTimeout(2),
    clamper_sys.ClampCmd(ClamperSys::UNCLAMPED),
    new DelayCommand(300),
    intake_sys.ConveyorStopCmd(),
    drive_sys.DriveForwardCmd(24, vex::fwd, 0.6)->withTimeout(1),

    new DelayCommand(100000),
    };
    // clang-format on
    cc.run();
}