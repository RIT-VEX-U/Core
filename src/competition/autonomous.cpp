#include "competition/autonomous.h"
#include "robot-config.h"

void game_auto_blue();
void game_auto_red();
void skills();

// bool conveyor_started = false;

// bool color_sensor_enabled = true;

void autonomous() {
    // vexDelay(700);

    // game_auto_red();
    skills();
}

void game_auto_red() {
    intake_sys.color_sort_on();

    CommandController cc{
      // Goal Rush
      new Parallel{
        new DriveForwardCommand(drive_sys, drive_motioncontroller, 49, vex::reverse, 1, 0),
        new InOrder{
          new DelayCommand(1380), // goal_grabber_command(true)
          clamper_sys.ClampCmd(ClamperSys::CLAMPED),
        }
      },

      // Reverse a bit with the goal
      drive_sys.DriveForwardCmd(22, vex::forward, 1, 0)->withTimeout(2),
      intake_sys.ConveyorInCmd(),
      intake_sys.IntakeCmd(),

      // First set of rings
      drive_sys.TurnToPointCmd(48, 24, vex::fwd, 0.6, 0)->withTimeout(1), // conveyor_intake_command(),
      drive_sys.DriveToPointCmd({48, 24}, vex::forward, 0.8, 0)->withTimeout(2),
      drive_sys.DriveForwardCmd(8, vex::forward, 0.8, 0)->withTimeout(2),
      drive_sys.DriveToPointCmd({48, 24}, vex::reverse, 0.8, 0)->withTimeout(2),

      // Second set of rings
      drive_sys.TurnToHeadingCmd(180, 0.8, 0)->withTimeout(1),
      drive_sys.DriveToPointCmd({24, 24}, vex::fwd, 0.8, 0)->withTimeout(2),
      drive_sys.DriveForwardCmd(10, vex::forward, 0.8, 0)->withTimeout(2),
      drive_sys.DriveForwardCmd(10, vex::reverse, 0.8, 0)->withTimeout(2),

      // Corner shit
      drive_sys.TurnToHeadingCmd(-137, 1, 0)->withTimeout(1),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.7, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.5, 0)->withTimeout(1.5),
      drive_sys.TurnToHeadingCmd(-137, 1, 0)->withTimeout(1),
      new DelayCommand(1500),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.7, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.5, 0)->withTimeout(1.5),
      drive_sys.TurnToHeadingCmd(-137, 1, 0)->withTimeout(1),
      new DelayCommand(1500),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.7, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.5, 0)->withTimeout(1.5),
      drive_sys.TurnToHeadingCmd(-137, 1, 0)->withTimeout(1),
      new DelayCommand(1500),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.7, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.5, 0)->withTimeout(1.5),

      // Drop goal in corner
      drive_sys.TurnToHeadingCmd(45, 0.8, 0)->withTimeout(1),

      clamper_sys.ClampCmd(ClamperSys::UNCLAMPED),
      intake_sys.IntakeStopCmd(),
      intake_sys.ConveyorStopCmd(),
      drive_sys.DriveTankCmd(-0.4, -0.4)->withTimeout(0.5),
      drive_sys.DriveTankCmd(0.4, 0.4)->withTimeout(0.1),
    };
    cc.run();
}

void game_auto_blue() {
    intake_sys.color_sort_on();

    CommandController cc{
      // Goal Rush
      new Parallel{
        new DriveForwardCommand(drive_sys, drive_motioncontroller, 49, vex::reverse, 1, 0),
        new InOrder{
          new DelayCommand(1380), // goal_grabber_command(true)
          clamper_sys.ClampCmd(ClamperSys::CLAMPED),
        }
      },

      // Reverse a bit with the goal
      drive_sys.DriveForwardCmd(22, vex::forward, 1, 0)->withTimeout(2),
      intake_sys.ConveyorInCmd(),
      intake_sys.IntakeCmd(),

      // First set of rings
      drive_sys.TurnToPointCmd(96, 24, vex::fwd, 0.6, 0)->withTimeout(1), // conveyor_intake_command(),
      drive_sys.DriveToPointCmd({96, 24}, vex::forward, 0.8, 0)->withTimeout(2),
      drive_sys.DriveForwardCmd(8, vex::forward, 0.8, 0)->withTimeout(2),
      drive_sys.DriveToPointCmd({96, 24}, vex::reverse, 0.8, 0)->withTimeout(2),

      // Second set of rings
      drive_sys.TurnToHeadingCmd(0, 0.8, 0)->withTimeout(1),
      drive_sys.DriveToPointCmd({120, 24}, vex::fwd, 0.8, 0)->withTimeout(2),
      drive_sys.DriveForwardCmd(10, vex::forward, 0.8, 0)->withTimeout(2),
      drive_sys.DriveForwardCmd(10, vex::reverse, 0.8, 0)->withTimeout(2),

      // Corner shit
      drive_sys.TurnToHeadingCmd(-43, 1, 0)->withTimeout(1),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.7, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.5, 0)->withTimeout(1.5),
      drive_sys.TurnToHeadingCmd(-43, 1, 0)->withTimeout(1),
      new DelayCommand(1500),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.7, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.5, 0)->withTimeout(1.5),
      drive_sys.TurnToHeadingCmd(-43, 1, 0)->withTimeout(1),
      new DelayCommand(1500),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.7, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.5, 0)->withTimeout(1.5),
      drive_sys.TurnToHeadingCmd(-43, 1, 0)->withTimeout(1),
      new DelayCommand(1500),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.7, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.5, 0)->withTimeout(1.5),

      // Drop goal in corner
      drive_sys.TurnToHeadingCmd(135, 0.8, 0)->withTimeout(1),

      clamper_sys.ClampCmd(ClamperSys::UNCLAMPED),
      intake_sys.IntakeStopCmd(),
      intake_sys.ConveyorStopCmd(),
      drive_sys.DriveTankCmd(-0.4, -0.4)->withTimeout(0.5),
      drive_sys.DriveTankCmd(0.4, 0.4)->withTimeout(0.1),
    };
    cc.run();
}

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

void skills() {
    class DebugCommand : public AutoCommand {
      public:
        bool run() override {
            drive_sys.stop();
            pose_t pos = odom.get_position();
            printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
            while (true) {
                double straight = (double)con.Axis3.position() / 100;
                double turn = (double)con.Axis1.position() / 100;

                // drive_sys.drive_arcade(straight, turn * -1.75, 1, TankDrive::BrakeType::None);

                vexDelay(100);
            }
            return true;
        }
    };

    intake_sys.color_sort_on();
    wallstakemech_sys.set_setpoint(from_degrees(200));
    wallstakemech_sys.hold = true;
    clamper_sys.unclamp();

    vexDelay(1000);
    // clang-format off
    CommandController cc{
      // drop the intake
      // drive_sys.DriveForwardCmd(1, vex::reverse, 1, 0)->withTimeout(0.3), intake_sys.IntakeCmd(),
      intake_sys.IntakeCmd(),
      drive_sys.DriveToPointCmd({48, 46}, vex::forward, 0.5, 0)->withTimeout(2.0),

      // turn and drive to goal
      drive_sys.TurnToHeadingCmd(90)->withTimeout(1.0),
      drive_sys.DriveToPointCmd({49.2, 27.0}, vex::reverse, 0.4, 0)->withTimeout(2.0),
      clamper_sys.ClampCmd(ClamperSys::CLAMPED),
      intake_sys.ConveyorInCmd(),
      new DelayCommand(400),

      drive_sys.TurnToPointCmd(68.8,22,vex::fwd, 1)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({68.8, 22}, vex::fwd, 0.4, 0)->withTimeout(2.0), 


      intake_sys.SetColorSortCmd(false),

      drive_sys.DriveToPointCmd({69.0, 23}, vex::fwd, 0.4, 0)->withTimeout(1.5), 
      new DelayCommand(400),
      wallstakemech_sys.set_setpoint_command(from_degrees(170)),
      drive_sys.TurnToHeadingCmd(-90.0, 1.0)->withTimeout(0.8),


      drive_sys.DriveForwardCmd(8, vex::fwd, 0.4)->withTimeout(2.0),
      get_into_wallstake(),
      drive_sys.DriveForwardCmd(12, vex::fwd, 0.3)->withTimeout(1.5),
      wallstakemech_sys.set_setpoint_command(from_degrees(45)),
      new DelayCommand(1000),
      drive_sys.DriveForwardCmd(8, vex::reverse, 0.3)->withTimeout(1.0),

      drive_sys.TurnToPointCmd(36, 36,vex::fwd, 1)->withTimeout(0.5),
      wallstakemech_sys.set_setpoint_command(from_degrees(200)),
      
      // Stage for center
      drive_sys.TurnToPointCmd(40 ,43.5, vex::fwd, 1)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({40, 43.5}, vex::fwd, 0.4, 0)->withTimeout(1), // conveyor_intake_command(),
      // center
      drive_sys.TurnToPointCmd(58,58,vex::fwd, 1)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({52.9, 58}, vex::fwd, 0.4, 0)->withTimeout(1), // conveyor_intake_command(),

      // center
      drive_sys.TurnToPointCmd(69, 69,vex::fwd, 1)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({71, 71}, vex::fwd, 0.2, 0)->withTimeout(2.0),

      drive_sys.DriveToPointCmd({40,40},vex::reverse,0.5),

      drive_sys.TurnToPointCmd(16.15, 20, vex::fwd, 1)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({16.15, 20}, vex::fwd, 0.2, 0)->withTimeout(2.0), 

      drive_sys.TurnToHeadingCmd(45, 1.0),
      drive_sys.DriveForwardCmd(12, vex::reverse, 0.3),
      clamper_sys.ClampCmd(ClamperSys::UNCLAMPED),
      drive_sys.DriveForwardCmd(12, vex::fwd, 0.6),
      new DelayCommand(1000000),

    };
    cc.run();
    // clang-format on
}