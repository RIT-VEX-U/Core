#include "competition/autonomous.h"
#include "robot-config.h"

void game_auto_blue();
void game_auto_red();
void skills();

int goal_counter = 0;
int color_sensor_counter = 0;

bool conveyor_started = false;

bool blue_alliance = true;

bool color_sensor_enabled = true;

void autonomous()
{
	// vexDelay(700);

	game_auto_blue();
}



AutoCommand *intake_command(double amt = 12.0) {
    return new FunctionCommand([=]() {
        intake(amt);
        return true;
    });
}

AutoCommand *outtake_command(double amt = 12.0) {
    return new FunctionCommand([=]() {
        intake(-amt);
        return true;
    });
}

AutoCommand *stop_intake_command() {
    return new FunctionCommand([=]() {
        intake_motor.stop();
        return true;
    });
}

AutoCommand *conveyor_intake_command(double amt = 12.0) {
    return new FunctionCommand([=]() {
        conveyor_intake(amt);
        conveyor_started = true;
        return true;
    });
}

AutoCommand *conveyor_outtake_command(double amt = 12.0) {
    return new FunctionCommand([=]() {
        conveyor_intake(-amt);
        return true;
    });
}

AutoCommand *stop_conveyor_command() {
    return new FunctionCommand([=]() {
        conveyor.stop();
        conveyor_started = false;
        return true;
    });
}

AutoCommand *goal_grabber_command(bool value) {
    return new FunctionCommand([=]() {
        goal_grabber_sol.set(value);
        goal_counter = 50;
        return true;
    });
}

void game_auto_red() {
    CommandController cc{
      new Async(new FunctionCommand([]() {
        while (true) {
          OdometryBase *odombase = &odom;
          pose_t pos = odombase->get_position();
          // printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
          vexDelay(20);

          if (goal_sensor.objectDistance(vex::mm) < 40 && goal_counter == 0) {
            goal_grabber_sol.set(true);
          }

          if (goal_counter > 0) {
            goal_counter--;
          }

          if (blue_alliance) {
            if (color_sensor.hue() > 0 && color_sensor.hue() < 30 && color_sensor_counter == 0) {
              color_sensor_counter = 30;
            }
          } else {
            if (color_sensor.hue() > 100 && color_sensor.hue() < 220 && color_sensor_counter == 0) {
              color_sensor_counter = 30;
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
        return true;
      })),

      // Goal Rush
      new Parallel{
        drive_sys.DriveForwardCmd(49, vex::reverse, 1, 0),
        new Async{new InOrder{new DelayCommand(970), goal_grabber_command(true)}}
      },

      // Reverse a bit with the goal
      drive_sys.DriveForwardCmd(22, vex::forward, 1, 0)->withTimeout(2),

      // First set of rings
      drive_sys.TurnToPointCmd(48, 24, vex::forward, 0.8, 0)->withTimeout(0.5), 
      conveyor_intake_command(),
      drive_sys.DriveToPointCmd({48, 24}, vex::forward, 1, 0)->withTimeout(2),
      drive_sys.DriveForwardCmd(8, vex::forward, 0.8, 0)->withTimeout(2),
      drive_sys.DriveForwardCmd(8, vex::reverse, 0.8, 0)->withTimeout(2),

      // Second set of rings
      drive_sys.TurnToHeadingCmd(180, 0.8, 0)->withTimeout(1),
      drive_sys.DriveToPointCmd({24, 24}, vex::fwd, 1, 0)->withTimeout(2),
      drive_sys.DriveForwardCmd(10, vex::forward, 0.8, 0)->withTimeout(2),
      drive_sys.DriveForwardCmd(10, vex::reverse, 0.8, 0)->withTimeout(2),

      // Corner shit
      drive_sys.TurnToHeadingCmd(-208, 0.8, 0)->withTimeout(1),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.4, 0)->withTimeout(1.5),

      // Drop goal in corner
      drive_sys.TurnToHeadingCmd(45, 0.8, 0)->withTimeout(0.5),
      goal_grabber_command(false),
      drive_sys.DriveTankCmd(-0.3, -0.3)->withTimeout(0.5),

      // Align for goal handoff
      drive_sys.DriveForwardCmd(8, vex::forward, 0.5, 0)->withTimeout(1),
      drive_sys.TurnToHeadingCmd(-45, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveForwardCmd(6, vex::forward, 0.5, 0),

      // new DebugCommand(),
    };
    cc.run();
}

void game_auto_blue() {
    CommandController cc{
      new Async(new FunctionCommand([]() {
        while (true) {
          OdometryBase *odombase = &odom;
          pose_t pos = odombase->get_position();
          // printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
          vexDelay(20);

          if (goal_sensor.objectDistance(vex::mm) < 40 && goal_counter == 0) {
            goal_grabber_sol.set(true);
          }

          if (goal_counter > 0) {
            goal_counter--;
          }

          if (blue_alliance) {
            if (color_sensor.hue() > 0 && color_sensor.hue() < 30 && color_sensor_counter == 0) {
              color_sensor_counter = 30;
            }
          } else {
            if (color_sensor.hue() > 100 && color_sensor.hue() < 220 && color_sensor_counter == 0) {
              color_sensor_counter = 30;
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
        return true;
      })),

      // Goal Rush
      new Parallel{
        new DriveForwardCommand(drive_sys, drive_motioncontroller, 49, vex::reverse, 1, 0),
        new InOrder{new DelayCommand(1380), goal_grabber_command(true)}
      },

      // Reverse a bit with the goal
      new DriveForwardCommand(drive_sys, drive_motioncontroller, 22, vex::forward, 1, 0),

      // First set of rings
      drive_sys.TurnToPointCmd(96, 24, vex::forward, 1, 0), conveyor_intake_command(),
      new DriveToPointCommand(drive_sys, drive_motioncontroller, {96, 24}, vex::forward, 1, 0),
      drive_sys.DriveForwardCmd(8, vex::forward, 0.8, 0)->withTimeout(2),
      drive_sys.DriveForwardCmd(8, vex::reverse, 0.8, 0)->withTimeout(2),

      // Second set of rings
      drive_sys.TurnToHeadingCmd(0, 1, 0)->withTimeout(1),
      drive_sys.DriveToPointCmd({120, 24}, vex::fwd, 1, 0)->withTimeout(2),
      drive_sys.DriveForwardCmd(10, vex::forward, 0.8, 0)->withTimeout(2),
      drive_sys.DriveForwardCmd(10, vex::reverse, 0.8, 0)->withTimeout(2),

      // Corner shit
      drive_sys.TurnToHeadingCmd(-43, 1, 0)->withTimeout(1),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(16, vex::forward, 0.4, 0)->withTimeout(1.5),
      drive_sys.DriveForwardCmd(13, vex::reverse, 0.4, 0)->withTimeout(1.5),

      // Drop goal in corner
      drive_sys.TurnToHeadingCmd(135, 0.8, 0)->withTimeout(0.5),
      goal_grabber_command(false),
      drive_sys.DriveTankCmd(-0.3, -0.3)->withTimeout(0.5),

      // Align for goal handoff
      drive_sys.DriveForwardCmd(8, vex::forward, 0.5, 0)->withTimeout(1),
      drive_sys.TurnToHeadingCmd(-45, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveForwardCmd(6, vex::forward, 0.5, 0),

      // new DebugCommand(),
    };
    cc.run();
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

  con.ButtonA.pressed([]() {
    // printf("testing\n");
    // FeedForward::ff_config_t config = drive_motioncontroller.tune_feedforward(drive_sys, odom, 0.6, 1);
    // printf("done\n");
    // printf("%f, %f, %f, %f\n", config.kS, config.kG, config.kV, config.kA);
    CommandController cc{
      new Async(new FunctionCommand([]() {
        while (true) {
          OdometryBase *odombase = &odom;
          pose_t pos = odombase->get_position();
          // printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
          vexDelay(20);

          if (goal_sensor.objectDistance(vex::mm) < 40 && goal_counter == 0) {
            goal_grabber_sol.set(true);
          }

          if (goal_counter > 0) {
            goal_counter--;
          }

          if (blue_alliance) {
            if (color_sensor.hue() > 0 && color_sensor.hue() < 30 && color_sensor_counter == 0) {
              color_sensor_counter = 30;
            }
          } else {
            if (color_sensor.hue() > 100 && color_sensor.hue() < 220 && color_sensor_counter == 0) {
              color_sensor_counter = 30;
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
        return true;
      })),

      // drop the intake
      drive_sys.DriveForwardCmd(5, vex::reverse, 1, 0)->withTimeout(0.3), intake_command(),

      // goal 1 ring 1
      drive_sys.DriveToPointCmd({48, 96}, vex::forward, 0.5, 0)->withTimeout(2),

      // goal 1 grab
      drive_sys.TurnToHeadingCmd(-90, 0.5, 0)->withTimeout(0.8),
      drive_sys.DriveToPointCmd({48, 120}, vex::reverse, 0.5, 0)->withTimeout(1), conveyor_intake_command(),

      // goal 1 ring 2
      drive_sys.TurnToHeadingCmd(0, 0.5, 0)->withTimeout(0.8),
      drive_sys.DriveToPointCmd({72, 120}, vex::forward, 0.5, 0)->withTimeout(1), new DelayCommand(350),

      // goal 1 ring 3
      drive_sys.TurnToHeadingCmd(90, 0.6, 0)->withTimeout(0.5),
      drive_sys.DriveForwardCmd(6, vex::forward, 0.8, 0)->withTimeout(0.5), new DelayCommand(100),
      drive_sys.DriveForwardCmd(6, vex::reverse, 0.8, 0)->withTimeout(0.5),

      // goal 1 ring 4
      drive_sys.TurnToHeadingCmd(180, 1, 0)->withTimeout(0.8),
      drive_sys.DriveToPointCmd({24, 120}, vex::forward, 0.6, 0)->withTimeout(4),

      // goal 1 ring 5
      drive_sys.TurnToHeadingCmd(135, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveForwardCmd(17, vex::forward, 0.3, 0)->withTimeout(1), new DelayCommand(500),
      drive_sys.DriveForwardCmd(10, vex::reverse, 0.4, 0)->withTimeout(1),

      // goal 1 drop
      drive_sys.TurnToHeadingCmd(-45, 0.8, 0)->withTimeout(1), drive_sys.DriveTankCmd(-0.3, -0.3)->withTimeout(1),
      new DelayCommand(1000), goal_grabber_command(false),

      // exit corner
      drive_sys.DriveToPointCmd({24, 120}, vex::forward, 0.5, 0)->withTimeout(2), stop_conveyor_command(),
      intake_command(),

      // goal 2 ring 1
      drive_sys.TurnToPointCmd(96, 120, vex::fwd, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({96, 118}, vex::fwd, 0.8, 0)->withTimeout(3),

      // goal 2 grab
      drive_sys.TurnToHeadingCmd(90, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({96, 96}, vex::reverse, 0.4, 0)->withTimeout(1), conveyor_intake_command(),
      new DelayCommand(1000),

      // goal 2 ring 2
      drive_sys.TurnToHeadingCmd(0, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({120, 96}, vex::forward, 0.6, 0)->withTimeout(1),

      // goal 2 ring 3
      drive_sys.TurnToHeadingCmd(90, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveToPointCmd({120, 120}, vex::forward, 0.6, 0)->withTimeout(1),

      // goal 2 ring 4
      drive_sys.TurnToHeadingCmd(45, 0.8, 0)->withTimeout(0.5),
      drive_sys.DriveForwardCmd(17, vex::forward, 0.3, 0)->withTimeout(1), new DelayCommand(500),
      drive_sys.DriveForwardCmd(10, vex::forward, 0.4, 0)->withTimeout(1),
      drive_sys.TurnToHeadingCmd(210, 0.8, 0)->withTimeout(1), new DelayCommand(200), goal_grabber_command(false),
      drive_sys.DriveTankCmd(-0.3, -0.3)->withTimeout(1),

      //   new DebugCommand(),
    };
    cc.run();
  });
}