#include "competition/autonomous.h"
#include "robot-config.h"

void game_auto();
void skills();

int goal_counter = 0;
int color_sensor_counter = 0;

bool conveyor_started = false;

bool blue_alliance = true;

void autonomous()
{
	vexDelay(700);

	game_auto();
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

/**
 * Approximate game auto path for Junior Jr. (what the new 15 inch robot will
 * likely be called)
 *
 * (Some values will definitely need to be changed, because these values are
 * trying to make the center of the robot move to the center of each point - so
 * it would try to drive to where the goal is instead of next to it)
 *
 * - Start at the point (12, 48) with the back of the robot (the goal grabber)
 *   facing 21.8 degrees south of east
 * - Drive 64.62 inches to the point (72, 24) and pick up the goal there
 * - Turn counterclockwise 21.8 degrees
 * - Drive 48 inches to the point (24, 24) and pick up the two rings of our
 *   color on that path
 * - Turn counterclockwise 45 degrees
 * - Drive 33.94 inches to the point (0, 0) and pick up the two rings in the
 *   corner
 * - Turn 180 degrees (clockwise is probably best so we can continue turning
 *   after more easily) and drop the goal in the positive corner
 * - Turn 153.43 degrees clockwise
 * - Drive 75.9 inches to the point (24, 72) and pick up the goal there
 * - Turn 71.57 degrees clockwise
 * - Drive 12 inches to (12, 72) and pick up the ring there
 * - Turn 158.2 degrees counterclockwise
 * - Drive 64.62 inches to the point (72, 48) to finish auton by touching the
 *   ladder
 */
void game_auto_red() {
    // CommandController cc {
    //     odom.SetPositionCmd({.x = 12, .y = 48, .rot = -21.8}),

    //     // NOTE: None of these have timeouts or cancel conditions since I don't
    //     // know how long each command will take to run, so those should get
    //     // added (probably during testing/debugging)

    //     // NOTE 2: Most (if not all) of these position values are going to be
    //     // wrong, but they should get the robot to somewhere near the correct
    //     // position

    //     // Drive to the first goal
    //     drive_sys.DriveToPointCommand({72, 24}, vex::reverse, .6),
    //     // Grab the goal
    //     goal_grabber_command(true),
    //     // Turn towards the next rings
    //     drive_sys.TurnDegreesCommand(21.8),
    //     // Start the intake and conveyor
    //     conveyor_intake_command(),
    //     intake_command(),
    //     // Drive while picking up the rings
    //     drive_sys.DriveToPointCommand({24, 24}, vex::forward, .6),
    //     // Turn and drive to the corner
    //     drive_sys.TurnDegreesCommand(45, .6),
    //     drive_sys.DriveToPointCommand({0, 0}, vex::forward, .6),
    //     // Turn and drop the goal in the positive corner
    //     drive_sys.TurnDegreesCommand(-180, .6),
    //     goal_grabber_command(false),
    //     // Turn and drive to the next goal
    //     drive_sys.TurnDegreesCommand(-153.43, .6),
    //     drive_sys.DriveToPointCommand({24, 72}, vex::reverse, .6),
    //     // Grab the goal
    //     goal_grabber_command(true),
    //     // Turn and drive to the next ring
    //     drive_sys.TurnDegreesCommand(-71.57),
    //     drive_sys.DriveToPointCommand({12, 72}, vex::forward, .6),
    //     // Turn and drive to the ladder
    //     drive_sys.TurnDegreesCommand(158.2, .6),
    //     drive_sys.DriveToPointCommand({72, 48}, vex::forward, .6),
    //     // Stop the intake and conveyor
    //     stop_conveyor_command(),
    //     stop_intake_command(),
    // };
    // cc.run();
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