#include "competition/autonomous.h"
#include "robot-config.h"

void game_auto();
void skills();

int goal_countera = 0;

void autonomous()
{
	vexDelay(700);

	game_auto();
}

// Some of these AutoCommands are basically the exact same as the previous
// Junior's commands, although some previous commands weren't needed

// AutoCommand *intake_command(double amt = 12.0) {
//     return new FunctionCommand([=]() {
//         intake(amt);
//         return true;
//     });
// }

// AutoCommand *outtake_command(double amt = 12.0) {
//     return new FunctionCommand([=]() {
//         intake(-amt);
//         return true;
//     });
// }

// AutoCommand *stop_intake_command() {
//     return new FunctionCommand([=]() {
//         intake_motor.stop();
//         return true;
//     });
// }

// class ConveyorStalled : public Condition {
//     bool test() override {
//         return conveyor.current() > 1.5;
//     }
// };

// AutoCommand *conveyor_intake_command(double amt = 12.0) {
//     return new FunctionCommand([=]() {
//         conveyor.spin(vex::directionType::fwd, amt, vex::volt);
//         while (conveyor.current() > 1.5) {
//             printf("stalls");
//             conveyor.spin(vex::directionType::rev, -1 * amt, vex::volt);
//         }
//         return true;
//     });
// }

// AutoCommand *conveyor_outtake_command(double amt = 12.0) {
//     return new FunctionCommand([=]() {
//         conveyor.spin(vex::directionType::rev, amt, vex::volt);
//         while (conveyor.current() > 1.5) {
//             printf("stalls");
//             conveyor.spin(vex::directionType::fwd, amt, vex::volt);
//         }
//         return true;
//     });
// }

// AutoCommand *stop_conveyor_command() {
//     return new FunctionCommand([=]() {
//         conveyor.spin(vex::directionType::rev, 0.0, vex::volt);
//         return true;
//     });
// }

// AutoCommand *goal_grabber_command(bool value) {
// 	return new FunctionCommand([=]() {
// 		goal_grabber_sol.set(value);
//         goal_countera = 10;
// 		return true;
// 	});
// }

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
void game_auto() {
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

// void skills() {
	// CommandController cc {
	// 	// odom.SetPositionCmd({.x = 9.5, .y = 72, .rot = 0}),

	// 	new Async(new FunctionCommand([]() {
	// 		while(true) {
	// 			OdometryBase *odombase = &odom;
    //             pose_t pos = odombase->get_position();
    //         	printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
	// 			vexDelay(100);

	// 			if((conveyor.current() > 2) && conveyor.velocity(rpm) < 0.5){
	// 				printf("Conveyor Stalling");
	// 				conveyor_intake(-12);
	// 				vexDelay(500);
	// 				conveyor_intake(12);
	// 			}

    //             if (goal_sensor.objectDistance(vex::mm) < 25 && goal_countera == 0) {
    //             goal_grabber_sol.set(true);
    //             }

    //             if (goal_countera > 0) {
    //                 goal_countera--;
    //             }
	// 		}
	// 		return true;
	// 	})),

	// 	// First Ring

    //     // drive_sys.DriveForwardCmd(24, fwd, 0.6)->withTimeout(2),
    //     // drive_sys.TurnToHeadingCmd(90, 0.6),
    //     // drive_sys.DriveForwardCmd(24, fwd, 0.6)->withTimeout(2),
    //     // drive_sys.TurnToHeadingCmd(180, 0.6),
    //     // drive_sys.DriveForwardCmd(24, fwd, 0.6)->withTimeout(2),
    //     // drive_sys.TurnToHeadingCmd(270, 0.6),
    //     // drive_sys.DriveForwardCmd(24, fwd, 0.6)->withTimeout(2),
    //     // drive_sys.TurnToHeadingCmd(360, 0.6),

    //     intake_command(),
	// 	drive_sys.DriveToPointCmd({50, 96}, vex::forward, .6) -> withTimeout(4),

    //     conveyor_stop_command(),

    //     // First Stake
    //     drive_sys.TurnToHeadingCmd(-90, .6) -> withTimeout(4),
    //     drive_sys.DriveToPointCmd({48, 120}, vex::reverse, .3) -> withTimeout(10),
    //     // goal_grabber_command(true),
    //     conveyor_intake_command(),
        

    //     // Second Ring
    //     drive_sys.TurnToPointCmd(72, 120, vex::directionType::fwd, .6) -> withTimeout(4),
    //     drive_sys.DriveToPointCmd({72, 120}, vex::forward, .6) -> withTimeout(4),

    //     // Third Ring
    //     // drive_sys.TurnToHeadingCmd(90, .6) -> withTimeout(2),
    //     drive_sys.TurnToPointCmd(72, 128, vex::directionType::fwd, .6) -> withTimeout(4),
    //     drive_sys.DriveToPointCmd({72, 128}, vex::forward, .6) -> withTimeout(4),

    //     // Fourth Ring
    //     drive_sys.TurnToPointCmd(24, 120, vex::directionType::fwd, .6) -> withTimeout(1),
    //     drive_sys.DriveToPointCmd({24, 120}, vex::forward, .6) -> withTimeout(1.5),

    //     // Fifth Ring
    //     drive_sys.TurnToPointCmd(4, 144, vex::directionType::fwd, .6) -> withTimeout(1),
    //     drive_sys.DriveToPointCmd({6, 137}, vex::forward, .6) -> withTimeout(1.05),
    //     //new DebugCommand(),
            
    //     // Deposit First Stake
    //     drive_sys.DriveForwardCmd(18, vex::directionType::rev, .6) -> withTimeout(.7),
    //     new DelayCommand(500),
    //     // drive_sys.TurnToPointCmd(96, 120, vex::directionType::fwd, .6) -> withTimeout(3),
    //     drive_sys.TurnToHeadingCmd(315, .5) -> withTimeout(1),
    //     //drive_sys.DriveForwardCmd(15, vex::directionType::rev, .8) -> withTimeout(.3),
    //     new DelayCommand(500),
    //     drive_sys.DriveTankCmd(-.5,-.5) -> withTimeout(1),
    //     //drive_sys.DriveForwardCmd(15, vex::directionType::rev, .6) -> withTimeout(.7),
    //     conveyor_stop_command(),
        
        

    //     //drop goal/move away
    //     goal_grabber_command(false),


    //     };
	// cc.run();
// }