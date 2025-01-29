#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

const vex::controller::button &goal_grabber =con.ButtonRight;
// const vex::controller::button &conveyor_button =con.ButtonR2;
// const vex::controller::button &conveyor_button_rev = con.ButtonR1;
const vex::controller::button &intake_button =con.ButtonR1;
const vex::controller::button &intake_button_rev = con.ButtonR2;

const vex::controller::button &wallstake_handoff = con.ButtonUp;
const vex::controller::button &wallstake_above_neutral = con.ButtonLeft;
const vex::controller::button &wallstake_on_neutral = con.ButtonDown;

void testing();

void auto__();

int goal_counter = 0;

/**
 * Main entrypoint for the driver control period
*/
void opcontrol()
{
    // testing();
    // con.ButtonA.pressed([]() {
    //     auto_();
    // });
    // auto__();
    // return;
    

goal_grabber.pressed([](){
    goal_grabber_sol.set(!goal_grabber_sol);
    goal_counter = 50;
    
});

// conveyor_button.pressed([](){
//     conveyor.spin(vex::directionType::fwd,12,vex::volt);
//     intake_roller.spin(vex::directionType::fwd,12,vex::volt);

// });
// conveyor_button_rev.pressed([](){
//     conveyor.spin(vex::directionType::rev,12,vex::volt);
//     intake_roller.spin(vex::directionType::rev,12,vex::volt);
// });

intake_button.pressed([](){
    printf("intaking... \n");
    intake();
    conveyor_intake();
});

intake_button_rev.pressed([](){
    printf("outtaking... \n");
    outtake();
    conveyor_outtake();

});

// wallstake_handoff.pressed([](){
//     wallstake_mech.set_state(HANDOFF);
// });

// wallstake_above_neutral.pressed([](){
//     wallstake_mech.set_state(ABOVE_NEUTRAL);
// });

// wallstake_on_neutral.pressed([](){
//     wallstake_mech.set_state(ON_NEUTRAL);
// });

con.ButtonA.pressed([]() {
    printf("Button A Pressed \n");
    CommandController cc{
        new Async(new FunctionCommand([]() {
			while(true) {
                pose_t pos = odom.get_position();
            	printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
				vexDelay(100);

				if((conveyor.current() > 2) && conveyor.velocity(rpm) < 0.5){
					printf("Conveyor Stalling");
					conveyor_intake(-12);
					vexDelay(500);
					conveyor_intake(12);
				}
			}
			return true;
		})),
        odom.SetPositionCmd({.x=0, .y = 0,.rot =0}),
        drive_sys.DriveForwardCmd(48, vex::forward, 1, 0),


    };
    cc.run();
  });





    // ================ INIT ================

    while (true)
    {
        // if(!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
        //     conveyor.stop();
        // }
        if(!intake_button.pressing() && ! intake_button_rev.pressing()){
            intake(0);
        }
        double left = (double)con.Axis3.position() / 100;
        double right = (double)con.Axis2.position() / 100;

        drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);

        pose_t pose = odom.get_position();

        printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pose.x, pose.y, pose.rot);

        if(!intake_button.pressing() && !intake_button_rev.pressing()) {
            intake_motor.stop();
            conveyor.stop();
        }


        if (goal_sensor.objectDistance(vex::mm) < 25 && goal_counter == 0) {
            goal_grabber_sol.set(true);
        }

        if (goal_counter > 0) {
            goal_counter--;
        }

        
        vexDelay(20);
        
         
    }
    
    // ================ PERIODIC ================


}

void testing() {

  class DebugCommand : public AutoCommand {
  public:
    bool run() override {
      drive_sys.stop();
      pose_t pos = odom.get_position();
      printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
      while (true) {
        double left = (double)con.Axis3.position() / 100;
        double right = (double)con.Axis2.position() / 100;

        drive_sys.drive_tank(left, right, 1, TankDrive::BrakeType::None);

        vexDelay(100);
      }
      return true;
    }

  };

  con.ButtonA.pressed([]() {
    printf("Button A Pressed \n");
    CommandController cc{
        new Async(new FunctionCommand([]() {
			while(true) {
                pose_t pos = odom.get_position();
            	printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
				vexDelay(100);

				if((conveyor.current() > 2) && conveyor.velocity(rpm) < 0.5){
					printf("Conveyor Stalling");
					conveyor_intake(-12);
					vexDelay(500);
					conveyor_intake(12);
				}
			}
			return true;
		})),
        odom.SetPositionCmd({.x=0, .y = 0,.rot =0}),
        drive_sys.DriveForwardCmd(48, vex::forward, 1, 0),


    };
    cc.run();
  });

  while(true){
    pose_t pos = odom.get_position();
    printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
    vexDelay(100);
  }


}















void auto__() {
	CommandController cc {
		// odom.SetPositionCmd({.x = 9.5, .y = 72, .rot = 0}),

		new Async(new FunctionCommand([]() {
			while(true) {
				OdometryBase *odombase = &odom;
                pose_t pos = odombase->get_position();
            	printf("ODO X: %.2f, Y: %.2f, R:%.2f, Concurr: %f\n", pos.x, pos.y, pos.rot, conveyor.current());
				vexDelay(100);

				if((conveyor.current() > 2) && conveyor.velocity(rpm) < 0.5){
					printf("Conveyor Stalling");
					conveyor_intake(-12);
					vexDelay(500);
					conveyor_intake(12);
				}

                if (goal_sensor.objectDistance(vex::mm) < 25 && goal_counter == 0) {
                goal_grabber_sol.set(true);
                }

                if (goal_counter > 0) {
                    goal_counter--;
                }
			}
			return true;
		})),

		// First Ring

        // drive_sys.DriveForwardCmd(24, fwd, 0.6)->withTimeout(2),
        // drive_sys.TurnToHeadingCmd(90, 0.6),
        // drive_sys.DriveForwardCmd(24, fwd, 0.6)->withTimeout(2),
        // drive_sys.TurnToHeadingCmd(180, 0.6),
        // drive_sys.DriveForwardCmd(24, fwd, 0.6)->withTimeout(2),
        // drive_sys.TurnToHeadingCmd(270, 0.6),
        // drive_sys.DriveForwardCmd(24, fwd, 0.6)->withTimeout(2),
        // drive_sys.TurnToHeadingCmd(360, 0.6),

        // intake_command(),
		drive_sys.DriveToPointCmd({29.5, 80}, vex::reverse, 0.5) -> withTimeout(4),

        };
	cc.run();
}