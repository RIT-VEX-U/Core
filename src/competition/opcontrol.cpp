#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

const vex::controller::button &goal_grabber =con.ButtonRight;
const vex::controller::button &conveyor_button =con.ButtonR2;
const vex::controller::button &conveyor_button_rev = con.ButtonR1;
const vex::controller::button &intake_button =con.ButtonL2;
const vex::controller::button &intake_button_rev = con.ButtonL1;

const vex::controller::button &wallstake_handoff = con.ButtonUp;
const vex::controller::button &wallstake_above_neutral = con.ButtonLeft;
const vex::controller::button &wallstake_on_neutral = con.ButtonDown;

/**
 * Main entrypoint for the driver control period
*/
void opcontrol()
{

goal_grabber.pressed([](){
    goal_grabber_sol.set(!goal_grabber_sol);
    
});

conveyor_button.pressed([](){
    conveyor.spin(vex::directionType::fwd,12,vex::volt);

});
conveyor_button_rev.pressed([](){
    conveyor.spin(vex::directionType::rev,12,vex::volt);

});

intake_button.pressed([](){
    intake.spin(vex::directionType::fwd,12,vex::volt);
});
intake_button_rev.pressed([](){
    intake.spin(vex::directionType::rev,12,vex::volt);

});

wallstake_handoff.pressed([](){
    wallstake_mech.set_state(HANDOFF);
});

wallstake_above_neutral.pressed([](){
    wallstake_mech.set_state(ABOVE_NEUTRAL);
});

wallstake_on_neutral.pressed([](){
    wallstake_mech.set_state(ON_NEUTRAL);
});





    // ================ INIT ================
    while (true)
    {
        if(!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
            conveyor.stop();
        }
        if(!intake_button.pressing() && ! intake_button_rev.pressing()){
            intake.stop();
        }
        double straight = (double)con.Axis3.position() / 100;
        double turn = (double)con.Axis1.position() / 100;

        drive_sys.drive_arcade(straight, turn * -1.75, 1, TankDrive::BrakeType::None);

        //pose_t pos = odom.get_position();
        //printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
        vexDelay(20);
        
         
    }
    
    // ================ PERIODIC ================


}
