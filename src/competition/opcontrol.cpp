#include "competition/opcontrol.h"
#include "robot-config.h"
#include "vex.h"

const vex::controller::button &goal_grabber = con.ButtonRight;
const vex::controller::button &conveyor_button = con.ButtonR2;
const vex::controller::button &conveyor_button_rev = con.ButtonR1;

const vex::controller::button &wallstake_handoff = con.ButtonUp;
const vex::controller::button &wallstake_up = con.ButtonL1;
const vex::controller::button &wallstake_down = con.ButtonL2;

void testing();

void auto__();

int goal_counter = 0;
int color_sensor_counter = 0;

bool conveyor_started = false;

bool blue_alliance = true;

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
 * Main entrypoint for the driver control period
 */
void opcontrol() {

    // con.ButtonA.pressed([]() {
    //     auto_();
    // });
    // auto__();
    // return;

    goal_grabber.pressed([]() {
        goal_grabber_sol.set(!goal_grabber_sol);
        goal_counter = 50;
    });

    conveyor_button.pressed([]() {
        double volts;
        if (color_sensor_counter == 0) {
            volts = 10;
            conveyor.setBrake(vex::brakeType::coast);
        } else {
            volts = 4;
            conveyor.setBrake(vex::brakeType::brake);
        }

        conveyor.spin(vex::directionType::fwd, volts, vex::volt);
        intake();
        mcglight_board.set(true);
    });
    conveyor_button_rev.pressed([]() {
        conveyor.spin(vex::directionType::rev, 10, vex::volt);
        outtake();
    });

    // wallstake_up.pressed([](){
    //     if (wallstake_mech.is_below_handoff()) {
    //         wallstake_mech.set_state(HANDOFF);
    //         wallstake_mech.hold = true;
    //     } else {
    //         wallstake_mech.set_voltage(-4);
    //         wallstake_mech.hold = false;
    //     }
    // });

    // wallstake_down.pressed([](){
    //     if (!wallstake_mech.is_below_handoff()) {
    //         wallstake_mech.set_voltage(4);
    //         wallstake_mech.hold = false;
    //     }
    // });

    // ================ INIT ================
    color_sensor.setLight(vex::ledState::on);
    color_sensor.setLightPower(100, vex::pct);

    while (true) {
        if (!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
            conveyor.stop();
            intake(0);
            mcglight_board.set(true);
        }

        // if (!wallstake_down.pressing() || !wallstake_up.pressing()) {
        //     wallstake_mech.hold = true;
        // } else {
        //     wallstake_mech.hold = false;
        // }

        // if (!intake_button.pressing() && !intake_button_rev.pressing()) {
        //     intake(0);
        // }
        double straight = (double)con.Axis3.position() / 100;
        double turn = (double)con.Axis1.position() / 100;

        drive_sys.drive_arcade(straight, turn * -1.75, 1, TankDrive::BrakeType::None);

        pose_t pos = odom.get_position();
        // printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
        printf("%f\n", color_sensor.hue());

        if (goal_sensor.objectDistance(vex::mm) < 25 && goal_counter == 0) {
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

        if (color_sensor_counter == 0 && conveyor_button.pressing()) {
          conveyor_intake();
        }

        vexDelay(20);
    }

    // ================ PERIODIC ================
}


