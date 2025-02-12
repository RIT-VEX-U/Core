#include "competition/opcontrol.h"
#include "robot-config.h"
#include "vex.h"

const vex::controller::button &goal_grabber = con.ButtonRight;
const vex::controller::button &conveyor_button = con.ButtonR2;
const vex::controller::button &conveyor_button_rev = con.ButtonR1;

const vex::controller::button &wallstake_toggler = con.ButtonL1;
const vex::controller::button &wallstake_stow = con.ButtonL2;
const vex::controller::button &wallstake_alliancestake = con.ButtonDown;
const vex::controller::button &toggle_colorsort = con.ButtonLeft;

void testing();

void auto__();

int goal_counterd = 0;
int color_sensor_counterd = 0;

bool conveyor_startedd = false;

bool blue_allianced = true;

bool color_sort_enabledd = true;

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

// AutoCommand *conveyor_intake_command(double amt = 12.0) {
//     return new FunctionCommand([=]() {
//         conveyor_intake(amt);
//         conveyor_started = true;
//         return true;
//     });
// }

// AutoCommand *conveyor_outtake_command(double amt = 12.0) {
//     return new FunctionCommand([=]() {
//         conveyor_intake(-amt);
//         return true;
//     });
// }

// AutoCommand *stop_conveyor_command() {
//     return new FunctionCommand([=]() {
//         conveyor.stop();
//         conveyor_started = false;
//         return true;
//     });
// }

// AutoCommand *goal_grabber_command(bool value) {
//     return new FunctionCommand([=]() {
//         goal_grabber_sol.set(value);
//         goal_counter = 50;
//         return true;
//     });
// }

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
        goal_counterd = 50;
    });

    conveyor_button.pressed([]() {
        double volts;
        volts = 12;

        conveyor.spin(vex::directionType::fwd, volts, vex::volt);
        intake();
        if (color_sort_enabledd) {
          mcglight_board.set(true);
        } else {
          mcglight_board.set(false);
        }
    });
    conveyor_button_rev.pressed([]() {
        conveyor.spin(vex::directionType::rev, 12, vex::volt);
        outtake();
    });

    toggle_colorsort.pressed([] () {
      if (color_sort_enabledd) {
        color_sort_enabledd = false;
      } else {
        color_sort_enabledd = true;
      }
    });

    wallstake_toggler.pressed([]() {
        wallstake_mech.hold = true;
        if (wallstake_mech.get_angle().degrees() > 180) {
            wallstake_mech.set_setpoint(from_degrees(170));
        } else if (wallstake_mech.get_angle().degrees() < 180) {
            wallstake_mech.set_setpoint(from_degrees(50));
        }
    });

    wallstake_stow.pressed([]() {
        wallstake_mech.hold = true;
        wallstake_mech.set_setpoint(from_degrees(200));
    });

    // wallstake_alliancestake.pressed([]() {
    //     wallstake_mech.hold = true;
    //     wallstake_mech.set_setpoint(from_degrees(0));
    // });

    // ================ INIT ================
    color_sensor.setLight(vex::ledState::on);
    color_sensor.setLightPower(100, vex::pct);

    conveyor_intake(0);


    while (true) {
        if (!conveyor_button.pressing() && !conveyor_button_rev.pressing()) {
            conveyor.stop();
            intake(0);
            mcglight_board.set(false);
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

        drive_sys.drive_arcade(straight, turn * 1.75, 1, TankDrive::BrakeType::None);

        pose_t pos = odom.get_position();
        // printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
        // printf("%f\n", color_sensor.hue());

        // if (goal_sensor.objectDistance(vex::mm) < 25 && goal_counterd == 0) {
        //     goal_grabber_sol.set(true);
        // }

        // if (goal_counterd > 0) {
        //     goal_counterd--;
        // }

        if (color_sort_enabledd) {

          if (blue_allianced) {
              if (color_sensor.hue() > 0 && color_sensor.hue() < 30 && color_sensor_counterd == 0) {
                  color_sensor_counterd = 30;
              }
          } else {
              if (color_sensor.hue() > 100 && color_sensor.hue() < 220 && color_sensor_counterd == 0) {
                  color_sensor_counterd = 30;
              }
          }

          if (color_sensor_counterd == 25) {
              color_sensor_counterd--;
              conveyor.stop();
          }

          if (color_sensor_counterd > 0) {
              color_sensor_counterd--;

          }

          if (color_sensor_counterd == 0 && conveyor_button.pressing()) {
            conveyor_intake();
          }
        }

        vexDelay(20);
    }

    // ================ PERIODIC ================
}


