#include "TempSubSystems/Intake.h"
#include "robot-config.h"

IntakeSys::IntakeSys() { task = vex::task(thread_fn, this); }

void IntakeSys::color_sort_on() { color_sort_state = IntakeSys::ColorSortState::ON; }

void IntakeSys::color_sort_off() { color_sort_state = IntakeSys::ColorSortState::OFF; }

AutoCommand *IntakeSys::SetColorSortCmd(bool colorsort_on) {
    return new FunctionCommand([&]() {
        set_color_sort_bool(colorsort_on);
        return true;
    });
}

void IntakeSys::set_color_sort_bool(bool ColorSortOn) {
    if (ColorSortOn) {
        color_sort_state = IntakeSys::ColorSortState::ON;
    } else {
        color_sort_state = IntakeSys::ColorSortState::OFF;
    }
}

bool IntakeSys::should_stop_for_colorsort() {
    if (color_sort_state == ColorSortState::OFF) {
        return false;
    }
    return color_sensor_counter > 0 && color_sensor_counter < 16;
}

void IntakeSys::colorSort() {
    // printf("colorsorting, %d\n", color_sensor_counter);
    mcglight_board.set(true);
    if (blue_alliance()) {
        if (color_sensor.hue() > 0 && color_sensor.hue() < 30 && color_sensor_counter == 0) {
            color_sensor_counter = 20;
            conveyor.spin(vex::forward, intakeVolts, vex::volt);
            // wallstakemech_sys.set_setpoint(from_degrees(130));
        }
    } else {
        if (color_sensor.hue() > 160 && color_sensor.hue() < 240 && color_sensor_counter == 0) {
            color_sensor_counter = 20;
            conveyor.spin(vex::forward, intakeVolts, vex::volt);
            // wallstakemech_sys.set_setpoint(from_degrees(130));
        }
    }

    if (color_sensor_counter > 0) {
        color_sensor_counter--;
    }
}
void IntakeSys::intake(double volts) {
    intake_state = IntakeState::IN;
    intakeVolts = volts;
}

void IntakeSys::outtake(double volts) {
    intake_state = IntakeState::OUT;
    intakeVolts = volts;
}

void IntakeSys::intake_stop() { intake_state = IntakeState::STOP; }

void IntakeSys::conveyor_in(double volts) {
    conveyor_state = IntakeState::IN;
    conveyorVolts = volts;
}
void IntakeSys::conveyor_stop() { conveyor_state = IntakeState::STOP; }
void IntakeSys::conveyor_out(double volts) {
    conveyor_state = IntakeState::OUT;
    conveyorVolts = volts;
}

void IntakeSys::setLight(bool state) { mcglight_board.set(state); }

int IntakeSys::thread_fn(void *ptr) {
    IntakeSys &self = *(IntakeSys *)ptr;
    color_sensor.setLight(vex::ledState::on);
    color_sensor.setLightPower(100, vex::pct);

    while (true) {
        if (self.color_sort_state == ColorSortState::ON && self.intake_state == IntakeState::IN) {
            mcglight_board.set(true);
            self.colorSort();
        } else {
            mcglight_board.set(false);
        }

        if (self.intake_state == IntakeState::IN) {
            intake_motor.spin(vex::fwd, self.intakeVolts, vex::volt);
        } else if (self.intake_state == IntakeState::OUT) {
            // printf("IntakeState OUT ");
            intake_motor.spin(vex::reverse, self.intakeVolts, vex::volt);
        } else if (self.intake_state == IntakeState::STOP) {
            // printf("IntakeState STOP ");
            intake_motor.stop();
        }
        if (self.conveyor_state == IntakeState::IN) {
            // printf("ConveyorState IN \n");
            if (self.should_stop_for_colorsort()) {
                // conveyor.stop();
                printf("AHHHHHH\n");
                wallstakemech_sys.set_setpoint(from_degrees(130));

            } else {
                intake_motor.spin(vex::fwd, self.intakeVolts, vex::volt);
                conveyor.spin(vex::fwd, self.conveyorVolts, vex::volt);
                if (self.color_sort_state == ColorSortState::ON && self.color_sensor_counter <= 0) {
                    wallstakemech_sys.set_setpoint(from_degrees(200));
                    printf("AHHHHHH\n");
                }
            }

        } else if (self.conveyor_state == IntakeState::OUT) {
            // printf("ConveyorState OUT \n");
            conveyor.spin(vex::reverse, self.conveyorVolts, vex::volt);
        } else if (self.conveyor_state == IntakeState::STOP) {
            // printf("ConveyorState STOP \n");
            conveyor.stop();
        }
        this_thread::sleep_for(10);
    }
    return 0;
}

AutoCommand *IntakeSys::IntakeCmd(double amt) {
    return new FunctionCommand([this, amt]() {
        intake(amt);
        return true;
    });
}

AutoCommand *IntakeSys::OuttakeCmd(double amt) {
    return new FunctionCommand([this, amt]() {
        outtake(amt);
        return true;
    });
}

AutoCommand *IntakeSys::IntakeStopCmd() {
    return new FunctionCommand([this]() {
        intake_stop();
        return true;
    });
}

AutoCommand *IntakeSys::ConveyorInCmd(double amt) {
    return new FunctionCommand([this, amt]() {
        conveyor_in(amt);
        conveyorStarted = true;
        return true;
    });
}

AutoCommand *IntakeSys::ConveyorOutCmd(double amt) {
    return new FunctionCommand([this, amt]() {
        conveyor_out(amt);
        conveyorStarted = true;
        return true;
    });
}

AutoCommand *IntakeSys::ConveyorStopCmd() {
    return new FunctionCommand([this]() {
        conveyor_stop();
        conveyorStarted = false;
        return true;
    });
}