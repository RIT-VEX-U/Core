#include "TempSubsystems/Intake.h"
#include "robot-config.h"

IntakeSys::IntakeSys(){
    task = vex::task(thread_fn,this);
}


void IntakeSys::setColorSortOn(bool ColorSortOn){
    colorSortEnabled = ColorSortOn;
}

bool IntakeSys::getColorSortOn(){
    return colorSortEnabled;
}

void IntakeSys::colorSort(){
    mcglight_board.set(true);
    if (blue_alliance()) {
            if (color_sensor.hue() > 0 && color_sensor.hue() < 30 && color_sensor_counter == 0) {
                color_sensor_counter = 30;
                conveyor.spin(vex::forward, intakeVolts, vex::volt);
            }
        } else {
            if (color_sensor.hue() > 160 && color_sensor.hue() < 240 && color_sensor_counter == 0) {
                color_sensor_counter = 30;
                conveyor.spin(vex::forward, intakeVolts, vex::volt);
            }
        }

        if (color_sensor_counter == 25) {
            color_sensor_counter--;
            conveyor.stop();
        }

        if (color_sensor_counter > 0) {
            color_sensor_counter--;
            
        }

        if (conveyorStarted && color_sensor_counter == 0) {
            conveyor.spin(vex::forward, conveyorVolts, vex::volt);
        }
}
void IntakeSys::intake(double volts){
    intake_state = IntakeState::IN;
    intakeVolts = volts;
}

void IntakeSys::outtake(double volts){
    intake_state = IntakeState::OUT;
    intakeVolts = volts;
}

void IntakeSys::intake_stop(){
    intake_motor.stop();
}

void IntakeSys::conveyor_in(double volts){
    intake_motor.spin(vex::directionType::fwd, volts, vex::volt);
}
void IntakeSys::conveyor_stop(){
    conveyor.stop();
}
void IntakeSys::conveyor_out(double volts){
    conveyor.spin(vex::directionType::rev, volts, vex::volt);
}

void IntakeSys::setLight(bool state){
    mcglight_board.set(state);
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

int IntakeSys::thread_fn(void *ptr){
    IntakeSys &self = *(IntakeSys*)ptr;
    while(true){
        if(self.colorSortEnabled){
            mcglight_board.set(true);
            self.colorSort();
        }
        else{
            mcglight_board.set(false);
        }
        if(self.intake_state == IntakeState::IN){
        intake_motor.spin(vex::fwd, self.intakeVolts, vex::volt);
        }
        else if(self.intake_state == IntakeState::OUT){
            intake_motor.spin(vex::reverse, self.intakeVolts, vex::volt);
        }
        else if(self.intake_state == IntakeState::STOP){
            intake_motor.stop();
        }
        vexDelay(20);
    }
    return 69;
}