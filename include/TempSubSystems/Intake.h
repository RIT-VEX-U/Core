#pragma once
#include "vex.h"
#include "../core/include/utils/command_structure/auto_command.h"
class IntakeSys{
    public:
    IntakeSys();

    enum class ConveyorState {
        STOP,
        IN,
        OUT,
    };
    
    

    enum IntakeState {
        STOP,
        IN,
        OUT,

    };
    IntakeState get_intake_state(){
        return intake_state;
    }
    ConveyorState get_conveyor_state(){
        return conveyor_state;
    }
    

    void setColorSortOn(bool ColorSortOn);
    bool getColorSortOn();

    void colorSort();
    void intake(double volts = 12);

    void outtake(double volts = 12);



    void intake_stop();

    void conveyor_in(double volts = 10);
    void conveyor_stop();
    void conveyor_out(double volts = 10);

    void setLight(bool state);
    static int thread_fn(void *ptr);

    AutoCommand *IntakeCmd(double amt = 10.0);
    AutoCommand *OuttakeCmd(double amt = 10.0);
    AutoCommand *IntakeStopCmd();

    AutoCommand *ConveyorInCmd(double amt = 10.0);
    AutoCommand *ConveyorOutCmd(double amt = 10.0);
    AutoCommand *ConveyorStopCmd();

    private:
    vex::task task;
    IntakeState intake_state = IntakeState::STOP;
    ConveyorState conveyor_state = ConveyorState::STOP;
    double intakeVolts = 12;
    double conveyorVolts = 10;
    int color_sensor_counter = 0;
    bool conveyorStarted = false;
    double sortConveyorVolts = 12;
    bool colorSortEnabled = false;
};