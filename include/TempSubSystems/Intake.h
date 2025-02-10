#pragma once
#include "vex.h"
#include "../core/include/utils/command_structure/auto_command.h"
class IntakeSys{
    public:
    IntakeSys();
    
    enum RunningType{
        OPCONTROL,
        AUTONOMOUS,
    };
    

    enum IntakeState {
        STOP,
        IN,
        OUT,

    };

    enum ColorSortState{
        ON,
        OFF,
    };
    ColorSortState get_color_sort_state(){
        return color_sort_state;
    }
    bool get_color_sort_bool(){
        if(color_sort_state == ColorSortState::ON){
            return true;
        }
        else if(color_sort_state == ColorSortState::OFF){
            return false;
        }
    }
    RunningType get_run_type(){
        return run_type;
    }
    IntakeState get_intake_state(){
        return intake_state;
    }
    IntakeState get_conveyor_state(){
        return conveyor_state;
    }
    

    void colorSort();
    void intake(double volts = 12);

    void outtake(double volts = 12);



    void intake_stop();

    void conveyor_in(double volts = 10);
    void conveyor_stop();
    void conveyor_out(double volts = 10);

    void setLight(bool state);

    void color_sort_on();
    void color_sort_off();
    void set_color_sort_bool(bool colorSortingOn);
    void opcontrol_init();
    void autonomous_init();


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
    IntakeState conveyor_state = IntakeState::STOP;
    ColorSortState color_sort_state = ColorSortState::OFF;
    RunningType run_type = RunningType::OPCONTROL;
    double intakeVolts = 12;
    double conveyorVolts = 10;
    int color_sensor_counter = 0;
    bool conveyorStarted = false;
    double sortConveyorVolts = 12;
};