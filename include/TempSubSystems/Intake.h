#pragma once
#include "../core/include/utils/command_structure/auto_command.h"
#include "vex.h"
class IntakeSys {
  public:
    IntakeSys();

    enum RunningType {
        OPCONTROL,
        AUTONOMOUS,
    };

    enum IntakeState {
        STOP,
        IN,
        OUT,

    };

    enum ColorSortState {
        ON,
        OFF,
    };
    ColorSortState get_color_sort_state() { return color_sort_state; }
    bool get_color_sort_bool() {
        if (color_sort_state == ColorSortState::ON) {
            return true;
        } else {
            return false;
        }
    }
    IntakeState get_intake_state() { return intake_state; }
    IntakeState get_conveyor_state() { return conveyor_state; }

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

    bool should_stop_for_colorsort();
    static int thread_fn(void *ptr);

    AutoCommand *IntakeCmd(double amt = 10.0);
    AutoCommand *OuttakeCmd(double amt = 10.0);
    AutoCommand *IntakeStopCmd();

    AutoCommand *ConveyorInCmd(double amt = 10.0);
    AutoCommand *ConveyorOutCmd(double amt = 10.0);
    AutoCommand *ConveyorStopCmd();

    AutoCommand *SetColorSortCmd(bool colorsort_on);

  private:
    vex::task task;
    IntakeState intake_state = IntakeState::STOP;
    IntakeState conveyor_state = IntakeState::STOP;
    ColorSortState color_sort_state = ColorSortState::OFF;
    double intakeVolts = 12;
    double conveyorVolts = 10;
    int color_sensor_counter = 0;
    bool conveyorStarted = false;
    double sortConveyorVolts = 12;
};