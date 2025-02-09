#pragma once
#include "vex.h"
#include "../core/include/utils/command_structure/auto_command.h"

class ClamperSys{
    public:
    ClamperSys();
    enum ClamperState{
        CLAMPED,
        UNCLAMPED,
    };
    
    void autoClamp();

    void start_auto_clamping();
    void stop_auto_clamping();

    void clamp();
    void unclamp();

    bool is_clamped();
    AutoCommand *ClampCmd(ClamperState state);
    private:
    static int thread_fn(void *ptr);

    vex::task task;
    ClamperState clamper_state = ClamperState::UNCLAMPED;
    bool AutoClamping = false;
};