#pragma once
#include "../core/include/utils/command_structure/auto_command.h"
#include "vex.h"

class ClamperSys {
  public:
    ClamperSys();
    enum ClamperState {
        CLAMPED,
        UNCLAMPED,
    };

    void start_auto_clamping();
    void stop_auto_clamping();
    void toggle_clamp();

    void clamp();
    void unclamp();

    bool is_clamped();
    AutoCommand *ClampCmd(ClamperState state);

  private:
    void autoClamp();
    static int thread_fn(void *ptr);

    vex::task task;
    ClamperState clamper_state = ClamperState::UNCLAMPED;
    bool AutoClamping = false;
};