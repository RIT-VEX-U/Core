#pragma once
#include "../core/include/subsystems/tank_drive.h"
#include "pid.h"
#include "vex.h"

class ManualTuner {
  public:
    ManualTuner(PID &pid, TankDrive &drive_sys);

    void set_setpoint(double setpoint);
    void set_p(double p);
    void set_i(double i);
    void set_d(double d);
    void startTuning();
    void stopTuning();

    static int thread_fn(void *ptr);
    bool doTuning = false;

  private:
    TankDrive drive_sys;
    vex::task task;
    double setpoint = 0;
    PID pid;
};
