#include "../core/include/utils/controls/pid_tuners.h"

ManualTuner::ManualTuner(PID &pid, TankDrive &drive_sys) : pid(pid), drive_sys(drive_sys) {
    task = vex::task(thread_fn, this);
};

void ManualTuner::set_p(double p) { pid.config.p = p; }
void ManualTuner::set_i(double i) { pid.config.i = i; }
void ManualTuner::set_d(double d) { pid.config.d = d; }

void ManualTuner::startTuning() { doTuning = true; }
void ManualTuner::stopTuning() { doTuning = false; }

void ManualTuner::set_setpoint(double setpoint) { this->setpoint = setpoint; }

int ManualTuner::thread_fn(void *ptr) {
    ManualTuner &self = *(ManualTuner *)ptr;
    self.pid.config.deadband = 0;
    self.drive_sys.odometry->set_position({0, 0, 0});
    while (true) {
        if (self.doTuning) {
            if (self.pid.config.error_method == PID::ERROR_TYPE::ANGULAR) {
                self.drive_sys.turn_to_heading(self.setpoint, 1, 0);
            } else if (self.pid.config.error_method == PID::ERROR_TYPE::LINEAR) {
                self.drive_sys.drive_to_point(self.setpoint, 0, vex::fwd, 1, 0);
            }
        }
        vexDelay(20);
    }
    return 0;
}