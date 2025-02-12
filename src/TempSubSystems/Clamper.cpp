#include "TempSubSystems/Clamper.h"
#include "robot-config.h"

ClamperSys::ClamperSys() { task = vex::task(thread_fn, this); }

void ClamperSys::autoClamp() {
    if (goal_sensor.objectDistance(vex::mm) < 40) {
        clamper_state = ClamperState::CLAMPED;
    }
}

void ClamperSys::start_auto_clamping() { AutoClamping = true; };
void ClamperSys::stop_auto_clamping() { AutoClamping = false; };

void ClamperSys::clamp() { clamper_state = ClamperState::CLAMPED; };
void ClamperSys::unclamp() { clamper_state = ClamperState::UNCLAMPED; };

void ClamperSys::toggle_clamp() {
    if (is_clamped()) {
        unclamp();
    } else {
        clamp();
    }
}

// returns true if the piston is clamped down
bool ClamperSys::is_clamped() { return goal_grabber_sol.value(); };

AutoCommand *ClamperSys::ClampCmd(ClamperState state) {
    return new FunctionCommand([this, state]() {
        clamper_state = state;
        return true;
    });
}

int ClamperSys::thread_fn(void *ptr) {
    ClamperSys &self = *(ClamperSys *)ptr;
    while (true) {
        if (self.AutoClamping) {
            self.autoClamp();
        }
        if (self.clamper_state == ClamperState::CLAMPED) {
            goal_grabber_sol.set(true);
        } else if (self.clamper_state == ClamperState::UNCLAMPED) {
            goal_grabber_sol.set(false);
        }
    }
    return 420;
}