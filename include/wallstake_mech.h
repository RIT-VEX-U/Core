#pragma once

#include "core.h"
#include "vex.h"

enum WallStakeState {
    STOW = 199,
    HANDOFF = 170,
    ABOVE_ALLIANCE = 80,
    ON_ALLIANCE = 10
};

class WallStakeMech {
    public:

    bool hold = false;

    WallStakeMech(const vex::motor_group &motors, const vex::pot &pot, const Rotation2d &tolerance, const Rotation2d &setpoint, const double &pot_offset, PID pid);

    Rotation2d get_angle();
    

    bool is_at_setpoint();
    bool is_at_angle(const double &angle);
    bool is_at_state(const WallStakeState &state);

    void update();
    void set_voltage(const double &voltage);
    void set_state(const WallStakeState &state);
    void set_setpoint(const Rotation2d &new_setpoint);
    Rotation2d get_setpoint();

    AutoCommand *set_setpoint_command(const Rotation2d &new_setpoint);
    AutoCommand *set_state_command(const WallStakeState &new_state);


    /**
     * Function that runs in the background task. This function pointer is passed
     * to the vex::task constructor.
     *
     * @param ptr Pointer to OdometryBase object
     * @return Required integer return code. Unused.
     */
    static int background_task(void *ptr);

    /**
     * End the background task. Cannot be restarted.
     * If the user wants to end the thread but keep the data up to date,
     * they must run the update() function manually from then on.
     */
    void end_async();
    
    bool is_below_handoff();

    void spin(double volts);

    private:
    
    vex::motor_group motors;
    vex::pot pot;

    Rotation2d tolerance;
    Rotation2d setpoint;
    double pot_offset;

    bool end_task;

    vex::task *handle;

    PID wallstake_pid;

};
