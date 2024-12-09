#pragma once
#include "../core/include/subsystems/odometry/odometry_tank.h"
#include "../core/include/subsystems/tank_drive.h"

struct DriveTuningConfig{
    double startspeed;
    double endspeed;
    int distance;
    PID::pid_config_t *pidConfig;
    vex::directionType dir;
    vex::motor_group *right_motors;
    OdometryTank odom;
    TankDrive *driveSys;
};
class AutoTuningTools{
    public:
    TankDrive &driveSys;
    DriveTuningConfig cfg;
    AutoTuningTools(TankDrive *driveSys, DriveTuningConfig cfg) : driveSys(*driveSys), cfg(cfg){};
    void TuneDrivePID();
    void driveWithError();
};