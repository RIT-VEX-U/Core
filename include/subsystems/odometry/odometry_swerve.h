#pragma once

#include "vex.h"
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/subsystems/swerve_drive.h"

class SwerveDrive;

typedef struct
{
    double lf_dist, rf_dist, lr_dist, rr_dist;
} stored_info_t;

static int background_task(void *obj);

/**
 * Odometry controller for a 4x module swerve drivetrain. Can either run in the background or 
 * called manually via the update() function.
 */
class OdometrySwerve : public OdometryBase
{
public:

    /**
     * Create the odometry object, decide whether to keep it in the background or foreground.
     */
    OdometrySwerve(SwerveDrive &drive_system, vex::inertial &imu, bool is_async=true);

    /**
     * Manually update the position of the robot. Must be called many times each second for an accurate position.
     * Returns the new posiiton.
     */
    position_t update();

private:

    /**
     * Run the calculations required to find a new position based on a stored old position, stored information
     * on each swerve module, and information from the sensors stored in the drive_system / imu
     */
    static position_t calculate_new_pos(stored_info_t &stored_info, position_t &cur_pos, SwerveDrive &drive_system, vex::inertial &imu);

    SwerveDrive &drive_system;
    vex::inertial &imu;

    stored_info_t stored_info;
};
