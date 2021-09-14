#include "../core/include/subsystems/odometry/odometry_swerve.h"
#include "../core/include/utils/vector.h"

OdometrySwerve::OdometrySwerve(SwerveDrive &drive_system, vex::inertial &imu, bool is_async)
: drive_system(drive_system), imu(imu)
{
    memset(&stored_info, 0, sizeof(stored_info_t));

    if(is_async)
        handle = new vex::task(background_task, this);
}

/**
 * Manually update the position of the robot. Must be called many times each second for an accurate position.
 * Returns the new posiiton.
 */
position_t OdometrySwerve::update()
{
    position_t new_pos = calculate_new_pos(stored_info,current_pos, drive_system, imu);

    mut.lock();
    this->current_pos = new_pos;
    mut.unlock();

    return current_pos;
}

/**
 * Function containing the code running in the background, if this odometry is running 
 * asynchronously.
 */
int background_task(void *obj)
{
    OdometrySwerve &odom = *((OdometrySwerve*)obj);

    while(!odom.end_task)
    {
        odom.update();
        vexDelay(DOWNTIME);
    }

    return 0;
}

/**
 * Run the calculations required to find a new position based on a stored old position, stored information
 * on each swerve module, and information from the sensors stored in the drive_system / imu
 */
position_t OdometrySwerve::calculate_new_pos(stored_info_t &stored_info, position_t &cur_pos, SwerveDrive &drive_system, vex::inertial &imu)
{
    // Get the change in distance for each drive motor
    double lf_delta = drive_system.get_module(LEFT_FRONT).get_distance_driven() - stored_info.lf_dist;
    double rf_delta = drive_system.get_module(RIGHT_FRONT).get_distance_driven() - stored_info.rf_dist;
    double lr_delta = drive_system.get_module(LEFT_REAR).get_distance_driven() - stored_info.lr_dist;
    double rr_delta = drive_system.get_module(RIGHT_REAR).get_distance_driven() - stored_info.rr_dist;

    // Store the vector displacement
    Vector lf_disp(drive_system.get_module(LEFT_FRONT).get_module_angle(), lf_delta);
    Vector rf_disp(drive_system.get_module(RIGHT_FRONT).get_module_angle(), rf_delta);
    Vector lr_disp(drive_system.get_module(LEFT_REAR).get_module_angle(), lr_delta);
    Vector rr_disp(drive_system.get_module(RIGHT_REAR).get_module_angle(), rr_delta);

    // Get an overall displacement of the robot
    // Adding the vectors cancels out rotation, and leaves the robots actual displacement
    Vector delta_displacement = lf_disp + rf_disp + lr_disp + rr_disp;

    // Create a vector from the robot's current position,
    // representing the displacement from the origin
    Vector::point_t pt = {.x = cur_pos.x, .y = cur_pos.y};
    Vector old_displacement(pt);

    // Add the "change in displacement" to the existing displacement (from 0,0)
    Vector new_displacement = old_displacement + delta_displacement;

    position_t out = 
    {
    .x = new_displacement.get_x(),
    .y = new_displacement.get_y(),
    .rot = imu.heading()
    };

    // Store each module's previous distance driven for the next update
    stored_info.lf_dist = drive_system.get_module(LEFT_FRONT).get_distance_driven();
    stored_info.rf_dist = drive_system.get_module(RIGHT_FRONT).get_distance_driven();
    stored_info.lr_dist = drive_system.get_module(LEFT_REAR).get_distance_driven();
    stored_info.rr_dist = drive_system.get_module(RIGHT_REAR).get_distance_driven();

    return out;
}