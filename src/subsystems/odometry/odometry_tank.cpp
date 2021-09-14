#include "../core/include/subsystems/odometry/odometry_tank.h"

/**
 * Initialize the Odometry module, using the IMU to get rotation
 * @param left_side The left motors 
 * @param right_side The right motors
 * @param imu The robot's inertial sensor
 */
OdometryTank::OdometryTank(vex::motor_group &left_side, vex::motor_group &right_side, vex::inertial &imu, odometry_config_t &config, bool is_async)
: left_side(left_side), right_side(right_side), imu(&imu), config(config)
{
    // Make sure the last known info starts zeroed
    memset(&stored_info, 0, sizeof(stored_info_t));

    // Start the asynchronous background thread
    if (is_async)
        handle = new vex::task(background_task, this);
}

/**
 * Initialize the Odometry module, calculating the rotation from encoders
 * @param left_side The left motors 
 * @param right_side The right motors
 * @param imu The robot's inertial sensor
 */
OdometryTank::OdometryTank(vex::motor_group &left_side, vex::motor_group &right_side, odometry_config_t &config, bool is_async)
: left_side(left_side), right_side(right_side), imu(NULL), config(config)
{
    // Make sure the last known info starts zeroed
    memset(&stored_info, 0, sizeof(stored_info_t));

    // Start the asynchronous background thread
    if (is_async)
        handle = new vex::task(background_task, this);
 
}

/**
 * The background task constantly polling the motors and updating the position.
 */
int background_task(void* odom_obj)
{
    OdometryTank &odom = *((OdometryTank*) odom_obj);
    while(!odom.end_task)
    {
        odom.update();

        vexDelay(DOWNTIME);
    }

    return 0;
}

/**
 * Update, store and return the current position of the robot. Only use if not initializing
 * with a separate thread.
 */
position_t OdometryTank::update()
{
    position_t updated_pos;

    double lside_revs = left_side.position(vex::rotationUnits::rev) / config.gear_ratio;
    double rside_revs = right_side.position(vex::rotationUnits::rev) / config.gear_ratio;

    if(imu == NULL)
    {
        updated_pos = calculate_new_pos(config, stored_info, lside_revs, rside_revs);
    } else
    {
        double angle = imu->rotation(vex::rotationUnits::deg);
        updated_pos = calculate_new_pos(config, stored_info, lside_revs, rside_revs, &angle);
    }
    
    // Update the class' stored position
    mut.lock();
    this->current_pos = updated_pos;
    mut.unlock();

    return updated_pos;
}

/**
 * Using information about the robot's mechanical structure and sensors, calculate a new position
 * of the robot, relative to when this method was previously ran.
 */
position_t OdometryTank::calculate_new_pos(odometry_config_t &config, stored_info_t &stored_info, double lside_revs, double rside_revs, double *gyro_angle_deg)
{
    position_t new_pos;

    double angle = 0;

        // If the IMU data was passed in, use it for rotational data
        if(gyro_angle_deg != NULL)
            angle = *gyro_angle_deg;
        else
        {
            // Get the difference in distance driven between the two sides
            // Uses the absolute position of the encoders, so resetting them will result in
            // a bad angle.
            double distance_diff = (lside_revs - rside_revs) * PI * config.wheel_diam;

            //Use the arclength formula to calculate the angle.
            angle = (180.0 / PI) * (distance_diff / config.dist_between_wheels);
            
            //Limit the angle betwen 0 and 360. 
            //fmod (floating-point modulo) gets it between -359 and +359, so tack on another 360 if it's negative.
            angle = fmod(angle, 360.0);
            if(angle < 0)
                angle += 360;
        }

        angle *= PI / 180.0; // Degrees to radians

        // Convert the revolutions into "change in distance", and average the values for a "distance driven"
        double lside_diff = (lside_revs - stored_info.lside) * PI * config.wheel_diam;
        double rside_diff = (rside_revs - stored_info.rside) * PI * config.wheel_diam;
        double dist_driven = (lside_diff + rside_diff) / 2.0;

        // Create a vector from the change in distance in the current direction of the robot
        Vector chg_vec(angle, dist_driven);
        
        // Create a vector from the current position in reference to X,Y=0,0
        Vector::point_t curr_point = {.x = stored_info.pos.x, .y = stored_info.pos.y};
        Vector curr_vec(curr_point);

        // Tack on the "difference" vector to the current vector
        Vector new_vec = curr_vec + chg_vec;

        new_pos.x = new_vec.get_x();
        new_pos.y = new_vec.get_y();
        new_pos.rot = angle * (180.0 / PI);

        // Store the left and right encoder values to find the difference in the next iteration
        stored_info.lside = lside_revs;
        stored_info.rside = rside_revs;
        stored_info.pos = new_pos;

    return new_pos;
}