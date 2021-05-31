#include "../core/include/subsystems/odometry.h"

/**
 * Initialize the Odometry module, using the IMU to get rotation
 * @param left_side The left motors 
 * @param right_side The right motors
 * @param imu The robot's inertial sensor
 */
Odometry::Odometry(vex::motor_group &left_side, vex::motor_group &right_side, vex::inertial &imu, odometry_config_t &config)
: left_side(&left_side), right_side(&right_side), imu(&imu), config(&config)
{
    // Create a raw pointer containing a list of all the hardware pointers needed to run the thread.
    // This is done because callbacks do not allow class member functions
    void* list[] = {this->left_side, this->right_side, this->imu, this->config, &mut, &current_pos};

    // Initialize and start the background task
    handle = new vex::task(background_task, list);
}

/**
 * Initialize the Odometry module, calculating the rotation from encoders
 * @param left_side The left motors 
 * @param right_side The right motors
 * @param imu The robot's inertial sensor
 */
Odometry::Odometry(vex::motor_group &left_side, vex::motor_group &right_side, odometry_config_t &config)
: left_side(&left_side), right_side(&right_side), imu(NULL), config(&config)
{
    // Create a raw pointer containing a list of all the hardware pointers needed to run the thread.
    // This is done because callbacks do not allow class member functions
    void* list[] = {this->left_side, this->right_side, this->imu, this->config, &mut, &current_pos};

    // Initialize and start the background task
    handle = new vex::task(background_task, list);
 
}

/**
 * The background task constantly polling the motors and updating the position.
 */
int background_task(void* obj_ptrs)
{
    // Parse the hardware passed into the function
    void** list = (void**) obj_ptrs;

    // Dereference the pointers and store as object references to make it cleaner later
    vex::motor_group &left_side = *((vex::motor_group*) list[0]);
    vex::motor_group &right_side = *((vex::motor_group*) list[1]);
    vex::inertial *imu = (vex::inertial*) list[2]; // (keep as pointer to check for NULL)
    odometry_config_t &config = *((odometry_config_t*) list[3]);
    vex::mutex &mut = *((vex::mutex*) list[4]);
    position_t &current_pos = *((position_t*) list[5]);

    double last_lside = 0, last_rside = 0;

    while(true)
    {
        double angle = 0;

        // If the IMU was passed in, use it for rotational data
        if(imu != NULL)
            angle = imu->heading();
        else
        {
            // Get the difference in distance driven between the two sides
            double distance_diff = (left_side.position(rotationUnits::rev) - right_side.position(rotationUnits::rev)) * PI * config.wheel_diam;

            //Use the arclength formula to calculate the angle.
            angle = (180.0 / PI) * (distance_diff / config.dist_between_wheels);
            
            //Limit the angle betwen 0 and 360. 
            //fmod (floating-point modulo) gets it between -359 and +359, so tack on another 360 if it's negative.
            angle = fmod(angle, 360.0);
            if(angle < 0)
                angle += 360;
        }

        angle *= PI / 180.0; // Degrees to radians

        // Get the encoder values in revolutions (Make sure the gearsetting is set)
        double curr_lside = left_side.position(rotationUnits::rev);
        double curr_rside = right_side.position(rotationUnits::rev);

        // Convert the revolutions into "change in distance", and average the values for a "distance driven"
        double lside_diff = (curr_lside - last_lside) * config.wheel_diam;
        double rside_diff = (curr_rside - last_rside) * config.wheel_diam;
        double dist_driven = (lside_diff + rside_diff) / 2.0;

        // Create a vector from the change in distance in the current direction of the robot
        Vector chg_vec(angle, dist_driven);
        
        // Create a vector from the current position in reference to X,Y=0,0
        Vector::point_t curr_point = {.x = current_pos.x, .y = current_pos.y};
        Vector curr_vec(curr_point);

        // Tack on the "difference" vector to the current vector
        Vector new_vec = curr_vec + chg_vec;

        // Store the left and right encoder values to find the difference in the next iteration
        last_lside = curr_lside;
        last_rside = curr_rside;

        // Avoid race conditions by locking the mutex
        mut.lock();

        // Get the X and Y coordinates from the vector and save them to the "current position"
        current_pos.x = new_vec.get_x();
        current_pos.y = new_vec.get_y();
        current_pos.rot = angle;

        mut.unlock();

        vexDelay(DOWNTIME);
    }
}

/**
 * Gets the current position and rotation
 */
position_t Odometry::get_position()
{
    mut.lock();

    position_t out = 
    {
        .x = current_pos.x,
        .y = current_pos.y,
        .rot = current_pos.rot
    };

    mut.unlock();

    return out;   
}

/**
 * Sets the current position of the robot
 */
void Odometry::set_position(position_t &newpos)
{
    mut.lock();

    current_pos.x = newpos.x;
    current_pos.y = newpos.y;
    current_pos.rot = newpos.rot;

    mut.unlock();
}

/**
 * Get the distance between two points
 */
double Odometry::pos_diff(position_t &pos1, position_t &pos2)
{
    // Use the pythagorean theorem
    return sqrt(pow(pos2.x - pos1.x, 2) + pow(pos2.y - pos1.y, 2));

}

/**
 * Get the change in rotation between two points
 */
double Odometry::rot_diff(position_t &pos1, position_t &pos2)
{
    return pos2.rot - pos1.rot;
}

