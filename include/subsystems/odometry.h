#ifndef _ODOMETRY_
#define _ODOMETRY_

#include "vex.h"
#include "../core/include/utils/vector.h"

#define DOWNTIME 50 //milliseconds

#ifndef PI
#define PI 3.141592654
#endif

/**
 * The background task constantly polling the motors and updating the position.
 */
static int background_task(void* pos_ptr);

// Describes a single position and rotation
typedef struct
{
    double x;
    double y;
    double rot;
} position_t;

typedef struct
{
    double dist_between_wheels;
    double wheel_diam;
} odometry_config_t;

// Contains information stored between update()s required for calculating posiion
typedef struct
{
    position_t pos;
    double lside, rside;
} stored_info_t;

class Odometry
{
    public:

    /**
     * Initialize the Odometry module, using the IMU to get rotation
     * @param left_side The left motors 
     * @param right_side The right motors
     * @param imu The robot's inertial sensor
     * @param is_async If true, the robot will automatically poll it's position and update it in the background.
     *      If false, the update() function must be called periodically.
     */
    Odometry(vex::motor_group &left_side, vex::motor_group &right_side, vex::inertial &imu, odometry_config_t &config, bool is_async);

    /**
     * Initialize the Odometry module, calculating the rotation from encoders
     * @param left_side The left motors 
     * @param right_side The right motors
     * @param imu The robot's inertial sensor
     * @param is_async If true, the robot will automatically poll it's position and update it in the background.
     *      If false, the update() function must be called periodically.
     */
    Odometry(vex::motor_group &left_side, vex::motor_group &right_side, odometry_config_t &config, bool is_async);

    /**
     * Gets the current position and rotation
     */
    position_t get_position();

    /**
     * Sets the current position of the robot
     */
    void set_position(position_t &newpos);

    /**
     * Update the current position on the field based on the sensors
     */
    position_t update();

    void end_async();

    bool end_task = false;

    /**
     * Get the distance between two points
     */
    static double pos_diff(position_t &pos1, position_t &pos2);

    /**
     * Get the change in rotation between two points
     */
    static double rot_diff(position_t &pos1, position_t &pos2);

    private:

    /**
     * Get information from the input hardware and an existing position, and calculate a new current position
     */
    static position_t calculate_new_pos(odometry_config_t &config, stored_info_t &stored_info, double lside_revs, double rside_revs, double *gyro_angle_deg=NULL);

    vex::mutex mut;
    position_t current_pos;

    vex::motor_group &left_side, &right_side;
    vex::inertial *imu;
    odometry_config_t &config;

    vex::task *handle;

    stored_info_t stored_info;
    
};

#endif