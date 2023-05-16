#pragma once

#include "vex.h"
#include "../core/include/utils/geometry.h"
#include "../core/include/robot_specs.h"

#ifndef PI
#define PI 3.141592654
#endif



/**
 * OdometryBase
 * 
 * This base class contains all the shared code between different implementations of odometry.
 * It handles the asynchronous management, position input/output and basic math functions, and 
 * holds positional types specific to field orientation. 
 * 
 * All future odometry implementations should extend this file and redefine update() function. 
 * 
 * @author Ryan McGee
 * @date Aug 11 2021
 */
class OdometryBase
{
public:

    /**
     * Construct a new Odometry Base object
     * 
     * @param is_async True to run constantly in the background, false to call update() manually
     */
    OdometryBase(bool is_async);

    /**
    * Gets the current position and rotation
    * @return the position that the odometry believes the robot is at
    */
    pose_t get_position(void);

    /**
     * Sets the current position of the robot
     * @param newpos the new position that the odometry will believe it is at
     */
    virtual void set_position(const pose_t& newpos=zero_pos);

    /**
     * Update the current position on the field based on the sensors
     * @return the location that the robot is at after the odometry does its calculations
     */
    virtual pose_t update() = 0;

    /**
     * Function that runs in the background task. This function pointer is passed
     * to the vex::task constructor. 
     * 
     * @param ptr Pointer to OdometryBase object
     * @return Required integer return code. Unused.
     */
    static int background_task(void* ptr);

    /**
     * End the background task. Cannot be restarted.
     * If the user wants to end the thread but keep the data up to date,
     * they must run the update() function manually from then on.
     */
    void end_async();

    /**
     * Get the distance between two points
     * @param start_pos distance from this point
     * @param end_pos to this point
     * @return the euclidean distance between start_pos and end_pos
     */
    static double pos_diff(pose_t start_pos, pose_t end_pos);

    /**
     * Get the change in rotation between two points
     * @param pos1 position with initial rotation
     * @param pos2 position with final rotation
     * @return change in rotation between pos1 and pos2
     */
    static double rot_diff(pose_t pos1, pose_t pos2);

    /**
     * Get the smallest difference in angle between a start heading and end heading.
     * Returns the difference between -180 degrees and +180 degrees, representing the robot
     * turning left or right, respectively.
     * @param start_deg intitial angle (degrees)
     * @param end_deg final angle (degrees)
     * @return the smallest angle from the initial to the final angle. This takes into account the wrapping of rotations around 360 degrees 
     */
    static double smallest_angle(double start_deg, double end_deg);

    /// @brief end_task is true if we instruct the odometry thread to shut down
    bool end_task = false;

    /**
     * Get the current speed
     * @return the speed at which the robot is moving and grooving (inch/s)
     */
    double get_speed();

    /**
     * Get the current acceleration
     * @return the acceleration rate of the robot (inch/s^2)
     */
    double get_accel();

    /**
     * Get the current angular speed in degrees
     * @return the angular velocity at which we are turning (deg/s) 
     */
    double get_angular_speed_deg();

    /**
     * Get the current angular acceleration in degrees
     * @return the angular acceleration at which we are turning (deg/s^2)
     */
    double get_angular_accel_deg();

    /**
     * Zeroed position. X=0, Y=0, Rotation= 90 degrees
     */
    inline static constexpr pose_t zero_pos = {.x=0.0L, .y=0.0L, .rot=90.0L};

protected:
    /**
     * handle to the vex task that is running the odometry code
    */
    vex::task *handle;

    /**
     * Mutex to control multithreading
     */
    vex::mutex mut;

    /**
     * Current position of the robot in terms of x,y,rotation
     */
    pose_t current_pos;

    double speed; /**< the speed at which we are travelling (inch/s)*/
    double accel; /**< the rate at which we are accelerating (inch/s^2)*/
    double ang_speed_deg; /**< the speed at which we are turning (deg/s)*/
    double ang_accel_deg; /**< the rate at which we are accelerating our turn (deg/s^2)*/
};