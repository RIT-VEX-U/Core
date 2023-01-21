#pragma once

#include "vex.h"
#include "../core/include/robot_specs.h"

#ifndef PI
#define PI 3.141592654
#endif

// Describes a single position and rotation
typedef struct
{
    double x;
    double y;
    double rot;
} position_t;

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
    */
    position_t get_position(void);

    /**
     * Sets the current position of the robot
     */
    virtual void set_position(const position_t &newpos=zero_pos);

    /**
     * Update the current position on the field based on the sensors
     */
    virtual position_t update() = 0;

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
     */
    static double pos_diff(position_t pos1, position_t pos2);

    /**
     * Get the change in rotation between two points
     */
    static double rot_diff(position_t pos1, position_t pos2);

    /**
     * Get the smallest difference in angle between a start heading and end heading.
     * Returns the difference between -180 degrees and +180 degrees, representing the robot
     * turning left or right, respectively.
     */
    static double smallest_angle(double start_deg, double end_deg);

    bool end_task = false;

    double get_speed();
    double get_accel();

    inline static constexpr position_t zero_pos = {.x=0.0L, .y=0.0L, .rot=90.0L};

protected:
    vex::task *handle;
    vex::mutex mut;
    position_t current_pos;

    double speed, accel;
};