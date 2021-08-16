#ifndef _ODOMETRY_
#define _ODOMETRY_

#include "vex.h"

#ifndef PI
#define PI 3.141592654
#endif

#define DOWNTIME 50 //milliseconds

// Describes a single position and rotation
typedef struct
{
    double x;
    double y;
    double rot;
} position_t;

/**
 * Base odometry class to simplify implementations of multiple drivetrains.
 */
class OdometryBase
{
public:
    /**
    * Gets the current position and rotation
    */
    position_t get_position(void);

    /**
     * Sets the current position of the robot
     */
    void set_position(position_t &newpos);

    /**
     * Update the current position on the field based on the sensors
     */
    virtual position_t update() = 0;

    /**
     * End the background task. Cannot be restarted.
     * If the user wants to end the thread but keep the data up to date,
     * they must run the update() function manually from then on.
     */
    void end_async();

    /**
     * Get the distance between two points
     */
    static double pos_diff(position_t pos1, position_t pos2, bool use_negatives=false);

    /**
     * Get the change in rotation between two points
     */
    static double rot_diff(position_t &pos1, position_t &pos2);

    bool end_task = false;

protected:
    vex::task *handle;
    vex::mutex mut;
    position_t current_pos;
};

#endif