#include "../core/include/subsystems/odometry/odometry_base.h"

/**
* Gets the current position and rotation
*/
position_t OdometryBase::get_position(void)
{
    mut.lock();

    // Create a new struct to pass-by-value
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
void OdometryBase::set_position(position_t &newpos)
{
    mut.lock();

    current_pos.x = newpos.x;
    current_pos.y = newpos.y;
    current_pos.rot = newpos.rot;

    mut.unlock();
}

/**
 * End the background task. Cannot be restarted.
 * If the user wants to end the thread but keep the data up to date,
 * they must run the update() function manually from then on.
 */
void OdometryBase::end_async()
{
    this->end_task = true;
}

/**
 * Get the distance between two points
 */
double OdometryBase::pos_diff(position_t &pos1, position_t &pos2)
{
    // Use the pythagorean theorem
    return sqrt(pow(pos2.x - pos1.x, 2) + pow(pos2.y - pos1.y, 2));
}

/**
 * Get the change in rotation between two points
 */
double OdometryBase::rot_diff(position_t &pos1, position_t &pos2)
{
    return pos2.rot - pos1.rot;
}