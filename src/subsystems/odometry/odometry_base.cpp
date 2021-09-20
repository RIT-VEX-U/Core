#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/utils/vector.h"

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
void OdometryBase::set_position(const position_t &newpos)
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
double OdometryBase::pos_diff(position_t pos1, position_t pos2, bool use_negatives)
{
    int negative_multiplier = 1;

    // If we are using negative distances, define a "negative distance" as when the angle
    // of a vector made from those two points is between 3PI/4 and 7PI/4
    if(use_negatives)
    {
        Vector::point_t point_diff = {
            .x = pos1.x - pos2.x,
            .y = pos1.y - pos2.y
        };

        Vector v_diff(point_diff);
        double angle = rad2deg(v_diff.get_dir());

        if(angle > 135 && angle < 315)
            negative_multiplier = -1;
    }

    // Use the pythagorean theorem
    return negative_multiplier * sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
}

/**
 * Get the change in rotation between two points
 */
double OdometryBase::rot_diff(position_t &pos1, position_t &pos2)
{
    return pos2.rot - pos1.rot;
}

/**
 * Get the smallest difference in angle between a start heading and end heading.
 * Returns the difference between -180 degrees and +180 degrees, representing the robot
 * turning left or right, respectively.
 */
double OdometryBase::smallest_angle(double start_deg, double end_deg)
{
  double retval;
  // get the difference between 0 and 360
  retval = fmod(end_deg - start_deg, 360.0);
  if(retval < 0)
    retval += 360.0;

  // Get the closest angle, now between -180 (turn left) and +180 (turn right)
  if(retval > 180)
    retval -= 360;

  return retval;
}