#include "../core/include/subsystems/odometry/odometry_tank.h"

/**
 * Initialize the Odometry module, calculating position from the drive motors.
 * @param left_side The left motors 
 * @param right_side The right motors
 * @param imu The robot's inertial sensor. If not included, rotation is calculated from the encoders.
 */
OdometryTank::OdometryTank(vex::motor_group &left_side, vex::motor_group &right_side, robot_specs_t &config, vex::inertial *imu, bool is_async)
: left_side(&left_side), right_side(&right_side), left_enc(NULL), right_enc(NULL), imu(imu), config(config)
{
    // Make sure the last known info starts zeroed
    memset(&current_pos, 0, sizeof(position_t));

    // Start the asynchronous background thread
    if (is_async)
        handle = new vex::task(background_task, this);
}

/**
 * Initialize the Odometry module, calculating posiiton from encoders on "dead wheels"
 * @param left_side The left motors 
 * @param right_side The right motors
 * @param imu The robot's inertial sensor. If not included, rotation is calculated from the encoders.
 */
OdometryTank::OdometryTank(CustomEncoder &left_enc, CustomEncoder &right_enc, robot_specs_t &config, vex::inertial *imu, bool is_async)
: left_side(NULL), right_side(NULL), left_enc(&left_enc), right_enc(&right_enc), imu(imu), config(config)
{
    // Make sure the last known info starts zeroed
    memset(&current_pos, 0, sizeof(position_t));

    // Start the asynchronous background thread
    if (is_async)
        handle = new vex::task(background_task, this);
}

/**
 * Resets the position and rotational data to the input.
 * 
 */
void OdometryTank::set_position(const position_t &newpos)
{
  rotation_offset = newpos.rot - (current_pos.rot - rotation_offset);

  OdometryBase::set_position(newpos);
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

    double lside_revs = 0, rside_revs = 0;

    if(left_side != NULL && right_side != NULL)
    {
      lside_revs = left_side->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
      rside_revs = right_side->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
    }else if(left_enc != NULL && right_enc != NULL)
    {
      lside_revs = left_enc->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
      rside_revs = right_enc->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
    }

    double angle = 0;

    // If the IMU data was passed in, use it for rotational data
    if(imu == NULL)
    {
      // Get the difference in distance driven between the two sides
      // Uses the absolute position of the encoders, so resetting them will result in
      // a bad angle.
      // Get the arclength of the turning circle of the robot
      double distance_diff = (rside_revs - lside_revs) * PI * config.odom_wheel_diam;

      printf("dist_diff: %f, ", distance_diff);

      //Use the arclength formula to calculate the angle. Add 90 to make "0 degrees" to starboard
      angle = ((180.0 / PI) * (distance_diff / config.dist_between_wheels)) + 90;

      printf("angle: %f, ", (180.0 / PI) * (distance_diff / config.dist_between_wheels));

    } else
    {
        // Translate "0 forward and clockwise positive" to "90 forward and CCW negative"
        angle = -imu->rotation(vex::rotationUnits::deg) + 90;
    }

    // Offset the angle, if we've done a set_position
    angle += rotation_offset;

    //Limit the angle betwen 0 and 360. 
    //fmod (floating-point modulo) gets it between -359 and +359, so tack on another 360 if it's negative.
    angle = fmod(angle, 360.0);
    if(angle < 0)
        angle += 360;

    updated_pos = calculate_new_pos(config, current_pos, lside_revs, rside_revs, angle);

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
position_t OdometryTank::calculate_new_pos(robot_specs_t &config, position_t &curr_pos, double lside_revs, double rside_revs, double angle_deg)
{
    position_t new_pos;

    static double stored_lside_revs = lside_revs;
    static double stored_rside_revs = rside_revs;

    // Convert the revolutions into "change in distance", and average the values for a "distance driven"
    double lside_diff = (lside_revs - stored_lside_revs) * PI * config.odom_wheel_diam;
    double rside_diff = (rside_revs - stored_rside_revs) * PI * config.odom_wheel_diam;
    double dist_driven = (lside_diff + rside_diff) / 2.0;

    double angle = angle_deg * PI / 180.0; // Degrees to radians

    // Create a vector from the change in distance in the current direction of the robot
    Vector chg_vec(angle, dist_driven);
    
    // Create a vector from the current position in reference to X,Y=0,0
    Vector::point_t curr_point = {.x = curr_pos.x, .y = curr_pos.y};
    Vector curr_vec(curr_point);

    // Tack on the "difference" vector to the current vector
    Vector new_vec = curr_vec + chg_vec;

    new_pos.x = new_vec.get_x();
    new_pos.y = new_vec.get_y();
    new_pos.rot = angle_deg;

    // Store the left and right encoder values to find the difference in the next iteration
    stored_lside_revs = lside_revs;
    stored_rside_revs = rside_revs;

    return new_pos;
}

double OdometryTank::get_speed()
{
  static position_t last_pos = get_position();
  static timer speed_tmr;

  double out = 0;
  if(speed_tmr.value() != 0)
  {
    out = pos_diff(last_pos, get_position()) / speed_tmr.time(timeUnits::sec);
    speed_tmr.reset();
  }
  last_pos = get_position();
  return out;
}