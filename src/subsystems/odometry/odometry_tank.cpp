#include "core/subsystems/odometry/odometry_tank.h"

/**
 * Initialize the Odometry module, calculating position from the drive motors.
 * @param left_side The left motors
 * @param right_side The right motors
 * @param config the specifications that supply the odometry with descriptions of the robot. See robot_specs_t for what
 * is contained
 * @param imu The robot's inertial sensor. If not included, rotation is calculated from the encoders.
 * @param is_async If true, position will be updated in the background continuously. If false, the programmer will have
 * to manually call update().
 */
OdometryTank::OdometryTank(
  vex::motor_group &left_side, vex::motor_group &right_side, robot_specs_t &config, vex::inertial *imu, bool is_async
)
    : OdometryBase(is_async), left_side(&left_side), right_side(&right_side), left_custom_enc(NULL),
      right_custom_enc(NULL), left_vex_enc(NULL), right_vex_enc(NULL), imu(imu), config(config) {}

/**
 * Initialize the Odometry module, calculating position from the drive motors.
 * @param left_custom_enc The left custom encoder
 * @param right_custom_enc The right custom encoder
 * @param config the specifications that supply the odometry with descriptions of the robot. See robot_specs_t for what
 * is contained
 * @param imu The robot's inertial sensor. If not included, rotation is calculated from the encoders.
 * @param is_async If true, position will be updated in the background continuously. If false, the programmer will have
 * to manually call update().
 */
OdometryTank::OdometryTank(
  CustomEncoder &left_custom_enc, CustomEncoder &right_custom_enc, robot_specs_t &config, vex::inertial *imu,
  bool is_async
)
    : OdometryBase(is_async), left_side(NULL), right_side(NULL), left_custom_enc(&left_custom_enc),
      right_custom_enc(&right_custom_enc), left_vex_enc(NULL), right_vex_enc(NULL), imu(imu), config(config) {}

/**
 * Initialize the Odometry module, calculating position from the drive motors.
 * @param left_vex_enc The left vex encoder
 * @param right_vex_enc The right vex encoder
 * @param config the specifications that supply the odometry with descriptions of the robot. See robot_specs_t for what
 * is contained
 * @param imu The robot's inertial sensor. If not included, rotation is calculated from the encoders.
 * @param is_async If true, position will be updated in the background continuously. If false, the programmer will have
 * to manually call update().
 */
OdometryTank::OdometryTank(
  vex::encoder &left_vex_enc, vex::encoder &right_vex_enc, robot_specs_t &config, vex::inertial *imu, bool is_async
)
    : OdometryBase(is_async), left_side(NULL), right_side(NULL), left_custom_enc(NULL), right_custom_enc(NULL),
      left_vex_enc(&left_vex_enc), right_vex_enc(&right_vex_enc), imu(imu), config(config) {}

/**
 * Resets the position and rotational data to the input.
 *
 */
void OdometryTank::set_position(const Pose2d &newpos) {
    mut.lock();
    rotation_offset = newpos.rotation().degrees() - (current_pos.rotation().degrees() - rotation_offset);
    mut.unlock();

    OdometryBase::set_position(newpos);
}

/**
 * Update, store and return the current position of the robot. Only use if not initializing
 * with a separate thread.
 */
Pose2d OdometryTank::update() {
    double lside_revs = 0, rside_revs = 0;

    if (left_side != NULL && right_side != NULL) {
        lside_revs = left_side->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
        rside_revs = right_side->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
    } else if (left_custom_enc != NULL && right_custom_enc != NULL) {
        lside_revs = left_custom_enc->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
        rside_revs = right_custom_enc->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
    } else if (left_vex_enc != NULL && right_vex_enc != NULL) {
        lside_revs = left_vex_enc->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
        rside_revs = right_vex_enc->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
    }

    double angle = 0;
    if (!imu->installed()) {
        // printf("NO IMU DETECTED\n");
    }
    // If the IMU data was passed in, use it for rotational data
    if (imu == NULL || imu->installed() == false) {
        // Get the difference in distance driven between the two sides
        // Uses the absolute position of the encoders, so resetting them will result in
        // a bad angle.
        // Get the arclength of the turning circle of the robot
        double distance_diff = (rside_revs - lside_revs) * PI * config.odom_wheel_diam;

        // Use the arclength formula to calculate the angle.
        angle = ((180.0 / PI) * (distance_diff / config.dist_between_wheels));

    } else {
        // Translate "clockwise positive" to "CCW negative"
        angle = -imu->rotation(vex::rotationUnits::deg);
    }

    // Offset the angle, if we've done a set_position
    angle += rotation_offset;

    // Limit the angle betwen 0 and 360.
    // fmod (floating-point modulo) gets it between -359 and +359, so tack on another 360 if it's negative.
    angle = fmod(angle, 360.0);
    if (angle < 0) {
        angle += 360;
    }

    current_pos = calculate_new_pos(config, current_pos, lside_revs, rside_revs, angle);

    static Pose2d last_pos = current_pos;
    static double last_speed = 0;
    static double last_ang_speed = 0;
    static timer tmr;
    bool update_vel_accel = tmr.time(sec) > 0.02;

    // This loop runs too fast. Only check at LEAST every 1/10th sec
    if (update_vel_accel) {
        // Calculate robot velocity
        double this_speed = current_pos.translation().distance(last_pos.translation()) / tmr.time(sec);
        ema.add_entry(this_speed);
        speed = ema.get_value();
        // Calculate robot acceleration
        accel = (speed - last_speed) / tmr.time(sec);

        // Calculate robot angular velocity (deg/sec)
        ang_speed_deg = smallest_angle(current_pos.rotation().degrees(), last_pos.rotation().degrees()) / tmr.time(sec);

        // Calculate robot angular acceleration (deg/sec^2)
        ang_accel_deg = (ang_speed_deg - last_ang_speed) / tmr.time(sec);

        tmr.reset();
        last_pos = current_pos;
        last_speed = speed;
        last_ang_speed = ang_speed_deg;
    }

    return current_pos;
}

/**
 * Using information about the robot's mechanical structure and sensors, calculate a new position
 * of the robot, relative to when this method was previously ran.
 */
Pose2d OdometryTank::calculate_new_pos(
  robot_specs_t &config, Pose2d &curr_pos, double lside_revs, double rside_revs, double angle_deg
) {
    Pose2d new_pos(0, 0, 0);

    static double stored_lside_revs = lside_revs;
    static double stored_rside_revs = rside_revs;

    // Convert the revolutions into "change in distance", and average the values for a "distance driven"
    double lside_diff = (lside_revs - stored_lside_revs) * PI * config.odom_wheel_diam;
    double rside_diff = (rside_revs - stored_rside_revs) * PI * config.odom_wheel_diam;
    double dist_driven = (lside_diff + rside_diff) / 2.0;

    double angle = angle_deg * PI / 180.0; // Degrees to radians

    // Create a vector from the change in distance in the current direction of the robot
    // deg2rad((smallest_angle(curr_pos.rot, angle_deg)/2 + curr_pos.rot, dist_driven)
    Translation2d chg_point(dist_driven, Rotation2d(angle));

    // Create a vector from the current position in reference to X,Y=0,0
    Translation2d curr_point(curr_pos.x(), curr_pos.y());

    // Tack on the "difference" vector to the current vector
    Translation2d new_point = curr_point + chg_point;
    new_pos = Pose2d(new_point, from_degrees(angle_deg));

    // Store the left and right encoder values to find the difference in the next iteration
    stored_lside_revs = lside_revs;
    stored_rside_revs = rside_revs;

    return new_pos;
}