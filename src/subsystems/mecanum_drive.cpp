#include "../core/include/subsystems/mecanum_drive.h"
#include "../core/include/utils/vector2d.h"
#include "../core/include/utils/math_util.h"

/**
* Create the Mecanum drivetrain object
*/
MecanumDrive::MecanumDrive(vex::motor &left_front, vex::motor &right_front, vex::motor &left_rear, vex::motor &right_rear, 
                           vex::rotation *lateral_wheel, vex::inertial *imu, mecanumdrive_config_t *config)
: left_front(left_front), right_front(right_front), left_rear(left_rear), right_rear(right_rear), // MOTOR CONFIG
  config(config), // CONFIG ...uh... config
  lateral_wheel(lateral_wheel), // NON-DRIVEN WHEEL CONFIG
  imu(imu) //IMU CONFIG
{

  // If the configuration exists, then allocate memory for the drive and turn pids
  if(config != NULL)
  {
    drive_pid = new PID(config->drive_pid_conf);
    drive_gyro_pid = new PID(config->drive_gyro_pid_conf);
    turn_pid = new PID(config->turn_pid_conf);
  }

}

/**
  * Drive the robot using vectors. This handles all the math required for mecanum control.
  *
  * @param direction_deg  the direction to drive the robot, in degrees. 0 is forward,
  *                       180 is back, clockwise is positive, counterclockwise is negative.
  * @param magnitude      How fast the robot should drive, in percent: 0.0->1.0
  * @param rotation       How fast the robot should rotate, in percent: -1.0->+1.0
  */
void MecanumDrive::drive_raw(double direction_deg, double magnitude, double rotation)
{
  double direction = deg2rad(direction_deg);
  
  // ALGORITHM - "rotate" the vector by 45 degrees and apply each corner to a wheel
  // .. Oh, and mix rotation too
  double lf = (magnitude * cos(direction - (PI / 4.0))) + rotation;
  double rf = (magnitude * cos(direction + (PI / 4.0))) - rotation;
  double lr = (magnitude * cos(direction + (PI / 4.0))) + rotation;
  double rr = (magnitude * cos(direction - (PI / 4.0))) - rotation;
  
  // Limit the output between -1.0 and +1.0
  lf = clamp(lf, -1.0, 1.0);
  rf = clamp(rf, -1.0, 1.0);
  lr = clamp(lr, -1.0, 1.0);
  rr = clamp(rr, -1.0, 1.0);

  // Finally, spin the motors
  left_front.spin(vex::directionType::fwd, lf * 100.0, vex::velocityUnits::pct);
  right_front.spin(vex::directionType::fwd, rf * 100.0, vex::velocityUnits::pct);
  left_rear.spin(vex::directionType::fwd, lr * 100.0, vex::velocityUnits::pct);
  right_rear.spin(vex::directionType::fwd, rr * 100.0, vex::velocityUnits::pct);
}

/**
 * Drive the robot with a mecanum-style / arcade drive. Inputs are in percent (-100.0 -> 100.0) straight from the controller.
 * Controls are mixed, so the robot can drive forward / strafe / rotate all at the same time. 
 *
 * @param left_y left joystick, Y axis (forward / backwards)
 * @param left_x left joystick, X axis (strafe left / right)
 * @param right_x right joystick, X axis (rotation left / right)
 * @param power = 2 how much of a "curve" there should be on drive controls; better for low speed maneuvers.
 *                Leave blank for a default curve of 2 (higher means more fidelity)
 */
void MecanumDrive::drive(double left_y, double left_x, double right_x, int power)
{
  // LATERAL CONTROLS - convert cartesion to a vector
  double magnitude = sqrt(pow(left_y / 100.0, 2) + pow(left_x / 100.0, 2));
  magnitude = pow(magnitude, power);
  
  double direction = atan2(left_x / 100.0, left_y / 100.0);

  // ROTATIONAL CONTROLS - just the right x joystick
  double rotation = right_x / 100.0;

  //  
  rotation = sign(rotation) * fabs(pow(rotation, power));
  
  return this->drive_raw(rad2deg(direction), magnitude, rotation);  
}

/**
  * Drive the robot in a straight line automatically.
  * If the inertial was declared in the constructor, use it to correct while driving.
  * If the lateral wheel was declared in the constructor, use it for more accurate positioning while strafing.
  *
  * @param inches   How far the robot should drive, in inches
  * @param direction    What direction the robot should travel in, in degrees.
  *                     0 is forward, +/-180 is reverse, clockwise is positive.
  * @param speed    The maximum speed the robot should travel, in percent: -1.0->+1.0
  * @param gyro_correction = true   Whether or not to use the gyro to help correct while driving.
  *                               Will always be false if no gyro was declared in the constructor.
  * @return Whether or not the maneuver is complete.
  */
bool MecanumDrive::auto_drive(double inches, double direction, double speed, bool gyro_correction)
{
  if(config == NULL || drive_pid == NULL)
  {
    fprintf(stderr, "Failed to run MecanumDrive::auto_drive - Missing mecanumdrive_config_t in constructor\n");
    return true; // avoid an infinte loop within auto
  }

  bool enable_gyro = gyro_correction && (imu != NULL);
  bool enable_wheel = (lateral_wheel != NULL);


  // INITIALIZE - only run ONCE "per drive" on startup
  if(init == true)
  {

    // Reset all driven encoders, and PID
    left_front.resetPosition();
    right_front.resetPosition();
    left_rear.resetPosition();
    right_rear.resetPosition();

    drive_pid->reset();

    // Reset only if gyro exists
    if(enable_gyro)
    {
      imu->resetRotation();
      drive_gyro_pid->reset();
      drive_gyro_pid->set_target(0.0);
    }
    // reset only if lateral wheel exists
    if(enable_wheel)
      lateral_wheel->resetPosition();

    // Finish setting up the PID loop - max speed and position target
    drive_pid->set_limits(-fabs(speed), fabs(speed));
    drive_pid->set_target(fabs(inches));

    init = false;
  }

  double dist_avg = 0.0;
  double drive_avg = 0.0;

  // This algorithm should be DEFINITELY good for forward/back, left/right.
  // Directions other than 0, 180, -90 and 90 will be hit or miss, but should be mostly right.
  // Recommend THOUROUGH testing at many angles.

  // IF in quadrant 1 or 3, use left front and right rear wheels as "drive" movement
  // ELSE in quadrant 2 or 4, use left rear and right front wheels as "drive" movement
  // Some wheels are NOT being averaged at any given time since the general mecanum algorithm makes them go slower than our robot speed
  // somewhat of a nasty hack, but wheel slippage should make up for it, and multivariable calc is hard.
  if( (direction > 0 && direction <= 90) || (direction < -90 && direction > -180))
  {
    drive_avg = fabs(left_front.position(rotationUnits::rev) * config->drive_wheel_diam * PI)
              + fabs(right_rear.position(rotationUnits::rev) * config->drive_wheel_diam * PI)
              / 2.0;
  } else
  {
    drive_avg = fabs(left_rear.position(rotationUnits::rev) * config->drive_wheel_diam * PI)
              + fabs(right_front.position(rotationUnits::rev) * config->drive_wheel_diam * PI)
              / 2.0;
  }

  // Only use the encoder wheel if it exists.
  // Without the wheel should be usable, but with it will be muuuuch more accurate.
  if(enable_wheel)
  {
    // Distance driven = Magnitude = sqrt(x^2 + y^2)
    // Since drive_avg is already a polar magnitude, turn it into "Y" with cos(theta)
    dist_avg = sqrt(
      pow(lateral_wheel->position(rotationUnits::rev) * config->lateral_wheel_diam * PI, 2)
      + pow(drive_avg * cos(direction * (PI / 180.0)), 2)
    );
  }else
  {
    dist_avg = drive_avg;
  }

  // ...double check to avoid an infinite loop
  dist_avg = fabs(dist_avg);


  // ROTATION CORRECTION
  double rot = 0;

  if(enable_gyro)
  {
    drive_gyro_pid->update(imu->rotation());
    rot = drive_gyro_pid->get();
  }

  // Update the PID and drive
  drive_pid->update(drive_avg);

  this->drive_raw(direction, drive_pid->get(), rot);

  // Stop and return true whenever the robot has completed it's drive.
  if(drive_pid->is_on_target())
  {
    drive_raw(0, 0, 0);
    init = true;
    return true;
  }

  // Return false while the robot is still driving.
  return false;
}

/**
* Autonomously turn the robot X degrees over it's center point. Uses a closed loop
* for control.
* @param degrees How many degrees to rotate the robot. Clockwise postive.
* @param speed What percentage to run the motors at: 0.0 -> 1.0
* @param ignore_imu = false Whether or not to use the Inertial for determining angle.
*        Will instead use circumference formula + robot's wheelbase + encoders to determine.
* 
* @return whether or not the robot has finished the maneuver
*/
bool MecanumDrive::auto_turn(double degrees, double speed, bool ignore_imu)
{
  // Make sure the configurations exist before continuing
  if(config == NULL || turn_pid == NULL)
  {
    fprintf(stderr, "Failed to run MecanumDrive::auto_turn - Missing mecanumdrive_config_t in constructor\n");
    return true;
  }

  // Decide whether or not to use the Inertial
  ignore_imu = ignore_imu || (this->imu == NULL);

  // INITIALIZE - clear encoders / imu / pid loops
  if(init == true)
  {
    if(ignore_imu)
    {
      this->left_front.resetPosition();
      this->right_front.resetPosition();
      this->left_rear.resetPosition();
      this->right_rear.resetPosition();
    }else
    {
      this->imu->resetRotation();
    }

    this->turn_pid->reset();
    this->turn_pid->set_limits(-fabs(speed), fabs(speed));
    this->turn_pid->set_target(degrees);

    init = false;
  }

  // RUN PERIODICALLY

  double current_angle = 0.0;

  if(ignore_imu)
  {
    double avg = (left_front.position(rotationUnits::rev) + left_rear.position(rotationUnits::rev) 
                - right_front.position(rotationUnits::rev) - right_rear.position(rotationUnits::rev)) / 4.0;

    // Current arclength = (avg * wheel_diam * PI) = (theta * (wheelbase / 2.0)). then convert to degrees
    current_angle = (360.0 * avg * config->drive_wheel_diam) / config->wheelbase_width;
  } else
  {
    current_angle = imu->rotation();
  }

  this->turn_pid->update(current_angle);
  this->drive_raw(0, 0, turn_pid->get());

  // We have reached the target.
  if(this->turn_pid->is_on_target())
  {
    this->drive_raw(0, 0, 0);
    init = true;
    return true;
  }

  return false;
}