#include "../core/include/subsystems/swerve_module.h"


/**
 * Create a single swerve module, made up of a Drive motor and a Direction motor.
 * 
 * the Drive motor is the central one, with the Direction motor being offset.
 */
SwerveModule::SwerveModule(vex::motor &drive, vex::motor &direction)
    : drive(drive), direction(direction)
{
  lastStoredHeading = 0.0;
  inverseDrive = false;
  driveMulitplier = 0.0;
}

/**
 * Sets both the speed and direction of the module, taking into account how the motors are geared
 * together, and each gear ratio.
 * 
 * @param direction_deg Degrees of rotation for the module's direction (clockwise positive, from top)
 * @param speed_pct Speed of the wheel for the module, in percent (-1.0 -> 1.0, positive fwd)
 * @param power=2 Square / Cube the input for a exponential curve for more lower speed control
 */
void SwerveModule::set(double direction_deg, double speed_pct, int power)
{
  // Don't move the direction wheel unless we need to
  if(speed_pct == 0.0)
    direction_deg = lastStoredHeading;
  else
    lastStoredHeading = direction_deg;

  set_direction(direction_deg);
  if(power % 2 == 0)
    set_speed((speed_pct > 0 ? 1.0 : -1.0) * pow(speed_pct, power));
  else
    set_speed(pow(speed_pct, power));
}

/**
 * Sets the direction of the module to X degrees (clockwise positive, from top perspective).
 * 
 * If set_speed is not called (even when the robot is not moving), then the wheel WILL rotate
 * despite not being set, due to how the motors are geared together.
 * 
 * Calling set_speed(0) will compensate for this.
 * 
 * @param deg position to set the motor, counter clockwise from the top.
 */
bool SwerveModule::set_direction(double deg)
{
  double pos = direction.position(vex::rotationUnits::deg) * DIR_GEAR_RATIO;
  
  // Find the degrees of the direction module between -180 and +180, with 0 being forward.
  int modpos = mod(pos, 360);
  modpos -= (modpos > 180) ? 360 : 0;

  // Delta between where we are (modpos) and where we need to be (deg)
  // Normalize the delta to make sure the maximum movement of the direction motor is 90 degrees.
  // If the delta is above 90, then the wheel will go to the nearest 180 from the target, and move the drive wheel backwards.
  int delta = deg - modpos;
  int normalizedDelta = delta + (abs(delta) > 90 ? (delta > 0 ? -180 : 180) : 0);

  inverseDrive = abs(delta) > 90 ? true : false;

  // Slow down the drive if we aren't close to the set direction yet (use the cube of the error)
  driveMulitplier = pow(1 - (abs(normalizedDelta) / 90.0), 3);
  double setpnt = (normalizedDelta + pos) / DIR_GEAR_RATIO;
  
  direction.spinTo(setpnt, vex::rotationUnits::deg, 100, vex::velocityUnits::pct, false);

  return fabs(setpnt - direction.rotation(rotationUnits::deg)) < 2;
}

/**
 * Sets the speed of the drive motor, taking into account the speed of the direction motor,
 * in percent units (-1.0 -> 1.0)
 */
void SwerveModule::set_speed(double percent)
{
  // take into account how the RPM of the direction motor affects the RPM of the drive wheel
  //double speed_diff_dps = 0;//direction.velocity(vex::velocityUnits::dps) * DIR_GEAR_RATIO * -.2;
  // Difference is negligable. Not worth the effort of getting it right.
  drive.setReversed(inverseDrive);

  drive.spin(vex::directionType::fwd, percent * 100.0, vex::velocityUnits::pct);
}

/**
 * Reset the drive encoder to zero
 */
void SwerveModule::reset_distance_driven()
{
  drive.resetPosition();
}

/**
 * Get 'distance' from the drive motor.
 * Will ALWAYS be positive.
 */
double SwerveModule::get_distance_driven()
{
  // return drive.position(vex::rotationUnits::rev);
  return fabs(WHEEL_DIAM * PI * drive.position(vex::rotationUnits::rev) * DRIVE_GEAR_RATIO);
}

/**
 * Return the direction that the module is pointing
 */
double SwerveModule::get_module_angle()
{
  // Limit to 0->360
  return fmod(direction.rotation(vex::rotationUnits::deg) * DIR_GEAR_RATIO, 360);
}

/**
 * Grab the maximum degrees per second of a motor with a certain gearset
 * 36 : 1 = reds
 * 18 : 1 = greens
 * 6 : 1 = blues
 */
double SwerveModule::gearset_dps(vex::gearSetting gearing)
{
    switch (gearing)
    {
    case vex::gearSetting::ratio36_1:
        return (1.0 / 36.0) * MOTOR_MAX_RPM * 360.0;
    case vex::gearSetting::ratio18_1:
        return (1.0 / 18.0) * MOTOR_MAX_RPM * 360.0;
    case vex::gearSetting::ratio6_1:
        return (1.0 / 6.0) * MOTOR_MAX_RPM * 360.0;
    default:
        return 0.0;
    }
}

/**
 * Improved modulus which correctly calculates negatives
 */
int SwerveModule::mod(int a, int b)
{
    return (b + (a % b)) % b;
}