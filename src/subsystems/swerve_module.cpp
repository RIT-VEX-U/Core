#include "../Core/include/subsystems/swerve_module.h"

/**
 * Create a single swerve module, made up of a Drive motor and a Direction motor.
 * 
 * the Drive motor is the central one, with the Direction motor being offset.
 */
SwerveModule::SwerveModule(vex::motor &drive, vex::gearSetting drive_gearing, vex::motor &direction, vex::gearSetting dir_gearing)
    : drive(drive), drive_gearing(drive_gearing), direction(direction), dir_gearing(dir_gearing)
{
}

/**
 * Sets both the speed and direction of the module, taking into account how the motors are geared
 * together, and each gear ratio.
 * 
 * @param direction_deg Degrees of rotation for the module's direction (clockwise positive, from top)
 * @param speed_pct Speed of the wheel for the module, in percent (-1.0 -> 1.0, positive fwd)
 */
void SwerveModule::set(double direction_deg, double speed_pct)
{
    set_direction(direction_deg);
    set_speed(speed_pct);
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
void SwerveModule::set_direction(double deg)
{

    int modpos = mod(deg, 360);

    // If the position is more than 180 degrees away, then turn the other way
    if(fabs(modpos - direction.rotation(vex::rotationUnits::deg)) > 180)
        modpos = mod(modpos + 180, 360);

    direction.spinToPosition(modpos / DIR_GEAR_RATIO, vex::rotationUnits::deg, false);
}

/**
 * Sets the speed of the drive motor, taking into account the speed of the direction motor,
 * in percent units (-1.0 -> 1.0)
 */
void SwerveModule::set_speed(double percent)
{
    // The speed of the Drive motor is influenced by how fast the direciton motor is spinning,
    // due to the way they are geared together. Therefore, subtract that RPM from what will be set.
    double rpm = (percent * gearset_max_rpm(drive_gearing)) - (direction.rotation(vex::rotationUnits::rev) * DIR_GEAR_RATIO);

    drive.spin(vex::directionType::fwd, rpm / gearset_max_rpm(drive_gearing), vex::percentUnits::pct);
}

/**
 * Grab the maximum RPM of a motor with a certain gearset
 * 
 * values are:
 *  RED: 100
 *  GREEN: 200
 *  BLUE: 600
 */
double SwerveModule::gearset_max_rpm(vex::gearSetting gearing)
{
    switch (gearing)
    {
    case vex::gearSetting::ratio36_1:
        return (1.0 / 36.0) * MOTOR_MAX_RPM;
    case vex::gearSetting::ratio18_1:
        return (1.0 / 18.0) * MOTOR_MAX_RPM;
    case vex::gearSetting::ratio6_1:
        return (1.0 / 6.0) * MOTOR_MAX_RPM;
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