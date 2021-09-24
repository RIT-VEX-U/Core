#ifndef _SWERVEMODULE_
#define _SWERVEMODULE_

#include "vex.h"
using namespace vex;

#ifndef PI
#define PI 3.141592654
#endif

#ifndef PI
#define PI 3.141592654
#endif

// Gear teeth (input to output): 16, 35
#define DIR_GEAR_RATIO (16.0/35.0) // ~0.457

// Gear teeth (input to output): 21, 10, 12, 30
#define DRIVE_GEAR_RATIO ((21.0/10.0)*(12.0/30.0)) // 0.84

#define MOTOR_MAX_RPM 3600

#define WHEEL_DIAM 2.75 //inches

class SwerveModule
{
    public:

    /**
     * Create a single swerve module, made up of a Drive motor and a Direction motor.
     * 
     * the Drive motor is the central one, with the Direction motor being offset.
     */
    SwerveModule(vex::motor &drive, vex::motor &direction);

    /**
     * Sets both the speed and direction of the module, taking into account how the motors are geared
     * together, and each gear ratio.
     * 
     * @param direction_deg Degrees of rotation for the module's direction (clockwise positive, from top)
     * @param speed_pct Speed of the wheel for the module, in percent (-1.0 -> 1.0, positive fwd)
     */
    void set(double direction_deg, double speed_pct, int power=2);

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
    bool set_direction(double deg);

    /**
     * Sets the speed of the drive motor, taking into account the speed of the direction motor,
     * in percent units (-1.0 -> 1.0)
     */
    void set_speed(double percent);

    /**
    * Reset the drive encoder to zero
    * 
    * @deprecated Resetting encoders will screw up OdometrySwerve; do not use with odometry.
    */
    void reset_distance_driven();

    /**
     * Get 'distance' from the drive motor
     * 
     * TODO take into account the effect of the direction motor on the driven distance
     */
    double get_distance_driven();

    /**
     * Return the direction that the module is pointing
     */
    double get_module_angle();

    bool auto_reverse = false;

    private:

    /**
     * Grab the maximum RPM of a motor with a certain gearset
     * 
     * values are:
     *  RED: 100
     *  GREEN: 200
     *  BLUE: 600
     */
    static double gearset_dps(vex::gearSetting gearing);

    /**
     * Improved modulus which correctly calculates negatives
     */
    static int mod(int var1, int var2);

    vex::motor &drive, &direction;

    bool inverseDrive;
    double lastStoredHeading;
    double driveMulitplier;

};

#endif