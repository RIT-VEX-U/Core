#include "../Core/include/subsystems/swerve_drive.h"

/**
 * Construct the SwerveDrive object.
 */
SwerveDrive::SwerveDrive(SwerveModule &left_front, SwerveModule &left_rear, SwerveModule &right_front, SwerveModule &right_rear)
:left_front(left_front), left_rear(left_rear), right_front(right_front), right_rear(right_rear)
{

}

/**
 * Drive the robot using controller inputs. Deadbands are automatically taken into
 * account before passing to the main control method.
 * 
 * @param leftY Left joystick, Y axis (-100 -> 100)
 * @param leftX Left joystick, X axis (-100 -> 100)
 * @param rightX Right joystick, X axis(-100 -> 100)
 */
void SwerveDrive::drive(int32_t leftY, int32_t leftX, int32_t rightX)
{
    Vector::point_t p = {.x=(leftX / 100.0), .y=(leftY / 100.0)};

    Vector input_lat(p);

    // Lateral Deadband
    if(fabs(input_lat.get_mag()) < LAT_DEADBAND)
        Vector input_lat(0,0);

    // Rotational Deadband
    if(fabs(rightX / 100.0) < LAT_DEADBAND)
        rightX = 0;

    // Convert input to a vector, and pass into main control method
    return this->drive(input_lat, rightX);    
}

/**
 * The main control method. Takes a vector and a rotation, and performs vector math to
 * calculate the position/speed for each wheel.
 */
void SwerveDrive::drive(Vector lateral, double rotation)
{
    // Each wheel has a different rotation vector
    Vector rot_lf(d2r(45), rotation);
    Vector rot_rf(d2r(45+90), rotation);
    Vector rot_rr(d2r(45+180), rotation);
    Vector rot_lr(d2r(45+270), rotation);

    // Perform vector addition between the rotation vector and lateral vectors
    Vector lf_out = rot_lf + lateral;
    Vector rf_out = rot_rf + lateral;
    Vector rr_out = rot_rr + lateral;
    Vector lr_out = rot_lr + lateral;

    // Set each swerve module to the respective direction / speed
    left_front.set(r2d(lf_out.get_dir()), lf_out.get_mag());
    right_front.set(r2d(rf_out.get_dir()), rf_out.get_mag());
    right_rear.set(r2d(rr_out.get_dir()), rr_out.get_mag());
    left_rear.set(r2d(lr_out.get_dir()), lr_out.get_mag());
}