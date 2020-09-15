#ifndef _SWERVEDRIVE_
#define _SWERVEDRIVE_

#include "../Core/include/subsystems/swerve_module.h"
#include "../Core/include/utils/vector.h"
#include "../Core/include/utils/pid.h"

#define ROT_DEADBAND 0.2
#define LAT_DEADBAND 0.2

class SwerveDrive
{

public:

/**
 * Construct the SwerveDrive object.
 */
SwerveDrive(SwerveModule &left_front, SwerveModule &left_rear, SwerveModule &right_front, SwerveModule &right_rear);

/**
 * Drive the robot using controller inputs. Deadbands are automatically taken into
 * account before passing to the main control method.
 */
void drive(int32_t leftY, int32_t leftX, int32_t rightX);

/**
 * The main control method. Takes a vector and a rotation, and performs vector math to
 * calculate the position/speed for each wheel.
 */
void drive(Vector lateral, double rotation);

/**
 * Autonomously drive the robot in (degrees) direction, at (-1.0 -> 1.0) speed, for (inches) distance.
 * Indicate a negative speed or distance, or (preferably) a direction of +-180 degrees for backwards.
 */
bool auto_drive(double direction, double speed, double distance);

void set_drive_pid(PID::pid_config_t &config);
void set_turn_pid(PID::pid_config_t &config);

private:

SwerveModule &left_front, &left_rear, &right_front, &right_rear;
bool auto_drive_init = true;
double auto_drive_enc_reset = 0.0;

PID *drive_pid, *turn_pid;

};

#endif