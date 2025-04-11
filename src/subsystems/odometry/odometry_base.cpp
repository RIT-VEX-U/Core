#include "core/subsystems/odometry/odometry_base.h"

/**
 * Construct a new Odometry Base object
 *
 * @param is_async True to run constantly in the background, false to call update() manually
 */
OdometryBase::OdometryBase(bool is_async) : current_pos(this->zero_pos) {
    if (is_async) {
        handle = new vex::task(background_task, (void *)this);
    }
}

/**
 * Function that runs in the background task. This function pointer is passed
 * to the vex::task constructor.
 *
 * @param ptr Pointer to OdometryBase object
 * @return Required integer return code. Unused.
 */
int OdometryBase::background_task(void *ptr) {
    OdometryBase &obj = *((OdometryBase *)ptr);
    vexDelay(1000);
    while (!obj.end_task) {
        obj.mut.lock();
        obj.update();
        obj.mut.unlock();
        vexDelay(5);
    }

    return 0;
}

/**
 * End the background task. Cannot be restarted.
 * If the user wants to end the thread but keep the data up to date,
 * they must run the update() function manually from then on.
 */
void OdometryBase::end_async() { this->end_task = true; }

/**
 * Gets the current position and rotation
 */
Pose2d OdometryBase::get_position(void) {
    mut.lock();

    // Create a new struct to pass-by-value
    Pose2d out = current_pos;

    mut.unlock();

    return out;
}

/**
 * Sets the current position of the robot
 */
void OdometryBase::set_position(const Pose2d &newpos) {
    mut.lock();

    current_pos = newpos;

    mut.unlock();
}

AutoCommand *OdometryBase::SetPositionCmd(const Pose2d &newpos) {
    return new FunctionCommand([&]() {
        set_position(newpos);
        return true;
    });
}

/**
 * Get the smallest difference in angle between a start heading and end heading.
 * Returns the difference between -180 degrees and +180 degrees, representing the robot
 * turning left or right, respectively.
 */
double OdometryBase::smallest_angle(double start_deg, double end_deg) {
    double retval;
    // get the difference between 0 and 360
    retval = fmod(end_deg - start_deg, 360.0);
    if (retval < 0) {
        retval += 360.0;
    }

    // Get the closest angle, now between -180 (turn left) and +180 (turn right)
    if (retval > 180) {
        retval -= 360;
    }

    return retval;
}

double OdometryBase::get_speed() {
    mut.lock();
    double retval = speed;
    mut.unlock();

    return retval;
}

double OdometryBase::get_accel() {
    mut.lock();
    double retval = accel;
    mut.unlock();

    return retval;
}

double OdometryBase::get_angular_speed_deg() {
    mut.lock();
    double retval = ang_speed_deg;
    mut.unlock();

    return retval;
}

double OdometryBase::get_angular_accel_deg() {
    mut.lock();
    double retval = ang_accel_deg;
    mut.unlock();

    return retval;
}
