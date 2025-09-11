#include "core/subsystems/tank_drive.h"
#include "core/utils/command_structure/drive_commands.h"
#include "core/utils/controls/pidff.h"
#include "core/utils/geometry.h"
#include "core/utils/math_util.h"

TankDrive::TankDrive(motor_group &left_motors, motor_group &right_motors, robot_specs_t &config, OdometryBase *odom)
    : left_motors(left_motors), right_motors(right_motors), correction_pid(config.correction_pid), odometry(odom),
      config(config) {
    drive_default_feedback = config.drive_feedback;
    turn_default_feedback = config.turn_feedback;
}

AutoCommand *
TankDrive::DriveToPointCmd(Feedback &fb, Translation2d pt, vex::directionType dir, double max_speed, double end_speed) {
    return new DriveToPointCommand(*this, fb, pt, dir, max_speed, end_speed);
}

AutoCommand *TankDrive::DriveToPointCmd(Translation2d pt, vex::directionType dir, double max_speed, double end_speed) {
    return new DriveToPointCommand(*this, *drive_default_feedback, pt, dir, max_speed, end_speed);
}

AutoCommand *
TankDrive::DriveToPointCmd(double x, double y, vex::directionType dir, double max_speed, double end_speed) {
    return new DriveToPointCommand(*this, *drive_default_feedback, Translation2d(x, y), dir, max_speed, end_speed);
}

AutoCommand *TankDrive::DriveForwardCmd(double dist, vex::directionType dir, double max_speed, double end_speed) {
    return new DriveForwardCommand(*this, *drive_default_feedback, dist, dir, max_speed, end_speed);
}

AutoCommand *
TankDrive::DriveForwardCmd(Feedback &fb, double dist, vex::directionType dir, double max_speed, double end_speed) {
    return new DriveForwardCommand(*this, fb, dist, dir, max_speed, end_speed);
}

AutoCommand *TankDrive::TurnToHeadingCmd(double heading, double max_speed, double end_speed) {
    return new TurnToHeadingCommand(*this, *turn_default_feedback, heading, max_speed, end_speed);
}
AutoCommand *TankDrive::TurnToHeadingCmd(Feedback &fb, double heading, double max_speed, double end_speed) {
    return new TurnToHeadingCommand(*this, fb, heading, max_speed, end_speed);
}

AutoCommand *
TankDrive::TurnToPointCmd(Translation2d point, vex::directionType dir, double max_speed, double end_speed) {
    return new TurnToPointCommand(*this, point, dir, max_speed, end_speed);
}
AutoCommand *TankDrive::TurnToPointCmd(double x, double y, vex::directionType dir, double max_speed, double end_speed) {
    return new TurnToPointCommand(*this, x, y, dir, max_speed, end_speed);
}

AutoCommand *TankDrive::TurnDegreesCmd(double degrees, double max_speed, double end_speed) {
    return new TurnDegreesCommand(*this, *turn_default_feedback, degrees, max_speed, end_speed);
}
AutoCommand *TankDrive::TurnDegreesCmd(Feedback &fb, double degrees, double max_speed, double end_speed) {
    return new TurnDegreesCommand(*this, fb, degrees, max_speed, end_speed);
}
AutoCommand *TankDrive::PurePursuitCmd(PurePursuit::Path path, directionType dir, double max_speed, double end_speed) {
    return new PurePursuitCommand(*this, *drive_default_feedback, path, dir, max_speed, end_speed);
}
AutoCommand *TankDrive::PurePursuitCmd(
  Feedback &feedback, PurePursuit::Path path, directionType dir, double max_speed, double end_speed
) {
    return new PurePursuitCommand(*this, feedback, path, dir, max_speed, end_speed);
}

Condition *TankDrive::DriveStalledCondition(double stall_time) {
    class DriveStalledCondition : public Condition {
      public:
        DriveStalledCondition(TankDrive &td, double stall_time) : td(td), stalled_for(stall_time) {}
        bool test() override {
            if (!func_initialized) {
                stopped_timer.reset();
                func_initialized = true;
            }
            if (td.odometry->get_speed() > 0) {
                stopped_timer.reset();
            }
            return stopped_timer.value() > stalled_for;
        }
        TankDrive &td;
        vex::timer stopped_timer;
        double stalled_for = 10.0;
        bool func_initialized = false;
    };
    return new DriveStalledCondition(*this, stall_time);
}
AutoCommand *TankDrive::DriveTankCmd(double left, double right) {
    class DriveTankCommand : public AutoCommand {
      public:
        DriveTankCommand(TankDrive &td, double left, double right) : td(td), left(left), right(right) {}
        bool run() override {
            td.drive_tank(left, right);
            return false;
        }
        std::string toString() override {
            return "Driving Tank with left: " + double_to_string(left) + " right: " + double_to_string(right);
        }
        void on_timeout() override { td.stop(); }
        TankDrive &td;
        double left = 0;
        double right = 0;
    };
    return new DriveTankCommand(*this, left, right);
}

/**
 * Reset the initialization for autonomous drive functions
 */
void TankDrive::reset_auto() { func_initialized = false; }

/**
 * Stops rotation of all the motors using their "brake mode"
 */
void TankDrive::stop() {
    left_motors.stop();
    right_motors.stop();
}

/**
 * Returns the Robot position as a Pose2d
 */
Pose2d TankDrive::get_position() { return this->odometry->get_position(); }

void TankDrive::drive_tank_raw(double left_norm, double right_norm) {
    left_motors.spin(directionType::fwd, left_norm * 12, voltageUnits::volt);
    right_motors.spin(directionType::fwd, right_norm * 12, voltageUnits::volt);
}
/**
 * Drive the robot using differential style controls. left_motors controls the
 * left motors, right_motors controls the right motors.
 *
 * left_motors and right_motors are in "percent": -1.0 -> 1.0
 */
bool captured_position = false;
bool was_breaking = false;

void TankDrive::drive_tank(double left, double right, int power, BrakeType bt) {

    left = modify_inputs(left, power);
    right = modify_inputs(right, power);
    double brake_threshold = 0.05;
    bool should_brake = (bt != BrakeType::None) && fabs(left) < brake_threshold && fabs(right) < brake_threshold;

    if (!should_brake) {
        drive_tank_raw(left, right);
        was_breaking = false;
        return;
    }
    if (should_brake && !was_breaking) {
        captured_position = false;
    }
    static PID::pid_config_t zero_vel_cfg = {.p = 0.005, .d = 0.0005};
    static PID zero_vel_pid = PID(zero_vel_cfg);

    if (bt == BrakeType::ZeroVelocity) {
        zero_vel_pid.set_target(0);
        double vel = left_motors.velocity(vex::velocityUnits::pct) + right_motors.velocity(vex::velocityUnits::pct);
        double outp = zero_vel_pid.update(vel);
        left_motors.spin(directionType::fwd, outp, voltageUnits::volt);
        right_motors.spin(directionType::fwd, outp, voltageUnits::volt);
    } else if (bt == BrakeType::Smart) {
        static Pose2d target_pose(0.0, 0.0, 0.0);

        zero_vel_pid.set_target(0);
        double vel = odometry->get_speed();
        if (fabs(vel) <= 0.01 && !captured_position) {
            target_pose = odometry->get_position();
            captured_position = true;
        } else if (captured_position) {
            double dist_to_target = target_pose.translation().distance(odometry->get_position().translation());
            if (dist_to_target < 12.0) {
                drive_to_point(target_pose.x(), target_pose.y(), vex::fwd);
            } else {
                target_pose = odometry->get_position();
                reset_auto();
            }
        } else {
            double outp = zero_vel_pid.update(vel);
            left_motors.spin(directionType::fwd, outp, voltageUnits::volt);
            right_motors.spin(directionType::fwd, outp, voltageUnits::volt);
        }
    }
    was_breaking = should_brake;
}

/**
 * Drive the robot using arcade style controls. forward_back controls the linear
 * motion, left_right controls the turning.
 *
 * left_motors and right_motors are in "percent": -1.0 -> 1.0
 */
void TankDrive::drive_arcade(double forward_back, double left_right, int power, BrakeType bt) {
    forward_back = modify_inputs(forward_back, power);
    left_right = modify_inputs(left_right, power);

    double left = forward_back + left_right;
    double right = forward_back - left_right;

    drive_tank(left, right, 1, bt);
}

/**
 * Use odometry to drive forward a certain distance using a custom feedback
 * controller
 *
 * Returns whether or not the robot has reached it's destination.
 * @param inches     the distance to drive forward
 * @param dir        the direction we want to travel forward and backward
 * @param feedback   the custom feedback controller we will use to travel. controls the rate at which we accelerate and
 * drive.
 * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
 * @param end_speed  the movement profile will attempt to reach this velocity by its completion
 */
bool TankDrive::drive_forward(
  double inches, directionType dir, Feedback &feedback, double max_speed, double end_speed
) {
    static Pose2d pos_setpt(0, 0, 0);

    // We can't run the auto drive function without odometry
    if (odometry == NULL) {
        fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
        fflush(stderr);
        return true;
    }

    // Generate a point X inches forward of the current position, on first startup
    if (!func_initialized) {
        Pose2d cur_pos = odometry->get_position();

        // forwards is positive Y axis, backwards is negative
        if (dir == directionType::rev) {
            printf("going backwards\n");
            inches = -fabs(inches);
        } else {
            printf("going forwards\n");
            inches = fabs(inches);
        }
        // Use vector math to get an X and Y
        Translation2d current_pos(cur_pos.x(), cur_pos.y());
        Translation2d delta_pos(inches, cur_pos.rotation());
        Translation2d setpt_vec = current_pos + delta_pos;

        // Save the new X and Y values as a Pose
        pos_setpt = Pose2d(setpt_vec.x(), setpt_vec.y(), pos_setpt.rotation());
    }

    // Call the drive_to_point with updated point values
    return drive_to_point(pos_setpt.x(), pos_setpt.y(), dir, feedback, max_speed, end_speed);
}
/**
 * Autonomously drive the robot forward a certain distance
 *
 *
 * @param inches      degrees by which we will turn relative to the robot (+)
 * turns ccw, (-) turns cw
 * @param dir        the direction we want to travel forward and backward
 * @param max_speed   the maximum percentage of robot speed at which the robot
 * will travel. 1 = full power
 * @param end_speed   the movement profile will attempt to reach this velocity
 * by its completion
 * @return true if we have finished driving to our point
 */
bool TankDrive::drive_forward(double inches, directionType dir, double max_speed, double end_speed) {
    if (drive_default_feedback != NULL) {
        return drive_forward(inches, dir, *drive_default_feedback, max_speed, end_speed);
    }

    printf("tank_drive.cpp: Cannot run drive_forward without a feedback "
           "controller!\n");
    fflush(stdout);
    return true;
}

/**
 * Autonomously turn the robot X degrees to counterclockwise (negative for
 * clockwise), with a maximum motor speed of percent_speed (-1.0 -> 1.0)
 *
 * Uses the specified feedback for it's control.
 *
 * @param degrees     degrees by which we will turn relative to the robot (+)
 * turns ccw, (-) turns cw
 * @param feedback    the feedback controller we will use to travel. controls
 * the rate at which we accelerate and drive.
 * @param max_speed   the maximum percentage of robot speed at which the robot
 * will travel. 1 = full power
 * @param end_speed   the movement profile will attempt to reach this velocity
 * by its completion
 * @return true if we have turned our target number of degrees
 */
bool TankDrive::turn_degrees(double degrees, Feedback &feedback, double max_speed, double end_speed) {
    // We can't run the auto drive function without odometry
    if (odometry == NULL) {
        fprintf(stderr, "Odometry is NULL. Unable to run turn_degrees()\n");
        fflush(stderr);
        return true;
    }

    static double target_heading = 0;

    // On the first run of the funciton, reset the gyro position and PID
    if (!func_initialized) {
        double start_heading = odometry->get_position().rotation().degrees();
        target_heading = start_heading + degrees;
    }

    return turn_to_heading(target_heading, feedback, max_speed, end_speed);
}

/**
 * Autonomously turn the robot X degrees to counterclockwise (negative for
 * clockwise), with a maximum motor speed of percent_speed (-1.0 -> 1.0)
 *
 * Uses the defualt turning feedback of the drive system.
 *
 * @param degrees     degrees by which we will turn relative to the robot (+)
 * turns ccw, (-) turns cw
 * @param max_speed   the maximum percentage of robot speed at which the robot
 * will travel. 1 = full power
 * @param end_speed   the movement profile will attempt to reach this velocity
 * by its completion
 * @return true if we turned te target number of degrees
 */
bool TankDrive::turn_degrees(double degrees, double max_speed, double end_speed) {
    if (turn_default_feedback != NULL) {
        return turn_degrees(degrees, *turn_default_feedback, max_speed, end_speed);
    }

    printf("tank_drive.cpp: Cannot run turn_degrees without a feedback "
           "controller!\n");
    fflush(stdout);
    return true;
}

/**
 * Use odometry to automatically drive the robot to a point on the field.
 * X and Y is the final point we want the robot.
 *
 * Returns whether or not the robot has reached it's destination.
 * @param x          the x position of the target
 * @param y          the y position of the target
 * @param dir        the direction we want to travel forward and backward
 * @param feedback   the feedback controller we will use to travel. controls the
 * rate at which we accelerate and drive.
 * @param max_speed  the maximum percentage of robot speed at which the robot
 * will travel. 1 = full power
 * @param end_speed  the movement profile will attempt to reach this velocity by
 * its completion
 * @return true if we have reached our target point
 */
bool TankDrive::drive_to_point(
  double x, double y, vex::directionType dir, Feedback &feedback, double max_speed, double end_speed
) {
    // We can't run the auto drive function without odometry
    if (odometry == NULL) {
        fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
        fflush(stderr);
        return true;
    }

    if (!func_initialized) {

        double initial_dist = odometry->get_position().translation().distance(Translation2d(x, y));

        // Reset the control loops
        correction_pid.init(0, 0);
        feedback.init(-initial_dist, 0);

        correction_pid.set_limits(-1, 1);
        feedback.set_limits(-1, 1);

        func_initialized = true;
    }

    // Store the initial position of the robot
    Pose2d current_pos = odometry->get_position();
    Pose2d end_pos(x, y, 0);

    // Create a point (and vector) to get the direction
    Translation2d pos_diff_pt = {x - current_pos.x(), y - current_pos.y()};

    Translation2d the_point(pos_diff_pt);

    // Get the distance between 2 points
    double dist_left = current_pos.translation().distance(end_pos.translation());

    int sign = 1;

    // Make an imaginary perpendicualar line to that between the bot and the
    // point. If the point is behind that line, and the point is within the
    // robot's radius, use negatives for feedback control.

    double angle_to_point = atan2(y - current_pos.y(), x - current_pos.x()) * 180.0 / PI;
    double angle = fmod(current_pos.rotation().degrees() - angle_to_point, 360.0);

    // Normalize the angle between 0 and 360
    if (angle > 360) {
        angle -= 360;
    }
    if (angle < 0) {
        angle += 360;
    }

    // If the angle is behind the robot, report negative.
    if (dir == directionType::fwd && angle > 90 && angle < 270) {
        sign = -1;
    } else if (dir == directionType::rev && (angle < 90 || angle > 270)) {
        sign = -1;
    }

    if (fabs(dist_left) < config.drive_correction_cutoff) {
        // When inside the robot's cutoff radius, report the distance to the point along the robot's forward axis,
        // so we always "reach" the point without having to do a lateral translation
        dist_left *= fabs(cos(angle * PI / 180.0));
    }

    // Get the heading difference between where we are and where we want to be
    // Optimize that heading so we don't turn clockwise all the time
    double heading = the_point.theta().wrapped_degrees_360();
    double delta_heading = 0;

    // Going backwards "flips" the robot's current heading
    if (dir == directionType::fwd) {
        delta_heading = OdometryBase::smallest_angle(current_pos.rotation().degrees(), heading);
    } else {
        delta_heading = OdometryBase::smallest_angle(current_pos.rotation().degrees() - 180, heading);
    }

    // Update the PID controllers with new information
    correction_pid.update(delta_heading);
    feedback.update(sign * -1 * dist_left);

    // Disable correction when we're close enough to the point
    double correction = 0;
    if (is_pure_pursuit || fabs(dist_left) > config.drive_correction_cutoff) {
        correction = correction_pid.get();
    }

    // Reverse the drive_pid output if we're going backwards
    double drive_pid_rval;
    if (dir == directionType::rev) {
        drive_pid_rval = feedback.get() * -1;
    } else {
        drive_pid_rval = feedback.get();
    }

    // Combine the two pid outputs
    double lside = drive_pid_rval + correction;
    double rside = drive_pid_rval - correction;

    // limit the outputs between -1 and +1
    lside = clamp(lside, -max_speed, max_speed);
    rside = clamp(rside, -max_speed, max_speed);

    drive_tank(lside, rside);

    // Check if the robot has reached it's destination
    if (feedback.is_on_target()) {
        if (end_speed == 0) {
            stop();
        }
        func_initialized = false;
        return true;
    }

    return false;
}

/**
 * Use odometry to automatically drive the robot to a point on the field.
 * X and Y is the final point we want the robot.
 * Here we use the default feedback controller from the drive_sys
 *
 * Returns whether or not the robot has reached it's destination.
 * @param x          the x position of the target
 * @param y          the y position of the target
 * @param dir        the direction we want to travel forward and backward
 * @param max_speed  the maximum percentage of robot speed at which the robot
 * will travel. 1 = full power
 * @param end_speed  the movement profile will attempt to reach this velocity by
 * its completion
 * @return true if we have reached our target point
 */
bool TankDrive::drive_to_point(double x, double y, vex::directionType dir, double max_speed, double end_speed) {
    if (drive_default_feedback != NULL) {
        return this->drive_to_point(x, y, dir, *drive_default_feedback, max_speed, end_speed);
    }

    printf("tank_drive.cpp: Cannot run drive_to_point without a feedback "
           "controller!\n");
    fflush(stdout);
    return true;
}

/**
 * Turn the robot in place to an exact heading relative to the field.
 * 0 is forward.
 *
 * @param heading_deg the heading to which we will turn
 * @param feedback    the feedback controller we will use to travel. controls
 * the rate at which we accelerate and drive.
 * @param max_speed  the maximum percentage of robot speed at which the robot
 * will travel. 1 = full power
 * @param end_speed  the movement profile will attempt to reach this velocity by
 * its completion
 * @return true if we have reached our target heading
 */
bool TankDrive::turn_to_heading(double heading_deg, Feedback &feedback, double max_speed, double end_speed) {
    // We can't run the auto drive function without odometry
    if (odometry == NULL) {
        fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
        fflush(stderr);
        return true;
    }

    if (!func_initialized) {
        double initial_delta = OdometryBase::smallest_angle(odometry->get_position().rotation().degrees(), heading_deg);
        feedback.init(-initial_delta, 0);
        feedback.set_limits(-fabs(max_speed), fabs(max_speed));

        func_initialized = true;
    }

    // Get the difference between the new heading and the current, and decide
    // whether to turn left or right.
    double delta_heading = OdometryBase::smallest_angle(odometry->get_position().rotation().degrees(), heading_deg);
    feedback.update(-delta_heading);

    fflush(stdout);

    drive_tank(-feedback.get(), feedback.get());

    // When the robot has reached it's angle, return true.
    if (feedback.is_on_target()) {
        func_initialized = false;
        stop();
        return true;
    }
    return false;
}
/**
 * Turn the robot in place to an exact heading relative to the field.
 * 0 is forward. Uses the defualt turn feedback of the drive system
 *
 * @param heading_deg the heading to which we will turn
 * @param max_speed  the maximum percentage of robot speed at which the robot
 * will travel. 1 = full power
 * @param end_speed  the movement profile will attempt to reach this velocity by
 * its completion
 * @return true if we have reached our target heading
 */
bool TankDrive::turn_to_heading(double heading_deg, double max_speed, double end_speed) {
    if (turn_default_feedback != NULL) {
        return turn_to_heading(heading_deg, *turn_default_feedback, max_speed, end_speed);
    }

    printf("tank_drive.cpp: Cannot run turn_to_heading without a feedback "
           "controller!\n");
    fflush(stdout);
    return true;
}

/**
 * Modify the inputs from the controller by squaring / cubing, etc
 * Allows for better control of the robot at slower speeds
 * @param input the input signal -1 -> 1
 * @param power the power to raise the signal to
 * @return input^power accounting for any sign issues that would arise with this
 * naive solution
 */
double TankDrive::modify_inputs(double input, int power) { return sign(input) * pow(std::abs(input), power); }

/**
 * Drive the robot autonomously using a pure-pursuit algorithm - Input path with
 * a set of waypoints - the robot will attempt to follow the points while
 * cutting corners (radius) to save time (compared to stop / turn / start)
 *
 * @param path The list of coordinates to follow, in order
 * @param dir Run the bot forwards or backwards
 * @param feedback The feedback controller determining speed
 * @param max_speed Limit the speed of the robot (for pid / pidff feedbacks)
 * @return True when the path is complete
 */
bool TankDrive::pure_pursuit(
  PurePursuit::Path path, directionType dir, Feedback &feedback, double max_speed, double end_speed
) {
    std::vector<Translation2d> points = path.get_points();
    if (!path.is_valid()) {
        printf("WARNING: Unexpected pure pursuit path - some segments intersect or are too close\n");
    }
    Pose2d robot_pose = odometry->get_position();

    // On function initialization, send the path-length estimate to the feedback controller
    if (!func_initialized) {
        if (dir != directionType::rev) {
            feedback.init(-estimate_path_length(points), 0);
        } else {
            feedback.init(estimate_path_length(points), 0);
        }

        func_initialized = true;
    }

    Translation2d lookahead = PurePursuit::get_lookahead(points, odometry->get_position(), path.get_radius());
    Translation2d localized = lookahead - robot_pose.translation();

    Translation2d last_point = points[points.size() - 1];
    bool is_last_point = (lookahead == last_point);

    double correction = 0;
    double dist_remaining = PurePursuit::estimate_remaining_dist(points, robot_pose, path.get_radius());
    double angle_diff = 0;

    // Robot is facing forwards / backwards, change the bot's angle by 180
    if (dir != directionType::rev) {
        angle_diff =
          OdometryBase::smallest_angle(robot_pose.rotation().degrees(), rad2deg(atan2(localized.y(), localized.x())));
    } else {
        angle_diff = OdometryBase::smallest_angle(
          robot_pose.rotation().degrees() + 180, rad2deg(atan2(localized.y(), localized.x()))
        );
    }

    // Correct the robot's heading until the last cut-off
    if (!(is_last_point && robot_pose.translation().distance(last_point) < config.drive_correction_cutoff)) {
        correction_pid.update(angle_diff);
        correction = correction_pid.get();
    } else // Inside cut-off radius, ignore horizontal diffs
    {
        dist_remaining *= cos(angle_diff * (PI / 180.0));
    }

    if (dir != directionType::rev) {
        feedback.update(-dist_remaining);
    } else {
        feedback.update(dist_remaining);
    }

    max_speed = fabs(max_speed);

    double left = clamp(feedback.get(), -max_speed, max_speed);
    double right = clamp(feedback.get(), -max_speed, max_speed);

    left += correction;
    right -= correction;

    drive_tank(left, right);

    // When the robot has reached the end point and feedback reports on target, end pure pursuit
    if (is_last_point && feedback.is_on_target()) {
        func_initialized = false;
        stop();
        return true;
    }
    return false;
}

/**
 * Drive the robot autonomously using a pure-pursuit algorithm - Input path with
 * a set of waypoints - the robot will attempt to follow the points while
 * cutting corners (radius) to save time (compared to stop / turn / start)
 *
 * Use the default drive feedback
 *
 * @param path The list of coordinates to follow, in order
 * @param dir Run the bot forwards or backwards
 * @param max_speed Limit the speed of the robot (for pid / pidff feedbacks)
 * @return True when the path is complete
 */
bool TankDrive::pure_pursuit(PurePursuit::Path path, directionType dir, double max_speed, double end_speed) {
    return pure_pursuit(path, dir, *config.drive_feedback, max_speed, end_speed);
}