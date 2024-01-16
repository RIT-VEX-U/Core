#include "../core/include/utils/controls/motion_controller.h"
#include "../core/include/subsystems/screen.h"
#include "../core/include/utils/math_util.h"
#include <vector>

/**
 * @brief Construct a new Motion Controller object
 *
 * @param config The definition of how the robot is able to move
 * max_v Maximum velocity the movement is capable of
 * accel Acceleration / deceleration of the movement
 * pid_cfg Definitions of kP, kI, and kD
 * ff_cfg Definitions of kS, kV, and kA
 */
MotionController::MotionController(m_profile_cfg_t &config)
    : config(config), pid(config.pid_cfg), ff(config.ff_cfg),
      profile(config.max_v, config.accel) {}

/**
 * @brief Initialize the motion profile for a new movement
 * This will also reset the PID and profile timers.
 * @param start_pt Movement starting position
 * @param end_pt Movement ending posiiton
 * @param start_vel Movement starting velocity
 * @param end_vel Movement ending velocity
 */
void MotionController::init(double start_pt, double end_pt, double start_vel,
                            double end_vel) {
    profile.set_endpts(start_pt, end_pt);
    profile.set_vel_endpts(start_vel, end_vel);
    pid.reset();
    tmr.reset();

    this->end_pt = end_pt;
}

/**
 * @brief Update the motion profile with a new sensor value
 *
 * @param sensor_val Value from the sensor
 * @return the motor input generated from the motion profile
 */
double MotionController::update(double sensor_val) {
    cur_motion = profile.calculate(tmr.time(timeUnits::sec), sensor_val);
    pid.set_target(cur_motion.pos);
    pid.update(sensor_val);

    this->current_pos = sensor_val;

    out = pid.get() + ff.calculate(cur_motion.vel, cur_motion.accel, pid.get());

    if (lower_limit != upper_limit) {
        out = clamp(out, lower_limit, upper_limit);
    }

    return out;
}

/**
 * @return the last saved result from the feedback controller
 */
double MotionController::get() { return out; }

/**
 * Clamp the upper and lower limits of the output. If both are 0, no limits
 * should be applied.
 *
 * @param lower Upper limit
 * @param upper Lower limit
 */
void MotionController::set_limits(double lower, double upper) {
    lower_limit = lower;
    upper_limit = upper;
}

/**
 * @return Whether or not the movement has finished, and the PID
 * confirms it is on target
 */
bool MotionController::is_on_target() {
    return (tmr.time(timeUnits::sec) > profile.get_movement_time()) &&
           pid.is_on_target() &&
           fabs(end_pt - current_pos) < pid.config.deadband;
}

/**
 * @return The current postion, velocity and acceleration setpoints
 */
motion_t MotionController::get_motion() const { return cur_motion; }

/**
 * This method attempts to characterize the robot's drivetrain and automatically
 * tune the feedforward. It does this by first calculating the kS (voltage to
 * overcome static friction) by slowly increasing the voltage until it moves.
 *
 * Next is kV (voltage to sustain a certain velocity), where the robot will
 * record it's steady-state velocity at 'pct' speed.
 *
 * Finally, kA (voltage needed to accelerate by a certain rate), where the robot
 * will record the entire movement's velocity and acceleration, record a plot of
 * [X=(pct-kV*V-kS), Y=(Acceleration)] along the movement, and since kA*Accel =
 * pct-kV*V-kS, the reciprocal of the linear regression is the kA value.
 *
 * @param drive The tankdrive to operate on
 * @param odometry The robot's odometry subsystem
 * @param pct Maximum velocity in percent (0->1.0)
 * @param duration Amount of time the robot should be moving for the test
 * @return A tuned feedforward object
 */
FeedForward::ff_config_t
MotionController::tune_feedforward(TankDrive &drive, OdometryTank &odometry,
                                   double pct, double duration) {
    FeedForward::ff_config_t out = {};

    pose_t start_pos = odometry.get_position();

    // ========== kS Tuning =========
    // Start at 0 and slowly increase the power until the robot starts moving
    double power = 0;
    while (odometry.pos_diff(start_pos, odometry.get_position()) < 0.05) {
        drive.drive_tank(power, power, 1);
        power += 0.001;
        vexDelay(100);
    }
    out.kS = power;
    drive.stop();

    // ========== kV / kA Tuning =========

    std::vector<std::pair<double, double>> vel_data_points;   // time, velocity
    std::vector<std::pair<double, double>> accel_data_points; // time, accel

    double max_speed = 0;
    double max_accel = 0;
    timer tmr;
    double time;

    MovingAverage vel_ma(3);
    MovingAverage accel_ma(3);

    // Move the robot forward at a fixed percentage for X seconds while taking
    // velocity and accel measurements

    drive.drive_tank(pct, pct, 1);
    do {
        time = tmr.time(sec);

        vel_ma.add_entry(odometry.get_speed());
        accel_ma.add_entry(odometry.get_accel());

        double speed = vel_ma.get_value();
        double accel = accel_ma.get_value();
        // For kV:
        if (speed > max_speed) {
            max_speed = speed;
        }
        if (accel > max_accel) {
            max_accel = accel;
        }

        // For kA:
        // Filter out the acceleration dampening due to motor inductance
        if (time > 0.25) {
            vel_data_points.push_back(std::pair<double, double>(time, speed));
            accel_data_points.push_back(std::pair<double, double>(time, accel));
        }

        // Theoretical polling rate = 100hz (it won't be that much, cause,
        // y'know, vex.)
        vexDelay(10);
    } while (time < duration);

    drive.stop();

    // Calculate kV (volts/12 per unit per second)
    out.kV = (pct - out.kS) / max_speed;

    printf("Max speed achieved: %.4f\n", max_speed);
    printf("Max accel achieved: %.4f\n", max_accel);

    // Calculate kA (volts/12 per unit per second^2)
    std::vector<std::pair<double, double>> accel_per_pct;
    for (int i = 0; i < vel_data_points.size(); i++) {
        accel_per_pct.push_back(std::pair<double, double>(
            pct - out.kS -
                (vel_data_points[i].second *
                 out.kV), // Acceleration-causing percent (X variable)
            accel_data_points[i].second // Measured acceleration (Y variable)
            ));
    }

    // kA is the reciprocal of the slope of the linear regression
    double regres_slope = calculate_linear_regression(accel_per_pct).first;
    out.kA = 1.0 / regres_slope;

    return out;
}

class MotionControllerPage : public screen::Page {
  public:
    MotionControllerPage(const MotionController &mc) : mc(mc) {}

    void update(bool was_pressed, int x, int y) override {}
    void draw(vex::brain::lcd &screen, bool first_draw,
              unsigned int frame_number) {
        const motion_t mot = mc.get_motion();

        // Text - top right
        screen.printAt(240, 20, "pos: %.2fin", mot.pos);
        screen.printAt(240, 40, "vel: %.2fin", mot.vel);
        screen.printAt(240, 60, "acc: %.2fin", mot.accel);
        screen.printAt(240, 80, "%.2fs of %.2fs", mc.tmr.value(),
                       mc.profile.get_movement_time());

        // Trapezoid - bottom right
        const int32_t trap_width = 200;
        const int32_t trap_height = 98;
        const double seconds = mc.tmr.value();
        const double full_time = mc.profile.get_movement_time();
        const double x_pct = seconds / full_time;
        const double y_pct = mot.vel / mc.profile.get_max_v();
        const int32_t x_pos = (int32_t)(x_pct * trap_width) + 240;
        const int32_t y_pos = (int32_t)(-y_pct * trap_height) + 100;
        const int32_t max_vel_y = 120;
        const int32_t end_acc_x = 260;
        const int32_t start_dec_x = 420;

        // trapezoid
        screen.setPenColor(vex::red);
        // accelerating
        screen.drawLine(240, 200, end_acc_x, max_vel_y);
        // staying
        screen.drawLine(end_acc_x, max_vel_y, start_dec_x, max_vel_y);
        // decellerating
        screen.drawLine(440, 200, start_dec_x, max_vel_y);

        // dot where we are
        screen.setFillColor(vex::red);
        screen.setPenColor(vex::white);
        screen.drawCircle(x_pos, y_pos, 3);
    }

  private:
    const MotionController &mc;
};

screen::Page *MotionController::Page() {
    return new MotionControllerPage(*this);
}