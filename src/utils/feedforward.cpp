#include "../core/include/utils/feedforward.h"


/**
* tune_feedforward takes a group of motors and finds the feedforward conifg parameters automagically.
*  @param motor the motor group to use 
*  @param pct Maximum velocity in percent (0->1.0)
 * @param duration Amount of time the motors spin for the test
 * @return A tuned feedforward object
 */
FeedForward::ff_config_t tune_feedforward(vex::motor_group &motor, double pct, double duration)
{
    FeedForward::ff_config_t out = {};
    
    double start_pos = motor.position(vex::rotationUnits::rev);

    // ========== kS Tuning =========
    // Start at 0 and slowly increase the power until the robot starts moving
    double power = 0;
    while(fabs(motor.position(vex::rotationUnits::rev) - start_pos) < 0.05)
    {
        motor.spin(vex::directionType::fwd, power, vex::voltageUnits::volt);
        power += 0.001;
        vexDelay(100);
    }
    out.kS = power;
    motor.stop();


    // ========== kV / kA Tuning =========

    std::vector<std::pair<double, double>> vel_data_points; // time, velocity
    std::vector<std::pair<double, double>> accel_data_points; // time, accel

    double max_speed = 0;
    vex::timer tmr;
    double time = 0;

    MovingAverage vel_ma(3);
    MovingAverage accel_ma(3);

    // Move the robot forward at a fixed percentage for X seconds while taking velocity and accel measurements
    do
    {
        double last_time = time;
        time = tmr.time(vex::sec);
        double dt = time - last_time;

        vel_ma.add_entry(motor.velocity(vex::velocityUnits::rpm));
        accel_ma.add_entry(motor.velocity(vex::velocityUnits::rpm)/dt);

        double speed = vel_ma.get_average();
        double accel = accel_ma.get_average();

        // For kV:
        if(speed > max_speed)
            max_speed = speed;

        // For kA:
        // Filter out the acceleration dampening due to motor inductance
        if(time > 0.25)
        {
            vel_data_points.push_back(std::pair<double, double>(time, speed));
            accel_data_points.push_back(std::pair<double, double>(time, accel));
        }

        // Theoretical polling rate = 100hz (it won't be that much, cause, y'know, vex.)
        vexDelay(10); 
    } while(time < duration);

    motor.stop();

    // Calculate kV (volts/12 per unit per second)
    out.kV = (pct - out.kS) / max_speed;

    // Calculate kA (volts/12 per unit per second^2)
    std::vector<std::pair<double, double>> accel_per_pct;
    for (int i = 0; i < vel_data_points.size(); i++)
    {
        accel_per_pct.push_back(std::pair<double, double>(
            pct - out.kS - (vel_data_points[i].second * out.kV),   // Acceleration-causing percent (X variable)
            accel_data_points[i].second                            // Measured acceleration (Y variable)
        ));
    }
    
    // kA is the reciprocal of the slope of the linear regression
    double regres_slope = calculate_linear_regression(accel_per_pct).first;
    out.kA = 1.0 / regres_slope; 

    return out;
}