#pragma once

typedef struct
{
    double pos;
    double vel;
    double accel;

} motion_t;

/**
 * Trapezoid Profile
 * 
 * This is a motion profile defined by an acceleration, maximum velocity, start point and end point.
 * Using this information, a parametric function is generated, with a period of acceleration, constant
 * velocity, and deceleration. The velocity graph looks like a trapezoid, giving it it's name.
 * 
 * If the maximum velocity is set high enough, this will become a S-curve profile, with only acceleration and deceleration.
 * 
 * This class is designed for use in properly modelling the motion of the robots to create a feedfoward
 * and target for PID. Acceleration and Maximum velocity should be measured on the robot and tuned down
 * slightly to account for battery drop.
 * 
 * Here are the equations graphed for ease of understanding:
 * https://www.desmos.com/calculator/rkm3ivu1yk
 * 
 * @author Ryan McGee
 * @date 7/12/2022
 * 
 */
class TrapezoidProfile
{
    public:

    /**
     * @brief Construct a new Trapezoid Profile object
     * 
     * @param max_v Maximum velocity the robot can run at
     * @param accel Maximum acceleration of the robot
     */
    TrapezoidProfile(double max_v, double accel);

    /**
     * @brief Run the trapezoidal profile based on the time that's ellapsed
     * 
     * @param time_s Time since start of movement
     * @return motion_t Position, velocity and acceleration
     */
    motion_t calculate(double time_s);

    void set_endpts(double start, double end);

    void set_accel(double accel);

    void set_max_v(double max_v);

    double get_movement_time();

    private:
    double start, end;
    double max_v, accel;
    double time;
    

};