#pragma once

/**
 * motion_t is a description of 1 dimensional motion at a point in time.
*/
typedef struct
{
    double pos;   ///< 1d position at this point in time
    double vel;   ///< 1d velocity at this point in time
    double accel; ///< 1d acceleration at this point in time

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

    /**
     * set_endpts defines a start and end position 
     * @param start the starting position of the path
     * @param end the ending position of the path
     */
    void set_endpts(double start, double end);

    /**
     * set_accel sets the acceleration this profile will use (the left and right legs of the trapezoid)
     * @param accel the acceleration amount to use
    */
    void set_accel(double accel);

    /**
     * sets the maximum velocity for the profile
     * (the height of the top of the trapezoid)
     * @param max_v the maximum velocity the robot can travel at
    */
    void set_max_v(double max_v);

    /**
     * uses the kinematic equations to and specified accel and max_v to figure out how long moving along the profile would take
     * @return the time the path will take to travel 
    */
    double get_movement_time();

    private:
    double start, end; ///< the start and ending position of the profile
    double max_v; ///< the maximum velocity to travel at for this profile
    double accel; ///< the rate of acceleration to use for this profile.
    double time; ///< the current point in time along the path
    

};