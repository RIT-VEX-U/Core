#pragma once

const int MAX_TRAPEZOID_PROFILE_SEGMENTS = 4;

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
 * trapezoid_profile_segment_t is a description of one constant acceleration segment of a trapezoid motion profile
 */
typedef struct
{
    double pos_after; ///< 1d position after this segment concludes
    double vel_after; ///< 1d velocity after this segment concludes
    double accel;     ///< 1d acceleration during the segment
    double duration;  ///< duration of the segment
} trapezoid_profile_segment_t;

/**
 * Trapezoid Profile
 *
 * This is a motion profile defined by:
 * - maximum acceleration
 * - maximum velocity
 * - start position and velocity
 * - end position and velocity
 *
 * Using this information, a parametric function is generated, with a period of acceleration, constant
 * velocity, and deceleration. The velocity graph usually looks like a trapezoid, giving it its name.
 *
 * If the maximum velocity is set high enough, this will become a S-curve profile, with only acceleration and deceleration.
 *
 * If the initial velocity is in the wrong direction, the profile will first come to a stop, then continue a normal
 * trapezoid profile.
 *
 * If the initial velocity is higher than the maximum velocity, the profile will first try to achieve the maximum velocity.
 *
 * If the end velocity is not achievable, the profile will try to get as close as possible. The end velocity must
 * be in the direction of the end point.
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
     * @brief Run the trapezoidal profile based on the time and distance that's elapsed
     *
     * @param time_s Time since start of movement
     * @param pos_s The current position
     * @return motion_t Position, velocity and acceleration
     */
    motion_t calculate(double time_s, double pos_s);

    /**
     * @brief Run the trapezoidal profile based on the time that's elapsed
     *
     * @param time_s Time since start of movement
     * @return motion_t Position, velocity and acceleration
     */
    motion_t calculate_time_based(double time_s);

    /**
     * @brief set_endpts defines a start and end position
     *
     * @param start the starting position of the path
     * @param end the ending position of the path
     */
    void set_endpts(double start, double end);

    /**
     * @brief set start and end velocities
     *
     * @param start the starting velocity of the path
     * @param end the ending velocity of the path
     */
    void set_vel_endpts(double start, double end);

    /**
     * @brief set_accel sets the acceleration this profile will use (the left and right legs of the trapezoid)
     *
     * @param accel the acceleration amount to use
    */
    void set_accel(double accel);

    /**
     * @brief sets the maximum velocity for the profile (the height of the top of the trapezoid)
     *
     * @param max_v the maximum velocity the robot can travel at
    */
    void set_max_v(double max_v);

    /**
     * @brief uses the kinematic equations to and specified accel and max_v to figure out how long moving along the profile would take
     *
     * @return the time the path will take to travel
    */
    double get_movement_time();

    private:
    double si, sf; ///< the initial and final position of the profile
    double vi, vf; ///< the initial and final velocity of the profile
    double max_v; ///< the maximum velocity to travel at for this profile
    double accel; ///< the rate of acceleration to use for this profile.

    trapezoid_profile_segment_t segments[MAX_TRAPEZOID_PROFILE_SEGMENTS];
    int num_acceleration_phases;

    bool precalculated; ///< whether or not the segment array is up to date

    /**
     * Attempt to generate the motion profile for the given parameters
     *
     * @return False if there was a problem with the parameters
     */
    bool precalculate();

    /**
     * Calculate a trapezoid segment given a target velocity that accelerates to the given velocity
     *
     * @param si segment initial position
     * @param vi segment initial velocity
     * @param v_target target velocity
     * @return a trapezoid_profile_segment_t that represents constant acceleration to the target velocity
     */
    trapezoid_profile_segment_t calculate_kinetic_motion(double si, double vi, double v_target);

    /**
     * Calculate the next segment of the trapezoid motion profile
     *
     * @param s segment starting position
     * @param v segment starting velocity
     * @return the next segment of the trapezoid motion profile
     */
    trapezoid_profile_segment_t calculate_next_segment(double s, double v);
};