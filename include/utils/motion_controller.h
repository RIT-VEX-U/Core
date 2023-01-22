#pragma once
#include "../core/include/utils/pid.h"
#include "../core/include/utils/feedforward.h"
#include "../core/include/utils/trapezoid_profile.h"
#include "../core/include/utils/feedback_base.h"
#include "../core/include/subsystems/tank_drive.h"
#include "vex.h"

/**
 * Motion Controller class
 * 
 * This class defines a top-level motion profile, which can act as an intermediate between
 * a subsystem class and the motors themselves
 *
 * This takes the constants kS, kV, kA, kP, kI, kD, max_v and acceleration and wraps around 
 * a feedforward, PID and trapezoid profile. It does so with the following formula:
 * 
 * out = feedfoward.calculate(motion_profile.get(time_s)) + pid.get(motion_profile.get(time_s))
 * 
 * For PID and Feedforward specific formulae, see pid.h, feedforward.h, and trapezoid_profile.h
 * 
 * @author Ryan McGee
 * @date 7/13/2022
 */
class MotionController : public Feedback
{
    public:

    typedef struct
    {
        double max_v;
        double accel;
        PID::pid_config_t pid_cfg;
        FeedForward::ff_config_t ff_cfg;
    } m_profile_cfg_t;

    /**
     * @brief Construct a new Motion Controller object
     * 
     * @param config The definition of how the robot is able to move
     *    max_v Maximum velocity the movement is capable of
     *    accel Acceleration / deceleration of the movement
     *    pid_cfg Definitions of kP, kI, and kD
     *    ff_cfg Definitions of kS, kV, and kA
     */
    MotionController(m_profile_cfg_t &config);

    /**
     * @brief Initialize the motion profile for a new movement
     * This will also reset the PID and profile timers.
     * 
     * @param start_pt Movement starting position
     * @param end_pt Movement ending posiiton 
     */
    void init(double start_pt, double end_pt) override;
    
    /**
     * @brief Update the motion profile with a new sensor value
     * 
     * @param sensor_val Value from the sensor
     * @return The motor input generated from the motion profile 
     */
    double update(double sensor_val) override;

    /**
     * @return the last saved result from the feedback controller
     */
    double get() override;

    /**
     * Clamp the upper and lower limits of the output. If both are 0, no limits should be applied.
     * 
     * @param lower Upper limit
     * @param upper Lower limit
     * @return double 
     */
    void set_limits(double lower, double upper) override;

    /** 
     * @return Whether or not the movement has finished, and the PID
     * confirms it is on target
     */
    bool is_on_target() override;

    /**
     * This method attempts to characterize the robot's drivetrain and automatically tune the feedforward.
     * It does this by first calculating the kS (voltage to overcome static friction) by slowly increasing
     * the voltage until it moves.
     * 
     * Next is kV (voltage to sustain a certain velocity), where the robot will record it's steady-state velocity
     * at 'pct' speed.
     * 
     * Finally, kA (voltage needed to accelerate by a certain rate), where the robot will record the entire movement's
     * velocity and acceleration, record a plot of [X=(pct-kV*V-kS), Y=(Acceleration)] along the movement,
     * and since kA*Accel = pct-kV*V-kS, the reciprocal of the linear regression is the kA value.
     * 
     * @param drive The tankdrive to operate on
     * @param odometry The robot's odometry subsystem
     * @param pct Maximum velocity in percent (0->1.0)
     * @param duration Amount of time the robot should be moving for the test
     * @return A tuned feedforward object
     */
    static FeedForward::ff_config_t tune_feedforward(TankDrive &drive, OdometryTank &odometry, double pct=0.6, double duration=2);

    private: 

    m_profile_cfg_t config;

    PID pid;
    FeedForward ff;
    TrapezoidProfile profile;

    double lower_limit = 0, upper_limit = 0;
    double out = 0;
     
    vex::timer tmr;

};