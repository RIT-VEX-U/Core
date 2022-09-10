#pragma once

#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/subsystems/custom_encoder.h"
#include "../core/include/utils/vector.h"
#include "../core/include/robot_specs.h"

static int background_task(void* odom_obj);

class OdometryTank : public OdometryBase
{
public:
    /**
    * Initialize the Odometry module, calculating position from the drive motors.
    * @param left_side The left motors 
    * @param right_side The right motors
    * @param imu The robot's inertial sensor. If not included, rotation is calculated from the encoders.
    * @param is_async If true, position will be updated in the background continuously. If false, the programmer will have to manually call update().
    */
    OdometryTank(vex::motor_group &left_side, vex::motor_group &right_side, robot_specs_t &config, vex::inertial *imu=NULL, bool is_async=true);

    /**
    * Initialize the Odometry module, calculating posiiton from encoders on "dead wheels"
    * @param left_side The left motors 
    * @param right_side The right motors
    * @param imu The robot's inertial sensor. If not included, rotation is calculated from the encoders.
    * @param is_async If true, position will be updated in the background continuously. If false, the programmer will have to manually call update().
    */
    OdometryTank(CustomEncoder &left_enc, CustomEncoder &right_enc, robot_specs_t &config, vex::inertial *imu=NULL, bool is_async=true);

    /**
     * Update the current position on the field based on the sensors
     */
    position_t update() override;


    void set_position(const position_t &newpos=zero_pos) override;

    double get_speed();

private:
    /**
     * Get information from the input hardware and an existing position, and calculate a new current position
     */
    static position_t calculate_new_pos(robot_specs_t &config, position_t &stored_info, double lside_diff, double rside_diff, double angle_deg);

    vex::motor_group *left_side, *right_side;
    CustomEncoder *left_enc, *right_enc;
    vex::inertial *imu;
    robot_specs_t &config;

    double rotation_offset = 0;
    
};