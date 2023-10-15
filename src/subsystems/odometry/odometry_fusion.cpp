#include "../core/include/subsystems/odometry/odometry_fusion.h"

OdometryFusion::OdometryFusion(vex::motor_group &left_side, vex::motor_group &right_side, robot_specs_t &config, vex::inertial imu, bool is_async = true)
: OdometryTank(left_side, right_side, config, &imu, is_async)
{

}

OdometryFusion::OdometryFusion(CustomEncoder &left_custom_enc, CustomEncoder &right_custom_enc, robot_specs_t &config, vex::inertial imu, bool is_async = true)
: OdometryTank(left_custom_enc, right_custom_enc, config, &imu, is_async)
{

}

/**
 * Get information from the input hardware and an existing position, and calculate a new current position
 */
pose_t OdometryFusion::calculate_new_pos(robot_specs_t &config, pose_t &stored_info, double lside_diff, double rside_diff, double angle_deg)
{
    
}