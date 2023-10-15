#pragma once
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/subsystems/custom_encoder.h"

typedef struct 
{
    int movingavg_buffersize;
} odomfus_cfg_t;

class OdometryFusion : OdometryBase
{
public:

    OdometryFusion(vex::motor_group &left_side, vex::motor_group &right_side, robot_specs_t &config, vex::inertial &imu, bool is_async = true);
    OdometryFusion(CustomEncoder &left_custom_enc, CustomEncoder &right_custom_enc, robot_specs_t &config, vex::inertial &imu, bool is_async = true);

    pose_t update() override;


private:

    pose_t calculate_new_pose(robot_specs_t &config, pose_t &stored_info, double lside_rev, double rside_rev, double side_accel, double angle_deg);

    vex::motor_group *left_side, *right_side;
    CustomEncoder *left_custom_enc, *right_custom_enc;
    robot_specs_t &config;
    
    vex::inertial &imu;
    
};