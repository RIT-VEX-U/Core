#pragma once

// These are required for Eigen to compile
// https://www.vexforum.com/t/eigen-integration-issue/61474/5
#undef __ARM_NEON__
#undef __ARM_NEON
#include <Eigen/Dense>

#include "../core/include/subsystems/custom_encoder.h"
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/utils/math_util.h"

/**
 * OdometryNWheel
 *
 * This class handles the code for an N-pod odometry setup, where there are N <WHEELS> free spinning omni wheels
 * (dead wheels) placed in any known configuration on the robot.
 *
 * Example of a possible wheel configuration:
 *
 *  +Y   ---------------
 *  ^    |    ===      |
 *  |    |             |
 *  |    | ||   O      |
 *  |    | ||          |
 *  |    |      ===    |
 *  |    ---------------
 *  |
 *  +-------------------> + X
 *
 * Where O is the center of rotation. The robot will monitor the changes in rotation of these wheels, use this to
 * calculate a pose delta, then integrate the deltas over time to determine the robot's position.
 *
 * This is a "set and forget" class, meaning once the object is created, the robot will immediately begin
 * tracking it's movement in the background.
 *
 * https://rit.enterprise.slack.com/files/U04112Y5RB6/F080M01KPA5/predictperpindiculars2.pdf
 * 2024-2025 Notebook: Entries/Software Entries/Localization/N-Pod Odometry
 *
 * @author Jack Cammarata, Richie Sommers
 * @date Nov 14 2024
 *
 */

/**
 * tracking_wheel_cfg_t holds all the specifications for a single tracking wheel
 * The units for x, y, and radius will determine the units of the position estimate
 */
typedef struct {
  double x;         /**< x position of the center of the wheel */
  double y;         /**< y position of the center of the wheel */
  double theta_rad; /**< angle between wheel direction and x axis in the robot frame */
  double radius;    /**< radius of the wheel */
} tracking_wheel_cfg_t;

template <int WHEELS> class OdometryNWheel : public OdometryBase {
public:
  /**
   * Construct a new Odometry N Wheel object
   *
   * @param encoders std::array containing each CustomEncoder
   * @param wheel_configs std::array containing each tracking_wheel_cfg
   * @param is_async true to constantly run in the background
   */
  OdometryNWheel(
    const std::array<CustomEncoder, WHEELS> &encoders, const std::array<tracking_wheel_cfg_t, WHEELS> wheel_configs,
    vex::inertial *imu, bool is_async
  )
      : OdometryBase(is_async), imu(imu), encoders(encoders) {
    Eigen::Matrix<double, WHEELS, 3> transfer_matrix;
    for (int i = 0; i < WHEELS; i++) {
      double x = wheel_configs[i].x;
      double y = wheel_configs[i].y;
      double theta_rad = wheel_configs[i].theta_rad;
      double radius = wheel_configs[i].radius;
      wheel_radii(i) = radius;

      double x_factor = cos(theta_rad);
      double y_factor = -sin(theta_rad);
      double theta_factor = -(x * sin(theta_rad)) - (y * cos(theta_rad));

      if (abs(y_factor) < 0.000000001) {
        y_factor = 0;
      }
      if (abs(x_factor) < 0.000000001) {
        x_factor = 0;
      }

      transfer_matrix.row(i) << x_factor, y_factor, theta_factor;
    }

    transfer_matrix_pseudoinverse = transfer_matrix.completeOrthogonalDecomposition().pseudoInverse();

    old_heading = 0;
  }

  /**
   * Update the current position of the robot once, using the current state of
   * the encoders and the previous known location
   *
   * @return the robot's updated position
   */
  pose_t update() override {
    Eigen::Vector<double, WHEELS> radian_deltas;

    for (int i = 0; i < WHEELS; i++) {
      double angle_rad = encoders[i].position(rev) * M_PI * 2;
      radian_deltas(i) = angle_rad - old_wheel_angles[i];

      old_wheel_angles[i] = angle_rad;
    }

    pose_t updated_pos = calculate_new_pos(radian_deltas, current_pos);

    angle = 0;

    // Translate "0 forward and clockwise positive" to "CCW positive and radians"
    angle = -imu->rotation(vex::rotationUnits::rev) * 2 * M_PI;

    // Offset the angle, if we've done a set_position
    angle += rotation_offset;

    static pose_t last_pos = updated_pos;
    static double last_speed = 0;
    static double last_ang_speed = 0;
    static timer tmr;

    double speed_local = 0;
    double accel_local = 0;
    double ang_speed_local = 0;
    double ang_accel_local = 0;
    bool update_vel_accel = tmr.time(sec) > 0.1;

    // This loop runs too fast. Only check at LEAST every 1/10th sec
    if (update_vel_accel) {
      // Calculate robot velocity
      speed_local = pos_diff(updated_pos, last_pos) / tmr.time(sec);

      // Calculate robot acceleration
      accel_local = (speed_local - last_speed) / tmr.time(sec);

      // Calculate robot angular velocity (deg/sec)
      ang_speed_local = smallest_angle(updated_pos.rot, last_pos.rot) / tmr.time(sec);

      // Calculate robot angular acceleration (deg/sec^2)
      ang_accel_local = (ang_speed_local - last_ang_speed) / tmr.time(sec);

      tmr.reset();
      last_pos = updated_pos;
      last_speed = speed_local;
      last_ang_speed = ang_speed_local;
    }

    this->current_pos = updated_pos;
    if (update_vel_accel) {
      this->speed = speed_local;
      this->accel = accel_local;
      this->ang_speed_deg = ang_speed_local;
      this->ang_accel_deg = ang_accel_local;
    }

    old_heading = angle;

    return current_pos;
  }

  /**
   * Resets the position and rotational data to the input.
   */
  void set_position(const pose_t &newpos) override {
    mut.lock();
    rotation_offset = newpos.rot - (current_pos.rot - rotation_offset);
    mut.unlock();

    OdometryBase::set_position(newpos);
  }

private:
  /**
   * Applies a twist (pose delta) to a pose by including first order dynamics of heading.
   * Can be thought of as applying a twist as following an arc rather than a straight line.
   *
   * https://file.tavsys.net/control/controls-engineering-in-frc.pdf#section.10.2
   *
   * @param old_pose  The pose to which the twist will be applied
   * @param twist     The twist, represents a pose delta
   */
  pose_t pose_exponential(const Eigen::Vector3d old_pose, const Eigen::Vector3d twist) {
    double dtheta = twist(2);

    double sinTheta = sin(dtheta);
    double cosTheta = cos(dtheta);

    double sinOldTheta = sin(old_pose(2));
    double cosOldTheta = cos(old_pose(2));

    Eigen::Matrix3d rotation{{cosOldTheta, -sinOldTheta, 0}, {sinOldTheta, cosOldTheta, 0}, {0, 0, 1}};
    Eigen::Matrix3d integrated_rotation;

    // when the angle change is very small, we use a taylor series to approximate
    if (abs(dtheta) < 1e-9) {
      integrated_rotation.row(0) << 1.0 - ((dtheta * dtheta) / 6.0), -(dtheta / 2.0), 0;
      integrated_rotation.row(1) << (dtheta / 2.0), 1.0 - ((dtheta * dtheta) / 6.0), 0;
      integrated_rotation.row(2) << 0, 0, 1;
    } else {
      integrated_rotation.row(0) << sinTheta / dtheta, (cosTheta - 1.0) / dtheta, 0;
      integrated_rotation.row(1) << (1 - cosTheta) / dtheta, sinTheta / dtheta, 0;
      integrated_rotation.row(2) << 0, 0, 1;
    }

    Eigen::Vector3d global_pose_delta = rotation * integrated_rotation * twist;

    pose_t newPose;

    newPose.x = old_pose(0) + global_pose_delta(0);
    newPose.y = old_pose(1) + global_pose_delta(1);
    newPose.rot = wrap_angle_rad(old_pose(2) + global_pose_delta(2));

    return newPose;
  }

  /**
   * Calculation method for the robot's new position using the change in encoders, and the old pose, the wheel
   * configurations are stored as class members.
   *
   * @param radian_deltas vector containing the change of angle of each wheel (radians)
   * @param old_pose The robot's previous position (x, y, rot)
   * @return The robot's new position (x, y, rot)
   */
  pose_t calculate_new_pos(Eigen::Vector<double, WHEELS> radian_deltas, pose_t old_pose) {
    // Mr T = E -> Mr^{+} E = T
    Eigen::Vector3d pose_delta = transfer_matrix_pseudoinverse * (radian_deltas.asDiagonal() * wheel_radii);
    Eigen::Vector3d old_pose_vector{old_pose.x, old_pose.y, old_pose.rot};

    // we achieve better performance by using the imu for rotation directly
    pose_delta(2) = angle - old_heading;

    pose_t new_pose = pose_exponential(old_pose_vector, pose_delta);
    new_pose.rot = angle;

    return new_pose;
  }

  double angle;
  double old_heading;
  double rotation_offset;

  vex::inertial *imu;

  Eigen::Matrix<double, 3, WHEELS> transfer_matrix_pseudoinverse;

  std::array<CustomEncoder, WHEELS> encoders;
  Eigen::Vector<double, WHEELS> wheel_radii;
  Eigen::Vector<double, WHEELS> old_wheel_angles;
};
