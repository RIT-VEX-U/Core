#pragma once

// These are required for Eigen to compile
// https://www.vexforum.com/t/eigen-integration-issue/61474/5
#undef __ARM_NEON__
#undef __ARM_NEON
#include <Eigen/Dense>

#include "../core/include/subsystems/custom_encoder.h"
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/utils/math_util.h"

#include "../core/include/utils/math/geometry/pose2d.h"

/**
 * OdometrySerial
 *
 * This class handles the code for an odometry setup where calculations are done on an external coprocessor.
 * Data is sent to the brain via smart port, using a generic serial (UART) connection.
 *
 *
 *
 * This is a "set and forget" class, meaning once the object is created, the robot will immediately begin
 * tracking it's movement in the background.
 *
 * https://rit.enterprise.slack.com/files/U04112Y5RB6/F080M01KPA5/predictperpindiculars2.pdf
 * 2024-2025 Notebook: Entries/Software Entries/Localization/N-Pod Odometry
 *
 * @author Jack Cammarata
 * @date Jan 16 2025
 */
class OdometrySerial : OdometryBase {
  public:
    /**
     * Construct a new Odometry Serial Object
     */
    OdometrySerial(const bool &is_async, const bool &calc_vel_acc_on_brain, const pose_t &initial_pose, const int32_t &port, const int32_t &baudrate);

    void send_config(const pose_t &initial_pose, const bool &calc_vel_acc_on_brain);

    /**
     * Update the current position of the robot once by reading a single packet from the serial port
     *
     * @return the robot's updated position
     */
    pose_t update();

    /**
     * Resets the position and rotational data to the input.
     */
    void set_position(const Pose2d &new_pose);

    int receive_cobs_packet(uint32_t port, uint8_t *buffer, size_t buffer_size);

    pose_t get_position(void);

    Pose2d get_pose2d(void);

    static int background_task(void *ptr);

    size_t cobs_decode(const uint8_t *buffer, size_t length, void *data);

    size_t cobs_encode(const void *data, size_t length, uint8_t *buffer);

  private:
    int32_t _port;

    bool calc_vel_acc_on_brain;

    Pose2d pose;

    Transform2d pose_offset;

    double speed;
    double accel;
    double ang_speed_deg;
    double ang_accel_deg;
};
