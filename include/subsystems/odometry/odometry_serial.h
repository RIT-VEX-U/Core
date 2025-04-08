#pragma once

#include "../core/include/utils/math/eigen_interface.h"
#include <v5_api.h>

#include "../core/include/subsystems/custom_encoder.h"
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/utils/math_util.h"

#include "../core/include/utils/math/geometry/pose2d.h"

// conversion factors
#define METER_TO_INCH 39.37f
#define INCH_TO_METER 0.0254f
#define RADIAN_TO_DEGREE 57.2958f
#define DEGREE_TO_RADIAN 0.0174533f

// Conversion factor for the linear position registers. 16-bit signed
// registers with a max value of 10 meters (394 inches) gives a resolution
// of about 0.0003 mps (0.012 ips)
#define METER_TO_INT16 3276.8f
#define INT16_TO_METER 0.00030517578125f

// Conversion factor for the linear velocity registers. 16-bit signed
// registers with a max value of 5 mps (197 ips) gives a resolution of about
// 0.00015 mps (0.006 ips)
#define MPS_TO_INT16 6553.6f
#define INT16_TO_MPS 0.000152587890625f

// Conversion factor for the linear acceleration registers. 16-bit signed
// registers with a max value of 157 mps^2 (16 g) gives a resolution of
// about 0.0048 mps^2 (0.49 mg)
#define MPSS_TO_INT16 208.8378804178797f
#define INT16_TO_MPSS 0.0047884033203125f

// Conversion factor for the angular position registers. 16-bit signed
// registers with a max value of pi radians (180 degrees) gives a resolution
// of about 0.00096 radians (0.0055 degrees)
#define RAD_TO_INT16 10430.37835047045f
#define INT16_TO_RAD 9.587379924285257e-5f

// Conversion factor for the angular velocity registers. 16-bit signed
// registers with a max value of 34.9 rps (2000 dps) gives a resolution of
// about 0.0011 rps (0.061 degrees per second)
#define RPS_TO_INT16 938.733649223929f
#define INT16_TO_RPS 0.0010652648925781f

// Conversion factor for the angular acceleration registers. 16-bit signed
// registers with a max value of 3141 rps^2 (180000 dps^2) gives a
// resolution of about 0.096 rps^2 (5.5 dps^2)
#define RPSS_TO_INT16 10.43037835047045f
#define INT16_TO_RPSS 0.0958737992428526

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
class OdometrySerial : public OdometryBase {
  public:
    /**
     * Construct a new Odometry Serial Object
     */
    OdometrySerial(bool is_async, int32_t port, int32_t baudrate);

    /**
     * Update the current position of the robot once by reading a single packet from the serial port
     *
     * @return the robot's updated position
     */
    Pose2d update() override;

    int receive_packet(uint32_t port, uint8_t *buffer, size_t buffer_size);

    int receive_cobs_packet(uint32_t port, uint8_t *buffer, size_t buffer_size);

    size_t cobs_decode(const uint8_t *buffer, size_t length, void *data);

    size_t cobs_encode(const void *data, size_t length, uint8_t *buffer);

    Pose2d regs_to_pose(uint8_t *raw, float raw_to_xy, float raw_to_h);

    void pose_to_regs(uint8_t *raw, Pose2d &pose, float raw_to_xy, float raw_to_h);

  private:
    int32_t _port;

    Pose2d pos{0, 0, 0};
    Pose2d vel{0, 0, 0};
    Pose2d acc{0, 0, 0};

    double speed;
    double accel;
    double ang_speed_deg;
    double ang_accel_deg;
};
