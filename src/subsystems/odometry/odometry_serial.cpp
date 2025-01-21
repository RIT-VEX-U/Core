// These are required for Eigen to compile
// https://www.vexforum.com/t/eigen-integration-issue/61474/5
#undef __ARM_NEON__
#undef __ARM_NEON
#include <Eigen/Dense>

#include "../core/include/subsystems/odometry/odometry_serial.h"

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

/**
 * Construct a new Odometry Serial Object
 */
OdometrySerial::OdometrySerial(
  bool is_async, bool calc_vel_acc_on_brain, pose_t initial_pose, pose_t sensor_offset, int32_t port,
  int32_t baudrate
)
    : OdometryBase(is_async), calc_vel_acc_on_brain(calc_vel_acc_on_brain), pose(Pose2d(0, 0, 0)),
      pose_offset(Transform2d(0, 0, 0)), _port(port) {
    vexGenericSerialEnable(_port, 0);
    vexGenericSerialBaudrate(_port, baudrate);
    send_config(initial_pose, sensor_offset, calc_vel_acc_on_brain);

    if (is_async) {
      printf("thread started\n");
    }
}

void OdometrySerial::send_config(const pose_t &initial_pose, const pose_t &sensor_offset, const bool &calc_vel_acc_on_brain) {
    uint8_t raw[(sizeof(initial_pose)) + sizeof(calc_vel_acc_on_brain)];
    uint8_t cobs_encoded[sizeof(raw) + 1];

    float initialx = (float)initial_pose.x;
    float initialy = (float)initial_pose.y;
    float initialrot = (float)initial_pose.rot;

    float offsetx = (float)initial_pose.x;
    float offsety = (float)initial_pose.y;
    float offsetrot = (float)initial_pose.rot;

    memcpy(&raw[0], &initial_pose.x, sizeof(float));
    memcpy(&raw[4], &initial_pose.y, sizeof(float));
    memcpy(&raw[8], &initial_pose.rot, sizeof(float));
    memcpy(&raw[12], &sensor_offset.x, sizeof(float));
    memcpy(&raw[16], &sensor_offset.y, sizeof(float));
    memcpy(&raw[20], &sensor_offset.rot, sizeof(float));
    memcpy(&raw[24], &calc_vel_acc_on_brain, sizeof(bool));

    cobs_encode(raw, sizeof(raw), cobs_encoded);

    vexGenericSerialTransmit(_port, cobs_encoded, sizeof(cobs_encoded));

    // vexDelay(100);
}

int OdometrySerial::receive_cobs_packet(uint32_t port, uint8_t *buffer, size_t buffer_size) {
    size_t index = 0;

    while (true) {
        // wait for a byte (we read byte by byte into our own buffer rather than grabbing the whole packet all at
        // once)
        if (vexGenericSerialReceiveAvail(port) > 0) {
            uint8_t character = vexGenericSerialReadChar(port);

            // if delimiter
            if (character == 0x00) {
                return index; // return packet length
            }

            // store character in buffer
            if (index < buffer_size) {
                buffer[index++] = character;
            } else {
                // buffer overflow
                return -1;
            }
        }
        vex::this_thread::yield();
    }
}

/**
 * Update the current position of the robot once by reading a single packet from the serial port
 *
 * @return the robot's updated position
 */
pose_t OdometrySerial::update() {
    uint8_t cobs_encoded_size;
    uint8_t packet_size;

    if (calc_vel_acc_on_brain) {
        cobs_encoded_size = 13;
        packet_size = 12;
    } else {
        cobs_encoded_size = 37;
        packet_size = 36;
    }

    uint8_t cobs_encoded[cobs_encoded_size];
    uint8_t decoded_packet[packet_size];

    int packet_length = receive_cobs_packet(_port, cobs_encoded, cobs_encoded_size);
    Pose2d updated_pose(0, 0, 0);

    if (packet_length == cobs_encoded_size) {
        if (cobs_decode(cobs_encoded, packet_length, decoded_packet) == 12) {
            float *floats = (float *)decoded_packet;
            updated_pose = Pose2d(Translation2d(floats[0], floats[1]), from_degrees(floats[2]));

        } else {
            printf("OdometrySerial: Invalid COBS encoding\n");
            return {0, 0, 0};
        }
    } else if (packet_length == -1) {
        printf("OdometrySerial: Buffer overflow\n");
        return {0, 0, 0};
    }

    if (calc_vel_acc_on_brain) {
        static Pose2d last_pose = updated_pose;
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
            speed_local = (updated_pose.translation().distance(last_pose.translation())) / tmr.time(sec);

            // Calculate robot acceleration
            accel_local = (speed_local - last_speed) / tmr.time(sec);

            // Calculate robot angular velocity (deg/sec)
            ang_speed_local =
              (updated_pose.rotation() - last_pose.rotation()).wrapped_degrees_180() / tmr.time(sec);

            // Calculate robot angular acceleration (deg/sec^2)
            ang_accel_local = (ang_speed_local - last_ang_speed) / tmr.time(sec);

            tmr.reset();
            last_pose = updated_pose;
            last_speed = speed_local;
            last_ang_speed = ang_speed_local;
        }

        this->pose = updated_pose;
        if (update_vel_accel) {
            this->speed = speed_local;
            this->accel = accel_local;
            this->ang_speed_deg = ang_speed_local;
            this->ang_accel_deg = ang_accel_local;
        }

        std::cout << vexSystemHighResTimeGet() << ", " << pose.rotation().wrapped_degrees_360() << std::endl;

        return {pose.x(), pose.y(), pose.rotation().wrapped_degrees_180()};
    }

    return {pose.x(), pose.y(), pose.rotation().wrapped_degrees_360()};
}

/**
 * Resets the position and rotational data to the input.
 */
void OdometrySerial::set_position(const Pose2d &new_pose) {
    // regrettably we can't use the functions for this because it rotates the translation... ughies
    pose_offset = Transform2d(new_pose.translation() - pose.translation(), new_pose.rotation() - pose.rotation());
}

/**
 * Gets the current position and rotation
 * @return the position that the odometry believes the robot is at
 */
pose_t OdometrySerial::get_position(void) {
    Pose2d pose = get_pose2d();
    return pose_t{pose.x(), pose.y(), pose.rotation().wrapped_degrees_360()};
}

Pose2d OdometrySerial::get_pose2d(void) {
    // regrettably we can't use the functions for this because it rotates the translation... ughies
    return Pose2d(pose.translation() + pose_offset.translation(), pose.rotation() + pose_offset.rotation());
}

/** COBS encode data to buffer
      @param data Pointer to input data to encode
      @param length Number of bytes to encode
      @param buffer Pointer to encoded output buffer
      @return Encoded buffer length in bytes
      @note Does not output delimiter byte
*/
size_t OdometrySerial::cobs_encode(const void *data, size_t length, uint8_t *buffer) {
    assert(data && buffer);

    uint8_t *encode = buffer;  // Encoded byte pointer
    uint8_t *codep = encode++; // Output code pointer
    uint8_t code = 1;          // Code value

    for (const uint8_t *byte = (const uint8_t *)data; length--; ++byte) {
        if (*byte) // Byte not zero, write it
            *encode++ = *byte, ++code;

        if (!*byte || code == 0xff) // Input is zero or block completed, restart
        {
            *codep = code, code = 1, codep = encode;
            if (!*byte || length)
                ++encode;
        }
    }
    *codep = code; // Write final code value

    return (size_t)(encode - buffer);
}

/** COBS decode data from buffer
  @param buffer Pointer to encoded input bytes
  @param length Number of bytes to decode
  @param data Pointer to decoded output data
  @return Number of bytes successfully decoded
  @note Stops decoding if delimiter byte is found
*/
size_t OdometrySerial::cobs_decode(const uint8_t *buffer, size_t length, void *data) {
    assert(buffer && data);

    const uint8_t *byte = buffer;      // Encoded input byte pointer
    uint8_t *decode = (uint8_t *)data; // Decoded output byte pointer

    for (uint8_t code = 0xff, block = 0; byte < buffer + length; --block) {
        if (block) // Decode block byte
            *decode++ = *byte++;
        else {
            block = *byte++;             // Fetch the next block length
            if (block && (code != 0xff)) // Encoded zero, write it unless it's delimiter.
                *decode++ = 0;
            code = block;
            if (!code) // Delimiter code found
                break;
        }
    }

    return (size_t)(decode - (uint8_t *)data);
}
