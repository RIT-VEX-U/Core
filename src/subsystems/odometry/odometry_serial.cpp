#include "../core/include/utils/math/eigen_interface.h"

#include "../core/include/subsystems/odometry/odometry_serial.h"
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/utils/math_util.h"
#include "../core/include/utils/math/geometry/pose2d.h"

/**
 * OdometrySerial
 *
 * This class handles the code for an odometry setup where calculations are done on an external coprocessor.
 * Data is sent to the brain via smart port, using a generic serial (UART) connection.
 *
 * This is a "set and forget" class, meaning once the object is created, the robot will immediately begin
 * tracking it's movement in the background.
 *
 * @author Jack Cammarata
 * @date Apr 8 2025
 */

/**
 * Construct a new Odometry Serial object
 */
OdometrySerial::OdometrySerial(
  bool is_async, int32_t port,
  int32_t baudrate
)
    : OdometryBase(is_async), _port(port) {
    vexGenericSerialEnable(_port, 0);
    vexGenericSerialBaudrate(_port, baudrate);
}

/**
 * Attempts to receive a packet given a length, this automatically decodes it.
 * 
 * @param port the port number the serial is plugged into
 * @param buffer pointer to a uint8_t[] where we put the data
 * @param buffer_size length in bytes of the buffer, after being decoded
 */
int OdometrySerial::receive_packet(uint32_t port, uint8_t *buffer, size_t buffer_size) {
    uint8_t cobs_encoded[buffer_size + 2];
    receive_cobs_packet(port, cobs_encoded, buffer_size + 2);  
    
    return cobs_decode(cobs_encoded, buffer_size + 2, buffer);
}

/**
 * Attempts to recieve an entire packet encoded with COBS, stops at delimiter or there's a buffer overflow
 * 
 * @param port the port number the serial is plugged into
 * @param buffer pointer to a uint8_t[] where we put the data
 * @param buffer_size length in bytes of the encoded buffer
 * @return 0 success
 */
int OdometrySerial::receive_cobs_packet(uint32_t port, uint8_t *buffer, size_t buffer_size) {
    size_t index = 0;

    while (true) {        
        // wait for a byte (we read byte by byte into our own buffer rather than grabbing the whole packet all at once)
        if (vexGenericSerialReceiveAvail(port) > 0) {
            uint8_t character = vexGenericSerialReadChar(port);

            // if delimiter
            if (character == 0x00) {
                buffer[index++] = character;
                return index; // return packet length
            }

            // store character in buffer
            if (index < buffer_size) {
                buffer[index++] = character;
            } else {
                // buffer overflow
                printf("bufferoverflow\n");
                return -1;
            }
        }
        vex::this_thread::yield();
    }
}

/**
 * Update the current position of the robot once by reading a single packet from the serial port, then using it to
 * update pos, vel, acc, speed, accel, angular speed, angular accel
 *
 * @return the robot's updated position
 */
Pose2d OdometrySerial::update() {
    receive_packet(_port, raw, sizeof(raw));

    this->pos = this->pos + (regs_to_pose(raw, INT16_TO_METER, INT16_TO_RAD) - prev);
    this->vel = regs_to_pose(raw + 6, INT16_TO_MPS, INT16_TO_RPS);
    this->acc = regs_to_pose(raw + 12, INT16_TO_MPSS, INT16_TO_RPSS);

    // speed = ||v||
    this->speed = vel.translation().norm();
    // dspeed/dt
    // (v * a) / ||v||
    this->accel = (vel.translation() * acc.translation()) / (vel.translation().norm());
    this->ang_speed_deg = vel.rotation().degrees();
    this->ang_accel_deg = acc.rotation().degrees();

    this->prev = regs_to_pose(raw, INT16_TO_METER, INT16_TO_RAD);

    return this->pos;
}

/** COBS encode data to buffer
 * 
 * @param data Pointer to input data to encode
 * @param length Number of bytes to encode
 * @param buffer Pointer to encoded output buffer
 * 
 * @return Encoded buffer length in bytes
 * @note Does not output delimiter byte
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

    *encode++ = 0x00; // Append 0x00 delimiter

    return (size_t)(encode - buffer);
}

/** COBS decode data from buffer
 * @param buffer Pointer to encoded input bytes
 * @param length Number of bytes to decode
 * @param data Pointer to decoded output data
 * 
 * @return Number of bytes successfully decoded
 * @note Stops decoding if delimiter byte is found
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

Pose2d OdometrySerial::regs_to_pose(uint8_t *raw, float raw_to_xy, float raw_to_h) {
    int16_t raw_x = (raw[1] << 8) | raw[0];
    int16_t raw_y = (raw[3] << 8) | raw[2];
    int16_t raw_h = (raw[5] << 8) | raw[4];

    double x = raw_x * raw_to_xy * METER_TO_INCH;
    double y = raw_y * raw_to_xy * METER_TO_INCH;
    double h = raw_h * raw_to_h;

    return Pose2d(x, y, h);
}

void OdometrySerial::pose_to_regs(uint8_t *raw, Pose2d &pose, float xy_to_raw, float h_to_raw) {
    int16_t rawx = (float)(pose.x()) * xy_to_raw;
    int16_t rawy = (float)(pose.y()) * xy_to_raw;
    int16_t rawh = (float)(pose.rotation().wrapped_radians_360()) * h_to_raw;

    raw[0] = rawx & 0xFF;
    raw[1] = (rawx >> 8) & 0xFF;
    raw[2] = rawy & 0xFF;
    raw[3] = (rawy >> 8) & 0xFF;
    raw[4] = rawh & 0xFF;
    raw[5] = (rawh >> 8) & 0xFF;
}
