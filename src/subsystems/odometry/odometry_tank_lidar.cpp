// #include "core/subsystems/odometry/odometry_tank_lidar.h"

// #include "core/utils/math/eigen_interface.h"
// #include "core/utils/math/estimator/kalman_filter.h"
// #include "core/utils/math/estimator/unscented_kalman_filter.h"
// #include "core/utils/math/geometry/pose2d.h"
// #include "core/utils/math/geometry/rotation2d.h"
// #include "robot-config.h"
// #include "vex.h"
// #include <functional>

// OdometryTankLidar::OdometryTankLidar(
//   const double &gear_ratio, const double &circumference, const Pose2d &initial_pose, const EVec<3> &P, const EVec<3>
//   &Q, const EVec<2> &R_lidar, vex::inertial &imu, vex::motor_group &left_side, vex::motor_group &right_side, const
//   uint32_t &port, const uint32_t &baudrate, const Transform2d &lidar_offset, const double &Kvl, const double &Kal,
//   const double &Kva, const double &Kaa
// )
//     : observer_(
//         f, h_lidar, RK2_with_input<3, 2>, Q, R_lidar, mean_func_X, mean_func_Y, residual_func_X, residual_func_Y,
//         add_func_X
//       ),
//       OdometryBase(false), gear_ratio_(gear_ratio), circumference_(circumference), imu_(imu), left_side_(left_side),
//       right_side_(right_side), port_(port), baudrate_(baudrate) {
//     this->lidar_offset_ = lidar_offset;
//     this->Kvl_ = Kvl;
//     this->Kal_ = Kal;
//     this->Kva_ = Kva;
//     this->Kaa_ = Kaa;
//     // auto odom_func = [](void *ptr) {
//     //     OdometryTankLidar *self = (OdometryTankLidar *)ptr;
//     //     return self->odom_thread(ptr);
//     // };
//     // auto lidar_func = [](void *ptr) {
//     //     OdometryTankLidar *self = (OdometryTankLidar *)ptr;
//     //     return self->lidar_thread(ptr);
//     // };
//     // vex::task *handle = new vex::task{odom_func, (void *)this};
//     // printf("inited\n");

//     observer_.set_xhat(initial_pose.vector());
//     lidar_handle = new vex::task{lidar_thread, (void *)this};
//     // odom_handle = new vex::task{odom_thread, (void *)this};
// }

// void OdometryTankLidar::set_position(const Pose2d &newpos) {
//     observer_.set_xhat(EVec<3>{newpos.x(), newpos.y(), newpos.rotation().wrapped_radians_180()});
// }

// Pose2d OdometryTankLidar::get_position() const {
//     return Pose2d{observer_.xhat(0), observer_.xhat(1), from_radians(wrap_radians_360(observer_.xhat(2)))};
// }

// Pose2d OdometryTankLidar::update() { return Pose2d(); }

// EVec<3> OdometryTankLidar::h_odom(const EVec<3> &xhat, const EVec<2> &u) {
//     // const double h = xhat(2);
//     // const double l = xhat(3);
//     // const double r = xhat(4);
//     return EVec<3>{0, 0, 0};
// }

// // the walls are 0.25in from the edge of the field, we're measuring inside the walls, not the field
// EVec<2> OdometryTankLidar::h_lidar(const EVec<3> &xhat, const EVec<2> &u) {
//     const double top_right = 142;
//     const double bot_left = 0.75;

//     Pose2d robot_pose = Pose2d{xhat(0), xhat(1), from_radians(xhat(2))};
//     EVec<3> lidar_xhat = (robot_pose + lidar.lidar_offset_).vector();
//     double beam_theta = wrap_radians_180(deg2rad(-u(1))) + lidar_xhat(2);

//     double c = std::cos(beam_theta);
//     double s = std::sin(beam_theta);

//     double d_left = (c < 0) ? ((lidar_xhat(0) - bot_left) / -c) : std::numeric_limits<double>::infinity();
//     double d_right = (c > 0) ? ((top_right - (lidar_xhat(0))) / c) : std::numeric_limits<double>::infinity();
//     double d_bottom = (s < 0) ? ((lidar_xhat(1) - bot_left) / -s) : std::numeric_limits<double>::infinity();
//     double d_top = (s > 0) ? ((top_right - (lidar_xhat(1))) / s) : std::numeric_limits<double>::infinity();

//     double min_distance = std::min({d_left, d_right, d_bottom, d_top});

//     return EVec<2>{min_distance, u(1)};
// }

// EVec<3> OdometryTankLidar::h_full_state(const EVec<3> &xhat, const EVec<2> &u) {
//     return EVec<3>{xhat(0), xhat(1), xhat(2)};
// }

// EVec<3> OdometryTankLidar::f(const EVec<3> &xhat, const EVec<2> &u) {
//     const double x = xhat(0);
//     const double y = xhat(1);
//     const double h = xhat(2);

//     const double v = u(0);
//     const double dx = v * std::cos(h);
//     const double dy = v * std::sin(h);

//     return EVec<3>{dx, dy, u(1)};
// }

// int OdometryTankLidar::odom_thread(void *ptr) {
//     OdometryTankLidar &obj = *((OdometryTankLidar *)ptr);
//     while (obj.running_) {
//         uint64_t now = vexSystemHighResTimeGet() - obj.first_time;

//         // this_thread::sleep_until(now + obj.first_time + 100);

//         if (now - obj.last_time >= 10000) {
//             obj.mutex.lock();
//             obj.odom_update(now - obj.last_time);
//             obj.mutex.unlock();
//         }

//         obj.last_time = now;
//         this_thread::yield();
//     }
//     return 0;
// }
// int OdometryTankLidar::lidar_thread(void *ptr) {
//     OdometryTankLidar &obj = *((OdometryTankLidar *)ptr);
//     vexGenericSerialEnable(obj.port_, 0);
//     vexGenericSerialBaudrate(obj.port_, obj.baudrate_);
//     vexDelay(100);
//     vexGenericSerialFlush(obj.port_);
//     constexpr size_t BUFFER_SIZE = 4;
//     uint8_t buf[BUFFER_SIZE];

//     while (obj.running_) {
//         if (receive_packet(obj.port_, buf, BUFFER_SIZE) != 4) {
//             continue;
//         };

//         uint16_t angle_q6;
//         uint16_t dist;

//         memcpy(&angle_q6, buf, sizeof(angle_q6));
//         memcpy(&dist, buf + sizeof(angle_q6), sizeof(dist));

//         double angle = fmod((angle_q6 * 0.015625) + 351.5, 360);
//         double distance = (dist / 25.4); // to inches

//         if (obj.observer_.xhat(0) > 140 || obj.observer_.xhat(1) > 140 || obj.observer_.xhat(0) < 0 ||
//             obj.observer_.xhat(1) < 0) {
//             obj.observer_.set_xhat(0, gps_sensor.xPosition(vex::distanceUnits::in) + 71.25);
//             obj.observer_.set_xhat(1, gps_sensor.yPosition(vex::distanceUnits::in) + 71.25);
//             obj.observer_.set_xhat(2, deg2rad(wrap_degrees_180(gps_sensor.heading(vex::rotationUnits::deg) + 90)));
//         }

//         if (angle > 360 || angle < 0) {
//             continue;
//         }
//         if (distance > 203 || distance < 0) {
//             continue;
//         }

//         // if inside robot
//         if (angle > 160 && angle < 296) {
//             continue;
//         }

//         // if clamped
//         if (clamper_sys.is_clamped() && angle >= 296) {
//             continue;
//         }

//         if (std::abs(h_lidar(obj.observer_.xhat(), EVec<2>{distance, angle})(0) - distance) > 5) {
//             continue;
//         }

//         // printf("%f,%f\n", distance, angle);
//         // std::cout << obj.observer_.xhat();

//         obj.mutex.lock();
//         obj.lidar_update(distance, angle);
//         obj.mutex.unlock();
//     }
//     vex::this_thread::yield();

//     return 0;
// }
// void OdometryTankLidar::kill_threads() { running_ = false; }

// // Happens every 10ms, when we get data
// // u = [v, omega]
// // we compute v based on 10 ms time interval, and hold it constant on all subsequent lidar updates
// void OdometryTankLidar::odom_update(uint64_t dt_us) {
//     double omega = deg2rad(imu_.gyroRate(vex::axisType::xaxis, vex::dps));
//     double velocity = get_drive_velocity();

//     // predict using saved velocity and angular velocity
//     observer_.predict(EVec<2>{velocity, omega}, dt_us / 1000000.0);
//     printf(
//       " predict, %f, %f, %f, %f, %f\n", observer_.xhat(0), observer_.xhat(1), rad2deg(observer_.xhat(2)), omega,
//       velocity
//     );
// }

// // Happens immediately when we get data
// // y = [distance, angle]
// // the way this does this makes me sad
// // we should latency compensate or use a buffer or some such... ugh
// void OdometryTankLidar::lidar_update(const double &distance_in, const double &angle_deg_cw) {
//     static bool first = true;
//     if (first) {
//         first_time = vexSystemHighResTimeGet();
//         first = false;
//     }
//     uint64_t now = vexSystemHighResTimeGet() - first_time;

//     double omega = deg2rad(imu_.gyroRate(vex::axisType::xaxis, vex::dps));
//     double velocity = get_drive_velocity();

//     double pred = h_lidar(observer_.xhat(), EVec<2>{distance_in, angle_deg_cw})(0);

//     // predict using saved velocity and angular velocity
//     observer_.predict(EVec<2>{velocity, omega}, (now - last_time) / 1000000.0);
//     observer_.correct(
//       EVec<2>{distance_in, angle_deg_cw}, EVec<2>{distance_in, angle_deg_cw}, EVec<2>{distance_in / 20, 1}
//     );
//     if (gps_sensor.quality() == 100) {
//         observer_.correct<3>(
//           EVec<2>{0, 0},
//           EVec<3>{
//             gps_sensor.xPosition(vex::distanceUnits::in) + 71.25, gps_sensor.yPosition(vex::distanceUnits::in)
//             + 71.25, deg2rad(wrap_degrees_180(gps_sensor.heading(vex::rotationUnits::deg) + 90))
//           },
//           h_full_state, EVec<3>{0.1, 0.1, 0.1}, mean_func_X, residual_func_X, residual_func_X, add_func_X
//         );
//     }
//     // observer_.set_xhat(2, 0);
//     last_time = now;

//     if (observer_.xhat(0) > 140 || observer_.xhat(1) > 140 || observer_.xhat(0) < 0 || observer_.xhat(1) < 0) {
//         observer_.set_xhat(0, gps_sensor.xPosition(vex::distanceUnits::in) + 71.25);
//         observer_.set_xhat(1, gps_sensor.yPosition(vex::distanceUnits::in) + 71.25);
//         observer_.set_xhat(2, deg2rad(wrap_degrees_180(gps_sensor.heading(vex::rotationUnits::deg) + 90)));
//     }

//     // printf(
//     //   "%llu, %f, %f, %f, %f, %f, %f\n", now, observer_.xhat(0), observer_.xhat(1),
//     //   rad2deg(observer_.xhat(2)), pred, distance_in, angle_deg_cw
//     // );
//     // printf("%f,%f\n", distance_in, angle_deg_cw);
// }

// /**
//  * COBS decode data from buffer.
//  *
//  * @param buffer Pointer to encoded input bytes.
//  * @param length Number of bytes to decode.
//  * @param data Pointer to decoded output data.
//  *
//  * @return Number of bytes successfully decoded.
//  * @note Stops decoding if delimiter byte is found.
//  */
// size_t cobsDecode(const uint8_t *buffer, size_t length, void *data) {
//     assert(buffer && data);

//     const uint8_t *byte = buffer;      // Encoded input byte pointer
//     uint8_t *decode = (uint8_t *)data; // Decoded output byte pointer

//     for (uint8_t code = 0xff, block = 0; byte < buffer + length; --block) {
//         if (block) // Decode block byte
//             *decode++ = *byte++;
//         else {
//             block = *byte++;             // Fetch the next block length
//             if (block && (code != 0xff)) // Encoded zero, write it unless it's delimiter.
//                 *decode++ = 0;
//             code = block;
//             if (!code) // Delimiter code found
//                 break;
//         }
//     }
//     return (size_t)(decode - (uint8_t *)data);
// }

// /**
//  * Attempts to recieve an entire packet encoded with COBS, stops at delimiter or there's a buffer overflow.
//  *
//  * @param port the port number the serial is plugged into.
//  * @param buffer pointer to a uint8_t[] where we put the data.
//  * @param buffer_size length in bytes of the encoded buffer.
//  * @return 0 success.
//  */
// int receive_cobs_packet(uint32_t port, uint8_t *buffer, size_t buffer_size) {
//     size_t index = 0;

//     while (true) {
//         // wait for a byte (we read byte by byte into our own buffer rather than grabbing the whole packet all at
//         // once)
//         int32_t avail = vexGenericSerialReceiveAvail(port);
//         // printf("%ld\n", avail);
//         if (avail > 0) {
//             uint8_t character = vexGenericSerialReadChar(port);
//             // printf("%X\n", character);

//             // if delimiter
//             if (character == 0x00) {
//                 buffer[index++] = character;
//                 return index; // return packet length
//             }

//             // store character in buffer
//             if (index < buffer_size) {
//                 buffer[index++] = character;
//             } else {
//                 // buffer overflow
//                 printf("bufferoverflow\n");
//                 return -1;
//             }
//         }
//         vex::this_thread::yield();
//     }
// }

// /**
//  * Attempts to receive a packet given a length, this automatically decodes it.
//  *
//  * @param port the port number the serial is plugged into.
//  * @param buffer pointer to a uint8_t[] where we put the data.
//  * @param buffer_size length in bytes of the buffer, after being decoded.
//  */
// int receive_packet(uint32_t port, uint8_t *buffer, size_t buffer_size) {
//     uint8_t cobs_encoded[buffer_size + 2];
//     receive_cobs_packet(port, cobs_encoded, buffer_size + 2);

//     return cobsDecode(cobs_encoded, buffer_size + 2, buffer);
// }

// double OdometryTankLidar::get_drive_velocity() {
//     double left1 = left_front_most.velocity(vex::velocityUnits::rpm) / 60.0 / gear_ratio_ * circumference_;
//     double left2 = left_front_middle.velocity(vex::velocityUnits::rpm) / 60.0 / gear_ratio_ * circumference_;
//     double left3 = left_back_middle.velocity(vex::velocityUnits::rpm) / 60.0 / gear_ratio_ * circumference_;
//     double left4 = left_back_most.velocity(vex::velocityUnits::rpm) / 60.0 / gear_ratio_ * circumference_;
//     double right1 = right_front_most.velocity(vex::velocityUnits::rpm) / 60.0 / gear_ratio_ * circumference_;
//     double right2 = right_front_middle.velocity(vex::velocityUnits::rpm) / 60.0 / gear_ratio_ * circumference_;
//     double right3 = right_back_middle.velocity(vex::velocityUnits::rpm) / 60.0 / gear_ratio_ * circumference_;
//     double right4 = right_back_most.velocity(vex::velocityUnits::rpm) / 60.0 / gear_ratio_ * circumference_;

//     double left = (left1 + left2 + left3 + left4) / 4.0;
//     double right = (right1 + right2 + right3 + right4) / 4.0;

//     return (left + right) / 2.0;
// }
