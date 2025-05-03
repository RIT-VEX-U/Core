#pragma once

#include "core/utils/math/eigen_interface.h"
#include "core/utils/math/estimator/unscented_kalman_filter.h"
#include "core/utils/math/geometry/pose2d.h"
#include "vex.h"

class OdometryTankLidar {
  public:
    OdometryTankLidar(
      const double &gear_ratio, const double &circumference, const Pose2d &initial_pose, const EVec<3> &P,
      const EVec<3> &Q, const EVec<2> &R_lidar, vex::inertial &imu, vex::motor_group &left_side,
      vex::motor_group &right_side, const uint32_t &port, const uint32_t &baudrate, const Transform2d &lidar_offset, const double &Kvl, const double &Kal, const double &Kva, const double &Kaa
    );

    void set_position(const Pose2d &newpos);
    Pose2d get_position() const;

    static EVec<3> h_odom(const EVec<3> &xhat, const EVec<2> &u);
    static EVec<2> h_lidar(const EVec<3> &xhat, const EVec<2> &u);
    static EVec<3> h_full_state(const EVec<3> &xhat, const EVec<2> &u);
    static EVec<3> f(const EVec<3> &xhat, const EVec<2> &u);

    static int odom_thread(void *ptr);
    static int lidar_thread(void *ptr);
    void kill_threads();

    // Happens every 10ms, when we get data
    // y = [dleft, dright, theta]
    // we compute v and omega based on 10ms time interval, ugh
    void odom_update(uint64_t dt_us);

    // Happens immediately when we get data
    // y = [distance, angle]
    // the way this does this makes me sad
    // we should latency compensate or use a buffer or some such... ugh
    void lidar_update(const double &distance_in, const double &angle_deg_cw);

    UKF<3, 2, 2> observer_;
    EVec<2> R_lidar;
    // EVec<2> R_odom;
    EVec<3> Q;

    double lidar_tolerance = 20;

    vex::inertial &imu_;
    vex::motor_group &left_side_;
    vex::motor_group &right_side_;

    double current_velocity;
    double current_angular_velocity;

    double prev_left;
    double prev_right;

    double circumference_;
    double gear_ratio_;
    std::atomic<uint64_t> last_time;
    std::atomic<uint64_t> first_time;
    Pose2d current_pos;

    double Kvl_, Kal_, Kva_, Kaa_;
    Transform2d lidar_offset_;



    vex::mutex mutex;

    uint32_t port_;
    uint32_t baudrate_;

    std::atomic<bool> running_{true};

    vex::task *lidar_handle;
    vex::task *odom_handle;


    static EVec<3> mean_func_X(const EMat<3, 5> &sigmas, const EVec<5> &Wm) {
        EVec<3> x = EVec<3>::Zero();

        double c = 0;
        double s = 0;

        for (int i = 0; i < 5; i++) {
            x(0) += sigmas(0, i) * Wm(i); // x
            x(1) += sigmas(1, i) * Wm(i); // y

            c += std::cos(sigmas(2, i)) * Wm(i); // h
            s += std::sin(sigmas(2, i)) * Wm(i); // h
        }
        x(2) = std::atan2(s, c); // h

        return x;
    }

    static EVec<2> mean_func_Y(const EMat<2, 5> &sigmas, const EVec<5> &Wm) {
        EVec<2> y = EVec<2>::Zero();

        double c = 0;
        double s = 0;

        for (int i = 0; i < 5; i++) {
            y(0) += sigmas(0, i) * Wm(i);
            c += std::cos(sigmas(1, i)) * Wm(i);
            s += std::sin(sigmas(1, i)) * Wm(i);
        }
        y(1) = std::atan2(s, c);
        return y;
    }

    static EVec<3> residual_func_X(const EVec<3> &a, const EVec<3> &b) {
        return {a(0) - b(0), a(1) - b(1), wrap_radians_180(a(2) - b(2))};
    }

    static EVec<3> add_func_X(const EVec<3> &a, const EVec<3> &b) {
        return {a(0) + b(0), a(1) + b(1), wrap_radians_180(a(2) + b(2))};
    }

    static EVec<2> residual_func_Y(const EVec<2> &a, const EVec<2> &b) {
        return {a(0) - b(0), wrap_radians_180(a(1) - b(1))};
    }

    static EVec<3> residual_func_Y_odom(const EVec<3> &a, const EVec<3> &b) {
        return {wrap_radians_180(a(0) - b(0)), a(1) - b(1), a(2) - b(2)};
    }

    double get_drive_velocity();


};

    /**
     * COBS decode data from buffer.
     *
     * @param buffer Pointer to encoded input bytes.
     * @param length Number of bytes to decode.
     * @param data Pointer to decoded output data.
     *
     * @return Number of bytes successfully decoded.
     * @note Stops decoding if delimiter byte is found.
     */
    static size_t cobsDecode(const uint8_t *buffer, size_t length, void *data);

    /**
     * Attempts to recieve an entire packet encoded with COBS, stops at delimiter or there's a buffer overflow.
     *
     * @param port the port number the serial is plugged into.
     * @param buffer pointer to a uint8_t[] where we put the data.
     * @param buffer_size length in bytes of the encoded buffer.
     * @return 0 success.
     */
    static int receive_cobs_packet(uint32_t port, uint8_t *buffer, size_t buffer_size);

    /**
     * Attempts to receive a packet given a length, this automatically decodes it.
     *
     * @param port the port number the serial is plugged into.
     * @param buffer pointer to a uint8_t[] where we put the data.
     * @param buffer_size length in bytes of the buffer, after being decoded.
     */
    static int receive_packet(uint32_t port, uint8_t *buffer, size_t buffer_size);


