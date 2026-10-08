#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "core/utils/kinematics/differential_drive_kinematics.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"
#include "core/utils/units.h"

using namespace units::literals;

/**
 * @brief Trajectory constraint that limits acceleration based on available battery voltage and
 * motor capabilities.
 */
class TankVoltageConstraint : public TrajectoryConstraint {
   public:
    /**
     * @brief Constructs a TankVoltageConstraint from track width.
     * @param Kv Linear velocity feedforward constant.
     * @param Ka Linear acceleration feedforward constant.
     * @param max_voltage Maximum available voltage.
     * @param track_width Robot track width (distance between left and right wheels).
     */
    TankVoltageConstraint(
            units::LinearVelocityFeedforward Kv,
            units::LinearAccelerationFeedforward Ka,
            units::Voltage max_voltage,
            units::Length track_width
    )
        : Kv_(Kv), Ka_(Ka), max_voltage_(max_voltage), kinematics_(track_width) {}

    /**
     * @brief Constructs a TankVoltageConstraint from DifferentialDriveKinematics.
     * @param Kv Linear velocity feedforward constant.
     * @param Ka Linear acceleration feedforward constant.
     * @param max_voltage Maximum available voltage.
     * @param kinematics Kinematics model for differential drive.
     */
    TankVoltageConstraint(
            units::LinearVelocityFeedforward Kv,
            units::LinearAccelerationFeedforward Ka,
            units::Voltage max_voltage,
            const DifferentialDriveKinematics& kinematics
    )
        : Kv_(Kv), Ka_(Ka), max_voltage_(max_voltage), kinematics_(kinematics) {}

    /**
     * @brief Computes maximum allowed velocity.
     * @param pose 2D position and orientation.
     * @param curvature Path curvature in rad/meter.
     * @param velocity Candidate linear velocity.
     * @return Constrained units::Velocity limit.
     */
    units::Velocity max_velocity(
            const Pose2d& pose, units::Curvature curvature, units::Velocity velocity
    ) const override {
        return units::Velocity(std::numeric_limits<double>::max());
    }

    /**
     * @brief Computes minimum and maximum allowed linear accelerations based on voltage limits.
     * @param pose 2D position and orientation.
     * @param curvature Path curvature in rad/meter.
     * @param speed Current scalar speed.
     * @return MinMax acceleration bounds struct.
     */
    MinMax min_max_acceleration(
            const Pose2d& pose, units::Curvature curvature, units::Velocity speed
    ) const override {
        units::AngularVelocity omega = speed * curvature;
        ChassisVelocities chassis_speeds(speed, 0_inps, omega);
        DifferentialDriveWheelVelocities wheel_speeds = kinematics_.to_wheel_velocities(chassis_speeds);

        units::Velocity max_wheel_speed = units::max(wheel_speeds.left, wheel_speeds.right);
        units::Velocity min_wheel_speed = units::min(wheel_speeds.left, wheel_speeds.right);

        units::Acceleration max_wheel_acceleration =
                (max_voltage_ - (Kv_ * max_wheel_speed)) / Ka_;
        units::Acceleration min_wheel_acceleration =
                (-max_voltage_ - Kv_ * min_wheel_speed) / Ka_;

        units::Acceleration max_chassis_acceleration;
        units::Acceleration min_chassis_acceleration;

        const double curvd = std::abs(curvature.to(units::radpm));
        const double twd = kinematics_.track_width.to(units::m);

        if (units::abs(speed) <= 1e-9_mps) {
            const double denom = 1.0 + (twd * curvd / 2.0);
            max_chassis_acceleration =
                    max_wheel_acceleration / (std::abs(denom) < 1e-6 ? 1e-6 : denom);
            min_chassis_acceleration =
                    min_wheel_acceleration / (std::abs(denom) < 1e-6 ? 1e-6 : denom);
        } else {
            const double speed_val = speed.to(units::inps);
            const double speed_sgn = speed_val < 0.0 ? -1.0 : 1.0;
            const double max_denom = 1.0 + (twd * curvd * speed_sgn / 2.0);
            const double min_denom = 1.0 - (twd * curvd * speed_sgn / 2.0);

            max_chassis_acceleration =
                    max_wheel_acceleration / (std::abs(max_denom) < 1e-6 ? 1e-6 : max_denom);
            min_chassis_acceleration =
                    min_wheel_acceleration / (std::abs(min_denom) < 1e-6 ? 1e-6 : min_denom);
        }

        if (units::abs(curvature) > 1E-9_radpm && (kinematics_.track_width / 2.0) > 1_rad / units::abs(curvature)) {
            if (speed > 0_mps && min_chassis_acceleration > 0_inps2) {
                min_chassis_acceleration = -min_chassis_acceleration;
            } else if (speed < 0_mps && max_chassis_acceleration < 0_inps2) {
                max_chassis_acceleration = -max_chassis_acceleration;
            }
        }

        return {min_chassis_acceleration, max_chassis_acceleration};
    }

    /**
     * @brief Polymorphic deep-copy factory method.
     * @return Unique pointer to cloned TankVoltageConstraint instance.
     */
    std::unique_ptr<TrajectoryConstraint> clone() const override {
        return std::unique_ptr<TrajectoryConstraint>(new TankVoltageConstraint(*this));
    }

   private:
    units::LinearVelocityFeedforward Kv_;      ///< Linear velocity feedforward constant
    units::LinearAccelerationFeedforward Ka_;  ///< Linear acceleration feedforward constant
    units::Voltage max_voltage_;               ///< Maximum available voltage
    DifferentialDriveKinematics kinematics_;   ///< Differential drive kinematics model
};
