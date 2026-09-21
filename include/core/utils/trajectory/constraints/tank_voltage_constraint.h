#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "core/utils/math_util.h"
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
     * @brief Constructs a TankVoltageConstraint.
     * @param Kv Linear velocity feedforward constant.
     * @param Ka Linear acceleration feedforward constant.
     * @param maxVoltage Maximum available voltage.
     * @param trackWidth Robot track width (distance between left and right wheels).
     */
    TankVoltageConstraint(
            LinearVelocityFeedforward Kv,
            LinearAccelerationFeedforward Ka,
            Voltage max_voltage,
            units::Length track_width
    )
        : Kv_(Kv), Ka_(Ka), max_voltage_(max_voltage), track_width_(track_width) {}

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
        units::Velocity left_velocity = (speed - (track_width_ / 2 * (speed * curvature / 1_rad)));
        units::Velocity right_velocity = (speed + (track_width_ / 2 * (speed * curvature / 1_rad)));

        units::Velocity max_wheel_speed = units::max(left_velocity, right_velocity);
        units::Velocity min_wheel_speed = units::min(left_velocity, right_velocity);

        units::Acceleration max_wheel_acceleration =
                (max_voltage_ - (Kv_ * units::max_wheel_speed)) / Ka_;
        units::Acceleration min_wheel_acceleration =
                (-max_voltage_ - Kv_ * units::min_wheel_speed) / Ka_;

        units::Acceleration max_chassis_acceleration;
        units::Acceleration min_chassis_acceleration;

        const double speed_val = speed.inps();
        const double speed_sgn = units::abs(speed_val) <= 1e-9 ? 1.0 : (speed_val < 0.0 ? -1.0 : 1.0);
        const double curvd = units::abs(curvature / 1_rad).internal();
        const double twd = track_width_.internal();
        const double max_denom = 1.0 + (twd * curvd * speed_sgn / 2.0);
        const double min_denom = 1.0 - (twd * curvd * speed_sgn / 2.0);

        max_chassis_acceleration =
                max_wheel_acceleration / (units::abs(max_denom) < 1e-6 ? 1e-6 : max_denom);
        min_chassis_acceleration =
                min_wheel_acceleration / (units::abs(min_denom) < 1e-6 ? 1e-6 : min_denom);

        if (units::abs(curvature) > 1E-9_radpm && (track_width_ / 2.0) > 1_rad / units::abs(curvature)) {
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
    LinearVelocityFeedforward Kv_;      ///< Linear velocity feedforward constant
    LinearAccelerationFeedforward Ka_;  ///< Linear acceleration feedforward constant
    Voltage max_voltage_;                ///< Maximum available voltage
    units::Length track_width_;          ///< Robot track width
};
