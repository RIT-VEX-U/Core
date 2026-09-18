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
            Voltage maxVoltage,
            units::Length trackWidth
    )
        : Kv_(Kv), Ka_(Ka), maxVoltage_(maxVoltage), trackWidth_(trackWidth) {}

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
        units::Velocity leftVelocity = (speed - (trackWidth_ / 2 * (speed * curvature / 1_rad)));
        units::Velocity rightVelocity = (speed + (trackWidth_ / 2 * (speed * curvature / 1_rad)));

        units::Velocity maxWheelSpeed = units::max(leftVelocity, rightVelocity);
        units::Velocity minWheelSpeed = units::min(leftVelocity, rightVelocity);

        units::Acceleration maxWheelAcceleration =
                (maxVoltage_ - (Kv_ * units::maxWheelSpeed)) / Ka_;
        units::Acceleration minWheelAcceleration =
                (-maxVoltage_ - Kv_ * units::minWheelSpeed) / Ka_;

        units::Acceleration maxChassisAcceleration;
        units::Acceleration minChassisAcceleration;

        const double speedVal = speed.inps();
        const double speedSgn = std::abs(speedVal) <= 1e-9 ? 1.0 : (speedVal < 0.0 ? -1.0 : 1.0);
        const double curvd = abs(curvature / 1_rad).internal();
        const double twd = trackWidth_.internal();
        const double maxDenom = 1.0 + (twd * curvd * speedSgn / 2.0);
        const double minDenom = 1.0 - (twd * curvd * speedSgn / 2.0);

        maxChassisAcceleration =
                maxWheelAcceleration / (std::abs(maxDenom) < 1e-6 ? 1e-6 : maxDenom);
        minChassisAcceleration =
                minWheelAcceleration / (std::abs(minDenom) < 1e-6 ? 1e-6 : minDenom);

        if (abs(curvature) > 1E-9_radpm && (trackWidth_ / 2.0) > 1_rad / abs(curvature)) {
            if (speed > 0_mps && minChassisAcceleration > 0_inps2) {
                minChassisAcceleration = -minChassisAcceleration;
            } else if (speed < 0_mps && maxChassisAcceleration < 0_inps2) {
                maxChassisAcceleration = -maxChassisAcceleration;
            }
        }

        return {minChassisAcceleration, maxChassisAcceleration};
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
    Voltage maxVoltage_;                ///< Maximum available voltage
    units::Length trackWidth_;          ///< Robot track width
};
