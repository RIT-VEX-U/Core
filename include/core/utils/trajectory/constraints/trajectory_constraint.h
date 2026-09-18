#pragma once

#include <limits>
#include <memory>

#include "core/utils/math/geometry/pose2d.h"
#include "core/utils/units.h"

using namespace units::literals;

/**
 * @brief Interface for defining physical trajectory velocity and acceleration constraints.
 */
class TrajectoryConstraint {
   public:
    TrajectoryConstraint() = default;

    TrajectoryConstraint(const TrajectoryConstraint&) = default;
    TrajectoryConstraint& operator=(const TrajectoryConstraint&) = default;

    TrajectoryConstraint(TrajectoryConstraint&&) = default;
    TrajectoryConstraint& operator=(TrajectoryConstraint&&) = default;

    virtual ~TrajectoryConstraint() = default;

    /**
     * @brief Struct representing minimum and maximum allowed linear accelerations.
     */
    struct MinMax {
        /**
         * @brief Constructs MinMax with explicit minimum and maximum acceleration bounds.
         * @param minAcceleration Lower acceleration limit.
         * @param maxAcceleration Upper acceleration limit.
         */
        MinMax(units::Acceleration minAcceleration, units::Acceleration maxAcceleration)
            : minAcceleration(minAcceleration), maxAcceleration(maxAcceleration) {}

        /** @brief Default constructor setting unbounded limits [-Inf, +Inf]. */
        MinMax()
            : minAcceleration(-std::numeric_limits<double>::max()),
              maxAcceleration(std::numeric_limits<double>::max()) {}

        units::Acceleration minAcceleration{-std::numeric_limits<double>::max()
        };  ///< Lower acceleration limit
        units::Acceleration maxAcceleration{std::numeric_limits<double>::max()
        };  ///< Upper acceleration limit
    };

    /**
     * @brief Computes maximum allowed velocity at a given pose and path curvature.
     * @param pose 2D position and orientation.
     * @param curvature Path curvature in rad/meter.
     * @param velocity Candidate linear velocity.
     * @return Constrained units::Velocity limit.
     */
    virtual units::Velocity max_velocity(
            const Pose2d& pose, units::Curvature curvature, units::Velocity velocity
    ) const = 0;

    /**
     * @brief Computes minimum and maximum allowed linear accelerations at a given pose, curvature,
     * and speed.
     * @param pose 2D position and orientation.
     * @param curvature Path curvature in rad/meter.
     * @param speed Current scalar speed.
     * @return MinMax acceleration bounds struct.
     */
    virtual MinMax min_max_acceleration(
            const Pose2d& pose, units::Curvature curvature, units::Velocity speed
    ) const = 0;

    /**
     * @brief Polymorphic deep-copy factory method.
     * @return Unique pointer to cloned TrajectoryConstraint instance.
     */
    virtual std::unique_ptr<TrajectoryConstraint> clone() const = 0;
};
