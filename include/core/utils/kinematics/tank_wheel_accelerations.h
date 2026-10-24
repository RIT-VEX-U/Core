#pragma once
#include "core/utils/units.h"

using namespace units::literals;

/**
 * @brief Represents the left and right accelerations of a tank drive.
 */
struct TankWheelAccelerations {
    /** @brief Left wheel acceleration. */
    units::Acceleration left = 0_inps2;
    /** @brief Right wheel acceleration. */
    units::Acceleration right = 0_inps2;

    TankWheelAccelerations() = default;

    TankWheelAccelerations(units::Acceleration left, units::Acceleration right)
        : left(left), right(right) {}

    bool operator==(const TankWheelAccelerations& other) const {
        return left == other.left && right == other.right;
    }

    bool operator!=(const TankWheelAccelerations& other) const {
        return !(*this == other);
    }
};
