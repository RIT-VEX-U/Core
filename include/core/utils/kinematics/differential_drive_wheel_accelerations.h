#pragma once
#include "core/utils/units.h"

using namespace units::literals;

/**
 * @brief Represents the left and right accelerations of a differential drive.
 */
struct DifferentialDriveWheelAccelerations {
    /** @brief Left wheel acceleration. */
    units::Acceleration left = 0_inps2;
    /** @brief Right wheel acceleration. */
    units::Acceleration right = 0_inps2;

    DifferentialDriveWheelAccelerations() = default;

    DifferentialDriveWheelAccelerations(units::Acceleration left, units::Acceleration right)
        : left(left), right(right) {}

    bool operator==(const DifferentialDriveWheelAccelerations& other) const {
        return left == other.left && right == other.right;
    }

    bool operator!=(const DifferentialDriveWheelAccelerations& other) const {
        return !(*this == other);
    }
};
