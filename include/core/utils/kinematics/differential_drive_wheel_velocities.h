#pragma once
#include "core/utils/units.h"

using namespace units::literals;

/**
 * @brief Represents the left and right velocities of a differential drive.
 */
struct DifferentialDriveWheelVelocities {
    /** @brief Left wheel velocity. */
    units::Velocity left = 0_inps;
    /** @brief Right wheel velocity. */
    units::Velocity right = 0_inps;

    DifferentialDriveWheelVelocities() = default;

    DifferentialDriveWheelVelocities(units::Velocity left, units::Velocity right)
        : left(left), right(right) {}

    bool operator==(const DifferentialDriveWheelVelocities& other) const {
        return left == other.left && right == other.right;
    }

    bool operator!=(const DifferentialDriveWheelVelocities& other) const {
        return !(*this == other);
    }
};
