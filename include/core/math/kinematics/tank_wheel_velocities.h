#pragma once
#include "core/math/units.h"

using namespace units::literals;

/**
 * @brief Represents the left and right velocities of a tank drive.
 */
struct TankWheelVelocities {
  /** @brief Left wheel velocity. */
  units::Velocity left = 0_inps;
  /** @brief Right wheel velocity. */
  units::Velocity right = 0_inps;

  TankWheelVelocities() = default;

  TankWheelVelocities(units::Velocity left, units::Velocity right) : left(left), right(right) {}

  bool operator==(const TankWheelVelocities& other) const {
    return left == other.left && right == other.right;
  }

  bool operator!=(const TankWheelVelocities& other) const { return !(*this == other); }
};
