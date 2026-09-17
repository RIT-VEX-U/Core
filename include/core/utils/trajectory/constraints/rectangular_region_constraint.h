#pragma once

#include <memory>

#include "core/utils/units.h"
#include "core/utils/math/geometry/pose2d.h"
#include "core/utils/math/geometry/translation2d.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"

using namespace units::literals;

/**
 * @brief Enforces a TrajectoryConstraint only when the robot's translation
 * falls within a specified 2D rectangular bounding box.
 */
class RectangularRegionConstraint : public TrajectoryConstraint {
 public:
  /**
   * @brief Constructs a RectangularRegionConstraint.
   * @param bottom_left The bottom-left coordinate of the bounding box.
   * @param top_right The top-right coordinate of the bounding box.
   * @param constraint The child constraint to apply when inside the box.
   */
  RectangularRegionConstraint(const Translation2d& bottom_left,
                              const Translation2d& top_right,
                              const TrajectoryConstraint& constraint)
      : bottom_left_(bottom_left),
        top_right_(top_right),
        constraint_(constraint.clone()) {}

  units::Velocity max_velocity(const Pose2d& pose, units::Curvature curvature,
                        units::Velocity velocity) const override {
    if (is_in_region(pose.translation())) {
      return constraint_->max_velocity(pose, curvature, velocity);
    }
    return velocity;
  }

  TrajectoryConstraint::MinMax min_max_acceleration(const Pose2d& pose, units::Curvature curvature,
                                                    units::Velocity velocity) const override {
    if (is_in_region(pose.translation())) {
      return constraint_->min_max_acceleration(pose, curvature, velocity);
    }
    return {};
  }

  std::unique_ptr<TrajectoryConstraint> clone() const override {
    return std::make_unique<RectangularRegionConstraint>(bottom_left_, top_right_, *constraint_);
  }

 private:
  Translation2d bottom_left_;
  Translation2d top_right_;
  std::unique_ptr<TrajectoryConstraint> constraint_;

  bool is_in_region(const Translation2d& point) const {
    return point.x() >= bottom_left_.x() && point.x() <= top_right_.x() &&
           point.y() >= bottom_left_.y() && point.y() <= top_right_.y();
  }
};
