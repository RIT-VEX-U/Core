#pragma once

#include <memory>

#include "core/units/units.h"
#include "core/utils/math/geometry/pose2d.h"
#include "core/utils/math/geometry/translation2d.h"
#include "core/utils/trajectory/constraints/trajectory_constraint.h"

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
      : m_bottom_left(bottom_left),
        m_top_right(top_right),
        m_constraint(constraint.clone()) {}

  Velocity max_velocity(const Pose2d& pose, Curvature curvature,
                        Velocity velocity) const override {
    if (is_in_region(pose.translation())) {
      return m_constraint->max_velocity(pose, curvature, velocity);
    }
    return velocity;
  }

  TrajectoryConstraint::MinMax min_max_acceleration(const Pose2d& pose, Curvature curvature,
                                                    Velocity velocity) const override {
    if (is_in_region(pose.translation())) {
      return m_constraint->min_max_acceleration(pose, curvature, velocity);
    }
    return {};
  }

  std::unique_ptr<TrajectoryConstraint> clone() const override {
    return std::make_unique<RectangularRegionConstraint>(m_bottom_left, m_top_right, *m_constraint);
  }

 private:
  Translation2d m_bottom_left;
  Translation2d m_top_right;
  std::unique_ptr<TrajectoryConstraint> m_constraint;

  bool is_in_region(const Translation2d& point) const {
    return point.x() >= m_bottom_left.x() && point.x() <= m_top_right.x() &&
           point.y() >= m_bottom_left.y() && point.y() <= m_top_right.y();
  }
};
