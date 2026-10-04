#pragma once

#include <vector>

#include "cevalm.hpp"
#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/math/geometry/transform2d.h"
#include "core/utils/math/geometry/translation2d.h"
#include "core/utils/math/geometry/twist2d.h"
#include "core/utils/units.h"

/**
 * Class representing a pose in 2d space with x, y, and rotational components
 *
 * Assumes conventional cartesian coordinate system:
 * Looking down at the coordinate plane,
 * +X is right
 * +Y is up
 * +Theta is counterclockwise
 */
class Pose2d {
   private:
    Translation2d translation_;
    Rotation2d rotation_;

   public:
    /// Default Constructor for Pose2d
    constexpr Pose2d() : translation_{Translation2d()}, rotation_{Rotation2d()} {}

    /**
     * Constructs a pose with given translation and rotation components.
     *
     * @param translation translational component.
     * @param rotation rotational component.
     */
    constexpr Pose2d(Translation2d translation, Rotation2d rotation)
        : translation_{translation}, rotation_{rotation} {}

    /**
     * Constructs a pose with given translation and rotation components.
     *
     * @param x x component.
     * @param y y component.
     * @param rotation rotational component.
     */
    constexpr Pose2d(units::Length x, units::Length y, Rotation2d rotation)
        : translation_{x, y}, rotation_{rotation} {}

    /**
     * Constructs a pose with given translation and rotation components.
     *
     * @param x x component.
     * @param y y component.
     * @param radians rotational component in radians.
     */
    constexpr Pose2d(units::Length x, units::Length y, double radians)
        : translation_{x, y}, rotation_{radians} {}

    /**
     * Constructs a pose with given translation and rotation components in a vector,
     * using the supplied units.
     *
     * @param pose_vector vector of the form [x, y, theta].
     * @param length_unit unit of length to use.
     * @param angle_unit unit of angle to use, defaults to radians.
     */
    constexpr Pose2d(
            const Eigen::Vector3d& pose_vector, units::Length length_unit,
            units::Angle angle_unit = units::radians
    )
        : translation_{
                  units::Length(pose_vector[0], length_unit),
                  units::Length(pose_vector[1], length_unit)
          },
          rotation_{units::Angle(pose_vector[2], angle_unit)} {}

    /// @returns the x value of the translation.
    constexpr units::Length x() const { return translation_.x(); }

    /// @returns x in the supplied length unit.
    constexpr double x(units::Length unit) const { return translation_.x(unit); }

    /// Sets the x value of the translation.
    constexpr void set_x(units::Length val) { translation_.set_x(val); }

    /// @returns the y value of the translation.
    constexpr units::Length y() const { return translation_.y(); }

    /// @returns y in the supplied length unit.
    constexpr double y(units::Length unit) const { return translation_.y(unit); }

    /// Sets the y value of the translation.
    constexpr void set_y(units::Length val) { translation_.set_y(val); }

    /// @returns the translation.
    constexpr Translation2d translation() const { return translation_; }

    /// Sets the translation.
    constexpr void set_translation(Translation2d val) { translation_ = val; }

    /// @returns the rotation.
    constexpr Rotation2d rotation() const { return rotation_; }

    /// Sets the rotation.
    constexpr void set_rotation(Rotation2d val) { rotation_ = val; }

    /// @returns the heading as an angle.
    constexpr units::Angle angle() const { return rotation_.angle(); }

    /// @returns the angle in the supplied unit.
    constexpr double angle(units::Angle unit) const { return rotation_.angle(unit); }

    /// Sets the rotation as an angle.
    constexpr void set_angle(units::Angle val) { rotation_ = Rotation2d(val); }

    /// @returns [x, y, theta] in the supplied units. Angles default to radians.
    EVec<3> as_vector(units::Length length_unit, units::Angle angle_unit = units::radians) const {
        return {translation_.x().to(length_unit), translation_.y().to(length_unit),
                rotation_.angle().to(angle_unit)};
    }

    /// @returns the distance from the pose to another point.
    constexpr units::Length distance(Translation2d point) const {
        return translation_.distance(point);
    }

    /// @returns the distance from this to another pose, ignoring rotation.
    constexpr units::Length distance(const Pose2d& other) const {
        return distance(other.translation_);
    }

    /// @returns the bearing from this to another point in the world frame.
    constexpr Rotation2d bearing_to(Translation2d point) const {
        const auto delta = point - translation_;
        if (delta.x().internal() == 0 && delta.y().internal() == 0) {
            return rotation_;
        }
        return delta.theta();
    }

    /// @returns the smallest angle to another point in the local frame.
    constexpr units::Angle angle_to(Translation2d point) const {
        return (bearing_to(point) - rotation_).angle();
    }

    /// Converts a point from this pose's local frame to the world frame.
    constexpr Translation2d local_to_world(Translation2d point) const {
        return translation_ + point.rotate_by(rotation_);
    }

    /// Converts a point from the world frame to this pose's local frame.
    constexpr Translation2d world_to_local(Translation2d point) const {
        return (point - translation_).rotate_by(-rotation_);
    }

    /**
     * Finds this pose in the frame of another pose rather than the origin.
     *
     * @param other new origin.
     * @return this pose relative to other.
     */
    constexpr Pose2d relative_to(const Pose2d& other) const {
        Transform2d transform = *this - other;
        return Pose2d{transform.translation(), transform.rotation()};
    }

    /**
     * Adds a transform to this pose.
     * Rotates the transform's translation into the pose's frame,
     * then adds the translation and rotation.
     *
     * @param transform the change in pose.
     * @return the pose after being transformed.
     */
    constexpr Pose2d transform_by(const Transform2d& transform) const {
        return Pose2d{
                translation_ + (transform.translation().rotate_by(rotation_)),
                rotation_ + transform.rotation()
        };
    }

    /**
     * Interpolates position along a straight line and heading along the shortest turn.
     * Fractions outside [0, 1] return the nearest endpoint.
     *
     * Does NOT move along an arc like a Twist, it's a straight line.
     */
    constexpr Pose2d interpolate(const Pose2d& end, double fraction) const {
        if (fraction <= 0) {
            return *this;
        }
        if (fraction >= 1) {
            return end;
        }
        return {translation_ + (end.translation_ - translation_) * fraction,
                rotation_ + (end.rotation_ - rotation_) * fraction};
    }

    /**
     * Applies a twist (pose delta) to a pose by integrating constant velocity and angular velocity
     * in the robot's frame of motion.
     *
     * When applying a twist, imagine a constant angular velocity, the
     * translational components must be rotated into the global frame at every
     * point along the twist, simply adding the deltas does not do this, and using
     * euler integration results in some error. This is the analytic solution for integrating
     * along an arc.
     *
     * Can also be thought of more simply as following an arc rather than a straight line.
     *
     * See this document for more information on the pose exponential and its
     * derivation.
     * https://file.tavsys.net/control/controls-engineering-in-frc.pdf#section.10.2
     *
     * @param twist     The twist, represents a pose delta.
     * @return new pose that has been moved forward according to the twist.
     */
    constexpr Pose2d exp(const Twist2d& twist) const {
        const units::Length dx = twist.dx();
        const units::Length dy = twist.dy();
        const double dtheta = twist.dtheta().to(units::radians);

        const double sin_theta = cevalm::sin(dtheta);
        const double cos_theta = cevalm::cos(dtheta);

        double s, c;
        if (cevalm::abs(dtheta) < 1e-9) {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        } else {
            s = sin_theta / dtheta;
            c = (1 - cos_theta) / dtheta;
        }

        const Transform2d transform{
                Translation2d{dx * s - dy * c, dx * c + dy * s}, Rotation2d{cos_theta, sin_theta}
        };

        return *this + transform;
    }

    /**
     * The inverse of the pose exponential.
     *
     * Determines the twist required to go from this pose to the given end pose.
     *
     * @param end_pose the end pose to find the mapping to.
     * @return the twist required to go from this pose to the given end
     */
    constexpr Twist2d log(const Transform2d& transform) const {
        const double dtheta = transform.rotation().radians();
        const double half_dtheta = dtheta / 2.0;

        const double cos_minus_one = transform.rotation().f_cos() - 1;

        double half_theta_by_tan_of_half_dtheta;

        if (cevalm::abs(cos_minus_one) < 1e-9) {
            half_theta_by_tan_of_half_dtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            half_theta_by_tan_of_half_dtheta =
                    -(half_dtheta * transform.rotation().f_sin()) / cos_minus_one;
        }

        const Translation2d translation_part =
                transform.translation().rotate_by(
                        {half_theta_by_tan_of_half_dtheta, -half_dtheta}
                ) *
                cevalm::hypot(half_theta_by_tan_of_half_dtheta, half_dtheta);

        return Twist2d{
                translation_part.x(), translation_part.y(), units::Angle(dtheta, units::radians)
        };
    }

    /**
     * Calculates the mean of a list of poses.
     *
     * @param list std::vector containing a list of poses.
     * @return the single pose mean of the list of poses.
     */
    static Pose2d wrapped_mean(const std::vector<Pose2d>& list) {
        if (list.size() == 0) {
            return Pose2d{};
        }

        units::Length sumx;
        units::Length sumy;

        double sum_sin = 0;
        double sum_cos = 0;

        for (Pose2d pose : list) {
            sumx += pose.x();
            sumy += pose.y();

            sum_sin += pose.rotation_.f_sin();
            sum_cos += pose.rotation_.f_cos();
        }

        return Pose2d{
                Translation2d{sumx / list.size(), sumy / list.size()},
                Rotation2d{sum_cos / list.size(), sum_sin / list.size()}
        };
    }

    /// Adds a transform to this pose by rotating it into the pose frame then adding.
    constexpr Pose2d operator+(const Transform2d& transform) const {
        return Pose2d{
                translation_ + (transform.translation().rotate_by(rotation_)),
                transform.rotation() + rotation_
        };
    }

    /// Adds a transform to this pose by rotating it into the pose frame then adding.
    constexpr Pose2d& operator+=(const Transform2d& transform) { return *this = *this + transform; }

    /// Subtracts another pose from this to find the transform between them.
    constexpr Transform2d operator-(const Pose2d& other) const {
        return Transform2d{
                (translation_ - other.translation_).rotate_by(-other.rotation_),
                rotation_ - other.rotation_
        };
    }

    /// Checks exact equality between this and another pose.
    constexpr bool operator==(const Pose2d& other) const {
        return (translation_ == other.translation_) && (rotation_ == other.rotation_);
    }

    /**
     * Checks translation distance and the smallest angle against tolerances.
     * Defaults to 1um and 1e-6 radians.
     */
    constexpr bool is_near(
            const Pose2d& other, units::Length distance_tolerance = units::Length(1e-6),
            units::Angle angle_tolerance = units::Angle(1e-6)
    ) const {
        return translation_.is_near(other.translation_, distance_tolerance) &&
               rotation_.is_near(other.rotation_, angle_tolerance);
    }
};
