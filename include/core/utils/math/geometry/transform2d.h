#pragma once

#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/math/geometry/translation2d.h"
#include "core/utils/units.h"

/**
 * Class representing a transformation of a pose2d, by rotating a translation into the frame
 * of the pose2d, then adding the translation, then rotating by theta.
 *
 * Assumes conventional cartesian coordinate system:
 * Looking down at the coordinate plane,
 * +X is right
 * +Y is up
 * +Theta is counterclockwise
 */
class Transform2d {
   private:
    Translation2d translation_;
    Rotation2d rotation_;

   public:
    /// Default Constructor for Transform2d
    constexpr Transform2d() = default;

    /**
     * Constructs a transform given translation and rotation components.
     *
     * @param translation the translational component of the transform.
     * @param rotation the rotational component of the transform.
     */
    constexpr Transform2d(Translation2d translation, Rotation2d rotation)
        : translation_(translation), rotation_(rotation) {}

    /**
     * Constructs a transform given translation and rotation components.
     *
     * @param x the x component of the transform.
     * @param y the y component of the transform.
     * @param rotation the rotational component of the transform.
     */
    constexpr Transform2d(units::Length x, units::Length y, Rotation2d rotation)
        : translation_(x, y), rotation_(rotation) {}

    /**
     * Constructs a transform given translation and rotation components.
     *
     * @param x the x component of the transform.
     * @param y the y component of the transform.
     * @param angle the rotational component of the transform.
     */
    constexpr Transform2d(units::Length x, units::Length y, units::Angle angle)
        : translation_(x, y), rotation_(angle) {}

    /**
     * Constructs a transform given translation and rotation components.
     *
     * @param translation the translational component of the transform.
     * @param angle the rotational component of the transform.
     */
    constexpr Transform2d(Translation2d translation, units::Angle angle)
        : translation_(translation), rotation_(angle) {}

    /**
     * Constructs a transform given translation and rotation components given as a
     * vector using the supplied units.
     *
     * @param transform_vector vector of the form [x, y, theta]
     * @param unit The length unit to use when assigning the translation.
     * @param angle_unit The angle unit, defaults to radians.
     */
    constexpr Transform2d(
            const Eigen::Vector3d& transform_vector, units::Length unit,
            units::Angle angle_unit = units::radians
    )
        : translation_({transform_vector(0), transform_vector(1)}, unit),
          rotation_(units::Angle(transform_vector(2), angle_unit)) {}

    /// Gets the x component
    constexpr units::Length x() const { return translation_.x(); }

    /// Gets x in the supplied length unit.
    constexpr double x(units::Length unit) const { return translation_.x(unit); }

    /// Sets the x component.
    constexpr void set_x(units::Length val) { translation_.set_x(val); }

    /// Gets the y component
    constexpr units::Length y() const { return translation_.y(); }

    /// Gets y in the supplied length unit.
    constexpr double y(units::Length unit) const { return translation_.y(unit); }

    /// Sets the y component.
    constexpr void set_y(units::Length val) { translation_.set_y(val); }

    /// Gets the translation.
    constexpr Translation2d translation() const { return translation_; }

    /// Sets the translation.
    constexpr void set_translation(Translation2d val) { translation_ = val; }

    /// Gets the rotation.
    constexpr Rotation2d rotation() const { return rotation_; }

    /// Sets the rotation.
    constexpr void set_rotation(Rotation2d val) { rotation_ = val; }

    /// Gets the rotation as an Angle
    constexpr units::Angle angle() const { return rotation_.angle(); }

    /// Gets the angle in the supplied unit.
    constexpr double angle(units::Angle unit) const { return rotation_.angle(unit); }

    /// Sets the rotation as an angle.
    constexpr void set_angle(units::Angle val) { rotation_ = Rotation2d(val); }

    /**
     * Returns [x, y, theta] in the supplied units. Angles default to radians.
     *
     * @param length_unit the unit of length to get the values as
     * @param angle_unit the unit of angle to get the rotation as, default radians
     * @return EVec<3> containing the values.
     */
    EVec<3> as_vector(units::Length length_unit, units::Angle angle_unit = units::radians) const {
        return {translation_.x().to(length_unit), translation_.y().to(length_unit),
                rotation_.angle().to(angle_unit)};
    }

    /// Inverts this transform
    constexpr Transform2d inverse() const {
        return Transform2d(-translation_.rotate_by(-rotation_), -rotation_);
    }

    /// Composes this transform and another transform.
    constexpr Transform2d operator+(const Transform2d& other) const {
        return {translation_ + other.translation_.rotate_by(rotation_),
                rotation_ + other.rotation_};
    }

    /// Composes this transform and another transform.
    constexpr Transform2d& operator+=(const Transform2d& other) { return *this = *this + other; }

    /// Inverts this transform.
    constexpr Transform2d operator-() const { return inverse(); }

    /// Multiplies this transform by a scalar.
    constexpr Transform2d operator*(double scalar) const {
        return Transform2d(translation_ * scalar, rotation_ * scalar);
    }

    /// Multiplies this transform by a scalar reverse order.
    friend constexpr Transform2d operator*(double scalar, const Transform2d& transform) {
        return transform * scalar;
    }

    /// Multiplies this transform by a scalar.
    constexpr Transform2d& operator*=(double scalar) { return *this = *this * scalar; }

    /// Divides this transform by a scalar.
    constexpr Transform2d operator/(double scalar) const {
        return Transform2d(translation_ / scalar, rotation_ / scalar);
    }

    /// Divides this transform by a scalar.
    constexpr Transform2d& operator/=(double scalar) { return *this = *this / scalar; }

    /// Checks exact equality between this and another transform.
    constexpr bool operator==(const Transform2d& other) const {
        return (translation_ == other.translation_) && (rotation_ == other.rotation_);
    }

    /**
     * Checks translation distance and the smallest angle against tolerances.
     * Defaults to 1um and 1e-6 radians.
     */
    constexpr bool is_near(
            const Transform2d& other, units::Length distance_tolerance = units::Length(1e-6),
            units::Angle angle_tolerance = units::Angle(1e-6)
    ) const {
        return translation_.is_near(other.translation_, distance_tolerance) &&
               rotation_.is_near(other.rotation_, angle_tolerance);
    }
};
