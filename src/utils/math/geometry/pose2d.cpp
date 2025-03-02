#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <vector>

#include "../core/include/utils/math/geometry/pose2d.h"
#include "../core/include/utils/math/geometry/rotation2d.h"
#include "../core/include/utils/math/geometry/transform2d.h"
#include "../core/include/utils/math/geometry/translation2d.h"
#include "../core/include/utils/math/geometry/twist2d.h"

/**
 * Constructs a pose with given translation and rotation components.
 *
 * @param translation translational component.
 * @param rotation rotational component.
 */
Pose2d::Pose2d(const Translation2d &translation, const Rotation2d &rotation)
    : m_translation{translation}, m_rotation{rotation} {}

/**
 * Constructs a pose with given translation and rotation components.
 *
 * @param x x component.
 * @param y y component.
 * @param rotation rotational component.
 */
Pose2d::Pose2d(const double &x, const double &y, const Rotation2d &rotation)
    : m_translation{x, y}, m_rotation{rotation} {}

/**
 * Constructs a pose with given translation and rotation components.
 *
 * @param x x component.
 * @param y y component.
 * @param radians rotational component in radians.
 */
Pose2d::Pose2d(const double &x, const double &y, const double &radians) : m_translation{x, y}, m_rotation{radians} {}

/**
 * Constructs a pose with given translation and rotation components.
 *
 * @param translation translational component.
 * @param radians rotational component in radians.
 */
Pose2d::Pose2d(const Translation2d &translation, const double &radians)
    : m_translation{translation}, m_rotation{radians} {}

/**
 * Constructs a pose with given translation and rotation components.
 *
 * @param pose_vector vector of the form [x, y, theta].
 */
Pose2d::Pose2d(const Eigen::Vector3d &pose_vector)
    : m_translation{pose_vector(0), pose_vector(1)}, m_rotation{pose_vector(2)} {}

/**
 * Returns the translational component.
 *
 * @return the translational component.
 */
Translation2d Pose2d::translation() const { return m_translation; }

/**
 * Returns the x value of the translational component.
 *
 * @return the x value of the translational component.
 */
double Pose2d::x() const { return m_translation.x(); }

/**
 * Returns the y value of the translational component.
 *
 * @return the y value of the translational component.
 */
double Pose2d::y() const { return m_translation.y(); }

/**
 * Returns the rotational component.
 *
 * @return the rotational component.
 */
Rotation2d Pose2d::rotation() const { return m_rotation; }

/**
 * Compares this to another pose.
 *
 * @param other the other pose to compare to.
 *
 * @return true if each of the components are within 1e-9 of each other.
 */
bool Pose2d::operator==(const Pose2d other) const {
  return (translation() == other.translation()) && (rotation() == other.rotation());
}

/**
 * Multiplies this pose by a scalar.
 * Simply multiplies each component.
 *
 * @param scalar the scalar value to multiply by.
 */
Pose2d Pose2d::operator*(const double &scalar) const { return Pose2d{m_translation * scalar, m_rotation * scalar}; }

/**
 * Divides this pose by a scalar.
 * Simply divides each component.
 *
 * @param scalar the scalar value to divide by.
 */
Pose2d Pose2d::operator/(const double &scalar) const { return *this * (1. / scalar); }

/**
 * Adds a transform to this pose.
 * Transforms the pose in the pose's frame.
 *
 * @param transform the change in pose.
 */
Pose2d Pose2d::operator+(const Transform2d &transform) const {
  return Pose2d{translation() + (transform.translation().rotate_by(rotation())), transform.rotation() + rotation()};
}

/**
 * Subtracts one pose from another to find the transform between them.
 *
 * @param other the pose to subtract.
 */
Transform2d Pose2d::operator-(const Pose2d &other) const {
  Pose2d pose_diff = relative_to(other);
  return Transform2d(pose_diff.translation(), pose_diff.rotation());
}

/**
 * Sends a pose to an output stream.
 * Ex.
 * std::cout << pose;
 *
 * prints "Pose2d[x: (value), y: (value), rad: (radians), deg: (degrees)]"
 */
std::ostream &operator<<(std::ostream &os, const Pose2d &pose) {
  os << "Pose2d[x: " << pose.x() << ", y: " << pose.y() << ", rad: " << pose.rotation().radians()
     << ", deg: " << pose.rotation().degrees() << "]";
  return os;
}

/**
 * Finds the pose equivalent to this pose relative to another arbitrary pose rather than the origin.
 *
 * @param other the pose representing the new origin.
 *
 * @return this pose relative to another pose.
 */
Pose2d Pose2d::relative_to(const Pose2d &other) const {
  Transform2d transform{other, *this};
  return Pose2d{transform.translation(), transform.rotation()};
}

/**
 * Adds a transform to this pose.
 * Simply adds each component.
 *
 * @param transform the change in pose.
 *
 * @return the pose after being transformed.
 */
Pose2d Pose2d::transform_by(const Transform2d &transform) const {
  return Pose2d{translation() + (transform.translation().rotate_by(rotation())), rotation() + transform.rotation()};
}

/**
 * Applies a twist (pose delta) to a pose by including first order dynamics of heading.
 *
 * When applying a twist, imagine a constant angular velocity, the translational components must
 * be rotated into the global frame at every point along the twist, simply adding the deltas does not do this,
 * and using euler integration results in some error. This is the analytic solution that that problem.
 *
 * Can also be thought of more simply as applying a twist as following an arc rather than a straight line.
 *
 * See this document for more information on the pose exponential and its derivation.
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf#section.10.2
 *
 * @param old_pose  The pose to which the twist will be applied.
 * @param twist     The twist, represents a pose delta.
 *
 * @return new pose that has been moved forward according to the twist.
 */
Pose2d Pose2d::exp(const Twist2d &twist) const {
  const double dx = twist.dx();
  const double dy = twist.dy();
  const double dtheta = twist.dtheta();

  const double sin_theta = std::sin(dtheta);
  const double cos_theta = std::cos(dtheta);

  double s, c;
  if (std::abs(dtheta) < 1e-9) {
    s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
    c = 0.5 * dtheta;
  } else {
    s = sin_theta / dtheta;
    c = (1 - cos_theta) / dtheta;
  }

  const Transform2d transform{Translation2d{dx * s - dy * c, dx * c + dy * s}, Rotation2d{cos_theta, sin_theta}};

  return *this + transform;
}

/**
 * The inverse of the pose exponential.
 *
 * Determines the twist required to go from this pose to the given end pose.
 * suppose you have Pose2d a, Twist2d twist
 * if a.exp(twist) = b then a.log(b) = twist
 *
 * @param end_pose the end pose to find the mapping to.
 *
 * @return the twist required to go from this pose to the given end
 */
Twist2d Pose2d::log(const Pose2d &end_pose) const {
  const Pose2d transform = end_pose.relative_to(*this);
  const double dtheta = transform.rotation().radians();
  const double halfDtheta = dtheta / 2.0;

  const double cosMinusOne = transform.rotation().f_cos() - 1;

  double halfThetaByTanOfHalfDtheta;

  if (std::abs(cosMinusOne) < 1e-9) {
    halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
  } else {
    halfThetaByTanOfHalfDtheta = -(halfDtheta * transform.rotation().f_sin()) / cosMinusOne;
  }

  const Translation2d translationPart = transform.translation().rotate_by({halfThetaByTanOfHalfDtheta, -halfDtheta}) *
                                        std::hypot(halfThetaByTanOfHalfDtheta, halfDtheta);

  return Twist2d{translationPart.x(), translationPart.y(), dtheta};
}

/**
 * Calculates the mean of a list of poses.
 *
 * @param list std::vector containing a list of poses.
 *
 * @return the single pose mean of the list of poses.
 */
Pose2d pose_mean(const std::vector<Pose2d> &list) {
  double sumx = 0;
  double sumy = 0;

  double sum_sin = 0;
  double sum_cos = 0;

  for (int i = 0; i < list.size(); i++) {
    sumx += list.at(i).x();
    sumy += list.at(i).y();

    sum_sin += list.at(i).rotation().f_sin();
    sum_cos += list.at(i).rotation().f_cos();
  }

  return Pose2d{
    Translation2d{sumx / list.size(), sumy / list.size()}, Rotation2d{sum_sin / list.size(), sum_cos / list.size()}
  };
}
