#pragma once

// These are required for Eigen to compile
// https://www.vexforum.com/t/eigen-integration-issue/61474/5
#undef __ARM_NEON__
#undef __ARM_NEON
#include <Eigen/Dense>

#include "../core/include/utils/math/geometry/translation2d.h"
#include "../core/include/utils/math/geometry/rotation2d.h"

/**
 * Class representing 
 */
class Transform2d {
    public:
    Transform2d(const Translation2d& translation, const Rotation2d& rotation);

    Transform2d(const double& x, const double& y, const Rotation2d& rotation);

    Transform2d(const double& x, const double& y, const double& radians);

    Transform2d(const Translation2d& translation, const double& radians);

    Transform2d(const Eigen::Vector3d& transform_vector);

    Translation2d translation();

    double x();

    double y();

    Rotation2d rotation();

    Transform2d operator+(const Transform2d& other);

    Transform2d operator-(const Transform2d& other);

    Transform2d operator*(const double& scalar);

    Transform2d operator/(const double& scalar);

    bool operator==(const Transform2d& other);

    private:
    Translation2d translation;
    Rotation2d rotation;


};
