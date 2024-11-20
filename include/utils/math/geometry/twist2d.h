#pragma once

// These are required for Eigen to compile
// https://www.vexforum.com/t/eigen-integration-issue/61474/5
#undef __ARM_NEON__
#undef __ARM_NEON
#include <Eigen/Dense>


class Twist2d {
    public:

    Twist2d(const double& dx, const double& dy, const double& dtheta);

    Twist2d(const Eigen::Vector3d& twist_vector);

    double dx();

    double dy();

    double dtheta();

    bool operator==(const Twist2d& other);

    Twist2d operator*(const double& scalar);

    private:
    double dx;
    double dy;
    double dtheta;
};
