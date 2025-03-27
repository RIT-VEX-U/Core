// #pragma once

// #include "../core/include/utils/math/eigen_interface.h"
// #include "../core/include/utils/math/systems/linear_system.h"
// #include "../core/include/utils/controls/state_space/linear_quadratic_regulator.h"
// #include "../core/include/utils/math/estimator/kalman_filter.h"


// class CascadedUnicycleDriveController {
//     public:
//     CascadedUnicycleDriveController(double kVl, double kAl, double kVr, double kAr, const double &max_v, const double &max_omega) : max_v_(max_v), max_omega_(max_omega) {
//         EMat<1, 1> drive_Al{-kVl/kAl};
//         EMat<1, 1> drive_Bl{1/kAl};
//         EMat<1, 1> drive_C{1};
//         EMat<1, 1> drive_D{0};

//         EMat<1, 1> drive_Ar{-kVr/kAr};
//         EMat<1, 1> drive_Br{1/kAr};


//         drive_plant_l_ = LinearSystem<1, 1, 1>(drive_Al, drive_Bl, drive_C, drive_D);
//         drive_plant_r_ = LinearSystem<1, 1, 1>(drive_Ar, drive_Br, drive_C, drive_D);

//         EVec<1> R{0.023}; // noise for velocity
//         EVec<1> Q{0.01};

//         kf_l_ = KalmanFilter<1, 1, 1>(drive_plant_l_, Q, R);
//         kf_r_ = KalmanFilter<1, 1, 1>(drive_plant_r_, Q, R);
//     }

//     EMat<3, 3> jac_A(const double &v, const double &theta) {
//         EMat<3, 3> A;
//         A << 0, 0, -v*std::sin(theta), 
//              0, 0,  v*std::cos(theta),
//              0, 0,  0;

//         return A;
//     }

//     EMat<3, 2> jac_B(const double &theta) {
//         EMat<3, 2> B;
//         B << std::cos(theta), 0,
//              std::sin(theta), 0,
//                            0, 1;

//         return B;
//     }

//     EMat<2, 3> K(const double &v, const double &theta, const double &max_v, const double &max_omega) {
//         auto [Ad, Bd] = discretize_AB(jac_A(v, theta), jac_B(theta), 0.01);

//         EVec<3> Q = cost_matrix(EVec<3>{1, 1, 1});
//         EVec<2> R = cost_matrix(EVec<2>{max_v, max_omega});

//         EMat<3, 3> S = DARE<3, 3>(Ad, Bd, Q, R);

//         // (BᵀSD + R) \ (BᵀSA)
//         return (Bd.transpose() * S * Bd + R).llt().solve(Bd.transpose() * S * Ad);
//     }

//     EVec<2> chassis_speeds_(EVec<3> x, EVec<3> r) {

//     }




//     private:

//     double max_v_;
//     double max_omega_;


//     LinearSystem<1, 1, 1> drive_plant_l_;
//     LinearSystem<1, 1, 1> drive_plant_r_;
//     KalmanFilter<1, 1, 1> kf_l_;
//     KalmanFilter<1, 1, 1> kf_r_;

// };