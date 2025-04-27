#pragma once

#include "core/utils/math/eigen_interface.h"
#include "../vendor/eigen/unsupported/Eigen/MatrixFunctions"

#include <cmath>
#include <tuple>

/**
 * Discretizes the continuous system and input matrices (A and B) over the
 * timestep dt in seconds.
 * 
 * @tparam STATES Dimension of the state matrix.
 * @tparam INPUTS Dimension of the input matrix.
 * 
 * @param Ac The continuous state matrix A.
 * @param Bc The continuous input matrix B.
 * @param dt The timestep in seconds.
 */
template <int STATES, int INPUTS>
std::tuple<EMat<STATES, STATES>, EMat<STATES, INPUTS>> discretize_AB(EMat<STATES, STATES> Ac, EMat<STATES, INPUTS> Bc, const double &dt) {
    // Form the intermediate matrix M
    //
    //       [A B]
    //   M = [0 0]
    //
    EMat<STATES + INPUTS, STATES + INPUTS> M;
    M.template block<STATES, STATES>(0, 0) = Ac;
    M.template block<STATES, INPUTS>(0, STATES) = Bc;
    M.template block<INPUTS, STATES + INPUTS>(STATES, 0).setZero();

    //
    //  M * T   [Ad Bd]
    // e      = [0   I]
    //
    EMat<STATES + INPUTS, STATES + INPUTS> phi = (M * dt).exp();

    // Extract Ad and Bd from phi and put them in a tuple
    return std::make_tuple(phi.template block<STATES, STATES>(0, 0), phi.template block<STATES, INPUTS>(0, STATES));
}
