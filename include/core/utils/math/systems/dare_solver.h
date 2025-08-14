#pragma once

#include "core/utils/math/eigen_interface.h"

/**
 * Copyright (c) FIRST and other WPILib contributors.
 * 
 * Computes the unique stabilizing solution X to the discrete-time algebraic
 * Riccati equation:
 *
 *   AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0
 *
 * This internal function skips expensive precondition checks for increased
 * performance. The solver may hang if any of the following occur:
 * <ul>
 *   <li>Q isn't symmetric positive semidefinite</li>
 *   <li>R isn't symmetric positive definite</li>
 *   <li>The (A, B) pair isn't stabilizable</li>
 *   <li>The (A, C) pair where Q = CᵀC isn't detectable</li>
 * </ul>
 * Only use this function if you're sure the preconditions are met.
 *
 * This algorithm is taken directly from WPILib. The version that I wrote was
 * slower, unsurprisingly, so we will use theirs instead.
 * 
 * This is also the algorithm used by Drake, an advanced robotics library.
 *
 * @tparam STATES Number of STATES.
 * @tparam INPUTS Number of INPUTS.
 * @param A The system matrix.
 * @param B The input matrix.
 * @param Q The state cost matrix.
 * @param R_llt The LLT decomposition of the input cost matrix.
 * @return Solution to the DARE.
 */
template <int STATES, int INPUTS>
EMat<STATES, STATES> DARE(
  const EMat<STATES, STATES> &A, const EMat<STATES, INPUTS> &B, const EMat<STATES, STATES> &Q,
  const EMat<INPUTS, INPUTS> &R
) {
    const Eigen::LLT<EMat<INPUTS, INPUTS>> R_llt = R.llt();
    using StateMatrix = EMat<STATES, STATES>;

    // Implements SDA algorithm on p. 5 of [1] (initial A, G, H are from (4)).
    //
    // [1] E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang "Structure-Preserving
    //     Algorithms for Periodic Discrete-Time Algebraic Riccati Equations",
    //     International Journal of Control, 77:8, 767-788, 2004.
    //     DOI: 10.1080/00207170410001714988

    // A₀ = A
    // G₀ = BR⁻¹Bᵀ
    // H₀ = Q

    StateMatrix A_k = A;
    StateMatrix G_k = B * R_llt.solve(B.transpose());
    StateMatrix H_k;
    StateMatrix H_k1 = Q;



    do {
        H_k = H_k1;

        // W = I + GₖHₖ
        StateMatrix W = StateMatrix::Identity(H_k.rows(), H_k.cols()) + G_k * H_k;

        auto W_solver = W.lu();

        // Solve WV₁ = Aₖ for V₁
        StateMatrix V_1 = W_solver.solve(A_k);

        // Solve V₂Wᵀ = Gₖ for V₂
        //
        // We want to put V₂Wᵀ = Gₖ into Ax = b form so we can solve it more
        // efficiently.
        //
        // V₂Wᵀ = Gₖ
        // (V₂Wᵀ)ᵀ = Gₖᵀ
        // WV₂ᵀ = Gₖᵀ
        //
        // The solution of Ax = b can be found via x = A.solve(b).
        //
        // V₂ᵀ = W.solve(Gₖᵀ)
        // V₂ = W.solve(Gₖᵀ)ᵀ
        StateMatrix V_2 = W_solver.solve(G_k.transpose()).transpose();

        // Gₖ₊₁ = Gₖ + AₖV₂Aₖᵀ
        // Hₖ₊₁ = Hₖ + V₁ᵀHₖAₖ
        // Aₖ₊₁ = AₖV₁
        G_k += A_k * V_2 * A_k.transpose();
        H_k1 = H_k + V_1.transpose() * H_k * A_k;
        A_k *= V_1;
        

        // while |Hₖ₊₁ − Hₖ| > ε |Hₖ₊₁|
    } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm());

    return H_k1;
}
