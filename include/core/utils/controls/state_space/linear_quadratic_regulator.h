#pragma once

#include "../vendor/eigen/unsupported/Eigen/MatrixFunctions"
#include "core/utils/math/eigen_interface.h"
#include "core/utils/math/systems/dare_solver.h"
#include "core/utils/math/systems/discretization.h"
#include "core/utils/math/systems/linear_system.h"

/**
 * Forms a cost matrix from a set of tolerances for each variable using Bryson's
 * Rule.
 *
 *   Q = 1 / tol²
 *
 * @tparam DIM The dimensions of the cost matrix.
 * @param tolerances Vector containing the tolerances for each variable.
 *
 * @return The cost matrix.
 */
template <int DIM>
EMat<DIM, DIM> cost_matrix(const EVec<DIM>& tolerances) {
  EMat<DIM, DIM> Q = EMat<DIM, DIM>::Zero();
  for (int i = 0; i < DIM; i++) {
    Q(i, i) = 1.0 / (tolerances(i) * tolerances(i));
  }
  return Q;
}

/**
 * Class implements an LQR controller. This finds the optimal gain matrix K
 * where:
 *
 *   u = K(r - x)
 *
 * K is optimized to minimize a cost function:
 *
 *       ∞
 *   J = ∑ xₖᵀQxₖ + uₖᵀRuₖ
 *      k=0
 *
 * Where Q and R are the state and control cost matrices.
 *
 * @tparam STATES The number of states in the system.
 * @tparam INPUTS The number of inputs to the system.
 */
template <int STATES, int INPUTS>
class LinearQuadraticRegulator {
 public:
  // Definitions to shorten some lines.
  using MatrixA = EMat<STATES, STATES>;
  using MatrixB = EMat<STATES, INPUTS>;
  using VectorX = EVec<STATES>;
  using VectorU = EVec<INPUTS>;

  /**
   * Constructs an LQR given a plant, a vector of tolerances for the states and inputs, and the timestep in seconds.
   *
   * @tparam OUTPUTS The number of outputs of the plant.
   * @param plant The linear system to control.
   * @param Qtolerances A vector of tolerances for each state.
   * @param Rtolerances A vector of tolerances for each input.
   */
  template <int OUTPUTS>
  LinearQuadraticRegulator(LinearSystem<STATES, INPUTS, OUTPUTS>& plant, const VectorX& Qtolerances,
                           const VectorU& Rtolerances, const double& dt)
      : LinearQuadraticRegulator(plant.A(), plant.B(), Qtolerances, Rtolerances, dt) {}

  /**
   * Constructs an LQR given state and input matrices, a vector of tolerances for the states and inputs, and the
   * timestep in seconds.
   *
   * @param A The state matrix of the linear system.
   * @param B The input matrix of the linear system.
   * @param Qtolerances A vector of tolerances for each state.
   * @param Rtolerances A vector of tolerances for each input.
   */
  LinearQuadraticRegulator(const MatrixA& A, const MatrixB& B, const VectorX& Qtolerances, const VectorU& Rtolerances,
                           const double& dt)
      : LinearQuadraticRegulator(A, B, cost_matrix(Qtolerances), cost_matrix(Rtolerances), dt) {}

  /**
   * Constructs an LQR given state and input matrices, the cost matrices of states and inputs, and the timestep in
   * seconds.
   *
   * @param A The state matrix of the linear system.
   * @param B The input matrix of the linear system.
   * @param Q The cost matrix of the states.
   * @param R The cost matrix of the inputs.
   */
  LinearQuadraticRegulator(const MatrixA& A, const MatrixB& B, const EMat<STATES, STATES>& Q,
                           const EMat<INPUTS, INPUTS>& R, const double& dt) {
    auto [Ad, Bd] = discretize_AB(A, B, dt);

    MatrixA S = DARE<STATES, INPUTS>(Ad, Bd, Q, R);

    // (BᵀSD + R) \ (BᵀSA)
    K_ = (Bd.transpose() * S * Bd + R).llt().solve(Bd.transpose() * S * Ad);
  }

  /**
   * Computes the control input u as:
   *
   *   u = K(r - x)
   *
   * @param x The current state.
   * @param r The reference state.
   */
  VectorU calculate(const VectorX& x, const VectorX& r) { return K_ * (r - x); }

  /**
   * Recomputes K to work for a time delayed state.
   *
   *   Kdelay = K(A - BK)^(delay / dt)
   *
   * @tparam OUTPUTS The number of outputs of the plant
   * @param plant The linear system.
   * @param dt The timestep in seconds.
   * @param input_delay The time delay of the system.
   */
  template <int OUTPUTS>
  void latency_compensate(LinearSystem<STATES, INPUTS, OUTPUTS>& plant, const double& dt, const double& input_delay) {
    auto [Ad, Bd] = discretize_AB(plant.A(), plant.B(), dt);

    // Kdelay = K(A - BK)^(delay / dt)
    K_ = K_ * (Ad - Bd * K_).pow(input_delay / dt);
  }

 private:
  EMat<INPUTS, STATES> K_;
};
