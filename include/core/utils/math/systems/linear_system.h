#pragma once

#include "core/utils/math/eigen_interface.h"
#include "core/utils/math/systems/discretization.h"

/**
 * This class represents a state-space model of a linear system.
 *
 * It contains the following continuous matrices:
 * A, System matrix
 * B, Input matrix
 * C, Output matrix
 * D, Feedthrough matrix
 */
template <int STATES, int INPUTS, int OUTPUTS>
class LinearSystem {
 public:
  using MatrixA = EMat<STATES, STATES>;
  using MatrixB = EMat<STATES, INPUTS>;
  using MatrixC = EMat<OUTPUTS, STATES>;
  using MatrixD = EMat<OUTPUTS, INPUTS>;

  using VectorX = EVec<STATES>;
  using VectorU = EVec<INPUTS>;
  using VectorY = EVec<OUTPUTS>;

  /**
   * Constructs a discrete linear system with the given continuous matrices.
   *
   * @param A The continuous system matrix
   * @param B The continuous input matrix
   * @param C The output matrix
   * @param D The feedthrough matrix
   */
  LinearSystem(const MatrixA& A, const MatrixB& B, const MatrixC& C, const MatrixD& D)
      : m_Ac(A), m_Bc(B), m_C(C), m_D(D) {}

  /**
   * Returns the continuous system matrix A.
   */
  MatrixA A() { return m_Ac; }

  /**
   * Returns the continuous input matrix B.
   */
  MatrixB B() { return m_Bc; }

  /**
   * Returns a tuple of A and B after being discretized.
   */
  const std::tuple<std::tuple<MatrixA, MatrixB>>& discAB(const double& dt) { return discretize_AB(m_Ac, m_Bc, dt); }

  /**
   * Returns the output matrix C.
   */
  MatrixC C() { return m_C; }

  /**
   * Returns the feedthrough matrix D.
   */
  MatrixD D() { return m_D; }

  /**
   * Computes the new state vector given the previous state vector, an input
   * vector, and the timestep in seconds.
   *
   * @param x The current state vector.
   * @param u The input vector.
   * @param dt The timestep in seconds.
   *
   * @return The new state vector.
   */
  VectorX compute_X(const VectorX& x, const VectorU& u, double dt) {
    // Discretize A and B
    auto [Ad, Bd] = discretize_AB(m_Ac, m_Bc, dt);

    return Ad * x + Bd * u;
  }

  /**
   * Computes the output vector given a state and an input.
   *
   * @param x The state vector.
   * @param u The input vector.
   *
   * @return The output vector.
   */
  VectorY compute_Y(const VectorX& x, const VectorU& u) { return m_C * x + m_D * u; }

 private:
  // Continuous system matrix
  MatrixA m_Ac;
  // Continuous input matrix
  MatrixB m_Bc;
  // Output matrix
  MatrixC m_C;
  // Feedthrough matrix
  MatrixD m_D;
};
