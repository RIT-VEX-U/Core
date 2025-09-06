#pragma once

#include <iostream>

#include "core/utils/math/eigen_interface.h"
#include "core/utils/math/systems/discretization.h"
#include "core/utils/math/systems/linear_system.h"

/**
 * This class computes a feedforward control input by inverting the discrete
 * plant dynamics. A continuous linear system is provided, it is then
 * discretized on some timestep, then the feedforward control input
 * is computed to satisfy:
 *
 *   B_d * u_ff = next_state - A_d * current_state
 */
template <int STATES, int INPUTS>
class LinearPlantInversionFeedforward {
 public:
  /**
   * Constructs a feedforward given a plant and the nominal timestep.
   *
   * @tparam OUTPUTS The number of outputs of the plant.
   * @param plant The linear system.
   * @param dt The nominal timestep in seconds.
   */
  template <int OUTPUTS>
  LinearPlantInversionFeedforward(LinearSystem<STATES, INPUTS, OUTPUTS>& plant, const double& dt)
      : LinearPlantInversionFeedforward(plant.A(), plant.B(), dt) {}

  /**
   * Construts a feedforward given the state and input matrices from a plant.
   *
   * @param A The state matrix of the linear system.
   * @param B The input matrix of the linear system.
   * @param dt The nominal timestep in seconds.
   */
  LinearPlantInversionFeedforward(const EMat<STATES, STATES>& A, const EMat<STATES, INPUTS>& B, const double& dt)
      : A_(A), B_(B), m_dt(dt) {
    auto [Ad, Bd] = discretize_AB(A, B, dt);
    Ad_ = Ad;
    Bd_ = Bd;
  }

  /**
   * Computes the feedforward control input given the current reference state
   * and the next reference state. This also sets the current reference state
   * to the next reference state for you.
   *
   * @param r The current reference state.
   * @param next_r The next reference state.
   */
  EVec<INPUTS> calculate(const EVec<STATES>& r, const EVec<STATES>& next_r) {
    // ẋ = Ax + Bu
    // Bu = ẋ - Ax
    // u = B \ (ẋ - Ax)
    // u = B \ (next_r - Br)
    uff_ = Bd_.householderQr().solve(next_r - (Ad_ * r));
    r_ = next_r;

    return uff_;
  }

  /**
   * Computes the feedforward control input given only the next reference state.
   * This assumes that the previous reference is already set.
   *
   * @param next_r The next reference state.
   */
  EVec<INPUTS> calculate(const EVec<STATES>& next_r) { return calculate(r_, next_r); }

  /**
   * Computes the feedforward control input given the current reference state
   * and the next reference state. This also sets the current reference state
   * to the next reference state for you. Use this function if your timestep
   * is not the same between each run.
   *
   * This is slower because it discretizes A and B on each run, requiring
   * computing a matrix exponential. Don't use this unless you have to.
   *
   * @param r The current reference state.
   * @param next_r The next reference state.
   * @param dt The timestep for this run.
   */
  EVec<INPUTS> calculate(const EVec<STATES>& r, const EVec<STATES>& next_r, const double& dt) {
    auto [Ad, Bd] = discretize_AB(A_, B_, dt);

    // ẋ = Ax + Bu
    // Bu = ẋ - Ax
    // u = B \ (ẋ - Ax)
    // u = B \ (next_r - Br)
    uff_ = Bd.householderQr().solve(next_r - (Ad * r));
    r_ = next_r;

    return uff_;
  }

  /**
   * Computes the feedforward control input given only the next reference state.
   * This assumes that the previous reference is already set.
   *
   * This is slower because it discretizes A and B on each run, requiring
   * computing a matrix exponential. Don't use this unless you have to.
   *
   * @param next_r The next reference state.
   * @param dt The timestep for this run.
   */
  EVec<INPUTS> calculate(const EVec<STATES>& next_r, const double& dt) { return calculate(r_, next_r, dt); }

  /**
   * Resets the reference to the given state, and the feedforward to zero.
   *
   * @param initial_state The state to set the current reference to.
   */
  void reset(const EVec<STATES>& initial_state) {
    r_ = initial_state;
    uff_.setZero();
  }

  /**
   * Resets the reference to all zeros, and the feedforward to zero.
   */
  void reset() {
    r_.setZero();
    uff_.setZero();
  }

  /**
   * Sets the current reference to a given state.
   *
   * @param r The state to set the current reference to.
   */
  void set_r(const EVec<STATES>& r) { r_ = r; }

 private:
  // The continuous system matrices
  EMat<STATES, STATES> A_;
  EMat<STATES, INPUTS> B_;

  // The discrete system matrices discretized on the nominal timestep
  EMat<STATES, STATES> Ad_;
  EMat<STATES, INPUTS> Bd_;

  // The feedforward control input
  EVec<INPUTS> uff_;
  // The current reference state
  EVec<STATES> r_;

  double m_dt;
};