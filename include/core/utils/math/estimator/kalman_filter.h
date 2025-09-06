#pragma once

#include "core/utils/math/eigen_interface.h"
#include "core/utils/math/systems/linear_system.h"

/**
 * Kalman filters combine predictions from a model and measurements to estimate
 * a system's true state.
 *
 * Each call of predict moves the state forward in time according to the matrix A,
 * and the covariance has white noise Q added.
 *
 * Each call of correct applies a measurement which moves the state more toward
 * the true state, and it reduces the state covariance.
 *
 * To read more about Kalman filters read:
 * https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
 *
 * @tparam STATES Dimension of the state vector.
 * @tparam INPUTS Dimension of the control input vector.
 * @tparam OUTPUTS Dimension of the measurement vector.
 */
template <int STATES, int INPUTS, int OUTPUTS>
class KalmanFilter {
 public:
  using StateVector = EVec<STATES>;
  using InputVector = EVec<INPUTS>;
  using OutputVector = EVec<OUTPUTS>;

  using StateMatrix = EMat<STATES, STATES>;
  using InputMatrix = EMat<STATES, INPUTS>;

  /**
   * Constructs a Kalman filter.
   *
   * @param plant The linear system the filter tracks.
   * @param state_stddevs The standard deviations of the states.
   * @param measurement_stddevs The standard deviations of the measurements.
   */
  KalmanFilter(LinearSystem<STATES, INPUTS, OUTPUTS>& plant, const StateVector& state_stddevs,
               const OutputVector& measurement_stddevs)
      : KalmanFilter(plant.A(), plant.B(), plant.C(), plant.D(), state_stddevs, measurement_stddevs) {}

  /**
   * Constructs a Kalman filter.
   *
   * @param A The state matrix.
   * @param B The input matrix.
   * @param C The measurement matrix.
   * @param D The feedthrough matrix.
   * @param state_stddevs The standard deviations of the states.
   * @param measurement_stddevs The standard deviations of the measurements.
   */
  KalmanFilter(const StateMatrix& A, const InputMatrix& B, const EMat<OUTPUTS, STATES>& C,
               const EMat<OUTPUTS, INPUTS>& D, const StateVector& state_stddevs,
               const OutputVector& measurement_stddevs) {
    A_ = A;
    B_ = B;
    C_ = C;
    D_ = D;

    Q_ = state_stddevs.asDiagonal();
    Q_ = Q_ * Q_.transpose();
    R_ = measurement_stddevs.asDiagonal();
    R_ = R_ * R_.transpose();

    reset();
  }

  /**
   * Returns the covariance matrix P.
   */
  StateMatrix P() const { return P_; }

  /**
   * Set the current covariance matrix P.
   *
   * @param P The covariance matrix P.
   */
  void set_P(const StateMatrix& P) { P_ = P; }

  /**
   * Returns the current state estimate x-hat.
   */
  const StateVector& xhat() const { return xhat_; }

  /**
   * Returns one element of the current state estimate x-hat.
   *
   * @param i Row of x-hat.
   */
  double xhat(int i) const { return xhat_(i); }

  /**
   * Set the current state estimate x-hat.
   */
  void set_xhat(const StateVector& xhat) { xhat_ = xhat; }

  /**
   * Set one element of the current state estimate x-hat.
   *
   * @param i Row of x-hat.
   */
  void set_xhat(int i, double value) { xhat_(i) = value; }

  /**
   * Resets the filter.
   */
  void reset() {
    xhat_.setZero();
    P_.setZero();
  }

  /**
   * Projects the state into the future by dt seconds with control input u.
   *
   * @param u The control input.
   * @param dt The timestep in seconds.
   */
  void predict(const InputVector& u, const double& dt) {
    // Q is discrete sqrt(process noise)
    EMat<STATES, STATES> Q = Q_ * dt;
    auto [A, B] = discretize_AB(A_, B_, dt);

    // Compute prior mean
    xhat_ = A * xhat_ + B * u;

    // Compute prior covariance
    P_ = A * P_ * A.transpose() + Q;
  }

  /**
   * Correct the state estimate using the measurements in y.
   *
   * @param y The vector of measurements.
   * @param u The control input used in the last predict step.
   */
  void correct(const OutputVector& y, const InputVector& u) { correct<OUTPUTS>(y, u, C_, D_, R_); }

  /**
   * Correct the state estimate using the measurements in y, and custom
   * measurement noise matrix. This is useful for when the noise in the
   * measurements vary.
   *
   * @param y The vector of measurements.
   * @param u The control input used in the last predict step.
   * @param R The measurement noise matrix to use for this step.
   */
  void correct(const OutputVector& y, const InputVector& u, const EMat<OUTPUTS, OUTPUTS>& R) {
    correct<OUTPUTS>(y, u, C_, D_, R);
  }

  /**
   * Correct the state estimate using the measurements in y, custom measurement
   * and feedthrough matrices, and custom measurement measurement noise.
   * This is useful for when a different set of measurements are being applied
   * than what the plant defines.
   *
   * @param y The vector of measurements.
   * @param u The control input used in the last predict step.
   * @param C The measurement matrix to use for this step.
   * @param D The feedthrough matrix to use for this step.
   * @param R The measurement noise matrix to use for this step.
   */
  template <int ROWS>
  void correct(const EVec<ROWS>& y, const InputVector& u, const EMat<ROWS, STATES>& C, const EMat<ROWS, INPUTS>& D,
               const EMat<ROWS, ROWS>& R) {
    // Compute the innovation covariance
    //
    //   Py = CPCᵀ + R
    //
    EMat<ROWS, ROWS> Py = C * P_ * C.transpose() + R;

    // Compute the optimal Kalamn gain
    //
    //   K = (Py \ CPᵀ)ᵀ
    //
    EMat<STATES, ROWS> K = Py.transpose().ldlt().solve(C * P_.transpose()).transpose();

    // Compute the posterior mean
    //
    //   x̂ = x̂ + K(y - (Cx̂ + Du))
    //
    xhat_ += K * (y - (C * xhat_ + D * u));

    // Compute the posterior covariance using the Joseph form update equation
    //
    // P = (I - KC)P(I - KC)ᵀ + KRKᵀ
    //
    P_ = (EMat<STATES, STATES>::Identity() - K * C) * P_ * (EMat<STATES, STATES>::Identity() - K * C).transpose() +
         K * R * K.transpose();
  }

 private:
  StateVector xhat_;
  StateMatrix P_;

  StateMatrix Q_;
  EMat<OUTPUTS, OUTPUTS> R_;

  StateMatrix A_;
  EMat<STATES, INPUTS> B_;
  EMat<OUTPUTS, STATES> C_;
  EMat<OUTPUTS, INPUTS> D_;

  double dt_;
};

// allow using both names
template <int STATES, int INPUTS, int OUTPUTS>
using KF = KalmanFilter<STATES, INPUTS, OUTPUTS>;
