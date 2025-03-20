#pragma once

#include "../core/include/utils/math/eigen_interface.h"

template <int STATES, int INPUTS, int OUTPUTS>
class SquareRootKalmanFilter {
 public:
  using StateVector = EVec<STATES>;
  using InputVector = EVec<INPUTS>;
  using OutputVector = EVec<OUTPUTS>;

  using StateMatrix = EMat<STATES, STATES>;
  using InputMatrix = EMat<STATES, INPUTS>;

  SquareRootKalmanFilter(const StateMatrix &F, const InputMatrix &B,
                         const EMat<OUTPUTS, STATES> &H,
                         const StateVector &state_std_devs,
                         const OutputVector &measurement_std_devs) {
    m_F = F;
    m_B = B;
    m_H = H;

    m_sqrt_Q = state_std_devs.asDiagonal();
    m_sqrt_R = measurement_std_devs.asDiagonal();

    reset();
  }

  const StateMatrix &S() const { return m_S; }

  double S(int i, int j) const { return m_S(i, j); }

  void set_S(const StateMatrix &S) { m_S = S; }

  StateMatrix P() const { return m_S.transpose() * m_S; }

  void set_P(const StateMatrix &P) { m_S = P.llt().matrixU(); }

  const StateVector &xhat() const { return m_xhat; }

  double xhat(int i) const { return m_xhat(i); }

  void set_xhat(const StateVector &xhat) { m_xhat = xhat; }

  void set_xhat(int i, double value) { m_xhat(i) = value; }

  void reset() {
    m_xhat.setZero();
    m_S.setZero();
  }

  void predict(const InputVector &u) {
    // Compute prior mean
    // (Tracy 4)
    m_xhat = m_F * m_xhat + m_B * u;

    // Form temporary matrix to compute prior state covariance
    EMat<STATES * 2, STATES> S_bar;
    S_bar.template block<STATES, STATES>(0, 0) = (m_S * m_F.transpose());
    S_bar.template block<STATES, STATES>(STATES, 0) = m_sqrt_Q;

    // Compute prior state covariance
    // (Tracy 5)
    m_S = S_bar.householderQr()
              .matrixQR()
              .template block<STATES, STATES>(0, 0)
              .template triangularView<Eigen::Upper>();
  }

  void correct(const OutputVector &y) { correct<OUTPUTS>(y, m_H, m_sqrt_R); }

  void correct(const OutputVector &y,
               const EMat<OUTPUTS, OUTPUTS> &R) {
    correct<OUTPUTS>(y, m_H, R);
  }

  template <int ROWS>
  void correct(const EVec<ROWS> &y,
               const EMat<ROWS, STATES> &H,
               const EMat<ROWS, ROWS> &R) {
    // Compute measurement innovation
    // (Tracy 8)
    EVec<ROWS> z = y - (H * m_xhat);

    // Form temporary matrix to compute innovation covariance
    EMat<STATES + ROWS, ROWS> Sy_bar;
    Sy_bar.template block<STATES, ROWS>(0, 0) = (m_S * H.transpose());
    Sy_bar.template block<ROWS, ROWS>(STATES, 0) = R;

    // Compute innovation covariance
    // (Tracy 9)
    EMat<ROWS, ROWS> Sy =
        Sy_bar.householderQr()
            .matrixQR()
            .template block<ROWS, ROWS>(0, 0)
            .template triangularView<Eigen::Upper>();

    // Compute Kalman gain
    // (Tracy 10)
    EMat<STATES, ROWS> K =
        (Sy.fullPivHouseholderQr().solve(
             (Sy.transpose().fullPivHouseholderQr().solve(H)) *
             (m_S.transpose() * m_S)))
            .transpose();

    // Compute posterior state
    // (Tracy 13)
    m_xhat = m_xhat + K * z;

    // Form temporary matrix to compute posterior state covariance
    EMat<STATES + ROWS, STATES> S_bar;
    S_bar.template block<STATES, STATES>(0, 0) =
        (m_S * (EMat<STATES, STATES>::Identity() - (K * H))
                   .transpose());
    S_bar.template block<ROWS, STATES>(STATES, 0) = (R * K.transpose());

    // Compute posterior state covariance
    // (Tracy 14)
    m_S = S_bar.householderQr()
              .matrixQR()
              .template block<STATES, STATES>(0, 0)
              .template triangularView<Eigen::Upper>();
  }

 private:
  StateVector m_xhat;
  StateMatrix m_S;

  StateMatrix m_sqrt_Q;
  EMat<OUTPUTS, OUTPUTS> m_sqrt_R;

  StateMatrix m_F;
  EMat<STATES, INPUTS> m_B;
  EMat<OUTPUTS, STATES> m_H;

  double m_dt;
};

template <int STATES, int INPUTS, int OUTPUTS>
using SRKF = SquareRootKalmanFilter<STATES, INPUTS, OUTPUTS>;
