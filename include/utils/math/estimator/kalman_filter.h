#pragma once

#include "../core/include/utils/math/eigen_interface.h"

#include "../core/include/utils/math/systems/LinearSystem.h"

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
 * This implementation uses the square-root form of the Kalman filter. The main
 * reason for this is to ensure that the covariance matrix always remains positive
 * definite. To see the formulation this implements, read:
 * https://arxiv.org/pdf/2208.06452
 *
 * @tparam STATES Dimension of the state vector.
 * @tparam INPUTS Dimension of the control input vector.
 * @tparam OUTPUTS Dimension of the measurement vector.
 */
template <int STATES, int INPUTS, int OUTPUTS> class KalmanFilter {
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
    KalmanFilter(const LinearSystem &plant, const StateVector &state_stddevs, const OutputVector &measurement_stddevs)
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
    KalmanFilter(
      const StateMatrix &A, const InputMatrix &B, const EMat<OUTPUTS, STATES> &C, const EMat<OUTPUTS, INPUTS> &D,
      const StateVector &state_stddevs, const OutputVector &measurement_stddevs
    ) {
        m_A = A;
        m_B = B;
        m_C = C;
        m_D = D;

        m_sqrtQ = state_std_devs.asDiagonal();
        m_sqrtR = measurement_std_devs.asDiagonal();

        reset();
    }

    /**
     * Returns the square-root covariance matrix S.
     */
    const StateMatrix &S() const { return m_S; }

    /**
     * Returns one element of the square-root covariance matrix S.
     *
     * @param i Row of S.
     * @param j Column of S.
     */
    double S(int i, int j) const { return m_S(i, j); }

    /**
     * Set the current square-root covariance matrix S.
     *
     * @param S The new square-root covariance matrix S.
     */
    void set_S(const StateMatrix &S) { m_S = S; }

    /**
     * Returns the reconstructed covariance matrix P.
     */
    StateMatrix P() const { return m_S.transpose() * m_S; }

    /**
     * Set the current square-root covariance matrix S to the square-root of P.
     *
     * @param P The covariance matrix P.
     */
    void set_P(const StateMatrix &P) { m_S = P.llt().matrixU(); }

    /**
     * Returns the current state estimate x-hat.
     */
    const StateVector &xhat() const { return m_xhat; }

    /**
     * Returns one element of the current state estimate x-hat.
     *
     * @param i Row of x-hat.
     */
    double xhat(int i) const { return m_xhat(i); }

    /**
     * Set the current state estimate x-hat.
     */
    void set_xhat(const StateVector &xhat) { m_xhat = xhat; }

    /**
     * Set one element of the current state estimate x-hat.
     *
     * @param i Row of x-hat.
     */
    void set_xhat(int i, double value) { m_xhat(i) = value; }

    /**
     * Resets the filter.
     */
    void reset() {
        m_xhat.setZero();
        m_S.setZero();
    }

    /**
     * Projects the state into the future by dt seconds with control input u.
     *
     * @param u The control input.
     * @param dt The timestep in seconds.
     */
    void predict(const InputVector &u, const double &dt) {
        // Q is discrete sqrt(process noise)
        EMat<STATES, STATES> Q = m_sqrtQ * sqrt(dt);
        auto [A, B] = discretize_AB(m_A, m_B);

        // Compute prior mean
        // equation (4)
        m_xhat = A * m_xhat + B * u;

        // Form temporary matrix to compute prior state covariance
        EMat<STATES * 2, STATES> S_bar;
        S_bar.template block<STATES, STATES>(0, 0) = (m_S * A.transpose());
        S_bar.template block<STATES, STATES>(STATES, 0) = m_sqrtQ;

        // Compute prior state covariance
        // equation (5)
        m_S = S_bar.householderQr().matrixQR().template block<STATES, STATES>(0, 0).template triangularView<Eigen::Upper>();
    }

    /**
     * Correct the state estimate using the measurements in y.
     *
     * @param y The vector of measurements.
     * @param u The control input used in the last predict step.
     */
    void correct(const OutputVector &y, const InputVector &u) { correct<OUTPUTS>(y, u, m_C, m_D, m_sqrtR); }

    /**
     * Correct the state estimate using the measurements in y, and custom standard
     * deviations. This is useful for when the noise in the measurements vary.
     *
     * @param y The vector of measurements.
     * @param u The control input used in the last predict step.
     * @param sqrtR The square root measurement noise matrix to use for this step.
     */
    void correct(const OutputVector &y, const InputVector &u, const EMat<OUTPUTS, OUTPUTS> &sqrtR) {
        correct<OUTPUTS>(y, u, m_C, m_D, sqrtR);
    }

    /**
     * Correct the state estimate using the measurements in y, custom measurement
     * and feedthrough matrices, and custom measurement standard deviations.
     * This is useful for when a different set of measurements are being applied
     * than what the plant defines.
     *
     * @param y The vector of measurements.
     * @param u The control input used in the last predict step.
     * @param C The measurement matrix to use for this step.
     * @param D The feedthrough matrix to use for this step.
     * @param sqrtR The square root measurement noise matrix to use for this step.
     */
    template <int ROWS>
    void correct(
      const EVec<ROWS> &y, const InputVector &u, const EMat<ROWS, STATES> &C, const EMat<ROWS, INPUTS> &D,
      const EMat<ROWS, ROWS> &sqrtR
    ) {
        // R is sqrt(measurement noise)

        // Compute measurement innovation
        // equation (8)
        EVec<ROWS> z = y - (C * m_xhat + D * u);

        // Form temporary matrix to compute innovation covariance
        EMat<STATES + ROWS, ROWS> Sy_bar;
        Sy_bar.template block<STATES, ROWS>(0, 0) = (m_S * C.transpose());
        Sy_bar.template block<ROWS, ROWS>(STATES, 0) = sqrtR;

        // Compute innovation covariance
        // equation (9)
        EMat<ROWS, ROWS> Sy =
          Sy_bar.householderQr().matrixQR().template block<ROWS, ROWS>(0, 0).template triangularView<Eigen::Upper>();

        // Compute Kalman gain
        // equation (10)
        EMat<STATES, ROWS> K =
          (Sy.template triangularView<Eigen::Upper>().solve(
             (Sy.transpose().template triangularView<Eigen::Lower>().solve(H)) * (m_S.transpose() * m_S)
           )).transpose();

        // Compute posterior state
        // equation (13)
        m_xhat = m_xhat + K * z;

        // Form temporary matrix to compute posterior state covariance
        EMat<STATES + ROWS, STATES> S_bar;
        S_bar.template block<STATES, STATES>(0, 0) = (m_S * (EMat<STATES, STATES>::Identity() - (K * H)).transpose());
        S_bar.template block<ROWS, STATES>(STATES, 0) = (sqrtR * K.transpose());

        // Compute posterior state covariance
        // equation (14)
        m_S = S_bar.householderQr().matrixQR().template block<STATES, STATES>(0, 0).template triangularView<Eigen::Upper>();
    }

  private:
    StateVector m_xhat;
    StateMatrix m_S;

    StateMatrix m_sqrtQ;
    EMat<OUTPUTS, OUTPUTS> m_sqrtR;

    StateMatrix m_A;
    EMat<STATES, INPUTS> m_B;
    EMat<OUTPUTS, STATES> m_C;
    EMat<OUTPUTS, INPUTS> m_D;

    double m_dt;
};

// allow using any of these names
template <int STATES, int INPUTS, int OUTPUTS> using KF = KalmanFilter<STATES, INPUTS, OUTPUTS>;
template <int STATES, int INPUTS, int OUTPUTS> using SRKF = KalmanFilter<STATES, INPUTS, OUTPUTS>;
template <int STATES, int INPUTS, int OUTPUTS> using SquareRootKalmanFilter = KalmanFilter<STATES, INPUTS, OUTPUTS>;
