#pragma once

#include <functional>

#include "core/utils/math/eigen_interface.h"
#include "core/utils/math/numerical/numerical_integration.h"

// Forward declare the sigma points class, it is at the bottom of this file.
template <int STATES> class ScaledSphericalSimplexSigmaPoints;

// Forward declare the Unscented Transform function, it is after the SRUKF class itself.
template <int COV_DIM, int STATES, int NUM_SIGMAS>
std::tuple<EVec<COV_DIM>, EMat<COV_DIM, COV_DIM>> square_root_ut(
  const EMat<COV_DIM, NUM_SIGMAS> &sigmas, const EVec<NUM_SIGMAS> &Wm, const EVec<NUM_SIGMAS> &Wc,
  const std::function<EVec<COV_DIM>(const EMat<COV_DIM, NUM_SIGMAS> &, const EVec<NUM_SIGMAS> &)> &mean_func,
  const std::function<EVec<COV_DIM>(const EVec<COV_DIM> &, const EVec<COV_DIM> &)> &residual_func,
  const EMat<COV_DIM, COV_DIM> &square_root_R
);

/**
 * Kalman filters combine predictions from a model and measurements to estimate
 * a system's true state.
 *
 * The Unscented Kalman Filter is a nonlinear estimator, meaning that the model
 * used to predict how the state changes over time can be nonlinear. The model
 * that determines the expected measurement given the current state can also be
 * nonlinear.
 *
 * At each timestep, sigma points are generated close to the mean, they are all
 * propagated forward in time according to the nonlinear model. The Unscented
 * Transform uses the propagated sigma points to compute the prior state and
 * covariance.
 *
 * When correcting the state and covariance with a measurement, sigma points are
 * again generated, but are transformed into the measurement space using the
 * measurement function. A Kalman gain matrix K is then computed, and used to
 * update the state and covariance.
 *
 * To read more about Kalman filters and the standard UKF read:
 * https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
 *
 * This implementation is somewhat non-standard. The square-root form of the UKF
 * (SR-UKF) is used, and the way the sigma points are generated is different than
 * most implementations. The square-root form is used to ensure that the
 * covariance matrix remains positive definite.
 *
 * To learn more about the SR-UKF, and see the exact formulation that most of this
 * implementation follows, read:
 * https://www.researchgate.net/publication/3908304
 *
 * The sigma points are not generated symmetrically around the mean, instead they
 * are generated as vertices of a simplex. Using N = # of states, this method uses
 * N + 2 sigma points instead of the standard 2N + 1 sigma points. This reduces
 * computation up to 50%. To learn more about this method, read:
 * https://www.sciencedirect.com/science/article/pii/S0888327020308190
 * 
 * This filter uses a method of "recalibrating" by essentially applying a measurement
 * twice instead of once, and only using it if it is more accurate than before the
 * measurement was applied. To learn more about this framework for nonlinear filters,
 * read:
 * https://arxiv.org/pdf/2407.05717
 *
 * @tparam STATES Dimension of the state vector.
 * @tparam INPUTS Dimension of the control input vector.
 * @tparam OUTPUTS Dimension of the measurement vector.
 */
template <int STATES, int INPUTS, int OUTPUTS> class SquareRootUnscentedKalmanFilter {
  public:
    static constexpr int NUM_SIGMAS = STATES + 2;

    using StateVector = EVec<STATES>;
    using InputVector = EVec<INPUTS>;
    using OutputVector = EVec<OUTPUTS>;

    using StateMatrix = EMat<STATES, STATES>;

    using WithInputIntegrator = std::function<EVec<STATES>(
      const WithInputDerivative<STATES, INPUTS> &f, const EVec<STATES> &x, const EVec<INPUTS> &u, const double &h
    )>;

    /**
     * Constructs an Unscented Kalman Filter.
     *
     * @param f A vector valued function of x and u that returns the derivative of
     * the state vector with respect to time.
     * @param h A vector valued function of x and u that returns the expected
     * measurement at the given state.
     * @param integrator A function from "numerical_integration.h" that integrates
     * a differential equation of the form f(x, u).
     * @param state_stddevs Standard deviations of the states in the model.
     * @param measurement_stddevs Standard deviations of the measurements.
     */
    SquareRootUnscentedKalmanFilter(
      const std::function<StateVector(const StateVector &, const InputVector &)> &f,
      const std::function<OutputVector(const StateVector &, const InputVector &)> &h,
      const WithInputIntegrator &integrator, const StateVector &state_stddevs, const OutputVector &measurement_stddevs
    )
        : m_f(f), m_h(h), m_integrator(integrator) {
        m_sqrt_Q = state_stddevs.asDiagonal();
        m_measurement_stddevs = measurement_stddevs;
        m_mean_func_X = [](const EMat<STATES, NUM_SIGMAS> &sigmas, const EVec<NUM_SIGMAS> &Wm) -> StateVector {
            return sigmas * Wm;
        };

        m_mean_func_Y = [](const EMat<OUTPUTS, NUM_SIGMAS> &sigmas, const EVec<NUM_SIGMAS> &Wc) -> OutputVector {
            return sigmas * Wc;
        };
        m_residual_func_X = [](const StateVector &a, const StateVector &b) -> StateVector { return a - b; };
        m_residual_func_Y = [](const OutputVector &a, const OutputVector &b) -> OutputVector { return a - b; };
        m_add_func_X = [](const StateVector &a, const StateVector &b) -> StateVector { return a + b; };

        reset();
    }

    /**
     * Constructs an Unscented Kalman Filter with custom mean, residual, and
     * addition functions. The most common use for these functions is when you
     * are estimating angles whose arithmetic operations need to be wrapped.
     *
     * @param f A vector valued function of x and u that returns the derivative of
     * the state vector with respect to time.
     * @param h A vector valued function of x and u that returns the expected
     * measurement at the given state.
     * @param integrator A function from "numerical_integration.h" that integrates
     * a differential equation of the form f(x, u).
     * @param state_stddevs Standard deviations of the states in the model.
     * @param measurement_stddevs Standard deviations of the measurements.
     * @param mean_func_X A function that computes the mean of a matrix
     * containing NUM_SIGMAS state sigma points with a set of weights for each.
     * @param mean_func_Y A function that computes the mean of a matrix
     * containing NUM_SIGMAS measurement sigma points with a set of weights for each.
     * @param residual_func_X A function that computes the residual of two state
     * vectors, usually by simple subtraction.
     * @param residual_func_Y A function that computes the residual of two measurement
     * vectors, usually by simple subtraction.
     * @param add_funx_X A function that adds two state vectors.
     */
    SquareRootUnscentedKalmanFilter(
      const std::function<StateVector(const StateVector &, const InputVector &)> &f,
      const std::function<OutputVector(const StateVector &, const InputVector &)> &h,
      const WithInputIntegrator &integrator, const StateVector &state_stddevs, const OutputVector &measurement_stddevs,
      const std::function<StateVector(const EMat<STATES, NUM_SIGMAS> &, const EVec<NUM_SIGMAS> &)> &mean_func_X,
      const std::function<OutputVector(const EMat<OUTPUTS, NUM_SIGMAS> &, const EVec<NUM_SIGMAS> &)> &mean_func_Y,
      const std::function<StateVector(const StateVector &, const StateVector &)> &residual_func_X,
      const std::function<OutputVector(const OutputVector &, const OutputVector &)> &residual_func_Y,
      const std::function<StateVector(const StateVector &, const StateVector &)> &add_func_X
    )
        : m_f(f), m_h(h), m_integrator(integrator), m_mean_func_X(mean_func_X), m_mean_func_Y(mean_func_Y),
          m_residual_func_X(residual_func_X), m_residual_func_Y(residual_func_Y), m_add_func_X(add_func_X) {
        m_sqrt_Q = state_stddevs.asDiagonal();
        m_measurement_stddevs = measurement_stddevs;

        reset();
    }

    /**
     * Returns the square-root covariance matrix S.
     */
    StateMatrix S() const { return m_S; }

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
    StateMatrix P() const { return m_S * m_S.transpose(); }

    /**
     * Set the current square-root covariance matrix S to the square-root of P.
     *
     * @param P The covariance matrix P.
     */
    void set_P(const StateMatrix &P) { m_S = P.llt().matrixL(); }

    /**
     * Returns the current state estimate x-hat.
     */
    StateVector xhat() const { return m_xhat; }

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
     * Make sure to explicitly set S after calling this.
     */
    void reset() {
        m_xhat.setZero();
        m_S.setZero();
        m_sigmas_F.setZero();
    }

    /**
     * Projects the state into the future by dt seconds with control input u.
     *
     * @param u The control input.
     * @param dt The timestep in seconds.
     */
    void predict(const InputVector &u, double dt) {
        // Our noise is continuous, so we need to discretize
        const EMat<STATES, STATES> Q = m_sqrt_Q * sqrt(dt);

        // Generate sigma points around the state mean
        //
        // equation (17)
        EMat<STATES, NUM_SIGMAS> sigmas = m_pts.square_root_sigma_points(m_xhat, m_S);

        // Project each sigma point forward in time according to the
        // dynamics f(x, u)
        //
        //   sigmas  = 𝒳ₖ₋₁
        //   sigmasF = 𝒳ₖ,ₖ₋₁ or just 𝒳 for readability
        //
        // equation (18)
        for (int i = 0; i < NUM_SIGMAS; ++i) {
            StateVector x = sigmas.template block<STATES, 1>(0, i);
            m_sigmas_F.template block<STATES, 1>(0, i) = m_integrator(m_f, x, u, dt);
        }

        // Pass the predicted sigmas (𝒳) through the Unscented Transform
        // to compute the prior state mean and covariance
        //
        // equations (18) (19) and (20)
        auto [xhat, S] = square_root_ut<STATES, STATES>(
          m_sigmas_F, m_pts.Wm(), m_pts.Wc(), m_mean_func_X, m_residual_func_X,
          Q.template triangularView<Eigen::Lower>()
        );

        m_xhat = xhat;
        m_S = S;
    }

    /**
     * Correct the state estimate using the measurements in y.
     *
     * @param u The control input used in the last predict step.
     * @param y The vector of measurements.
     */
    void correct(const InputVector &u, const OutputVector &y) {
        correct<OUTPUTS>(
          u, y, m_h, m_measurement_stddevs, m_mean_func_Y, m_residual_func_Y, m_residual_func_X, m_add_func_X
        );
    }

    /**
     * Correct the state estimate using the measurements in y, and custom standard
     * deviations. This is useful for when the noise in the measurements vary.
     *
     * @param u The control input used in the last predict step.
     * @param y The vector of measurements.
     * @param measurement_stddevs The vector of standard deviations for each
     * measurement to be used for this correct step.
     */
    void correct(const InputVector &u, const OutputVector &y, const EVec<OUTPUTS> &measurement_stddevs) {
        correct<OUTPUTS>(
          u, y, m_h, measurement_stddevs, m_mean_func_Y, m_residual_func_Y, m_residual_func_X, m_add_func_X
        );
    }

    /**
     * Correct the state estimate using the measurements in y, a custom measurement
     * function, and custom standard deviations. This is useful for when a different
     * set of measurements are being applied.
     *
     * @param u The control input used in the last predict step.
     * @param y The vector of measurements.
     * @param h A vector valued function of x and u that returns the expected
     * measurement at the given state.
     * @param measurement_stddevs The vector of standard deviations for each
     * measurement to be used for this correct step.
     */
    template <int ROWS>
    void correct(
      const InputVector &u, const EVec<ROWS> &y,
      const std::function<EVec<ROWS>(const StateVector &, const InputVector &)> &h,
      const EVec<ROWS> &measurement_stddevs
    ) {
        auto mean_func_Y = [](const EMat<OUTPUTS, NUM_SIGMAS> &sigmas, const EVec<NUM_SIGMAS> &Wc) -> EVec<ROWS> {
            return sigmas * Wc;
        };
        auto residual_func_X = [](const StateVector &a, const StateVector &b) -> StateVector { return a - b; };
        auto residual_func_Y = [](const EVec<ROWS> &a, const EVec<ROWS> &b) -> EVec<ROWS> { return a - b; };
        auto add_func_X = [](const StateVector &a, const StateVector &b) -> StateVector { return a + b; };

        correct<ROWS>(u, y, h, measurement_stddevs, mean_func_Y, residual_func_Y, residual_func_X, add_func_X);
    }

    /**
     * Correct the state estimate using the measurements in y, a custom measurement
     * function, custom standard deviations, and custom mean, residual, and addition
     * functions. This is useful for when a different set of measurements are being
     * applied, and they require custom arithmetic functions.
     *
     * @param u The control input used in the last predict step.
     * @param y The vector of measurements.
     * @param h A vector valued function of x and u that returns the expected
     * measurement at the given state.
     * @param measurement_stddevs The vector of standard deviations for each
     * measurement to be used for this correct step.
     * @param mean_func_Y A function that computes the mean of a matrix
     * containing NUM_SIGMAS measurement sigma points with a set of weights for each.
     * @param residual_func_X A function that computes the residual of two state
     * vectors, usually by simple subtraction.
     * @param residual_func_Y A function that computes the residual of two measurement
     * vectors, usually by simple subtraction.
     * @param add_funx_X A function that adds two state vectors.
     */
    template <int ROWS>
    void correct(
      const InputVector &u, const EVec<ROWS> &y,
      const std::function<EVec<ROWS>(const StateVector &, const InputVector &)> &h,
      const EVec<ROWS> measurement_stddevs,
      const std::function<EVec<ROWS>(const EMat<ROWS, NUM_SIGMAS> &, const EVec<NUM_SIGMAS> &)> &mean_func_Y,
      const std::function<EVec<ROWS>(const EVec<ROWS> &, const EVec<ROWS> &)> &residual_func_Y,
      const std::function<StateVector(const StateVector &, const StateVector &)> &residual_func_X,
      const std::function<StateVector(const StateVector &, const StateVector &)> &add_func_X
    ) {

      EMat<ROWS, ROWS> sqrt_R = measurement_stddevs.asDiagonal();

      // Generate new sigma points from the prior mean and covariance
      // and transform them into measurement space using h(x, u)
      //
      //   sigmas  = 𝒳
      //   sigmasH = 𝒴
      //
      // This differs from equation (22) which uses
      // the prior sigma points, regenerating them allows
      // multiple measurement updates per time update
      EMat<ROWS, NUM_SIGMAS> sigmas_H;
      EMat<STATES, NUM_SIGMAS> sigmas = m_pts.square_root_sigma_points(m_xhat, m_S);
      for (int i = 0; i < NUM_SIGMAS; ++i) {
        sigmas_H.template block<ROWS, 1>(0, i) =
            h(sigmas.template block<STATES, 1>(0, i), u);
      }
  
      // Pass the predicted measurement sigmas through the Unscented Transform
      // to compute the mean predicted measurement and square-root innovation
      // covariance.
      //
      // equations (23) (24) and (25)
      auto [yhat, Sy] = square_root_ut<ROWS, STATES, NUM_SIGMAS>(
          sigmas_H, m_pts.Wm(), m_pts.Wc(), mean_func_Y, residual_func_Y,
          sqrt_R.template triangularView<Eigen::Lower>());
  
      // Compute cross covariance of the predicted state and measurement sigma
      // points given as:
      //
      //           L+1
      //   P_{xy} = Σ Wᵢ⁽ᶜ⁾[𝒳ᵢ - x̂][𝒴ᵢ - ŷ⁻]ᵀ
      //           i=0
      //
      // equation (26)
      EMat<STATES, ROWS> Pxy;
      Pxy.setZero();
      for (int i = 0; i < NUM_SIGMAS; ++i) {
        Pxy += m_pts.Wc(i) *
               (residual_func_X(m_sigmas_F.template block<STATES, 1>(0, i),
                                m_xhat)) *
               (residual_func_Y(sigmas_H.template block<ROWS, 1>(0, i), yhat))
                   .transpose();
      }
  
      // Compute the Kalman gain. We use Eigen's forward and backward substitution
      // to solve. The equation in the paper uses MATLAB's / operator, but Eigen's
      // solvers act like the \ operator, so we need to rearrange the equation to
      // use those.
      //
      //   K = (P_{xy} / S_{y}ᵀ) / S_{y}
      //   K = (S_{y} \ P_{xy})ᵀ / S_{y}
      //   K = (S_{y}ᵀ \ (S_{y} \ P_{xy}ᵀ))ᵀ
      //
      // equation (27)
        EMat<STATES, ROWS> K = (Sy.transpose().template triangularView<Eigen::Upper>().solve(
                                  Sy.template triangularView<Eigen::Lower>().solve(Pxy.transpose())
                                ))
              .transpose();
  
      // Compute the posterior state mean
      //
      //   x̂ = x̂⁻ + K(y − ŷ⁻)
      //
      // second part of equation (27)
      EVec<STATES> xhat_dot = K * residual_func_Y(y, yhat);
      EVec<STATES> xhat = add_func_X(m_xhat, xhat_dot);
      EMat<STATES, STATES> S = m_S;
  
      // RECALIBRATE
  
      // Add the change of xhat to each of the sigma points in 𝒳.
      for (int i = 0; i < NUM_SIGMAS; i++) {
          sigmas.template block<STATES, 1>(0, i) += (xhat_dot);
      }
  
      // Pass those sigma points through the measurement function to transform
      // them into measurement space.
      for (int i = 0; i < NUM_SIGMAS; ++i) {
          sigmas_H.template block<ROWS, 1>(0, i) = h(sigmas.template block<STATES, 1>(0, i), u);
      }
  
      // Perform a second unscented transform, this time on the recalibrated
      // measurement sigma points.
      auto [yhat_k, Sy_k] = square_root_ut<ROWS, STATES, NUM_SIGMAS>(
          sigmas_H, m_pts.Wm(), m_pts.Wc(), mean_func_Y, residual_func_Y,
          sqrt_R.template triangularView<Eigen::Lower>());
  
      // Compute the cross covariance of the recalibrated sigma points.
      Pxy.setZero();
      for (int i = 0; i < NUM_SIGMAS; ++i) {
          Pxy += m_pts.Wc(i) *
                  (residual_func_X(sigmas.template block<STATES, 1>(0, i),
                                  xhat)) *
                  (residual_func_Y(sigmas_H.template block<ROWS, 1>(0, i), yhat_k))
                      .transpose();
      }
  
      // Compute the intermediate matrix U for downdating
      // the square-root covariance
      //
      // equation (28)
      const EMat<STATES, ROWS> U = K * Sy;
  
      // Downdate the posterior square-root state covariance
      //
      // equation (29)
      for (int i = 0; i < ROWS; i++) {
        Eigen::internal::llt_inplace<double, Eigen::Lower>::rankUpdate(
            S, U.template block<STATES, 1>(0, i), -1);
      }
  
      // BACK OUT
  
      // We only use the posterior state and covariance if it is more certain
      // than the prior.
      if (m_S.trace() > S.trace()) {
          m_xhat = xhat;
          m_S = S;
      }
    }

  private:
    std::function<StateVector(const StateVector &, const InputVector &)> m_f;
    std::function<OutputVector(const StateVector &, const InputVector &)> m_h;
    std::function<StateVector(const EMat<STATES, NUM_SIGMAS> &, const EVec<NUM_SIGMAS> &)> m_mean_func_X;
    std::function<OutputVector(const EMat<OUTPUTS, NUM_SIGMAS> &, const EVec<NUM_SIGMAS> &)> m_mean_func_Y;
    std::function<StateVector(const StateVector &, const StateVector &)> m_residual_func_X;
    std::function<OutputVector(const OutputVector &, const OutputVector &)> m_residual_func_Y;
    std::function<StateVector(const StateVector &, const StateVector &)> m_add_func_X;
    StateVector m_xhat;
    StateMatrix m_S;
    StateMatrix m_sqrt_Q;
    EVec<OUTPUTS> m_measurement_stddevs;
    EMat<STATES, NUM_SIGMAS> m_sigmas_F;

    ScaledSphericalSimplexSigmaPoints<STATES> m_pts;

    WithInputIntegrator m_integrator;
};

/**
 * Computes the Unscented Transform of a set of sigma points and their weights.
 * The mean and square-root covariance of the sigma points are returned in a tuple.
 *
 * @tparam COV_DIM Dimension of the covariance of the sigma points after they are
 * passed through a transforming function.
 * @tparam STATES Dimension of the state vector.
 *
 * @param sigmas Matrix containing the sigma points, each column is one sigma point.
 * @param Wm The weights for the mean.
 * @param Wc The weights for the covariance.
 * @param mean_func A function that computes the mean of NUM_SIGMAS sigma points
 * using a given set of weights.
 * @param residual_func A function that computes the residual of two sigma points.
 * This is usually a simple subtraction.
 * @param sqrt_R Square-root of the noise covariance of the sigma points.
 *
 * @return Tuple of x, and S, the mean and square-root covariance of the sigma points.
 */
template <int COV_DIM, int STATES, int NUM_SIGMAS>
std::tuple<EVec<COV_DIM>, EMat<COV_DIM, COV_DIM>> square_root_ut(
  const EMat<COV_DIM, NUM_SIGMAS> &sigmas, const EVec<NUM_SIGMAS> &Wm, const EVec<NUM_SIGMAS> &Wc,
  const std::function<EVec<COV_DIM>(const EMat<COV_DIM, NUM_SIGMAS> &, const EVec<NUM_SIGMAS> &)> &mean_func,
  const std::function<EVec<COV_DIM>(const EVec<COV_DIM> &, const EVec<COV_DIM> &)> &residual_func,
  const EMat<COV_DIM, COV_DIM> &sqrt_R
) {
    // New mean is usually just the sum of the sigmas * weights:
    //
    //      L+1
    //   x̂ = Σ Wᵢ⁽ᵐ⁾𝒳ᵢ
    //      i=0
    //
    // equations (19) and (23) in the paper show this,
    // but we allow a custom function, usually for angle wrapping
    EVec<COV_DIM> x = mean_func(sigmas, Wm);

    // Form an intermediate matrix S⁻ as:
    //
    //   [√{W₁⁽ᶜ⁾}(𝒳_{1:L+1} - x̂) √{Rᵛ}]
    //
    // the part of equations (20) and (24) within the "qr{}"
    EMat<COV_DIM, NUM_SIGMAS - 1 + COV_DIM> S_bar;
    for (int i = 0; i < NUM_SIGMAS - 1; i++) {
        S_bar.template block<COV_DIM, 1>(0, i) =
          std::sqrt(Wc[1]) * residual_func(sigmas.template block<COV_DIM, 1>(0, i + 1), x);
    }
    S_bar.template block<COV_DIM, COV_DIM>(0, NUM_SIGMAS - 1) = sqrt_R;

    // Compute the square-root covariance of the sigma points.
    //
    // We transpose S⁻ first because we formed it by horizontally
    // concatenating each part; it should be vertical so we can take
    // the QR decomposition as defined in the "QR Decomposition" passage
    // of section 3. "EFFICIENT SQUARE-ROOT IMPLEMENTATION"
    //
    // The resulting matrix R is the square-root covariance S, but it
    // is upper triangular, so we need to transpose it.
    //
    // equations (20) and (24)
    EMat<COV_DIM, COV_DIM> S = S_bar.transpose()
                                 .householderQr()
                                 .matrixQR()
                                 .template block<COV_DIM, COV_DIM>(0, 0)
                                 .template triangularView<Eigen::Upper>()
                                 .transpose();

    // Update or downdate the square-root covariance with (𝒳₀-x̂)
    // depending on whether its weight (W₀⁽ᶜ⁾) is positive or negative.
    //
    // equations (21) and (25)
    Eigen::internal::llt_inplace<double, Eigen::Lower>::rankUpdate(
      S, residual_func(sigmas.template block<COV_DIM, 1>(0, 0), x), Wc[0]
    );

    return std::make_tuple(x, S);
}

/**
 * Generates sigma points and weights according to the paper [1]
 * This is very different from Wan and Merwe's formulation.
 *
 * This only requires N + 2 sigma points instead of 2N + 1 sigma points.
 * Rather than generating sigma points symmetrically around the mean, it
 * generates them as vertices of an N-simplex.
 *
 * The performance of the filter using this reduced set of sigma points is
 * identical to the standard method, so there is no downside to using it here.
 *
 * [1] A Scaled Spherical Simplex Filter (S3F) with a decreased n + 2 sigma
 *     points set size and equivalent 2n + 1 Unscented Kalman Filter (UKF)
 *     accuracy
 *
 * @tparam STATES the dimension of the state. NUM_SIGMAS sigma points and
 * weights will be generated.
 */
template <int STATES> class ScaledSphericalSimplexSigmaPoints {
  public:
    static constexpr int NUM_SIGMAS = STATES + 2;

    /**
     * Constructs a sigma point generator for Spherical Simplex sigma points
     *
     * @param alpha Determines the spread of the sigma points around the mean.
     * Smaller values are closer to the mean, this is usually a small value.
     * @param beta Incorporates prior knowledge of the distribution of the state.
     * For Gaussian distributions, beta = 2 is optimal.
     */
    ScaledSphericalSimplexSigmaPoints(double alpha = 0.001, double beta = 2) { compute_weights(alpha, beta); }

    /**
     * Returns the number of sigma points, for simplex sigma points this is N+2.
     */
    int num_sigmas() { return NUM_SIGMAS; }

    /**
     * Computes the sigma points given a mean (x) and square-root covariance (S).
     *
     * @param x Vector of the means.
     * @param S Square-root covariance.
     *
     * @return Matrix containing the sigma points. Each column contains one sigma
     * point in the same space as x. The first column is the same as the mean,
     * with the others arranged around the mean.
     */
    EMat<STATES, NUM_SIGMAS> square_root_sigma_points(const EVec<STATES> &x, const EMat<STATES, STATES> &S) {

        EMat<STATES, NUM_SIGMAS> C = EMat<STATES, NUM_SIGMAS>::Zero();

        for (int row = 0; row < STATES; row++) {
            C.row(row).segment(1, row + 1).setConstant(-q(row) / (row + 1));
        }
        C.diagonal(2) = q;

        EMat<STATES, NUM_SIGMAS> sigmas = S * C;
        sigmas.colwise() += x;

        return sigmas;
    }

    /**
     * Returns a vector containing the weights of each sigma point for the mean.
     */
    const EVec<NUM_SIGMAS> &Wm() const { return m_Wm; }

    /**
     * Returns a vector containing the weights of each sigma point for the covariance.
     */
    const EVec<NUM_SIGMAS> &Wc() const { return m_Wc; }

    /**
     * Returns the weight for the i-th sigma point for the mean.
     *
     * @param i Element of the weights vector to return.
     */
    double Wm(int i) const { return m_Wm(i); }

    /**
     * Returns the weight for the i-th sigma point for the covariance.
     *
     * @param i Element of the weights vector to return.
     */
    double Wc(int i) const { return m_Wc(i); }

  private:
    EVec<NUM_SIGMAS> m_Wm;
    EVec<NUM_SIGMAS> m_Wc;
    EVec<STATES> q;

    /**
     * Computes the weights for the sigma points.
     *
     * @param alpha Determines the spread of the sigma points around the mean.
     * Smaller values are closer to the mean, this is usually a small value.
     * @param beta Incorporates prior knowledge of the distribution of the state.
     * For Gaussian distributions, beta = 2 is optimal.
     */
    void compute_weights(double alpha, double beta) {
        double c = 1 / (alpha * alpha * (STATES + 1));
        m_Wm = EVec<NUM_SIGMAS>::Constant(c);
        m_Wc = EVec<NUM_SIGMAS>::Constant(c);

        m_Wm(0) = 1 - (1 / (alpha * alpha));
        m_Wc(0) = 1 - (1 / (alpha * alpha)) + (1 - alpha * alpha + beta);

        EVec<STATES> t;
        for (int i = 0; i < STATES; i++) {
            t(i) = i + 1;
        }

        q = alpha * ((t * (STATES + 1)).cwiseQuotient(t + EVec<STATES>::Ones())).cwiseSqrt();
    }
};

// allow using both names
template <int STATES, int INPUTS, int OUTPUTS> using SRUKF = SquareRootUnscentedKalmanFilter<STATES, INPUTS, OUTPUTS>;
