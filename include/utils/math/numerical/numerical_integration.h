#pragma once

#include <../vendor/eigen/Eigen/Dense>

/**
 * This header provides a variety of methods for solving ODEs depending on the
 * needs of the system. First, second, fourth, and fifth order methods are
 * provided, with first order being the fastest and least accurate, and fifth
 * order being the slowest but most accurate.
 * 
 * Each method also provides a function for solving time-invariant ODEs:
 * dx/dt = f(x, u)
 * dx/dt = f(x)
 * and time-variant ODEs:
 * dy/dt = f(t, y)
 * 
 * Each function requires a function input, this can, and often is vector valued.
 */



/**
 * Performs first order numerical integration of dx/dt = f(x, u) using Euler's
 * method.
 *
 *   0|
 * ---|---
 *    |  1
 *
 * @param f The function to integrate, with two arguments x and u.
 * @param x The initial value of x.
 * @param u The input value u held constant over the integration period.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T, typename U>
T euler(F &&f, T x, U u, double dt) {
  double h = dt;
  T k1 = f(x, u);

  return x + h * k1;
}

/**
 * Performs first order numerical integration of dx/dt = f(x) using Euler's
 * method.
 *
 *   0|
 * ---|---
 *    |  1
 *
 * @param f The function to integrate, with one argument x.
 * @param x The initial value of x.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T>
T euler(F &&f, T x, double dt) {
  double h = dt;
  T k1 = f(x);

  return x + h * k1;
}

/**
 * Performs first order numerical integration of the time-varying system
 * dy/dt = f(t, y) using Euler's method.
 *
 *   0|
 * ---|---
 *    |  1
 *
 * @param f The function to integrate, with two arguments t and y.
 * @param t The initial value of t.
 * @param y The initial value of y.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T>
T euler(F &&f, double t, T y, double dt) {
  double h = dt;
  T k1 = f(t, y);

  return x + h * k1;
}

/**
 * Performs second order numerical integration of dx/dt = f(x, u) using Heun's
 * method, or the explicit trapezoid rule.
 *
 *   0|
 *   1|  1
 * ---|-------
 *    |1/2 1/2
 * 
 * @param f The function to integrate, with one argument x.
 * @param x The initial value of x.
 * @param u The input value u held constant over the integration period.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T, typename U>
T HeunRK2(F &&f, T x, U u, double dt) {
    const double h = dt;

    T k1 = f(x, u);
    T k2 = f(x + h * k1, u);

    return x + (h / 2.0) * (k1 + k2);
}

/**
 * Performs second order numerical integration of dx/dt = f(x) using Heun's
 * method, or the explicit trapezoid rule.
 *
 *   0|
 *   1|  1
 * ---|-------
 *    |1/2 1/2
 * 
 * @param f The function to integrate, with one argument x.
 * @param x The initial value of x.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T>
T HeunRK2(F &&f, T x, double dt) {
  const double h = dt;

  T k1 = f(x);
  T k2 = f(x + h * k1);

  return x + (h / 2.0) * (k1 + k2);
}

/**
 * Performs second order numerical integration of the time-varying system
 * dy/dt = f(t, y) using Heun's second order method, or the explicit trapezoid
 * rule.
 *
 *   0|
 *   1|  1
 * ---|-------
 *    |1/2 1/2
 * 
 * @param f The function to integrate, with two arguments t and y.
 * @param t The initial value of t.
 * @param y The initial value of y.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T>
T HeunRK2(F &&f, double t, T y, double dt) {
  const double h = dt;

  T k1 = f(t, y);
  T k2 = f(t + h, y + h * k1);

  return x + (h / 2.0) * (k1 + k2);
}

/**
 * Performs fourth order numerical integration of dx/dt = f(x, u) using the
 * fourth order Runge-Kutta method.
 *
 *   0|
 * 1/2|1/2
 * 1/2|  0 1/2
 *   1|  0   0   1
 * ---|---------------
 *    |1/6 1/3 1/3 1/6
 * 
 * @param f The function to integrate, with one argument x.
 * @param x The initial value of x.
 * @param u The input value u held constant over the integration period.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T, typename U>
T RK4(F &&f, T x, U u, double dt) {
  const double h = dt;

  T k1 = f(x, u);
  T k2 = f(x + h * 0.5 * k1, u);
  T k3 = f(x + h * 0.5 * k2, u);
  T k4 = f(x + h * k3, u);

  return x + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

/**
 * Performs fourth order numerical integration of dx/dt = f(x) using the
 * fourth order Runge-Kutta method.
 *
 *   0|
 * 1/2|1/2
 * 1/2|  0 1/2
 *   1|  0   0   1
 * ---|---------------
 *    |1/6 1/3 1/3 1/6
 * 
 * @param f The function to integrate, with one argument x.
 * @param x The initial value of x.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T>
T RK4(F &&f, T x, double dt) {
  const double h = dt;

  T k1 = f(x);
  T k2 = f(x + h * 0.5 * k1);
  T k3 = f(x + h * 0.5 * k2);
  T k4 = f(x + h * k3);

  return x + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

/**
 * Performs fourth order numerical integration of dy/dt = f(t, y) using the
 * fourth order Runge-Kutta method.
 *
 *   0|
 * 1/2|1/2
 * 1/2|  0 1/2
 *   1|  0   0   1
 * ---|---------------
 *    |1/6 1/3 1/3 1/6
 * 
 * @param f The function to integrate, with two arguments t and y.
 * @param t The initial value of t.
 * @param y The initial value of y.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T>
T RK4(F &&f, double t, T y, double dt) {
  const double h = dt;

  T k1 = f(t, y);
  T k2 = f(t + h * 0.5, y + h * 0.5 * k1);
  T k3 = f(t + h * 0.5, y + h * 0.5 * k2);
  T k4 = f(t + h, y + h * k3);

  return x + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

/**
 * Performs fifth order numerical integration of dx/dt = f(x, u) using Nystrom's
 * fifth order Runge-Kutta method.
 *
 *   0|
 * 1/3| 1/3
 * 2/5|4/25    6/25
 *   1| 1/4      -3      15/4
 * 2/3|2/27    10/9    -50/81   8/81
 * 4/5|2/25   12/15      2/15   8/75
 * ---|------------------------------------------------
 *    |23/192     0   125/192      0   -27/64   125/192
 * 
 * @param f The function to integrate, with one argument x.
 * @param x The initial value of x.
 * @param u The input value u held constant over the integration period.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T, typename U>
T NystromRK5(F &&f, T x, U u, double dt) {
  const double h = dt;

  constexpr double A[5][5]{
    {1.0 / 3.0, 0, 0, 0, 0},
    {4.0 / 25.0, 6.0 / 25.0, 0, 0, 0},
    {1.0 / 4.0, -3.0, 15.0 / 4.0, 0, 0},
    {2.0 / 27.0, 10.0 / 9.0, -50.0 / 81.0, 8.0 / 81.0, 0},
    {2.0 / 25.0, 12.0 / 15.0, 2.0 / 15.0, 8.0 / 75.0, 0}
  };

  T k1 = f(x, u);
  T k2 = f(x + h * (A[0][0] * k1), u);
  T k3 = f(x + h * (A[1][0] * k1 + A[1][1] * k2), u);
  T k4 = f(x + h * (A[2][0] * k1 + A[2][1] * k2 + A[2][2] * k3), u);
  T k5 = f(x + h * (A[3][0] * k1 + A[3][1] * k2 + A[3][2] * k3 + A[3][3] * k4), u);
  T k6 = f(x + h * (A[4][0] * k1 + A[4][1] * k2 + A[4][2] * k3 + A[4][3] * k4 + A[4][4] * k5), u);

  return x + (h / 192.0) * (23.0 * k1 + 0 * k2 + 125.0 * k3 + 0.0 * k4 + -81.0 * k5 + 125.0 * k6);
}

/**
 * Performs fifth order numerical integration of dx/dt = f(x) using Nystrom's
 * fifth order Runge-Kutta method.
 *
 *   0|
 * 1/3| 1/3
 * 2/5|4/25    6/25
 *   1| 1/4      -3      15/4
 * 2/3|2/27    10/9    -50/81   8/81
 * 4/5|2/25   12/15      2/15   8/75
 * ---|------------------------------------------------
 *    |23/192     0   125/192      0   -27/64   125/192
 * 
 * @param f The function to integrate, with one argument x.
 * @param x The initial value of x.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T>
T NystromRK5(F &&f, T x, double dt) {
  const double h = dt;

  constexpr double A[5][5]{
    {1.0 / 3.0, 0, 0, 0, 0},
    {4.0 / 25.0, 6.0 / 25.0, 0, 0, 0},
    {1.0 / 4.0, -3.0, 15.0 / 4.0, 0, 0},
    {2.0 / 27.0, 10.0 / 9.0, -50.0 / 81.0, 8.0 / 81.0, 0},
    {2.0 / 25.0, 12.0 / 15.0, 2.0 / 15.0, 8.0 / 75.0, 0}
  };

  T k1 = f(x, u);
  T k2 = f(x + h * (A[0][0] * k1));
  T k3 = f(x + h * (A[1][0] * k1 + A[1][1] * k2));
  T k4 = f(x + h * (A[2][0] * k1 + A[2][1] * k2 + A[2][2] * k3));
  T k5 = f(x + h * (A[3][0] * k1 + A[3][1] * k2 + A[3][2] * k3 + A[3][3] * k4));
  T k6 = f(x + h * (A[4][0] * k1 + A[4][1] * k2 + A[4][2] * k3 + A[4][3] * k4 + A[4][4] * k5));

  return x + (h / 192.0) * (23.0 * k1 + 0 * k2 + 125.0 * k3 + 0.0 * k4 + -81.0 * k5 + 125.0 * k6);
}

/**
 * Performs fifth order numerical integration of dy/dt = f(t, y) using Nystrom's
 * fifth order Runge-Kutta method.
 *
 *   0|
 * 1/3| 1/3
 * 2/5|4/25    6/25
 *   1| 1/4      -3      15/4
 * 2/3|2/27    10/9    -50/81   8/81
 * 4/5|2/25   12/15      2/15   8/75
 * ---|------------------------------------------------
 *    |23/192     0   125/192      0   -27/64   125/192
 * 
 * @param f The function to integrate, with one argument x.
 * @param x The initial value of x.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T>
T NystromRK5(F &&f, double t, T y, double dt) {
  const double h = dt;

  constexpr double A[5][5]{
    {1.0 / 3.0, 0, 0, 0, 0},
    {4.0 / 25.0, 6.0 / 25.0, 0, 0, 0},
    {1.0 / 4.0, -3.0, 15.0 / 4.0, 0, 0},
    {2.0 / 27.0, 10.0 / 9.0, -50.0 / 81.0, 8.0 / 81.0, 0},
    {2.0 / 25.0, 12.0 / 15.0, 2.0 / 15.0, 8.0 / 75.0, 0}
  };

  T k1 = f(t, y);
  T k2 = f(t + h * (1.0 / 3.0), y + h * (A[0][0] * k1));
  T k3 = f(t + h * (2.0 / 5.0), y + h * (A[1][0] * k1 + A[1][1] * k2));
  T k4 = f(t + h,               y + h * (A[2][0] * k1 + A[2][1] * k2 + A[2][2] * k3));
  T k5 = f(t + h * (2.0 / 3.0), y + h * (A[3][0] * k1 + A[3][1] * k2 + A[3][2] * k3 + A[3][3] * k4));
  T k6 = f(t + h * (4.0 / 5.0), y + h * (A[4][0] * k1 + A[4][1] * k2 + A[4][2] * k3 + A[4][3] * k4 + A[4][4] * k5));

  return x + (h / 192.0) * (23.0 * k1 + 0 * k2 + 125.0 * k3 + 0.0 * k4 + -81.0 * k5 + 125.0 * k6);
}
