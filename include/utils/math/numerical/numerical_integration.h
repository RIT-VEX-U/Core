#pragma once

/**
 * This header provides a variety of methods for solving ODEs depending on the
 * needs of the system. First, second, and fourth order methods are
 * provided, with first order being the fastest and least accurate, and fourth
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

  return y + h * k1;
}

/**
 * Performs second order numerical integration of dx/dt = f(x, u) using the
 * explicit midpoint method.
 *
 *   0|
 * 1/2|1/2
 * ---|-------
 *    |  0   1
 * 
 * @param f The function to integrate, with one argument x.
 * @param x The initial value of x.
 * @param u The input value u held constant over the integration period.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T, typename U>
T RK2(F &&f, T x, U u, double dt) {
    const double h = dt;

    T k1 = f(x, u);
    T k2 = f(x + h * 0.5 * k1, u);

    return x + h * k2;
}

/**
 * Performs second order numerical integration of dx/dt = f(x) using the
 * explicit midpoint method.
 *
 *   0|
 * 1/2|1/2
 * ---|-------
 *    |  0   1
 * 
 * @param f The function to integrate, with one argument x.
 * @param x The initial value of x.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T>
T RK2(F &&f, T x, double dt) {
  const double h = dt;

  T k1 = f(x);
  T k2 = f(x + h * 0.5 * k1);

  return x + h * k2;
}

/**
 * Performs second order numerical integration of the time-varying system
 * dy/dt = f(t, y) using the explicit midpoint method.
 *
 *   0|
 * 1/2|1/2
 * ---|-------
 *    |  0   1
 * 
 * @param f The function to integrate, with two arguments t and y.
 * @param t The initial value of t.
 * @param y The initial value of y.
 * @param dt The time over which to integrate.
 */
template <typename F, typename T>
T RK2(F &&f, double t, T y, double dt) {
  const double h = dt;

  T k1 = f(t, y);
  T k2 = f(t + h * 0.5, y + h * 0.5 * k1);

  return y + h * k2;
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

  return y + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}
