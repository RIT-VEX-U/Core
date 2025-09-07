#pragma once

#include <functional>

#include "core/utils/math/eigen_interface.h"

/**
 * This header provides a variety of methods for solving ODEs depending on the
 * needs of the system. First, second, and fourth order methods are
 * provided, with first order being the fastest and least accurate, and fourth
 * order being the slowest but most accurate.
 *
 * Each method also provides a function for solving time-invariant ODEs with
 * and without an input u:
 * dx/dt = f(x, u)
 * dx/dt = f(x)
 * and time-variant ODEs:
 * dy/dt = f(t, y)
 *
 * Each function here takes a std::function as an input. This must be a vector
 * valued function where x is some EVec<X>, u is some
 * EVec<U>, and t is a double.
 *
 * The template arguments are determined by the compiler as long as they are
 * valid, so you do not need to explicity state them when calling a function.
 *
 * To learn about Runge-Kutta methods in general read:
 * https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
 *
 * To see the specific methods used here read:
 * https://en.wikipedia.org/wiki/List_of_Runge%E2%80%93Kutta_methods
 */

template <int X, int U>
using WithInputDerivative = std::function<EVec<X>(const EVec<X>&, const EVec<U>&)>;

template <int X>
using WithoutInputDerivative = std::function<EVec<X>(const EVec<X>&)>;

template <int Y>
using TimeVariantDerivative = std::function<EVec<Y>(const double&, const EVec<Y>&)>;

/**
 * Performs first order numerical integration of the time-invariant differential
 * equation dx/dt = f(x, u) using Euler's method.
 *
 *   0|
 * ---|---
 *    |  1
 *
 * @param f The function to integrate, with two arguments x and u.
 * @param x The initial value of x.
 * @param u The input value u held constant over the integration period.
 * @param h The time over which to integrate.
 */
template <int X, int U>
EVec<X> euler_with_input(const WithInputDerivative<X, U>& f, const EVec<X>& x, const EVec<U>& u, const double& h) {
  EVec<X> k1 = f(x, u);

  return x + h * k1;
}

/**
 * Performs first order numerical integration of the time-invariant differential
 * equation dx/dt = f(x) using Euler's method.
 *
 *   0|
 * ---|---
 *    |  1
 *
 * @param f The function to integrate, with one argument x.
 * @param x The initial value of x.
 * @param h The time over which to integrate.
 */
template <int X>
EVec<X> euler_without_input(const WithoutInputDerivative<X>& f, const EVec<X>& x, const double& h) {
  EVec<X> k1 = f(x);

  return x + h * k1;
}

/**
 * Performs first order numerical integration of the time-variant differential
 * equation dy/dt = f(t, y) using Euler's method.
 *
 *   0|
 * ---|---
 *    |  1
 *
 * @param f The function to integrate, with two arguments t and y.
 * @param t The initial value of t.
 * @param y The initial value of y.
 * @param h The time over which to integrate.
 */
template <int Y>
EVec<Y> euler_time_variant(const TimeVariantDerivative<Y>& f, const double& t, const EVec<Y>& y, const double& h) {
  EVec<Y> k1 = f(t, y);

  return y + h * k1;
}

/**
 * Performs second order numerical integration of the time-invariant differential
 * equation dx/dt = f(x, u) using the explicit midpoint method.
 *
 *   0|
 * 1/2|1/2
 * ---|-------
 *    |  0   1
 *
 * @param f The function to integrate, with one argument x.
 * @param x The initial value of x.
 * @param u The input value u held constant over the integration period.
 * @param h The time over which to integrate.
 */
template <int X, int U>
EVec<X> RK2_with_input(const WithInputDerivative<X, U>& f, const EVec<X>& x, const EVec<U>& u, const double& h) {
  EVec<X> k1 = f(x, u);
  EVec<X> k2 = f(x + h * 0.5 * k1, u);

  return x + h * k2;
}

/**
 * Performs second order numerical integration of the time-invariant differential
 * equation dx/dt = f(x) using the explicit midpoint method.
 *
 *   0|
 * 1/2|1/2
 * ---|-------
 *    |  0   1
 *
 * @param f The function to integrate, with one argument x.
 * @param x The initial value of x.
 * @param h The time over which to integrate.
 */
template <int X>
EVec<X> RK2_without_input(const WithoutInputDerivative<X>& f, const EVec<X>& x, const double& h) {
  EVec<X> k1 = f(x);
  EVec<X> k2 = f(x + h * 0.5 * k1);

  return x + h * k2;
}

/**
 * Performs second order numerical integration of the time-variant differential
 * equation dy/dt = f(t, y) using the explicit midpoint method.
 *
 *   0|
 * 1/2|1/2
 * ---|-------
 *    |  0   1
 *
 * @param f The function to integrate, with two arguments t and y.
 * @param t The initial value of t.
 * @param y The initial value of y.
 * @param h The time over which to integrate.
 */
template <int Y>
EVec<Y> RK2_time_variant(const TimeVariantDerivative<Y>& f, const double& t, const EVec<Y>& y, const double& h) {
  EVec<Y> k1 = f(t, y);
  EVec<Y> k2 = f(t + h * 0.5, y + h * 0.5 * k1);

  return y + h * k2;
}

/**
 * Performs fourth order numerical integration of the time-invariant differential
 * equation dx/dt = f(x, u) using the fourth order Runge-Kutta method.
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
 * @param h The time over which to integrate.
 */
template <int X, int U>
EVec<X> RK4_with_input(const WithInputDerivative<X, U>& f, const EVec<X>& x, const EVec<U>& u, const double& h) {
  EVec<X> k1 = f(x, u);
  EVec<X> k2 = f(x + h * 0.5 * k1, u);
  EVec<X> k3 = f(x + h * 0.5 * k2, u);
  EVec<X> k4 = f(x + h * k3, u);

  return x + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

/**
 * Performs fourth order numerical integration of the time-invariant differential
 * equation dx/dt = f(x) using the fourth order Runge-Kutta method.
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
 * @param h The time over which to integrate.
 */
template <int X>
EVec<X> RK4_without_input(const WithoutInputDerivative<X>& f, const EVec<X>& x, const double& h) {
  EVec<X> k1 = f(x);
  EVec<X> k2 = f(x + h * 0.5 * k1);
  EVec<X> k3 = f(x + h * 0.5 * k2);
  EVec<X> k4 = f(x + h * k3);

  return x + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

/**
 * Performs second order numerical integration of the time-variant differential
 * equation dy/dt = f(t, y) using the fourth order Runge-Kutta method.
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
 * @param h The time over which to integrate.
 */
template <int Y>
EVec<Y> RK4_time_variant(const TimeVariantDerivative<Y>& f, const double& t, const EVec<Y>& y, const double& h) {
  EVec<Y> k1 = f(t, y);
  EVec<Y> k2 = f(t + h * 0.5, y + h * 0.5 * k1);
  EVec<Y> k3 = f(t + h * 0.5, y + h * 0.5 * k2);
  EVec<Y> k4 = f(t + h, y + h * k3);

  return y + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}
