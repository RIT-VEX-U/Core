#pragma once

#include <../vendor/eigen/Eigen/Dense>

#include <functional>

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
 * valued function where x is some Eigen::Vector<double, X>, u is some
 * Eigen::Vector<double, U>, and t is a double.
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
using WithInputDerivative =
  std::function<Eigen::Vector<double, X>(const Eigen::Vector<double, X> &, const Eigen::Vector<double, U> &)>;

template <int X>
using WithoutInputDerivative = std::function<Eigen::Vector<double, X>(const Eigen::Vector<double, X> &)>;

template <int Y>
using TimeVariantDerivative = std::function<Eigen::Vector<double, Y>(const double &, const Eigen::Vector<double, Y> &)>;

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
Eigen::Vector<double, X> euler_with_input(
  const WithInputDerivative<X, U> &f, const Eigen::Vector<double, X> &x, const Eigen::Vector<double, U> &u,
  const double &h
) {
    Eigen::Vector<double, X> k1 = f(x, u);

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
Eigen::Vector<double, X>
euler_without_input(const WithoutInputDerivative<X> &f, const Eigen::Vector<double, X> &x, const double &h) {
    Eigen::Vector<double, X> k1 = f(x);

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
Eigen::Vector<double, Y> euler_time_variant(
  const TimeVariantDerivative<Y> &f, const double &t, const Eigen::Vector<double, Y> &y, const double &h
) {
    Eigen::Vector<double, Y> k1 = f(t, y);

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
Eigen::Vector<double, X> RK2_with_input(
  const WithInputDerivative<X, U> &f, const Eigen::Vector<double, X> &x, const Eigen::Vector<double, U> &u,
  const double &h
) {
    Eigen::Vector<double, X> k1 = f(x, u);
    Eigen::Vector<double, X> k2 = f(x + h * 0.5 * k1, u);

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
Eigen::Vector<double, X>
RK2_without_input(const WithoutInputDerivative<X> &f, const Eigen::Vector<double, X> &x, const double &h) {
    Eigen::Vector<double, X> k1 = f(x);
    Eigen::Vector<double, X> k2 = f(x + h * 0.5 * k1);

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
Eigen::Vector<double, Y> RK2_time_variant(
  const TimeVariantDerivative<Y> &f, const double &t, const Eigen::Vector<double, Y> &y, const double &h
) {
    Eigen::Vector<double, Y> k1 = f(t, y);
    Eigen::Vector<double, Y> k2 = f(t + h * 0.5, y + h * 0.5 * k1);

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
Eigen::Vector<double, X> RK4_with_input(
  const WithInputDerivative<X, U> &f, const Eigen::Vector<double, X> &x, const Eigen::Vector<double, U> &u,
  const double &h
) {
    Eigen::Vector<double, X> k1 = f(x, u);
    Eigen::Vector<double, X> k2 = f(x + h * 0.5 * k1, u);
    Eigen::Vector<double, X> k3 = f(x + h * 0.5 * k2, u);
    Eigen::Vector<double, X> k4 = f(x + h * k3, u);

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
Eigen::Vector<double, X>
RK4_without_input(const WithoutInputDerivative<X> &f, const Eigen::Vector<double, X> &x, const double &h) {
    Eigen::Vector<double, X> k1 = f(x);
    Eigen::Vector<double, X> k2 = f(x + h * 0.5 * k1);
    Eigen::Vector<double, X> k3 = f(x + h * 0.5 * k2);
    Eigen::Vector<double, X> k4 = f(x + h * k3);

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
Eigen::Vector<double, Y> RK4_time_variant(
  const TimeVariantDerivative<Y> &f, const double &t, const Eigen::Vector<double, Y> &y, const double &h
) {
    Eigen::Vector<double, Y> k1 = f(t, y);
    Eigen::Vector<double, Y> k2 = f(t + h * 0.5, y + h * 0.5 * k1);
    Eigen::Vector<double, Y> k3 = f(t + h * 0.5, y + h * 0.5 * k2);
    Eigen::Vector<double, Y> k4 = f(t + h, y + h * k3);

    return y + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}
