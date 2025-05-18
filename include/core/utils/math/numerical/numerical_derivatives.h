#pragma once

#include "core/utils/math/eigen_interface.h"

#include <functional>

/**
 * Computes the derivative of a function f(x) at a point x.
 * 
 * @param f The function, returns a double and takes a double.
 * @param x The point to compute the derivative at.
 * 
 * @return The derivative of f(x) at x.
 */
double numerical_diff(const std::function<double(double)> &f, double x) {
    const double h = 1e-6;
    const double x_plus = f(x + h);
    const double x_minus = f(x - h);
    return (x_plus - x_minus) / (2 * h);
}

/**
 * Computes the gradient of a function f(x) at a point x.
 * 
 * @tparam X The dimension of the input vector x.
 * 
 * @param f The function, returns a double and takes a vector of size X.
 * @param x The point to compute the gradient at.
 * 
 * @return The gradient of f(x) at x as a vector of size X.
 */
template <int X>
EVec<X> numerical_gradient(const std::function<double(const EVec<X> &x)> &f, const EVec<X> &x) {
    EVec<X> grad = EVec<X>::Zero();
    const double h = 1e-6;
    for (int i = 0; i < X; i++) {
        EVec<X> x_plus = x;
        EVec<X> x_minus = x;
        x_plus(i) += h;
        x_minus(i) -= h;
        grad(i) = (f(x_plus) - f(x_minus)) / (2 * h);
    }
    return grad;
}

/**
 * Computes the directional derivative of a function f(x) at a point x in the direction of v.
 * 
 * @tparam X The dimension of the input vector x.
 * 
 * @param f The function, returns a double and takes a vector of size X.
 * @param x The point to compute the directional derivative at.
 * @param v The direction vector.
 * 
 * @return The directional derivative of f(x) at x in the direction of v.
 */
template <int X>
double numerical_directional_derivative(const std::function<double(const EVec<X> &x)> &f, const EVec<X> &x, const EVec<X> &v) {
    return v.dot(numerical_gradient(f, x));
}

/**
 * Computes the Jacobian of a function f(x) with respect to x.
 * 
 * @tparam X The dimension of the input vector x.
 * 
 * @param f The function, takes and returns vectors of size X.
 * @param x The point to compute the Jacobian at.
 * 
 * @return The Jacobian of f(x) wrt x.
 */
template <int X>
EMat<X, X> numerical_jacobian_wrt_x(const std::function<EVec<X>(const EVec<X> &)> &f, const EVec<X> &x) {
    EMat<X, X> jac = EMat<X, X>::Zero();
    const double h = 1e-6;
    for (int i = 0; i < X; i++) {
        EVec<X> x_plus = x;
        EVec<X> x_minus = x;
        x_plus(i) += h;
        x_minus(i) -= h;
        jac.col(i) = (f(x_plus) - f(x_minus)) / (2 * h);
    }
    return jac;
}

/**
 * Computes the Jacobian of a function f(x, u) with respect to x.
 * 
 * @tparam X The dimension of the input vector x.
 * @tparam U The dimension of the input vector u.
 * 
 * @param f The function, returns a vector of size X and takes vectors of size X and U.
 * @param x The point to compute the Jacobian at.
 * @param u The input to compute the Jacobian at.
 * 
 * @return The Jacobian of f(x, u) wrt x.
 */
template <int X, int U>
EMat<X, X> numerical_jacobian_wrt_x(const std::function<EVec<X>(const EVec<X> &x, const EVec<U> &u)> &f, const EVec<X> &x, const EVec<U> &u) {
    EMat<X, X> jac;
    const double h = 1e-6;
    for (int i = 0; i < X; i++) {
        EVec<X> x_plus = x;
        EVec<X> x_minus = x;
        x_plus(i) += h;
        x_minus(i) -= h;
        jac.col(i) = (f(x_plus, u) - f(x_minus, u)) / (2 * h);
    }
    return jac;
}

/**
 * Computes the Jacobian of a function f(x, u) with respect to u.
 * 
 * @tparam X The dimension of the input vector x.
 * @tparam U The dimension of the input vector u.
 * 
 * @param f The function, returns a vector of size X and takes vectors of size X and U.
 * @param x The point to compute the Jacobian at.
 * @param u The input to compute the Jacobian at.
 * 
 * @return The Jacobian of f(x, u) wrt u.
 */
template <int X, int U>
EMat<X, U> numerical_jacobian_wrt_u(const std::function<EVec<X>(const EVec<X> &x, const EVec<U> &u)> &f, const EVec<X> &x, const EVec<U> &u) {
    EMat<X, U> jac = EMat<X, U>::Zero();
    const double h = 1e-6;
    for (int i = 0; i < U; i++) {
        EVec<U> u_plus = u;
        EVec<U> u_minus = u;
        u_plus(i) += h;
        u_minus(i) -= h;
        jac.col(i) = (f(x, u_plus) - f(x, u_minus)) / (2 * h);
    }
    return jac;
}

/**
 * Computes the Hessian matrix of a function f(x) at a point x.
 * 
 * @tparam X The dimension of the input vector x.
 * 
 * @param f The function, returns a double and takes a vector of size X.
 * @param x The point to compute the Hessian at.
 * 
 * @return The Hessian of f(x) at x as an XxX matrix.
 */
template <int X>
EMat<X, X> numerical_hessian(const std::function<double(const EVec<X> &x)> &f, const EVec<X> &x) {
    EMat<X, X> hess = EMat<X, X>::Zero();
    const double h = 1e-6;
    
    // For each component, compute the derivative of the gradient.
    for (int i = 0; i < X; i++) {
        EVec<X> x_plus = x;
        EVec<X> x_minus = x;
        x_plus(i) += h;
        x_minus(i) -= h;
        
        EVec<X> grad_plus = numerical_gradient<X>(f, x_plus);
        EVec<X> grad_minus = numerical_gradient<X>(f, x_minus);
        
        // The i-th column is the derivative of the gradient with respect to the i-th component.
        hess.col(i) = (grad_plus - grad_minus) / (2 * h);
    }
    
    // Force symmatry.
    hess = (hess + hess.transpose()) / 2.0;
    
    return hess;
}
