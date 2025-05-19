#pragma once

#include "core/utils/math/autodiff/dual.h"
#include "core/utils/math/eigen_interface.h"

#include <functional>

namespace Autodiff {

/**
 * Computes the derivative of a function f(x) at a point x using forward automatic differentiation.
 * 
 * @param f The function, returns a Dual and takes a Dual.
 * @param x The point to compute the derivative at.
 * 
 * @return The derivative of f(x) at x.
 */
double diff(const std::function<Dual(Dual)>& f, double x) {
    Dual x_dual(x, 1.0);
    
    Dual result = f(x_dual);
    
    return result.derivative();
}

/**
 * Computes the gradient of a function f(x) at a point x using forward automatic differentiation.
 * 
 * @tparam X The dimension of the input vector x.
 * 
 * @param f The function, returns a Dual and takes a vector of Duals.
 * @param x The point to compute the gradient at.
 * 
 * @return The gradient of f(x) at x as a vector of size X.
 */
template <int X>
EVec<X> gradient(const std::function<Dual(const EVecX<Dual, X>&)>& f, const EVec<X>& x) {
    EVec<X> grad = EVec<X>::Zero();
    
    for (int i = 0; i < X; i++) {
        EVecX<Dual, X> x_dual;
        
        for (int j = 0; j < X; j++) {
            x_dual(j) = Dual(x(j), 0.0);
        }
        
        x_dual(i).set_derivative(1.0);
        
        Dual result = f(x_dual);
        grad(i) = result.derivative();
    }
    
    return grad;
}

/**
 * Computes the directional derivative of a function f(x) at a point x in the direction of v
 * using forward automatic differentiation.
 * 
 * @tparam X The dimension of the input vector x.
 * 
 * @param f The function, returns a Dual and takes a vector of Duals.
 * @param x The point to compute the directional derivative at.
 * @param v The direction vector.
 * 
 * @return The directional derivative of f(x) at x in the direction of v.
 */
template <int X>
double directional_derivative(const std::function<Dual(const EVecX<Dual, X>&)>& f, 
                                  const EVec<X>& x, 
                                  const EVec<X>& v) {
    EVec<X> v_normalized = v.normalized();
    
    // Create a dual vector with derivatives set to the normalized direction vector.
    // This is genius...
    EVecX<Dual, X> x_dual;
    for (int i = 0; i < X; i++) {
        x_dual(i) = Dual(x(i), v_normalized(i));
    }
    
    Dual result = f(x_dual);
    return result.derivative();
}

/**
 * Computes the Jacobian of a function f(x) with respect to x using forward automatic differentiation.
 * 
 * @tparam X The dimension of the input vector x.
 * @tparam Y The dimension of the output vector y.
 * 
 * @param f The function, returns a vector of Duals and takes a vector of Duals.
 * @param x The point to compute the Jacobian at.
 * 
 * @return The Jacobian of f(x) wrt x.
 */
template <int X, int Y>
EMat<Y, X> jacobian_wrt_x(
    const std::function<EVecX<Dual, Y>(const EVecX<Dual, X>&)>& f,
    const EVec<X> &x) {
    
    EMat<Y, X> jac = EMat<Y, X>::Zero();
    
    for (int i = 0; i < X; i++) {
        EVecX<Dual, X> x_dual;
        
        for (int j = 0; j < X; j++) {
            x_dual(j) = Dual(x(j), 0.0);
        }
        
        x_dual(i).set_derivative(1.0);
        
        EVecX<Dual, Y> result = f(x_dual);
        
        for (int j = 0; j < Y; j++) {
            jac(j, i) = result(j).derivative();
        }
    }
    
    return jac;
}

/**
 * Computes the Jacobian of a function f(x, u) with respect to x using forward automatic differentiation.
 * This overload is for two-parameter functions.
 * 
 * @tparam X The dimension of the input vector x.
 * @tparam U The dimension of the input vector u.
 * @tparam Y The dimension of the output vector y.
 * 
 * @param f The function, returns a vector of Duals and takes vectors of Duals.
 * @param x The point to compute the Jacobian at.
 * @param u The input to compute the Jacobian at.
 * 
 * @return The Jacobian of f(x, u) wrt x.
 */
template <int X, int U, int Y>
EMat<Y, X> jacobian_wrt_x(
    const std::function<EVecX<Dual, Y>(const EVecX<Dual, X>&, const EVecX<Dual, U>&)>& f,
    const EVec<X> &x,
    const EVec<U> &u) {
    
    EMat<Y, X> jac = EMat<Y, X>::Zero();
    
    for (int i = 0; i < X; i++) {
        EVecX<Dual, X> x_dual;
        EVecX<Dual, U> u_dual;
        
        for (int j = 0; j < X; j++) {
            x_dual(j) = Dual(x(j), 0.0);
        }
        
        for (int j = 0; j < U; j++) {
            u_dual(j) = Dual(u(j), 0.0);
        }
        
        x_dual(i).set_derivative(1.0);
        
        EVecX<Dual, Y> result = f(x_dual, u_dual);
        
        for (int j = 0; j < Y; j++) {
            jac(j, i) = result(j).derivative();
        }
    }
    
    return jac;
}

/**
 * Computes the Jacobian of a function f(x, u) with respect to u using forward automatic differentiation.
 * 
 * @tparam X The dimension of the input vector x.
 * @tparam U The dimension of the input vector u.
 * @tparam Y The dimension of the output vector y.
 * 
 * @param f The function, returns a vector of size Y and takes vectors of Dual values.
 * @param x The point to compute the Jacobian at.
 * @param u The input to compute the Jacobian at.
 * 
 * @return The Jacobian of f(x, u) wrt u.
 */
template <int X, int U, int Y>
EMat<Y, U> jacobian_wrt_u(
    const std::function<EVecX<Dual, Y>(
        const EVecX<Dual, X> &x, 
        const EVecX<Dual, U> &u)> &f,
    const EVec<X> &x,
    const EVec<U> &u) {
    
    EMat<Y, U> jac = EMat<Y, U>::Zero();
    
    for (int i = 0; i < U; i++) {
        EVecX<Dual, X> x_dual;
        EVecX<Dual, U> u_dual;
        
        for (int j = 0; j < X; j++) {
            x_dual(j) = Dual(x(j), 0.0);
        }
        
        for (int j = 0; j < U; j++) {
            u_dual(j) = Dual(u(j), 0.0);
        }
        
        u_dual(i).set_derivative(1.0);
        
        EVecX<Dual, Y> result = f(x_dual, u_dual);
        
        for (int j = 0; j < Y; j++) {
            jac(j, i) = result(j).derivative();
        }
    }
    
    return jac;
}

/**
 * Computes the Hessian matrix of a function f(x) at a point x using forward automatic differentiation.
 * 
 * @tparam X The dimension of the input vector x.
 * 
 * @param f The function, returns a Dual and takes a vector of Duals.
 * @param x The point to compute the Hessian at.
 * 
 * @return The Hessian of f(x) at x.
 */
template <int X>
EMat<X, X> hessian(const std::function<Dual(const EVecX<Dual, X>&)>& f, const EVec<X>& x) {
    EMat<X, X> hess = EMat<X, X>::Zero();
    const double h = 1e-6;
    
    // For each component, compute the derivative of the gradient using finite differences of gradients computed with autodiff.
    // This obviously isn't the most accurate, but it's easy and works better than doing it straight up numerically.
    for (int i = 0; i < X; i++) {
        EVec<X> x_plus = x;
        EVec<X> x_minus = x;
        x_plus(i) += h;
        x_minus(i) -= h;
        
        EVec<X> grad_plus = gradient<X>(f, x_plus);
        EVec<X> grad_minus = gradient<X>(f, x_minus);
        
        // The i-th column is the derivative of the gradient with respect to the i-th component.
        hess.col(i) = (grad_plus - grad_minus) / (2 * h);
    }
    
    // Force symmetry.
    hess = (hess + hess.transpose()) / 2.0;
    
    return hess;
}

} // namespace Autodiff
