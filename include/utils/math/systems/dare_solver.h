#pragma once

#include "../core/include/utils/math/eigen_interface.h"

template <typename MatrixType> double cond_inf(const MatrixType &M) {
    return M.template lpNorm<Eigen::Infinity>() * M.inverse().template lpNorm<Eigen::Infinity>();
}

template <int STATES, int INPUTS, int OUTPUTS>
double F_gamma(
  double gamma, const EMat<STATES, STATES> &A, const EMat<STATES, INPUTS> &B, const EMat<OUTPUTS, STATES> &C,
  const EMat<STATES, STATES> &Q, const EMat<STATES, STATES> &R
) {
    EMat<STATES, STATES> I = EMat<STATES, STATES>::Identity();
    EMat<STATES, STATES> Y = gamma * I;

    EMat<INPUTS, INPUTS> R_gamma = R + B.transpose() * Y * B;

    double f1 = cond_inf(R_gamma);

    double f2 = gamma * gamma * f1;

    auto R_gamma_ldlt = R_gamma.ldlt();

    EMat<STATES, STATES> G0 = B * R_gamma_ldlt.solve(B.transpose());

    EMat<STATES, STATES> A0 = (I - G0 * Y) * A - B * R_gamma_ldlt.solve(C);

    EMat<STATES, STATES> H0 = Q - Y 
        - C.transpose() * R_gamma_ldlt.solve(B.transpose() * Y * A)
        - A.transpose() * Y * B * R_gamma_ldlt.solve(C)
        - C.transpose() * R_gamma_ldlt.solve(C)
        + A.transpose() * Y * (I - G0 * Y) * A;

    double f3 = cond_inf(I + G0 * H0);

    return std::max({f1, f2, f3});
}

template <int STATES, int INPUTS, int OUTPUTS>
double gamma_opt(
    const EMat<STATES, STATES>& A,
    const EMat<STATES, INPUTS>& B,
    const EMat<OUTPUTS, STATES>& C,
    const EMat<STATES, STATES>& Q,
    const EMat<STATES, STATES>& R)
{
    // Define search interval for gamma.
    double a = 1e-6;
    double b = 1e2;
    double phi = (1.0 + std::sqrt(5.0)) / 2.0;
    
    double c = b - (b - a) / phi;
    double d = a + (b - a) / phi;
    
    double Fc = F_gamma<STATES, INPUTS, OUTPUTS>(c, A, B, C, Q, R);
    double Fd = F_gamma<STATES, INPUTS, OUTPUTS>(d, A, B, C, Q, R);
    
    // Perform exactly 4 iterations.
    for (int i = 0; i < 5; ++i) {
        if (Fc < Fd) {
            b = d;
            d = c;
            Fd = Fc;
            c = b - (b - a) / phi;
            Fc = F_gamma<STATES, INPUTS, OUTPUTS>(c, A, B, C, Q, R);
        } else {
            a = c;
            c = d;
            Fc = Fd;
            d = a + (b - a) / phi;
            Fd = F_gamma<STATES, INPUTS, OUTPUTS>(d, A, B, C, Q, R);
        }
    }
    
    return (a + b) / 2.0;
}

/**
 * Solves the DARE for the
 */
template <int STATES, int INPUTS, int OUTPUTS>
EMat<STATES, STATES> solve_DARE(
  const EMat<STATES, STATES> &A, const EMat<STATES, INPUTS> &B, const EMat<OUTPUTS, STATES> &C,
  const EMat<STATES, STATES> &Q, const EMat<STATES, STATES> &R
) {
    EMat<STATES, STATES> I = EMat<STATES, STATES>::Identity();

    double gamma = gamma_opt<STATES, INPUTS, OUTPUTS>(A, B, C, Q, R);
    EMat<STATES, STATES> Y = gamma * I;

    EMat<INPUTS, INPUTS> R_tilde = R + (B.transpose() * Y * B);
    auto R_tilde_ldlt = R_tilde.ldlt();

    EMat<STATES, STATES> G_k = B * R_tilde_ldlt.solve(B.transpose());
    EMat<STATES, STATES> A_k = (I - G_k * Y) * A - B * R_tilde_ldlt.solve(C);
    EMat<STATES, STATES> H_k = Q - Y - C.transpose() * R_tilde_ldlt.solve(B.transpose() * Y * A) -
                               A.transpose() * Y * B * R_tilde_ldlt.solve(C) - C.transpose() * R_tilde_ldlt.solve(C) +
                               A.transpose() * Y * (I - G_k * Y) * A;

    EMat<STATES, STATES> G_k1;
    EMat<STATES, STATES> A_k1;
    EMat<STATES, STATES> H_k1;

    double error = 1;

    while (error > 1e-10) {
        EMat<STATES, STATES> inv_I_GH = (I + G_k * H_k).ldlt().solve(I);
        EMat<STATES, STATES> inv_I_HG = (I + H_k * G_k).ldlt().solve(I);

        A_k1 = A_k * inv_I_GH * A_k.transpose();
        G_k1 = G_k + (A_k * G_k * inv_I_HG * A_k.transpose());
        H_k1 = H_k + (A_k.transpose() * inv_I_HG * H_k * A_k);

        error = (H_k1 - H_k).norm() / (std::max(1.0, H_k.norm()));

        A_k = A_k1;
        G_k = G_k1;
        H_k = H_k1;
    }

    EMat<STATES, STATES> X = H_k + Y;
    return X;
}
