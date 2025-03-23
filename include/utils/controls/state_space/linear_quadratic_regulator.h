#pragma once

#include "../core/include/utils/math/eigen_interface.h"

#include "../core/include/utils/math/systems/dare_solver.h"
#include "../core/include/utils/math/systems/discretization.h"
#include "../core/include/utils/math/systems/linear_system.h"

/**
 * Forms a cost matrix from a set of 
 */
template <int DIM>
EMat<DIM, DIM> cost_matrix(const EVec<DIM> &tolerances) {
    EMat<DIM, DIM> Q = EMat<DIM, DIM>::Zero();
    for (int i = 0; i < DIM; i++) {
        Q(i, i) = 1.0 / (tolerances(i) * tolerances(i));
    }
    return Q;
}

template <int STATES, int INPUTS>
class LinearQuadraticRegulator {
    public:

    using MatrixA = EMat<STATES, STATES>;
    using MatrixB = EMat<STATES, INPUTS>;
    using StateVector = EVec<STATES>;
    using InputVector = EVec<INPUTS>;

    template <int OUTPUTS>
    LinearQuadraticRegulator(LinearSystem<STATES, INPUTS, OUTPUTS> &plant, const StateVector &Qtolerances, const InputVector &Rtolerances, const double &dt) : LinearQuadraticRegulator(plant.A(), plant.B(), Qtolerances, Rtolerances, dt) {}

    LinearQuadraticRegulator(const MatrixA &A, const MatrixB &B, const StateVector &Qtolerances, const InputVector &Rtolerances, const double &dt) : LinearQuadraticRegulator(A, B, cost_matrix(Qtolerances), cost_matrix(Rtolerances), dt) {}

    LinearQuadraticRegulator(const MatrixA &A, const MatrixB &B, const EMat<STATES, STATES> &Q, const EMat<INPUTS, INPUTS> &R, const double &dt) {
        auto [Ad, Bd] = discretize_AB(A, B, dt);

        MatrixA S = DARE<STATES, INPUTS>(Ad, Bd, Q, R);

        m_K = (Bd.transpose() * S * Bd + R).llt().solve(Bd.transpose() * S * Ad); 
    }

    InputVector calculate(const StateVector &x, const StateVector &r) {
        return m_K * (r - x);
    }

    template <int OUTPUTS>
    void latency_compensate(LinearSystem<STATES, INPUTS, OUTPUTS> &plant, const double &dt, const double &input_delay) {
        auto [Ad, Bd] = discretize_AB(plant.A(), plant.B(), dt);

        m_K = m_K * (Ad - Bd * m_K).pow(input_delay / dt);
    }

    private:    
    EMat<INPUTS, STATES> m_K;
    
};