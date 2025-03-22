#pragma once

#include "../core/include/utils/math/eigen_interface.h"

#include "../core/include/utils/math/systems/linear_system.h"
#include <iostream>

template <int STATES, int INPUTS>
class LinearPlantInversionFeedforward {
    public:
    
    template <int OUTPUTS>
    LinearPlantInversionFeedforward(LinearSystem<STATES, INPUTS, OUTPUTS> &plant, const double &dt) : LinearPlantInversionFeedforward(plant.A(), plant.B(), dt) {}

    LinearPlantInversionFeedforward(const EMat<STATES, STATES> &A, const EMat<STATES, INPUTS> &B, const double &dt) : m_dt(dt) {
        auto [Ad, Bd] = discretize_AB(A, B, dt);
        m_Ad = Ad;
        m_Bd = Bd;
    }

    EVec<INPUTS> calculate(const EVec<STATES> &r, const EVec<STATES> &next_r) {
        m_uff = m_Bd.householderQr().solve(next_r - (m_Ad * r));
        m_r = next_r;

        return m_uff;
    }

    EVec<INPUTS> calculate(const EVec<STATES> &next_r) {
        return calculate(m_r, next_r);
    }

    EVec<INPUTS> calculate(const EVec<STATES> &r, const EVec<STATES> &next_r, const double &dt) {
        std::cout << "made it here" << std::endl;
        auto [Ad, Bd] = discretize_AB(A, B, dt);
        
        m_uff = Bd.householderQr().solve(next_r - (Ad * r));
        m_r = next_r;

        return m_uff;
    }

    EVec<INPUTS> calculate(const EVec<STATES> &next_r, const double &dt) {
        return calculate(m_r, next_r, dt);
    }

    void reset(const EVec<STATES> &initial_state) {
        m_r = initial_state;
        m_uff.setZero();
    }

    void reset() {
        m_r.setZero();
        m_uff.setZero();
    }

    private:
    EMat<STATES, STATES> A;
    EMat<STATES, INPUTS> B;

    EMat<STATES, STATES> m_Ad;
    EMat<STATES, INPUTS> m_Bd;

    EVec<INPUTS> m_uff;
    EVec<STATES> m_r;

    double m_dt;
};