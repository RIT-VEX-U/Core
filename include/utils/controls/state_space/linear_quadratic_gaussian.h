#pragma once

#include "../core/include/utils/controls/state_space/linear_plant_inversion.h"
#include "../core/include/utils/controls/state_space/linear_quadratic_regulator.h"
#include "../core/include/utils/math/estimator/kalman_filter.h"

template <int STATES, int INPUTS, int OUTPUTS>
class LinearQuadraticGaussian {
    public:
    
    

    private:
    LinearQuadraticRegulator lqr_;
    LinearPlantInversionFeedforward piff_;
    KalmanFilter kf_

    EVec<STATES> reference_;

};