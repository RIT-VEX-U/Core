#pragma once

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"

#include <Eigen/Dense>

#pragma clang diagnostic pop

/**
 * This header only serves to let you use Eigen's vectors and matrices without
 * having to write out
 * Eigen::Vector<double, DIM>
 * Eigen::Matrix<double, ROWS, COLS>
 *
 * include this and use:
 * EVec<DIM>
 * EMAT<ROWS, COLS>
 */

template <int DIM>
using EVec = Eigen::Vector<double, DIM>;

template <int ROWS, int COLS>
using EMat = Eigen::Matrix<double, ROWS, COLS>;
