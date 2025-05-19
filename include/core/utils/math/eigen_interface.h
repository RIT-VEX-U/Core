#pragma once

#include <Eigen/Dense>

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

template <typename T, int ROWS>
using EVecX = Eigen::Vector<T, ROWS>;

template <typename T, int ROWS, int COLS>
using EMatX = Eigen::Matrix<T, ROWS, COLS>;

