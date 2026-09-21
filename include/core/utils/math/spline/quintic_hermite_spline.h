#pragma once

#include <array>

#include "core/utils/math/spline/cubic_hermite_spline.h"

/**
 * @brief Quintic Hermite spline segment defined by endpoint positions, tangents, and second
 * derivatives.
 *
 * P(u) = c0 + c1*u + c2*u^2 + c3*u^3 + c4*u^4 + c5*u^5, where u in [0, 1].
 */
class QuinticHermiteSpline : public HermiteSpline<5> {
   public:
    /** @brief Default constructor. */
    QuinticHermiteSpline() = default;

    /**
     * @brief Constructs quintic Hermite spline from start and end HermitePoint structures.
     * @param start Initial endpoint with position, tangent, and second derivative.
     * @param end Final endpoint with position, tangent, and second derivative.
     * @param max_err Maximum allowable interpolation error in inches for the arc-length lookup table.
     */
    QuinticHermiteSpline(const HermitePoint &start, const HermitePoint &end, double max_err = 1e-4)
        : QuinticHermiteSpline(
                  start.point,
                  end.point,
                  start.tangent,
                  end.tangent,
                  start.second_derivative,
                  end.second_derivative
          ) {}

    /**
     * @brief Constructs quintic Hermite spline from explicit 2D position, tangent, and acceleration
     * vectors.
     * @param start_pos Start position vector.
     * @param end_pos End position vector.
     * @param start_tangent Start tangent vector.
     * @param end_tangent End tangent vector.
     * @param start_accel Start second derivative vector.
     * @param end_accel End second derivative vector.
     * @param max_err Maximum allowable interpolation error in inches for the arc-length lookup table.
     */
    QuinticHermiteSpline(
            const Translation2d &start_pos,
            const Translation2d &end_pos,
            const Translation2d &start_tangent,
            const Translation2d &end_tangent,
            const Translation2d &start_accel,
            const Translation2d &end_accel,
            double max_err = 1e-4
    ) {
        this->x_ = quintic_coeffs(start_pos.x(), end_pos.x(), start_tangent.x(), end_tangent.x(), start_accel.x(), end_accel.x());
        this->y_ = quintic_coeffs(start_pos.y(), end_pos.y(), start_tangent.y(), end_tangent.y(), start_accel.y(), end_accel.y());
        build_arc_table(max_err);
    }

   private:
    /**
     * @brief Computes 1D quintic Hermite polynomial coefficients [c0, c1, c2, c3, c4, c5].
     */
    static std::array<double, 6> quintic_coeffs(
            double start_pos, double end_pos, double start_tangent, double end_tangent, double start_accel, double end_accel
    ) {
        return {
                start_pos,
                start_tangent,
                start_accel / 2.0,
                (-20.0 * start_pos + 20.0 * end_pos - 12.0 * start_tangent - 8.0 * end_tangent - 3.0 * start_accel + end_accel) / 2.0,
                (30.0 * start_pos - 30.0 * end_pos + 16.0 * start_tangent + 14.0 * end_tangent + 3.0 * start_accel - 2.0 * end_accel) / 2.0,
                (-12.0 * start_pos + 12.0 * end_pos - 6.0 * start_tangent - 6.0 * end_tangent - start_accel + end_accel) / 2.0,
        };
    }
};
