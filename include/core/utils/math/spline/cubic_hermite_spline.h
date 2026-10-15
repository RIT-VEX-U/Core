#pragma once

#include <array>

#include "core/utils/math/spline/hermite_point.h"
#include "core/utils/math/spline/spline_base.h"

/**
 * @brief Cubic Hermite spline segment defined by endpoint positions and tangents.
 *
 * P(u) = c0 + c1*u + c2*u^2 + c3*u^3, where u in [0, 1].
 */
class CubicHermiteSpline : public HermiteSpline<3> {
   public:
    /** @brief Default constructor. */
    CubicHermiteSpline() = default;

    /**
     * @brief Constructs cubic Hermite spline from start and end HermitePoint structures.
     * @param start Initial endpoint with position and tangent.
     * @param end Final endpoint with position and tangent.
     * @param max_err Maximum allowable interpolation error in inches for the arc-length lookup table.
     */
    CubicHermiteSpline(const HermitePoint &start, const HermitePoint &end, double max_err = 1e-4)
        : CubicHermiteSpline(start.point, end.point, start.tangent, end.tangent, max_err) {}

    /**
     * @brief Constructs cubic Hermite spline from explicit 2D position and tangent vectors.
     * @param start_pos Start position vector.
     * @param end_pos End position vector.
     * @param start_tangent Start tangent vector.
     * @param end_tangent End tangent vector.
     * @param max_err Maximum allowable interpolation error in inches for the arc-length lookup table.
     */
    CubicHermiteSpline(
            const Translation2d &start_pos,
            const Translation2d &end_pos,
            const Translation2d &start_tangent,
            const Translation2d &end_tangent,
            double max_err = 1e-4
    ) {
        this->x_ = cubic_coeffs(start_pos.x(units::in), end_pos.x(units::in), start_tangent.x(units::in), end_tangent.x(units::in));
        this->y_ = cubic_coeffs(start_pos.y(units::in), end_pos.y(units::in), start_tangent.y(units::in), end_tangent.y(units::in));
        build_arc_table(max_err);
    }

   private:
    /**
     * @brief Computes 1D cubic Hermite polynomial coefficients [c0, c1, c2, c3].
     */
    static std::array<double, 4> cubic_coeffs(double start_pos, double end_pos, double start_tangent, double end_tangent) {
        return {
                start_pos,
                start_tangent,
                -3.0 * start_pos + 3.0 * end_pos - 2.0 * start_tangent - end_tangent,
                2.0 * start_pos - 2.0 * end_pos + start_tangent + end_tangent,
        };
    }
};
