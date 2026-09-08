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
     * @param du Parameter step size used to build the arc-length lookup table.
     */
    CubicHermiteSpline(const HermitePoint &start, const HermitePoint &end, double max_err = 1e-4)
        : CubicHermiteSpline(start.point, end.point, start.tangent, end.tangent) {}

    /**
     * @brief Constructs cubic Hermite spline from explicit 2D position and tangent vectors.
     * @param p0 Start position vector.
     * @param p1 End position vector.
     * @param t0 Start tangent vector.
     * @param t1 End tangent vector.
     * @param du Parameter step size used to build the arc-length lookup table.
     */
    CubicHermiteSpline(
      const Translation2d &p0,
      const Translation2d &p1,
      const Translation2d &t0,
      const Translation2d &t1,
      double max_err = 1e-4) {
        this->x_ = cubic_coeffs(p0.x(), p1.x(), t0.x(), t1.x());
        this->y_ = cubic_coeffs(p0.y(), p1.y(), t0.y(), t1.y());
        build_arc_table(max_err);
    }



  private:
    /**
     * @brief Computes 1D cubic Hermite polynomial coefficients [c0, c1, c2, c3].
     */
    static std::array<double, 4> cubic_coeffs(double p0, double p1, double v0, double v1) {
        return {
          p0,
          v0,
          -3.0 * p0 + 3.0 * p1 - 2.0 * v0 - v1,
          2.0 * p0 - 2.0 * p1 + v0 + v1,
        };
    }

};
