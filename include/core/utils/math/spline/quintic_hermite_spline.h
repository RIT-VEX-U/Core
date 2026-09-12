#pragma once

#include <array>

#include "core/utils/math/spline/cubic_hermite_spline.h"

/**
 * @brief Quintic Hermite spline segment defined by endpoint positions, tangents, and second derivatives.
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
     * @param du Parameter step size used to build the arc-length lookup table.
     */
    QuinticHermiteSpline(const HermitePoint &start, const HermitePoint &end, double max_err = 1e-4)
        : QuinticHermiteSpline(
            start.point,
            end.point,
            start.tangent,
            end.tangent,
            start.second_derivative,
            end.second_derivative) {}

    /**
     * @brief Constructs quintic Hermite spline from explicit 2D position, tangent, and acceleration vectors.
     * @param p0 Start position vector.
     * @param p1 End position vector.
     * @param t0 Start tangent vector.
     * @param t1 End tangent vector.
     * @param a0 Start second derivative vector.
     * @param a1 End second derivative vector.
     * @param du Parameter step size used to build the arc-length lookup table.
     */
    QuinticHermiteSpline(
      const Translation2d &p0,
      const Translation2d &p1,
      const Translation2d &t0,
      const Translation2d &t1,
      const Translation2d &a0,
      const Translation2d &a1,
      double max_err = 1e-4) {
        this->x_ = quintic_coeffs(p0.x(), p1.x(), t0.x(), t1.x(), a0.x(), a1.x());
        this->y_ = quintic_coeffs(p0.y(), p1.y(), t0.y(), t1.y(), a0.y(), a1.y());
        build_arc_table(max_err);
    }



  private:
    /**
     * @brief Computes 1D quintic Hermite polynomial coefficients [c0, c1, c2, c3, c4, c5].
     */
    static std::array<double, 6> quintic_coeffs(double p0, double p1, double v0, double v1, double a0, double a1) {
        return {
          p0,
          v0,
          a0 / 2.0,
          (-20.0 * p0 + 20.0 * p1 - 12.0 * v0 - 8.0 * v1 - 3.0 * a0 + a1) / 2.0,
          (30.0 * p0 - 30.0 * p1 + 16.0 * v0 + 14.0 * v1 + 3.0 * a0 - 2.0 * a1) / 2.0,
          (-12.0 * p0 + 12.0 * p1 - 6.0 * v0 - 6.0 * v1 - a0 + a1) / 2.0,
        };
    }

};
