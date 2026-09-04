#pragma once

#include <cmath>
#include <vector>

#include "core/units/units.h"
#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/math/geometry/translation2d.h"

/**
 * @brief Evaluated state sample at a specific parameter u or distance s along a spline.
 */
struct SplineSample {
    double u = 0.0;              ///< Spline parameter in range [0, 1]
    double s = 0.0;              ///< Arc length along spline from start in inches
    Translation2d position;     ///< 2D position (x, y)
    Translation2d velocity;     ///< 1D/2D parametric velocity vector (dx/du, dy/du)
    Translation2d acceleration; ///< 1D/2D parametric acceleration vector (d²x/du², d²y/du²)
    Rotation2d heading;         ///< Heading angle theta along path
    Curvature curvature = 0_radpm; ///< Path curvature (rad/meter)
};

/**
 * @brief Abstract base class for parametric 2D Hermite splines.
 *
 * Provides arc-length parameterization, heading, curvature, sampling,
 * and high-accuracy Gauss-Legendre quadrature integration.
 */
class SplineBase {
  public:
    virtual ~SplineBase() = default;

    /**
     * @brief Computes 2D position at spline parameter u.
     * @param u Parameter in range [0, 1].
     * @return 2D position vector.
     */
    virtual Translation2d position(double u) const = 0;

    /**
     * @brief Computes 2D parametric velocity (first derivative) at parameter u.
     * @param u Parameter in range [0, 1].
     * @return First derivative vector (dx/du, dy/du).
     */
    virtual Translation2d velocity(double u) const = 0;

    /**
     * @brief Computes 2D parametric acceleration (second derivative) at parameter u.
     * @param u Parameter in range [0, 1].
     * @return Second derivative vector (d²x/du², d²y/du²).
     */
    virtual Translation2d acceleration(double u) const = 0;

    /**
     * @brief Computes heading angle theta at spline parameter u.
     * @param u Parameter in range [0, 1].
     * @return Rotation2d heading angle.
     */
    Rotation2d heading(double u) const {
        const Translation2d vel = velocity(clamp_u(u));
        return vel.norm() > 1e-9 ? vel.theta() : Rotation2d();
    }

    /**
     * @brief Computes path curvature at parameter u.
     * @param u Parameter in range [0, 1].
     * @return Curvature in rad/meter.
     */
    Curvature curvature(double u) const {
        const Translation2d vel = velocity(clamp_u(u));
        const Translation2d acc = acceleration(clamp_u(u));
        const double denom = std::pow((vel.x() * vel.x()) + (vel.y() * vel.y()), 1.5);
        if (denom < 1e-9) {
            return 0_radpm;
        }

        // Convert 2D parametric curvature (in rad/inch) to canonical curvature (in rad/meter)
        constexpr double kInchesPerMeter = Length::from<meter_tag>(1.0).in();
        const double curvature_rad_per_in = ((vel.x() * acc.y()) - (vel.y() * acc.x())) / denom;
        return Curvature::from<radians_per_meter_tag>(curvature_rad_per_in * kInchesPerMeter);
    }

    /**
     * @brief Returns total arc length of the spline in inches.
     * @return Total arc length.
     */
    double length() const { return arc_lengths_.empty() ? 0.0 : arc_lengths_.back(); }

    /**
     * @brief Inverts arc distance s to spline parameter u.
     * @param s Arc distance along spline in inches.
     * @return Spline parameter u in range [0, 1].
     */
    double u_from_s(double s) const {
        if (arc_lengths_.empty() || us_.empty()) {
            return 0.0;
        }
        if (s <= 0.0) {
            return 0.0;
        }
        if (s >= length()) {
            return 1.0;
        }

        auto upper = std::lower_bound(arc_lengths_.begin(), arc_lengths_.end(), s);
        const size_t idx = static_cast<size_t>(upper - arc_lengths_.begin());
        if (idx == 0) {
            return us_.front();
        }

        const double s0 = arc_lengths_[idx - 1];
        const double s1 = arc_lengths_[idx];
        const double u0 = us_[idx - 1];
        const double u1 = us_[idx];
        const double alpha = (s - s0) / std::max(s1 - s0, 1e-9);
        return u0 + ((u1 - u0) * alpha);
    }

    /**
     * @brief Samples complete state (position, velocity, acceleration, heading, curvature) at parameter u.
     * @param u Parameter in range [0, 1].
     * @return SplineSample struct.
     */
    SplineSample sample(double u) const {
        const double clamped_u = clamp_u(u);
        SplineSample out;
        out.u = clamped_u;
        out.s = s_from_u(clamped_u);
        out.position = position(clamped_u);
        out.velocity = velocity(clamped_u);
        out.acceleration = acceleration(clamped_u);
        out.heading = heading(clamped_u);
        out.curvature = curvature(clamped_u);
        return out;
    }

    /**
     * @brief Samples complete state at arc distance s.
     * @param s Arc distance in inches.
     * @return SplineSample struct.
     */
    SplineSample sample_by_s(double s) const {
        SplineSample out = sample(u_from_s(s));
        out.s = clamp_value(s, 0.0, length());
        return out;
    }

    /**
     * @brief Samples 2D positions uniformly spaced in parameter u.
     * @param count Number of sample points.
     * @return Vector of 2D translation points.
     */
    std::vector<Translation2d> sample_by_count(int count) const {
        std::vector<Translation2d> points;
        if (count <= 0) {
            return points;
        }
        if (count == 1) {
            points.push_back(position(0.0));
            return points;
        }

        points.reserve(static_cast<size_t>(count));
        for (int i = 0; i < count; ++i) {
            const double u = static_cast<double>(i) / static_cast<double>(count - 1);
            points.push_back(position(u));
        }
        return points;
    }

    /**
     * @brief Samples 2D positions uniformly spaced by arc distance ds.
     * @param ds Arc distance step in inches.
     * @return Vector of 2D translation points.
     */
    std::vector<Translation2d> sample_by_spacing(double ds) const {
        std::vector<Translation2d> points;
        if (ds <= 0.0 || length() <= 0.0) {
            return points;
        }

        for (double s = 0.0; s < length(); s += ds) {
            points.push_back(sample_by_s(s).position);
        }
        points.push_back(position(1.0));
        return points;
    }

  protected:
    /**
     * @brief Evaluates polynomial P(u) using Horner's method.
     */
    template <size_t N> static double eval_poly(const std::array<double, N> &coeffs, double u) {
        double out = 0.0;
        for (size_t i = N; i-- > 0;) {
            out = (out * u) + coeffs[i];
        }
        return out;
    }

    /**
     * @brief Evaluates first derivative P'(u) using Horner's method.
     */
    template <size_t N> static double eval_poly_derivative(const std::array<double, N> &coeffs, double u) {
        double out = 0.0;
        for (size_t i = N - 1; i-- > 0;) {
            out = (out * u) + (coeffs[i + 1] * static_cast<double>(i + 1));
        }
        return out;
    }

    /**
     * @brief Evaluates second derivative P''(u) using Horner's method.
     */
    template <size_t N> static double eval_poly_second_derivative(const std::array<double, N> &coeffs, double u) {
        double out = 0.0;
        for (size_t i = N - 2; i-- > 0;) {
            out = (out * u) + (coeffs[i + 2] * static_cast<double>((i + 1) * (i + 2)));
        }
        return out;
    }

    /**
     * @brief Computes high-accuracy arc length over sub-interval [u0, u1] using 5-point Gauss-Legendre quadrature.
     */
    double integrate_segment_length(double u0, double u1) const {
        constexpr double x[5] = {
            0.0,
            -0.5384693101056831,
             0.5384693101056831,
            -0.9061798459386640,
             0.9061798459386640
        };
        constexpr double w[5] = {
            0.5688888888888889,
            0.4786286704993665,
            0.4786286704993665,
            0.2369268850561891,
            0.2369268850561891
        };

        const double half_len = (u1 - u0) * 0.5;
        const double mid = (u0 + u1) * 0.5;
        double sum = 0.0;
        for (int i = 0; i < 5; ++i) {
            const double u = mid + (half_len * x[i]);
            sum += w[i] * velocity(u).norm();
        }
        return half_len * sum;
    }

    /**
     * @brief Builds pre-computed parameter u and arc-length lookup tables using 5-point Gauss-Legendre quadrature.
     * @param du Parameter step size.
     */
    void build_arc_table(double du = 0.01) {
        us_.clear();
        arc_lengths_.clear();

        const double step = std::max(du, 1e-4);
        double accum = 0.0;

        us_.push_back(0.0);
        arc_lengths_.push_back(0.0);

        double prev_u = 0.0;
        for (double u = step; u < 1.0; u += step) {
            accum += integrate_segment_length(prev_u, u);
            us_.push_back(u);
            arc_lengths_.push_back(accum);
            prev_u = u;
        }

        accum += integrate_segment_length(prev_u, 1.0);
        us_.push_back(1.0);
        arc_lengths_.push_back(accum);
    }

    static double clamp_u(double u) { return clamp_value(u, 0.0, 1.0); }

    static double clamp_value(double value, double lo, double hi) {
        if (value < lo) {
            return lo;
        }
        if (value > hi) {
            return hi;
        }
        return value;
    }

  private:
    double s_from_u(double u) const {
        if (us_.empty() || arc_lengths_.empty()) {
            return 0.0;
        }
        if (u <= 0.0) {
            return 0.0;
        }
        if (u >= 1.0) {
            return length();
        }

        auto upper = std::lower_bound(us_.begin(), us_.end(), u);
        const size_t idx = static_cast<size_t>(upper - us_.begin());
        if (idx == 0) {
            return arc_lengths_.front();
        }

        const double u0 = us_[idx - 1];
        const double u1 = us_[idx];
        const double s0 = arc_lengths_[idx - 1];
        const double s1 = arc_lengths_[idx];
        const double alpha = (u - u0) / std::max(u1 - u0, 1e-9);
        return s0 + ((s1 - s0) * alpha);
    }

    std::vector<double> us_;
    std::vector<double> arc_lengths_;
};
