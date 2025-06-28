#pragma once

#include <algorithm>
#include <cmath>

#include "math/geometry/translation2d.h"

// we might not have std::clamp
#ifndef __cpp_lib_clamp
namespace core {
template <typename T>
T clamp(T value, T low, T high) {
  return std::max(low, std::min(high, value));
}
}  // namespace core
#else
// if we do then use it instead
namespace core {
using std::clamp;
}
#endif

/**
 * @brief Linearly intERPolate between values
 * @param a at t = 0, output = a
 * @param b at t = 1, output = b
 * @return a linear mixing of a and b according to t
 */
double lerp(double a, double b, double t) {
  return a * (1.0 - t) + b * t;
}

/**
 * Returns the sign of a number
 * @param x
 *
 * returns the sign +/-1 of x. 0 if x is 0
 **/
double sign(double x) {
  if (x < 0) {
    return -1;
  }
  return 1;
}

double wrap_angle_deg(double input) {
  double angle = fmod(input, 360);
  if (angle < 0) {
    angle += 360;
  }

  return angle;
}
double wrap_angle_rad(double input) {
  double angle = fmod(input, M_TWOPI);
  if (angle < 0) {
    angle += M_TWOPI;
  }

  return angle;
}

/*
Calculates the variance of  a set of numbers (needed for linear regression)
https://en.wikipedia.org/wiki/Variance
@param values   the values for which the variance is taken
@param mean     the average of values
*/
double variance(std::vector<double> const& values, double mean) {
  double total = 0.0;
  for (int i = 0; i < values.size(); i++) {
    total += (values[i] - mean) * (values[i] - mean);
  }
  return total / (values.size() - 1);
}

/*
Calculates the average of a vector of doubles
@param values   the list of values for which the average is taken
*/
double mean(std::vector<double> const& values) {
  double total = 0;
  for (int i = 0; i < values.size(); i++) {
    total += values[i];
  }
  return total / (double)values.size();
}

/*
Calculates the covariance of a set of points (needed for linear regression)
https://en.wikipedia.org/wiki/Covariance

@param points   the points for which the covariance is taken
@param meanx    the mean value of all x coordinates in points
@param meany    the mean value of all y coordinates in points
*/
double covariance(std::vector<std::pair<double, double>> const& points,
                  double meanx, double meany) {
  double covar = 0.0;
  for (int i = 0; i < points.size(); i++) {
    covar += (points[i].first - meanx) * (points[i].second - meany);
  }
  return covar;
}

/*
Calculates the slope and y intercept of the line of best fit for the data
@param points the points for the data
*/
std::pair<double, double> calculate_linear_regression(
    std::vector<std::pair<double, double>> const& points) {
  // Purely for convenience and the ability to reuse mean() and variance() - can be easily rewritten to avoid allocating
  // these if the code is repeatedly called
  std::vector<double> xs(points.size(), 0.0);
  std::vector<double> ys(points.size(), 0.0);
  for (int i = 0; i < points.size(); i++) {
    xs[i] = points[i].first;
    ys[i] = points[i].second;
  }

  double meanx = mean(xs);
  double meany = mean(ys);

  double slope = covariance(points, meanx, meany) / variance(xs, meanx);
  double y_intercept = meany - slope * meanx;

  return std::pair<double, double>(slope, y_intercept);
}

double estimate_path_length(const std::vector<Translation2d>& points) {
  double dist = 0;

  for (Translation2d p : points) {
    static Translation2d last_p = p;

    // Ignore the first point
    if (p == last_p) {
      continue;
    }

    dist += p.distance(last_p);
    last_p = p;
  }

  return dist;
}