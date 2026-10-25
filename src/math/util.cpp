#include "core/math/util.h"
#include <vector>

/**
 * Calculates the average of a vector of doubles
 * @param values   the list of values for which the average is taken
 */
double mean(std::vector<double> const &values) {
    double total = 0;
    for (int i = 0; i < values.size(); i++) {
        total += values[i];
    }
    return total / (double)values.size();
}

/**
 * Calculates the variance of  a set of numbers (needed for linear regression)
 * https://en.wikipedia.org/wiki/Variance
 * @param values   the values for which the covariance is taken
 * @param mean     the average of values
 */
double variance(std::vector<double> const &values, double mean) {
    double total = 0.0;
    for (int i = 0; i < values.size(); i++) {
        total += (values[i] - mean) * (values[i] - mean);
    }
    return total / (values.size() - 1);
}

/**
 * Calculates the covariance of a set of points (needed for linear regression)
 * (refactor to accept to sets of values not a set of points)
 * https://en.wikipedia.org/wiki/Covariance
 *
 * @param points   the points for which the covariance is taken
 * @param meanx    the mean value of all x coordinates in points
 * @param meany    the mean value of all y coordinates in points
 */
double covariance(std::vector<std::pair<double, double>> const &points, double meanx, double meany) {
    double covar = 0.0;
    for (int i = 0; i < points.size(); i++) {
        covar += (points[i].first - meanx) * (points[i].second - meany);
    }
    return covar;
}

/**
 * Calculates the slope and y intercept of the line of best fit for the data
 * @param points the points for the data
 * @return slope, y intercept. y = m(x)+b. std::pair<m, b>
 */
std::pair<double, double> calculate_linear_regression(std::vector<std::pair<double, double>> const &points) {
    /* 
     * Purely for convenience and the ability to reuse mean() and variance() - can be easily rewritten to avoid
     * allocating these if the code is repeatedly called
     */
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

double estimate_path_length(const std::vector<Translation2d> &points) {
    double dist = 0;

    for (Translation2d p : points) {
        static Translation2d last_p = p;

        // Ignore the first point
        if (p == last_p) {
            continue;
        }

        dist += p.distance(last_p).to(units::in);
        last_p = p;
    }

    return dist;
}
