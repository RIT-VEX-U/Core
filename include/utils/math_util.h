#pragma once
#include "math.h"
#include "vex.h"
#include <vector>

/**
* Constrain the input between a minimum and a maximum value
* 
* @param val  the value to be restrained
* @param low  the minimum value that will be returned
* @param high the maximum value that will be returned
**/
double clamp(double value, double low, double high);

/**
* Returns the sign of a number
* @param x
* 
* returns the sign +/-1 of x. 0 if x is 0
**/
double sign(double x);

double wrap_angle_deg(double input);
double wrap_angle_rad(double input);

/*
Calculates the variance of  a set of numbers (needed for linear regression)
https://en.wikipedia.org/wiki/Variance
@param values   the values for which the variance is taken
@param mean     the average of values
*/
double variance(std::vector<double> const &values, double mean);


/*
Calculates the average of a vector of doubles
@param values   the list of values for which the average is taken
*/
double mean(std::vector<double> const &values);

/*
Calculates the covariance of a set of points (needed for linear regression)
https://en.wikipedia.org/wiki/Covariance

@param points   the points for which the covariance is taken
@param meanx    the mean value of all x coordinates in points
@param meany    the mean value of all y coordinates in points
*/
double covariance(std::vector<std::pair<double, double>> const &points, double meanx, double meany);

/*
Calculates the slope and y intercept of the line of best fit for the data
@param points the points for the data
*/
std::pair<double, double> calculate_linear_regression(std::vector<std::pair<double, double>> const &points);

