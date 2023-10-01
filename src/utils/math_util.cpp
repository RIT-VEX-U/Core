#include "../core/include/utils/math_util.h"
#include <vector>

#ifndef PI
#define PI 3.141592654
#endif

/**
* Constrain the input between a minimum and a maximum value
* 
* @param val  the value to be restrained
* @param low  the minimum value that will be returned
* @param high the maximum value that will be returned
**/
double clamp(double val, double low, double high){
  if (val < low){
    return low;
  } else if (val > high){
    return high;
  }
  return val;
}
/**
* Returns the sign of a number
* @param x
* 
* returns the sign +/-1 of x. special case at 0 it returns +1
**/
double sign(double x){
  if (x<0){
    return -1;
  }
  return 1;
}


double wrap_angle_deg(double input)
{
  double angle = fmod(input, 360);
  if(angle < 0)
    angle += 360;

  return angle;
}

double wrap_angle_rad(double input)
{
  double angle = fmod(input, 2*PI);
  if(angle < 0)
    angle += 2*PI;

  return angle;
}
/*
Calculates the average of a vector of doubles
@param values   the list of values for which the average is taken
*/
double mean(std::vector<double> const &values){
    double total=0;
    for (int i=0; i<values.size(); i++){
        total += values[i];
    }
    return total / (double) values.size();
}


/*
Calculates the variance of  a set of numbers (needed for linear regression)
https://en.wikipedia.org/wiki/Variance
@param values   the values for which the covariance is taken
@param mean     the average of values
*/
double variance(std::vector<double> const &values, double mean) {
    double total = 0.0;
    for (int i=0; i<values.size(); i++){
        total += (values[i]-mean) * (values[i]-mean);
    }
    return total;
}

/*
Calculates the covariance of a set of points (needed for linear regression)
(refactor to accept to sets of values not a set of points)
https://en.wikipedia.org/wiki/Covariance

@param points   the points for which the covariance is taken
@param meanx    the mean value of all x coordinates in points
@param meany    the mean value of all y coordinates in points
*/
double covariance(std::vector<std::pair<double, double>> const &points, double meanx, double meany){
    double covar = 0.0;
	for (int i=0; i<points.size(); i++){
		covar += (points[i].first - meanx) * (points[i].second - meany);
    }
    return covar;
}

/*
* Calculates the slope and y intercept of the line of best fit for the data
* @param points the points for the data
* @return slope, y intercept. y = m(x)+b. std::pair<m, b>
*/
std::pair<double, double> calculate_linear_regression(std::vector<std::pair<double, double>> const &points){
    //Purely for convenience and the ability to reuse mean() and variance() - can be easily rewritten to avoid allocating these if the code is repeatedly called
    std::vector<double> xs(points.size(), 0.0);
    std::vector<double> ys(points.size(), 0.0);
    for (int i =0; i<points.size(); i++){
        xs[i] = points[i].first;
        ys[i] = points[i].second;
    }

    double meanx = mean(xs);
    double meany = mean(ys);

    double slope = covariance(points, meanx, meany) / variance(xs, meanx);
	  double y_intercept = meany - slope * meanx;

    return std::pair<double, double>(slope, y_intercept);
}


double estimate_path_length(const std::vector<point_t> &points)
{
  double dist = 0;

  for(point_t p : points)
  {
    static point_t last_p = p;

    // Ignore the first point
    if(p == last_p)
      continue;

    dist += p.dist(last_p);
    last_p = p;
  }
  
  return dist;
}