#pragma once

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