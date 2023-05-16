#pragma once


#include <cmath>
#include "../core/include/utils/geometry.h"

#ifndef PI
#define PI 3.141592654
#endif
/**
 * Vector2D is an x,y pair
 * Used to represent 2D locations on the field. 
 * It can also be treated as a direction and magnitude
*/
class Vector2D
{
public:
    /**
     * Construct a vector object.
     * 
     * @param dir Direction, in radians. 'foward' is 0, clockwise positive when viewed from the top.
     * @param mag Magnitude.
     */ 
    Vector2D(double dir, double mag);
    
    /**
     * Construct a vector object from a cartesian point.
     * 
     * @param p point_t.x , point_t.y
     */
    Vector2D(point_t p);

    /**
     * Get the direction of the vector, in radians.
     * '0' is forward, clockwise positive when viewed from the top.
     * 
     * Use r2d() to convert.
     * @return the direction of the vetctor in radians
     */
    double get_dir() const;

    /**
     * @return the magnitude of the vector
     */
    double get_mag() const;

    /**
     * @return the X component of the vector; positive to the right.
     */
    double get_x() const;

    /**
     * @return the Y component of the vector, positive forward.
     */
    double get_y() const;

    /**
     * Changes the magnitude of the vector to 1
     * @return the normalized vector
    */
    Vector2D normalize();

    /**
    * Returns a point from the vector
    * @return the point represented by the vector
    */
    point_t point();

/**
 * Scales a Vector2D by a scalar with the * operator
 * @param x the value to scale the vector by
 * @return the this Vector2D scaled by x
*/
    Vector2D operator*(const double &x);
    /**
     * Add the components of two vectors together
     * Vector2D + Vector2D = (this.x + other.x, this.y + other.y)
     * @param other the vector to add to this
     * @return the sum of the vectors
    */
    Vector2D operator+(const Vector2D &other);
    /**
     * Subtract the components of two vectors together
     * Vector2D - Vector2D = (this.x - other.x, this.y - other.y)
     * @param other the vector to subtract from this
     * @return the difference of the vectors
    */
    Vector2D operator-(const Vector2D &other);

private:

    double dir, mag;

};

/**
 * General function for converting degrees to radians
 * @param deg the angle in degrees
 * @return the angle in radians
 */
double deg2rad(double deg);

/**
 * General function for converting radians to degrees
 * @param r the angle in radians
 * @return the angle in degrees

 */
double rad2deg(double r);
