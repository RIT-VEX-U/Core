#ifndef _VECTOR_
#define _VECTOR_

#include <cmath>

#define PI 3.141592654

class Vector
{
public:

    struct point_t
    {
        double x, y;
    };

    /**
     * Construct a vector object.
     * 
     * @param dir Direction, in radians. 'foward' is 0, clockwise positive when viewed from the top.
     * @param mag Magnitude.
     */ 
    Vector(double dir, double mag);

    /**
     * Construct a vector object from a cartesian point.
     * 
     * @param p point_t.x , point_t.y
     */
    Vector(point_t &p);

    /**
     * Get the direction of the vector, in radians.
     * '0' is forward, clockwise positive when viewed from the top.
     * 
     * Use r2d() to convert.
     */
    double get_dir() const;

    /**
     * Get the magnitude of the vector
     */
    double get_mag() const;

    /**
     * Get the X component of the vector; positive to the right.
     */
    double get_x() const;

    /**
     * Get the Y component of the vector, positive forward.
     */
    double get_y() const;


    Vector operator+(const Vector &other);
    Vector operator-(const Vector &other);

private:

    double dir, mag;

};

/**
 * General function for converting degrees to radians
 */
double d2r(double deg);

/**
 * General function for converting radians to degrees
 */
double r2d(double r);

#endif