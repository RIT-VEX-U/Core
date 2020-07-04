#include "../Core/include/utils/vector.h"

/**
 * Construct a vector object.
 * 
 * @param dir Direction, in radians. 'foward' is 0, clockwise positive when viewed from the top.
 * @param mag Magnitude.
 */
Vector::Vector(double dir, double mag)
: dir(dir), mag(mag)
{

}

/**
 * Construct a vector object from a cartesian point.
 * 
 * @param p point_t.x , point_t.y
 */
Vector::Vector(point_t &p)
{
    this->dir = atan2(p.x, p.y);
    this->mag = sqrt( (p.x*p.x) + (p.y*p.y) );
}

/**
 * Get the direction of the vector, in radians.
 * '0' is forward, clockwise positive when viewed from the top.
 * 
 * Use r2d() to convert.
 */
double Vector::get_dir() const { return dir;}

/**
 * Get the magnitude of the vector
 */
double Vector::get_mag() const { return mag; }

/**
 * Get the X component of the vector; positive to the right.
 */
double Vector::get_x() const
{
return mag * sin(dir);
}

/**
 * Get the Y component of the vector, positive forward.
 */
double Vector::get_y() const
{
return mag * cos(dir);
}

/**
 * Correctly add vectors together with the + operator
 */
Vector Vector::operator+(const Vector &other)
{
    point_t p = 
    {
        .x = this->get_x() + other.get_x(),
        .y = this->get_y() + other.get_y()
    };

    return Vector( p );
}

/**
 * Correctly subtract vectors with the - operator
 */
Vector Vector::operator-(const Vector &other)
{
    point_t p = 
    {
        .x = this->get_x() - other.get_x(),
        .y = this->get_y() - other.get_y()
    };
    return Vector( p );
}

/**
 * General function for converting degrees to radians
 */
double d2r(double deg)
{
    return deg * (PI / 180.0);
}

/**
 * General function for converting radians to degrees
 */
double r2d(double rad)
{
    return rad * (180.0 / PI);
}