#include "../core/include/utils/vector.h"

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
    this->dir = atan2(p.y, p.x);
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
return mag * cos(dir);
}

/**
 * Get the Y component of the vector, positive forward.
 */
double Vector::get_y() const
{
return mag * sin(dir);
}

/**
 * Changes the magnetude of the vector to 1
*/
Vector Vector::normalize()
{
  return Vector(this->dir, 1);
}

/**
 * Returns a point from the vector
*/
Vector::point_t Vector::point()
{
  point_t p = 
  {
    .x = this->mag * cos(this->dir),
    .y = this->mag * sin(this->dir)
  };
  return p;
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
 * Multiplies a vector by a double with the * operator
*/
Vector Vector::operator*(const double &x)
{
  return Vector(this->dir, this->mag * x);
}

/**
 * General function for converting degrees to radians
 */
double deg2rad(double deg)
{
    return deg * (PI / 180.0);
}

/**
 * General function for converting radians to degrees
 */
double rad2deg(double rad)
{
    return rad * (180.0 / PI);
}
