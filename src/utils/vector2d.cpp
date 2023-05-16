#include "../core/include/utils/vector2d.h"

/**
 * Construct a vector object.
 * 
 * @param dir Direction, in radians. 'foward' is 0, clockwise positive when viewed from the top.
 * @param mag Magnitude.
 */
Vector2D::Vector2D(double dir, double mag)
: dir(dir), mag(mag)
{

}

/**
 * Construct a vector object from a cartesian point.
 * 
 * @param p point_t.x , point_t.y
 */
Vector2D::Vector2D(point_t p)
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
double Vector2D::get_dir() const { return dir;}

/**
 * Get the magnitude of the vector
 */
double Vector2D::get_mag() const { return mag; }

/**
 * Get the X component of the vector; positive to the right.
 */
double Vector2D::get_x() const
{
return mag * cos(dir);
}

/**
 * Get the Y component of the vector, positive forward.
 */
double Vector2D::get_y() const
{
return mag * sin(dir);
}

/**
 * Changes the magnetude of the vector to 1
*/
Vector2D Vector2D::normalize()
{
  return Vector2D(this->dir, 1);
}

/**
 * Convert a direction and magnitude representation to an x, y representation
 * @return the x, y representation of the vector 
*/
point_t Vector2D::point()
{
  point_t p = 
  {
    .x = this->mag * cos(this->dir),
    .y = this->mag * sin(this->dir)
  };
  return p;
}

/**
 * Add the components of two vectors together
 * Vector2D + Vector2D = (this.x + other.x, this.y + other.y)
 * @param other the vector to add to this
 * @return the sum of the vectors
*/

Vector2D Vector2D::operator+(const Vector2D &other)
{
    point_t p = 
    {
        .x = this->get_x() + other.get_x(),
        .y = this->get_y() + other.get_y()
    };

    return Vector2D( p );
}

/**
 * Subtract the components of two vectors together
 * Vector2D - Vector2D = (this.x - other.x, this.y - other.y)
 * @param other the vector to subtract from this
 * @return the difference of the vectors
*/
Vector2D Vector2D::operator-(const Vector2D &other)
{
    point_t p = 
    {
        .x = this->get_x() - other.get_x(),
        .y = this->get_y() - other.get_y()
    };
    return Vector2D( p );
}

/**
 * Scales a Vector2D by a scalar with the * operator
 * @param x the value to scale the vector by
 * @return the this Vector2D scaled by x
*/
Vector2D Vector2D::operator*(const double &x)
{
  return Vector2D(this->dir, this->mag * x);
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
