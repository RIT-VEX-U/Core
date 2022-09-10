#pragma once

#include <cmath>

#ifndef PI
#define PI 3.141592654
#endif

class Vector
{
public:

    /**
     * Data structure representing an X,Y coordinate
     */
    struct point_t
    {
        double x, y;

        double dist(const point_t other)
        {
          return sqrt(pow(this->x - other.x, 2) + pow(this->y - other.y, 2));
        }

        // Vector addition operation on points
        point_t operator+(const point_t &other)
        {
          point_t p 
          {
            .x = this->x + other.x,
            .y = this->y + other.y
          };
          return p;
        }

        // Vector subtraction operation on points
        point_t operator-(const point_t &other)
        {
          point_t p 
          {
            .x = this->x - other.x,
            .y = this->y - other.y
          };
          return p;
        }
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

    /**
     * Changes the magnetude of the vector to 1
    */
    Vector normalize();

    /**
    * Returns a point from the vector
    */
    Vector::point_t point();

    Vector operator*(const double &x);
    Vector operator+(const Vector &other);
    Vector operator-(const Vector &other);

private:

    double dir, mag;

};

/**
 * General function for converting degrees to radians
 */
double deg2rad(double deg);

/**
 * General function for converting radians to degrees
 */
double rad2deg(double r);
