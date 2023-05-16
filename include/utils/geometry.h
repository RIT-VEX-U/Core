#pragma once
#include <cmath>

/**
 * Data structure representing an X,Y coordinate
 */
struct point_t
{
    double x; ///< the x position in space
    double y; ///< the y position in space

    /**
     * dist calculates the euclidian distance between this point and another point using the pythagorean theorem
     * @param other the point to measure the distance from
     * @return the euclidian distance between this and other
     */
    double dist(const point_t other)
    {
        return std::sqrt(std::pow(this->x - other.x, 2) + pow(this->y - other.y, 2));
    }

    /**
     * Vector2D addition operation on points
     * @param other the point to add on to this
     * @return this + other (this.x + other.x, this.y + other.y)
     */
    point_t operator+(const point_t &other)
    {
        point_t p{
            .x = this->x + other.x,
            .y = this->y + other.y};
        return p;
    }

    /**
     * Vector2D subtraction operation on points
     * @param other the point_t to subtract from this
     * @return this - other (this.x - other.x, this.y - other.y)
     */
    point_t operator-(const point_t &other)
    {
        point_t p{
            .x = this->x - other.x,
            .y = this->y - other.y};
        return p;
    }
};


/**
 *  Describes a single position and rotation
 */
typedef struct
{
    double x;   ///< x position in the world
    double y;   ///< y position in the world
    double rot; ///< rotation in the world
} pose_t;