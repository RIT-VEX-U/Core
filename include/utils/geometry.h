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
    double dist(const point_t other) const
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

    point_t operator*(double s) const
    {
        return {x * s, y * s};
    }
    point_t operator/(double s) const
    {
        return {x / s, y / s};
    }

    point_t operator-() const
    {
        return {-x, -y};
    }
    point_t operator+() const
    {
        return {x, y};
    }

    bool operator==(const point_t &rhs)
    {
        return x == rhs.x && y == rhs.y;
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

    point_t get_point()
    {
        return point_t{.x = x, .y = y};
    }

} pose_t;

struct Mat2
{
    double X11, X12;
    double X21, X22;
    point_t operator*(const point_t p) const
    {
        double outx = p.x * X11 + p.y * X12;
        double outy = p.x * X21 + p.y * X22;
        return {outx, outy};
    }

    static Mat2 FromRotationDegrees(double degrees)
    {
        double rad = degrees * (M_PI / 180.0);
        double c = cos(rad);
        double s = sin(rad);
        return {c, -s, s, c};
    }
};