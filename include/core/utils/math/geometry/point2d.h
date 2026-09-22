#pragma once

#include <Eigen/Dense>

/**
 * Class representing a lattice point
 */
class Point2d {
public:
    /**
     * Default constructor for Point2d creating a lattice point at the origin
     */
    constexpr Point2d() : xcoord(0), ycoord(0) {}

    /**
     * Creates a lattice point at the coordinate (x,y)
     * @param x The x-coordinate of the point
     * @param y The y-coordinate of the point
     */
    Point2d(int x, int y);

    /**
     * Creates a lattice point with the values from the given vector
     * @param vector The vector whose values will be used
     */
    Point2d(Eigen::Vector2i vector);

    /**
     * Returns the X coordinate of the point
     * @returns The X coordinate of the point
     */
    int x() const;

    /**
     * Sets the X coordinate of the point
     */
    void setX(int x);

    /**
     * Returns the Y coordinate of the point
     * @returns The Y coordinate of the point
     */
    int y() const;

    /**
     * Sets the Y coordinate of the point
     */
    void setY(int y);

    /**
     * Returns the point as an Eigen::Vector2i
     * @returns Eigen::Vector2i with the same values as the point
     */
    Eigen::Vector2i as_vector() const;

    /**
     * Returns a vector in the canonical basis corresponding to the point in the basis of X and Y
     * @param X The vector corresponding to <1, 0> in the basis of X and Y
     * @param Y The vector corresponding to <0, 1> in the basis of X and Y
     * @returns The point as a linear combination of the X and Y vectors
     */
    Eigen::Vector2d as_vector(Eigen::Vector2d X, Eigen::Vector2d Y) const;

    /**
     * Returns the manhattan distance between two points
     * @returns The manhattan distance between two points
     */
    int manhattan_distance(Point2d other) const;

    /**
     * Returns the manhattan distance away from the origin
     * @returns The manhattan norm of the point
     */
    int manhattan_norm() const;

    /**
     * Returns the distance (as a continuous number) between two points
     * @returns The distance (as a continuous number) between two points
     */
    double distance(Point2d other) const;

    /**
     * Returns the distance (as a continuous number) away from the origin
     * @returns The norm (as a continuous number) of the point
     */
    double norm() const;

    /**
     * Compares two points
     * @param other The other Point2d to compare to
     * @return TRUE if the components of both points are equal, and FALSE if otherwise
     */
    bool operator==(Point2d other) const;

    /**
     * Returns the sum of two points
     *
     * [x] = [x] + [otherx]
     * [y] = [y] + [otherx]
     *
     * @param other The other point to be added
     * @return The sum of the two points
     */
    Point2d operator+(Point2d other) const;

    /**
     * Returns the difference of two points
     *
     * [x] = [x] - [otherx]
     * [y] = [y] - [otherx]
     *
     * @param other The point being subtracted from this one
     * @return The difference of the two points
     */
    Point2d operator-(Point2d other) const;

    /**
     * Returns the inverse of the point
     *
     * [x] = -[x]
     * [y] = -[y]
     *
     * @return The inverse of the point
     */
    Point2d operator-() const;

    /**
     * Returns this point multiplied by a scalar
     *
     * [x] = [x] * [scalar]
     * [y] = [y] * [scalar]
     *
     * @param scalar The scalar to multiply by
     * @return This point multiplied by a scalar
     */
    Point2d operator*(int scalar) const;

    /**
     * Returns the dot product of two points
     *
     * [scalar] = [x][otherx] + [y][othery]
     *
     * @param other The other point to find the dot product with
     * @return The scalar-valued dot product
     */
    int operator*(Point2d other) const;

    /**
     * Sends a point to an output stream. 
     * Ex:  The code `std::cout << point;` prints "Point2d[x: (value), y: (value)]"
     */
    friend std::ostream &operator<<(std::ostream &os, Point2d point);

private:
    int xcoord, ycoord;
};
