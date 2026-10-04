#pragma once

#include <Eigen/Dense>
#include "cevalm.hpp"

/// Class representing a lattice point
class Point2d {
   private:
    int x_, y_;

   public:
    /// Default constructor for Point2d creating a lattice point at the origin
    constexpr Point2d() : x_(0), y_(0) {}

    /**
     * Creates a lattice point at the coordinate (x,y)
     * @param x The x-coordinate of the point
     * @param y The y-coordinate of the point
     */
    constexpr Point2d(int x, int y) : x_(x), y_(y) {}

    /**
     * Creates a lattice point with the values from the given vector
     * @param vector The vector whose values will be used
     */
    constexpr Point2d(Eigen::Vector2i vector)
     : x_(vector(0)), y_(vector(1)) {}

    /// @returns the X coordinate of the point.
    constexpr int x() const { return x_; }

    /// Sets the x coordinate.
    constexpr void set_x(int val) { x_ = val; }

    /// @returns the Y coordinate of the point.
    constexpr int y() const { return y_; }

    /// Sets the y coordinate.
    constexpr void set_y(int val) { y_ = val; }

    /// @returns the point as an Eigen::Vector2i.
    constexpr Eigen::Vector2i as_vector() const { return Eigen::Vector2i(x_, y_); }

    /**
     * Returns a vector in the canonical basis corresponding to the point in the basis of X and Y
     * @param X The vector corresponding to <1, 0> in the basis of X and Y
     * @param Y The vector corresponding to <0, 1> in the basis of X and Y
     * @returns The point as a linear combination of the X and Y vectors
     */
    constexpr Eigen::Vector2d as_vector(Eigen::Vector2d X, Eigen::Vector2d Y) const {
        return Eigen::Vector2d(
                x_ * X(0) + y_ * Y(0),
                x_ * X(1) + y_ * Y(1)
        );
    }

    /// @returns the manhattan distance between two points.
    constexpr int manhattan_distance(Point2d other) const {
        return cevalm::abs(x_ - other.x_) + cevalm::abs(y_ - other.y_);
    }

    /// @returns the manhattan distance away from the origin.
    constexpr int manhattan_norm() const { return cevalm::abs(x_) + cevalm::abs(y_); }

    /// @returns the distance (as a continuous number) between two points.
    constexpr double distance(Point2d other) const { return cevalm::hypot(x_ - other.x_, y_ - other.y_);  }

    /// @returns the distance (as a continuous number) away from the origin.
    constexpr double norm() const { return cevalm::hypot(x_, y_); }

    /**
     * Returns the inverse of the point
     *
     * [x] = -[x]
     * [y] = -[y]
     *
     * @return The inverse of the point
     */
    constexpr Point2d inverse() const { return Point2d{-x_, -y_};; }

    /**
     * Returns the dot product of two points
     *
     * [scalar] = [x][otherx] + [y][othery]
     *
     * @param other The other point to find the dot product with
     * @return The scalar-valued dot product
     */
    constexpr int dot(Point2d other) const { return (x_ * other.x_) + (y_ * other.y_); }

    /**
     * Returns the sum of two points
     *
     * [x] = [x] + [otherx]
     * [y] = [y] + [otherx]
     *
     * @param other The other point to be added
     * @return The sum of the two points
     */
    constexpr Point2d operator+(Point2d other) const { return Point2d{x_ + other.x_, y_ + other.y_}; }

    /// Adds another point to this point
    constexpr Point2d& operator+=(Point2d other) { return *this = *this + other; }

    /**
     * Returns the difference of two points
     *
     * [x] = [x] - [otherx]
     * [y] = [y] - [otherx]
     *
     * @param other The point being subtracted from this one
     * @return The difference of the two points
     */
    constexpr Point2d operator-(Point2d other) const { return Point2d{x_ - other.x_, y_ - other.y_}; }

    /// Subtracts another point from this point
    constexpr Point2d& operator-=(Point2d other) { return *this = *this - other; }

    /**
     * Computes the inverse of the point
     *
     * [x] = -[x]
     * [y] = -[y]
     *
     * @return The inverse of the point
     */
    constexpr Point2d operator-() const { return inverse(); }

    /**
     * Returns this point multiplied by a scalar
     *
     * [x] = [x] * [scalar]
     * [y] = [y] * [scalar]
     *
     * @param scalar The scalar to multiply by
     * @return This point multiplied by a scalar
     */
    constexpr Point2d operator*(int scalar) const { return Point2d{x_ * scalar, y_ * scalar}; }

    /// Scales the point
    friend constexpr Point2d operator*(int scalar, Point2d point) { return point * scalar; }

    /**
     * Computes the dot product of two points
     *
     * [scalar] = [x][otherx] + [y][othery]
     *
     * @param other The other point to find the dot product with
     * @return The scalar-valued dot product
     */
    constexpr int operator*(Point2d other) const { return dot(other); }

    /// Multiplies this point by a scalar
    constexpr Point2d& operator*=(int scalar) { return *this = *this * scalar; }

    /**
     * Compares two points
     * @param other The other Point2d to compare to
     * @return TRUE if the components of both points are equal, and FALSE if otherwise
     */
    constexpr bool operator==(Point2d other) const { return x_ == other.x_ && y_ == other.y_; }
};
