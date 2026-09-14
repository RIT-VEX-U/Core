#include <cmath>
#include "core/utils/math/geometry/point2d.h"


/**
 * Creates a lattice point at the coordinate (x,y)
 * @param x The x-coordinate of the point
 * @param y The y-coordinate of the point
 */
Point2d::Point2d(const int& x, const int& y) : xcoord(x), ycoord(y) {}

/**
 * Creates a lattice point with the values from the given vector
 * @param vector The vector whose values will be used
 */
Point2d::Point2d(const Eigen::Vector2i &vector) : xcoord(vector(0)), ycoord(vector(1)) {}


/**
 * Returns the X coordinate of the point
 * @returns The X coordinate of the point
 */
int Point2d::x() const {
    return this->xcoord;
}

/**
 * Sets the X coordinate of the point
 */
void Point2d::setX(const int& x) {
    this->xcoord = x;
}

/**
 * Returns the Y coordinate of the point
 * @returns The Y coordinate of the point
 */
int Point2d::y() const {
    return this->ycoord;
}

/**
 * Sets the Y coordinate of the point
 */
void Point2d::setY(const int& y) {
    this->ycoord = y;
}

/**
 * Returns the vector as an Eigen::Vector2i
 * @returns Eigen::Vector2i with the same values as the point
 */
Eigen::Vector2i Point2d::as_vector() const {
    return Eigen::Vector2i(this->xcoord, this->ycoord);
}

/**
 * Returns a vector in the canonical basis corresponding to the point in the basis of X and Y
 * @param X The vector corresponding to <1, 0> in the basis of X and Y
 * @param Y The vector corresponding to <0, 1> in the basis of X and Y
 * @returns The point as a linear combination of the X and Y vectors
 */
Eigen::Vector2d Point2d::as_vector(Eigen::Vector2d X, Eigen::Vector2d Y) const {
    return Eigen::Vector2d(this->xcoord * X(0) + this->ycoord * Y(0), this->xcoord * X(1) + this->ycoord * Y(1));
}

/**
 * Returns the manhattan distance between two points
 * @returns The manhattan distance between two points
 */
int Point2d::manhattan_distance(const Point2d& other) const {
    return abs(this->xcoord - other.xcoord) + abs(this->ycoord - other.ycoord);
}

/**
 * Returns the manhattan distance away from the origin
 * @returns The manhattan norm of the point
 */
int Point2d::manhattan_norm() const {
    return abs(this->xcoord) + abs(this->ycoord);
}

/**
 * Returns the distance (as a continuous number) between two points
 * @returns The distance (as a continuous number) between two points
 */
double Point2d::distance(const Point2d& other) const {
    return hypot(this->xcoord - other.xcoord, this->ycoord - other.ycoord);
}

/**
 * Returns the distance (as a continuous number) away from the origin
 * @returns The norm (as a continuous number) of the point
 */
double Point2d::norm() const {
    return hypot(this->xcoord, this->ycoord);
}

/**
 * Compares two points
 * @param other The other Point2d to compare to
 * @return TRUE if the components of both points are equal, and FALSE if otherwise
 */
bool Point2d::operator==(const Point2d &other) const {
    return this->xcoord == other.xcoord && this->ycoord == other.ycoord;
}

/**
 * Returns the sum of two points
 *
 * [x] = [x] + [otherx]
 * [y] = [y] + [otherx]
 *
 * @param other The other point to be added
 * @return The sum of the two points
 */
Point2d Point2d::operator+(const Point2d &other) const {
    return Point2d{this->xcoord + other.xcoord, this->ycoord + other.ycoord};
}

/**
 * Returns the difference of two points
 *
 * [x] = [x] - [otherx]
 * [y] = [y] - [otherx]
 *
 * @param other The point being subtracted from this one
 * @return The difference of the two points
 */
Point2d Point2d::operator-(const Point2d &other) const {
    return Point2d{this->xcoord - other.xcoord, this->ycoord - other.ycoord};
}

/**
 * Returns the inverse of the point
 *
 * [x] = -[x]
 * [y] = -[y]
 *
 * @return The inverse of the point
 */
Point2d Point2d::operator-() const {
    return Point2d{-this->xcoord, -this->ycoord};
}

/**
 * Returns this point multiplied by a scalar
 *
 * [x] = [x] * [scalar]
 * [y] = [y] * [scalar]
 *
 * @param scalar The scalar to multiply by
 * @return This point multiplied by a scalar
 */
Point2d Point2d::operator*(const int &scalar) const {
    return Point2d{scalar * this->xcoord, scalar * this->ycoord};
}

/**
 * Returns the dot product of two points
 *
 * [scalar] = [x][otherx] + [y][othery]
 *
 * @param other The other point to find the dot product with
 * @return The scalar-valued dot product
 */
int Point2d::operator*(const Point2d &other) const {
    return (this->xcoord * other.xcoord) + (this->ycoord * other.ycoord);
}

/**
 * Sends a point to an output stream. 
 * Ex:  The code `std::cout << point;` prints "Point2d[x: (value), y: (value)]"
 */
std::ostream &operator<<(std::ostream &os, const Point2d &point) {
    os << "Point2d[x: " << point.x() << ", y: " << point.y() << "]";
    return os;
}