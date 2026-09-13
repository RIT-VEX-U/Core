#include <cmath>
#include "core/utils/math/geometry/vertex2d.h"


/**
 * Creates a vertex at the coordinate (x,y)
 * @param x The x-coordinate of the vertex
 * @param y The y-coordinate of the vertex
 */
Vertex2d::Vertex2d(const int& x, const int& y) : xcoord(x), ycoord(y) {}

/**
 * Creates a vertex with the values from the given vector
 * @param vector The vector whose values will be used
 */
Vertex2d::Vertex2d(const Eigen::Vector2i &vector) : xcoord(vector(0)), ycoord(vector(1)) {}


/**
 * Returns the X coordinate of the vertex
 * @returns The X coordinate of the vertex
 */
int Vertex2d::x() const {
    return this->xcoord;
}

/**
 * Sets the X coordinate of the vertex
 */
void Vertex2d::setX(const int& x) {
    this->xcoord = x;
}

/**
 * Returns the Y coordinate of the vertex
 * @returns The Y coordinate of the vertex
 */
int Vertex2d::y() const {
    return this->ycoord;
}

/**
 * Sets the Y coordinate of the vertex
 */
void Vertex2d::setY(const int& y) {
    this->ycoord = y;
}

/**
 * Returns the vector as an Eigen::Vector2i
 * @returns Eigen::Vector2i with the same values as the vertex
 */
Eigen::Vector2i Vertex2d::as_vector() const {
    return Eigen::Vector2i(xcoord, ycoord);
}

/**
 * Returns the manhattan distance between two vertices
 * @returns The manhattan distance between two vertices
 */
int Vertex2d::manhattan_distance(const Vertex2d& other) const {
    return abs(this->xcoord - other.xcoord) + abs(this->ycoord - other.ycoord);
}

/**
 * Returns the manhattan distance away from the origin
 * @returns The manhattan norm of the vertex
 */
int Vertex2d::manhattan_norm() const {
    return abs(this->xcoord) + abs(this->ycoord);
}

/**
 * Returns the distance (as a continuous number) between two vertices
 * @returns The distance (as a continuous number) between two vertices
 */
double Vertex2d::distance(const Vertex2d& other) const {
    return hypot(this->xcoord - other.xcoord, this->ycoord - other.ycoord);
}

/**
 * Returns the distance (as a continuous number) away from the origin
 * @returns The norm (as a continuous number) of the vertex
 */
double Vertex2d::norm() const {
    return hypot(this->xcoord, this->ycoord);
}

/**
 * Compares two vertices
 * @param other The other Vertex2d to compare to
 * @return TRUE if the components of both vertices are equal, and FALSE if otherwise
 */
bool Vertex2d::operator==(const Vertex2d &other) const {
    return this->xcoord == other.xcoord && this->ycoord == other.ycoord;
}

/**
 * Returns the sum of two vertices
 *
 * [x] = [x] + [otherx]
 * [y] = [y] + [otherx]
 *
 * @param other The other vertex to be added
 * @return The sum of the two vertices
 */
Vertex2d Vertex2d::operator+(const Vertex2d &other) const {
    return Vertex2d{this->xcoord + other.xcoord, this->ycoord + other.ycoord};
}

/**
 * Returns the difference of two vertices
 *
 * [x] = [x] - [otherx]
 * [y] = [y] - [otherx]
 *
 * @param other The vertex being subtracted from this one
 * @return The difference of the two vertices
 */
Vertex2d Vertex2d::operator-(const Vertex2d &other) const {
    return Vertex2d{this->xcoord - other.xcoord, this->ycoord - other.ycoord};
}

/**
 * Returns the inverse of the vertex
 *
 * [x] = -[x]
 * [y] = -[y]
 *
 * @return The inverse of the vertex
 */
Vertex2d Vertex2d::operator-() const {
    return Vertex2d{-this->xcoord, -this->ycoord};
}

/**
 * Returns this vertex multiplied by a scalar
 *
 * [x] = [x] * [scalar]
 * [y] = [y] * [scalar]
 *
 * @param scalar The scalar to multiply by
 * @return This vertex multiplied by a scalar
 */
Vertex2d Vertex2d::operator*(const int &scalar) const {
    return Vertex2d{scalar * this->xcoord, scalar * this->ycoord};
}

/**
 * Returns the dot product of two vertices
 *
 * [scalar] = [x][otherx] + [y][othery]
 *
 * @param other The other vertex to find the dot product with
 * @return The scalar-valued dot product
 */
int Vertex2d::operator*(const Vertex2d &other) const {
    return (this->xcoord * other.xcoord) + (this->ycoord * other.ycoord);
}

/**
 * Sends a vertex to an output stream. 
 * Ex:  The code `std::cout << vertex;` prints "Vertex2d[x: (value), y: (value)]"
 */
std::ostream &operator<<(std::ostream &os, const Vertex2d &vertex) {
    os << "Vertex2d[x: " << vertex.x() << ", y: " << vertex.y() << "]";
    return os;
}