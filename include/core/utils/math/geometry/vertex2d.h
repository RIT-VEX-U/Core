#pragma once

#include <Eigen/Dense>

/**
 * Class representing a point in a square lattice
 */
class Vertex2d {
public:
    /**
     * Default constructor for Vertex2d creating a vertex at the origin
     */
    constexpr Vertex2d() : xcoord(0), ycoord(0) {}

    /**
     * Creates a vertex at the coordinate (x,y)
     * @param x The x-coordinate of the vertex
     * @param y The y-coordinate of the vertex
     */
    Vertex2d(const int& x, const int& y);

    /**
     * Creates a vertex with the values from the given vector
     * @param vector The vector whose values will be used
     */
    Vertex2d(const Eigen::Vector2i &vector);

    /**
     * Returns the X coordinate of the vertex
     * @returns The X coordinate of the vertex
     */
    int x() const;

    /**
     * Sets the X coordinate of the vertex
     */
    void setX(const int& x);

    /**
     * Returns the Y coordinate of the vertex
     * @returns The Y coordinate of the vertex
     */
    int y() const;

    /**
     * Sets the Y coordinate of the vertex
     */
    void setY(const int& y);

    /**
     * Returns the vector as an Eigen::Vector2i
     * @returns Eigen::Vector2i with the same values as the vertex
     */
    Eigen::Vector2i as_vector() const;

    /**
     * Returns the manhattan distance between two vertices
     * @returns The manhattan distance between two vertices
     */
    int manhattan_distance(const Vertex2d& other) const;

    /**
     * Returns the manhattan distance away from the origin
     * @returns The manhattan norm of the vertex
     */
    int manhattan_norm() const;

    /**
     * Returns the distance (as a continuous number) between two vertices
     * @returns The distance (as a continuous number) between two vertices
     */
    double distance(const Vertex2d& other) const;

    /**
     * Returns the distance (as a continuous number) away from the origin
     * @returns The norm (as a continuous number) of the vertex
     */
    double norm() const;

    /**
     * Compares two vertices
     * @param other The other Vertex2d to compare to
     * @return TRUE if the components of both vertices are equal, and FALSE if otherwise
     */
    bool operator==(const Vertex2d &other) const;

    /**
     * Returns the sum of two vertices
     *
     * [x] = [x] + [otherx]
     * [y] = [y] + [otherx]
     *
     * @param other The other vertex to be added
     * @return The sum of the two vertices
     */
    Vertex2d operator+(const Vertex2d &other) const;

    /**
     * Returns the difference of two vertices
     *
     * [x] = [x] - [otherx]
     * [y] = [y] - [otherx]
     *
     * @param other The vertex being subtracted from this one
     * @return The difference of the two vertices
     */
    Vertex2d operator-(const Vertex2d &other) const;

    /**
     * Returns the inverse of the vertex
     *
     * [x] = -[x]
     * [y] = -[y]
     *
     * @return The inverse of the vertex
     */
    Vertex2d operator-() const;

    /**
     * Returns this vertex multiplied by a scalar
     *
     * [x] = [x] * [scalar]
     * [y] = [y] * [scalar]
     *
     * @param scalar The scalar to multiply by
     * @return This vertex multiplied by a scalar
     */
    Vertex2d operator*(const int &scalar) const;

    /**
     * Returns the dot product of two vertices
     *
     * [scalar] = [x][otherx] + [y][othery]
     *
     * @param other The other vertex to find the dot product with
     * @return The scalar-valued dot product
     */
    int operator*(const Vertex2d &other) const;

    /**
     * Sends a vertex to an output stream. 
     * Ex:  The code `std::cout << vertex;` prints "Vertex2d[x: (value), y: (value)]"
     */
    friend std::ostream &operator<<(std::ostream &os, const Vertex2d &translation);

private:
    int xcoord, ycoord;
};