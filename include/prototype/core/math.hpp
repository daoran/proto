/**
 * @file
 * @defgroup math math
 * @ingroup core
 */
#ifndef PROTOTYPE_CORE_MATH_HPP
#define PROTOTYPE_CORE_MATH_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "prototype/core/log.hpp"

namespace prototype {
/**
 * @addtogroup math
 * @{
 */

#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__
typedef Eigen::Vector2d vec2_t;
typedef Eigen::Vector3d vec3_t;
typedef Eigen::Vector4d vec4_t;
typedef Eigen::Matrix<double, 5, 1> vec5_t;
typedef Eigen::Matrix<double, 6, 1> vec6_t;
typedef Eigen::VectorXd vecx_t;

typedef Eigen::Matrix2d mat2_t;
typedef Eigen::Matrix3d mat3_t;
typedef Eigen::Matrix4d mat4_t;
typedef Eigen::MatrixXd matx_t;
typedef Eigen::Matrix<double, 3, 4> mat34_t;

typedef Eigen::Quaterniond quat_t;
#endif

/**
 * Print shape of a matrix
 *
 * @param[in] name Name of matrix
 * @param[in] A Matrix
 */
void print_shape(const std::string &name, const matx_t &A);

/**
 * Print shape of a vector
 *
 * @param[in] name Name of vector
 * @param[in] v Vector
 */
void print_shape(const std::string &name, const vecx_t &v);

/**
 * Print array
 *
 * @param[in] name Name of array
 * @param[in] array Target array
 * @param[in] size Size of target array
 */
void print_array(const std::string &name,
                 const double *array,
                 const size_t size);

/**
 * Array to string
 *
 * @param[in] array Target array
 * @param[in] size Size of target array
 * @returns String of array
 */
std::string array2str(const double *array, const size_t size);

/**
 * Convert double array to Eigen::Vector
 *
 * @param[in] x Input array
 * @param[in] size Size of input array
 * @param[out] y Output vector
 */
void array2vec(const double *x, const size_t size, vecx_t &y);

/**
 * Vector to array
 *
 * @param[in] v Vector
 * @returns Array
 */
double *vec2array(const vecx_t &v);

/**
 * Matrix to array
 *
 * @param[in] m Matrix
 * @returns Array
 */
double *mat2array(const matx_t &m);

/**
 * Quaternion to array
 *
 * *VERY IMPORTANT*: The returned array is (x, y, z, w).
 *
 * @param[in] q Quaternion
 * @returns Array
 */
double *quat2array(const quat_t &q);

/**
 * Vector to array
 *
 * @param[in] v Vector
 * @param[out] out Output array
 */
void vec2array(const vecx_t &v, double *out);

/**
 * Matrix to array
 *
 * @param[in] m Matrix
 * @param[in] out Output array
 */
void mat2array(const matx_t &m, double *out);

/**
 * Matrix to list of vectors
 *
 * @param[in] m Matrix
 * @param[in] row_wise Row wise
 * @returns Vectors
 */
std::vector<vecx_t> mat2vec(const matx_t &m, const bool row_wise = true);

/**
 * Matrix to list of vectors of size 3
 *
 * @param[in] m Matrix
 * @param[in] row_wise Row wise
 * @returns Vectors
 */
std::vector<vec3_t> mat2vec3(const matx_t &m, const bool row_wise = true);

/**
 * Matrix to list of vectors of size 3
 *
 * @param[in] m Matrix
 * @param[in] row_wise Row wise
 * @returns Vectors
 */
std::vector<vec2_t> mat2vec2(const matx_t &m, const bool row_wise = true);

/**
 * Vector to string
 *
 * @param[in] v Vector
 * @param[in] brackets Brakcets around vector string
 * @returns Vector as a string
 */
std::string vec2str(const vecx_t &v, const bool brackets = true);

/**
 * Create random integer
 *
 * @param[in] ub Upper bound
 * @param[in] lb Lower bound
 * @return Random integer
 */
int randi(const int ub, const int lb);

/**
 * Create random double
 *
 * @param[in] ub Upper bound
 * @param[in] lb Lower bound
 * @return Random floating point
 */
double randf(const double ub, const double lb);

/**
 * Sign of number
 *
 * @param[in] x Number to check sign
 * @return
 *    - 0: Number is zero
 *    - 1: Positive number
 *    - -1: Negative number
 */
int sign(const double x);

/**
 * Floating point comparator
 *
 * @param[in] f1 First value
 * @param[in] f2 Second value
 * @return
 *    - 0: if equal
 *    - 1: if f1 > f2
 *    - -1: if f1 < f2
 */
int fltcmp(const double f1, const double f2);

/**
 * Calculate median given an array of numbers
 *
 * @param[in] v Array of numbers
 * @return Median of given array
 */
double median(const std::vector<double> &v);

/**
 * Degrees to radians
 *
 * @param[in] d Degree to be converted
 * @return Degree in radians
 */
double deg2rad(const double d);

/**
 * Radians to degree
 *
 * @param[in] r Radian to be converted
 * @return Radian in degrees
 */
double rad2deg(const double r);

/**
 * Load std::vector of doubles to an Eigen::Matrix
 *
 * @param[in] x Matrix values
 * @param[in] rows Number of matrix rows
 * @param[in] cols Number of matrix colums
 * @param[out] y Output matrix
 */
void load_matrix(const std::vector<double> &x,
                 const int rows,
                 const int cols,
                 matx_t &y);

/**
 * Load an Eigen::Matrix into a std::vector of doubles
 *
 * @param[in] A Matrix
 * @param[out] x Output vector of matrix values
 */
void load_matrix(const matx_t A, std::vector<double> &x);

/**
 * Calculate binomial coefficient
 *
 * @param[in] n
 * @param[in] k
 * @returns Binomial coefficient
 */
double binomial(const double n, const double k);

/**
 * Wrap angle in degrees to 180
 *
 * @param[in] d Degrees
 * @return Angle wraped to 180
 */
double wrapTo180(const double d);

/**
 * Wrap angle in degrees to 360
 *
 * @param[in] d Degrees
 * @return Angle wraped to 360
 */
double wrapTo360(const double d);

/**
 * Wrap angle in radians to PI
 *
 * @param[in] r Radians
 * @return Angle wraped to PI
 */
double wrapToPi(const double r);

/**
 * Wrap angle in radians to 2 PI
 *
 * @param[in] r Radians
 * @return Angle wraped to 2 PI
 */
double wrapTo2Pi(const double r);

/**
 * Mean vector
 *
 * @param[in] x List of vectors
 * @return Mean vector
 */
vec3_t mean(const std::vector<vec3_t> &x);

/**
 * Cross-Track error based on waypoint line between p1, p2, and robot position
 *
 * @param[in] p1 Waypoint 1
 * @param[in] p2 Waypoint 2
 * @param[in] pos Robot position
 * @return Cross track error
 */
double cross_track_error(const vec2_t &p1, const vec2_t &p2, const vec2_t &pos);

/**
 * Check if point `pos` is left or right of line formed by `p1` and `p2`
 *
 * @param[in] p1 Waypoint 1
 * @param[in] p2 Waypoint 2
 * @param[in] pos Robot position
 * @returns
 *    - 1: Point is left of waypoint line formed by `p1` and `p2`
 *    - 2: Point is right of waypoint line formed by `p1` and `p2`
 *    - 0: Point is colinear with waypoint line formed by `p1` and `p2`
 */
int point_left_right(const vec2_t &p1, const vec2_t &p2, const vec2_t &pos);

/**
 * Calculate closest point given waypoint line between `p1`, `p2` and robot
 * position
 *
 * @param[in] p1 Waypoint 1
 * @param[in] p2 Waypoint 2
 * @param[in] p3 Robot position
 * @param[out] closest Closest point
 * @returns
 *    Unit number denoting where the closest point is on waypoint line. For
 *    example, a return value of 0.5 denotes the closest point is half-way
 *    (50%) of the waypoint line, alternatively a negative number denotes the
 *    closest point is behind the first waypoint.
 */
double closest_point(const vec2_t &p1,
                     const vec2_t &p2,
                     const vec2_t &p3,
                     vec2_t &closest);

/**
 * Linear interpolation between two points
 *
 * @param[in] a First point
 * @param[in] b Second point
 * @param[in] mu Unit number
 * @returns Linear interpolation
 */
vec2_t lerp(const vec2_t &a, const vec2_t &b, const double mu);

/** @} group math */
} //  namespace prototype
#endif // PROTOTYPE_CORE_MATH_HPP
