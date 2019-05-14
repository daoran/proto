#ifndef PROTOTYPE_CORE_MATH_HPP
#define PROTOTYPE_CORE_MATH_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "prototype/core/log.hpp"

namespace proto {

#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__
typedef Eigen::Vector2d vec2_t;
typedef Eigen::Vector3d vec3_t;
typedef Eigen::Vector4d vec4_t;
typedef Eigen::Matrix<double, 5, 1> vec5_t;
typedef Eigen::Matrix<double, 6, 1> vec6_t;
typedef Eigen::VectorXd vecx_t;

typedef Eigen::Vector2f vec2f_t;
typedef Eigen::Vector3f vec3f_t;
typedef Eigen::Vector4f vec4f_t;
typedef Eigen::Matrix<float, 5, 1> vec5f_t;
typedef Eigen::Matrix<float, 6, 1> vec6f_t;
typedef Eigen::VectorXf vecxf_t;

typedef Eigen::Matrix2d mat2_t;
typedef Eigen::Matrix3d mat3_t;
typedef Eigen::Matrix4d mat4_t;
typedef Eigen::MatrixXd matx_t;
typedef Eigen::Matrix<double, 3, 4> mat34_t;

typedef Eigen::Matrix2f mat2f_t;
typedef Eigen::Matrix3f mat3f_t;
typedef Eigen::Matrix4f mat4f_t;
typedef Eigen::MatrixXf matxf_t;
typedef Eigen::Matrix<float, 3, 4> mat34f_t;

typedef std::vector<vec2_t, Eigen::aligned_allocator<vec2_t>> vec2s_t;
typedef std::vector<vec3_t, Eigen::aligned_allocator<vec3_t>> vec3s_t;
typedef std::vector<vec4_t, Eigen::aligned_allocator<vec4_t>> vec4s_t;
typedef std::vector<vec5_t, Eigen::aligned_allocator<vec5_t>> vec5s_t;
typedef std::vector<vec6_t, Eigen::aligned_allocator<vec6_t>> vec6s_t;
typedef std::vector<vecx_t> vecxs_t;

typedef std::vector<vec2f_t, Eigen::aligned_allocator<vec2f_t>> vec2fs_t;
typedef std::vector<vec3f_t, Eigen::aligned_allocator<vec3f_t>> vec3fs_t;
typedef std::vector<vec4f_t, Eigen::aligned_allocator<vec4f_t>> vec4fs_t;
typedef std::vector<vec5f_t, Eigen::aligned_allocator<vec5f_t>> vec5fs_t;
typedef std::vector<vec6f_t, Eigen::aligned_allocator<vec6f_t>> vec6fs_t;
typedef std::vector<vecxf_t> vecxfs_t;

typedef std::vector<mat2_t, Eigen::aligned_allocator<mat2_t>> mat2s_t;
typedef std::vector<mat3_t, Eigen::aligned_allocator<mat3_t>> mat3s_t;
typedef std::vector<mat4_t, Eigen::aligned_allocator<mat4_t>> mat4s_t;
typedef std::vector<matx_t, Eigen::aligned_allocator<matx_t>> matxs_t;

typedef std::vector<mat2f_t, Eigen::aligned_allocator<mat2f_t>> mat2fs_t;
typedef std::vector<mat3f_t, Eigen::aligned_allocator<mat3f_t>> mat3fs_t;
typedef std::vector<mat4f_t, Eigen::aligned_allocator<mat4f_t>> mat4fs_t;
typedef std::vector<matxf_t, Eigen::aligned_allocator<matxf_t>> matxfs_t;

typedef Eigen::Quaterniond quat_t;
typedef std::vector<quat_t, Eigen::aligned_allocator<quat_t>> quats_t;

typedef Eigen::Quaternionf quatf_t;
typedef std::vector<quatf_t, Eigen::aligned_allocator<quat_t>> quatfs_t;
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
 * Print vector `v` with a `name`.
 */
void print_vector(const std::string &name, const vecx_t &v);

/**
 * Print matrix `m` with a `name`.
 */
void print_matrix(const std::string &name, const matx_t &m);

/**
 * Print quaternion `q` with a `name`.
 */
void print_quaternion(const std::string &name, const quat_t &q);

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
vec3s_t mat2vec3(const matx_t &m, const bool row_wise = true);

/**
 * Matrix to list of vectors of size 3
 *
 * @param[in] m Matrix
 * @param[in] row_wise Row wise
 * @returns Vectors
 */
vec2s_t mat2vec2(const matx_t &m, const bool row_wise = true);

/**
 * Vectors to matrix
 */
matx_t vecs2mat(const vec3s_t &vs);

/**
 * Vector to string
 *
 * @param[in] v Vector
 * @param[in] brackets Brakcets around vector string
 * @returns Vector as a string
 */
std::string vec2str(const vecx_t &v, const bool brackets = true);

/**
 * Array to string
 *
 * @param[in] arr Array
 * @param[in] len Length of array
 * @param[in] brackets Brakcets around vector string
 * @returns Array as a string
 */
std::string arr2str(const double *arr, const size_t len, bool brackets = true);

/**
 * Matrix to string
 *
 * @param[in] m Matrix
 * @param[in] indent Indent string
 * @returns Array as a string
 */
std::string mat2str(const matx_t &m, const std::string &indent = "  ");

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
 * Degrees to radians
 *
 * @param[in] d Degree to be converted
 * @return Degree in radians
 */
vec3_t deg2rad(const vec3_t d);

/**
 * Radians to degree
 *
 * @param[in] r Radian to be converted
 * @return Radian in degrees
 */
double rad2deg(const double r);

/**
 * Radians to degree
 *
 * @param[in] r Radian to be converted
 * @return Radian in degrees
 */
vec3_t rad2deg(const vec3_t &r);

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
double wrap180(const double d);

/**
 * Wrap angle in degrees to 360
 *
 * @param[in] d Degrees
 * @return Angle wraped to 360
 */
double wrap360(const double d);

/**
 * Wrap angle in radians to PI
 *
 * @param[in] r Radians
 * @return Angle wraped to PI
 */
double wrapPi(const double r);

/**
 * Wrap angle in radians to 2 PI
 *
 * @param[in] r Radians
 * @return Angle wraped to 2 PI
 */
double wrap2Pi(const double r);

/**
 * Mean vector
 *
 * @param[in] x List of vectors
 * @return Mean vector
 */
vec3_t mean(const vec3s_t &x);

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

/**
 * Normalize vector `v`.
 */
vec3_t normalize(const vec3_t &v);

/**
 * Zeros-matrix
 *
 * @param rows Number of rows
 * @param cols Number of cols
 * @returns Zeros matrix
 */
matx_t zeros(const int rows, const int cols);

/**
 * Zeros square matrix
 *
 * @param size Square size of matrix
 * @returns Zeros matrix
 */
matx_t zeros(const int size);

/**
 * Identity-matrix
 *
 * @param rows Number of rows
 * @param cols Number of cols
 * @returns Identity matrix
 */
matx_t I(const int rows, const int cols);

/**
 * Identity square matrix
 *
 * @param size Square size of matrix
 * @returns Identity square matrix
 */
matx_t I(const int size);

/**
 * Ones-matrix
 *
 * @param rows Number of rows
 * @param cols Number of cols
 * @returns Ones square matrix
 */
matx_t ones(const int rows, const int cols);

/**
 * Ones square matrix
 *
 * @param size Square size of matrix
 * @returns Ones square matrix
 */
matx_t ones(const int size);

/**
 * Horizontally stack matrices A and B
 *
 * @param A Matrix A
 * @param B Matrix B
 * @returns Stacked matrix
 */
matx_t hstack(const matx_t &A, const matx_t &B);

/**
 * Vertically stack matrices A and B
 *
 * @param A Matrix A
 * @param B Matrix B
 * @returns Stacked matrix
 */
matx_t vstack(const matx_t &A, const matx_t &B);

/**
 * Diagonally stack matrices A and B
 *
 * @param A Matrix A
 * @param B Matrix B
 * @returns Stacked matrix
 */
matx_t dstack(const matx_t &A, const matx_t &B);

/**
 * Skew symmetric-matrix
 *
 * @param w Input vector
 * @returns Skew symmetric matrix
 */
mat3_t skew(const vec3_t &w);

/**
 * Skew symmetric-matrix squared
 *
 * @param w Input vector
 * @returns Skew symmetric matrix squared
 */
mat3_t skewsq(const vec3_t &w);

/**
 * Enforce Positive Semi-Definite
 *
 * @param A Input matrix
 * @returns Positive semi-definite matrix
 */
matx_t enforce_psd(const matx_t &A);

/**
 * Null-space of A
 *
 * @param A Input matrix
 * @returns Null space of A
 */
matx_t nullspace(const matx_t &A);

/**
 * Return evenly spaced numbers over a specified interval.
 */
std::vector<double> linspace(const double start,
                             const double end,
                             const double num);

/**
 * Create look at matrix.
 */
mat4_t lookat(const vec3_t &cam_pos,
              const vec3_t &target,
              const vec3_t &up_axis=vec3_t{0.0, -1.0, 0.0});

/**
 * Shannon Entropy of a given covariance matrix `covar`.
 */
double shannon_entropy(const matx_t &covar);

} //  namespace proto
#endif // PROTOTYPE_CORE_MATH_HPP
