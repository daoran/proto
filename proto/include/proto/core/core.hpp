#ifndef PROTO_CORE_HPP
#define PROTO_CORE_HPP

#define ENABLE_MACROS 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>
#include <dirent.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <random>
#include <type_traits>
#include <set>
#include <list>
#include <deque>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/Splines>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/******************************************************************************
 * MACROS
 *****************************************************************************/
#ifdef ENABLE_MACROS

#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_ERROR(M, ...)                                                      \
  fprintf(stderr,                                                              \
          "\033[31m[ERROR] [%s:%d] " M "\033[0m\n",                            \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__)

#define LOG_INFO(M, ...) fprintf(stdout, "[INFO] " M "\n", ##__VA_ARGS__)
#define LOG_WARN(M, ...)                                                       \
  fprintf(stdout, "\033[33m[WARN] " M "\033[0m\n", ##__VA_ARGS__)

#define FATAL(M, ...)                                                          \
  fprintf(stdout,                                                              \
          "\033[31m[FATAL] [%s:%d] " M "\033[0m\n",                            \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__);                                                      \
  exit(-1)

#ifdef NDEBUG
#define DEBUG(M, ...)
#else
#define DEBUG(M, ...) fprintf(stdout, "[DEBUG] " M "\n", ##__VA_ARGS__)
#endif

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

#ifndef CHECK
#define CHECK(A, M, ...)                                                       \
  if (!(A)) {                                                                  \
    LOG_ERROR(M, ##__VA_ARGS__);                                               \
    goto error;                                                                \
  }
#endif

#endif  // ENABLE_MACROS ------------------------------------------------------

namespace proto {

#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__

#define col_major_t Eigen::ColMajor
#define row_major_t Eigen::RowMajor

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

template <int ROWS, int COLS, Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using mat_t = Eigen::Matrix<double, ROWS, COLS, STRIDE_TYPE>;

template <int ROWS, int COLS, Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using map_mat_t = Eigen::Map<Eigen::Matrix<double, ROWS, COLS, STRIDE_TYPE>>;

template <int ROWS>
using map_vec_t = Eigen::Map<Eigen::Matrix<double, ROWS, 1>>;
#endif

/******************************************************************************
 * FILESYSTEM
 *****************************************************************************/

/**
 * Open file in `path` with `mode` and set `nb_rows`.
 * @returns File pointer on success, nullptr on failure.
 */
FILE *file_open(const std::string &path,
                const std::string &mode,
                int *nb_rows = nullptr);

/**
 * Skip file line in file `fp`.
 */
void skip_line(FILE *fp);

/**
 * Get number of rows in file.
 * @returns Number of rows in file else -1 for failure.
 */
int file_rows(const std::string &file_path);

/**
 * Copy file from path `src` to path `dest.
 *
 * @returns 0 for success else -1 if `src` file could not be opend, or -2 if
 * `dest` file could not be opened.
 */
int file_copy(const std::string &src, const std::string &dest);

/**
 * Return file extension in `path`.
 */
std::string file_ext(const std::string &path);

/**
 * Return basename
 */
std::string basename(const std::string &path);

/**
 * Check if file exists
 *
 * @param path Path to file
 * @returns true or false
 */
bool file_exists(const std::string &path);

/**
 * Check if path exists
 *
 * @param path Path
 * @returns true or false
 */
bool dir_exists(const std::string &path);

/**
 * Create directory
 *
 * @param path Path
 * @returns 0 for success, -1 for failure
 */
int dir_create(const std::string &path);

/**
 * Return directory name
 *
 * @param path Path
 * @returns directory name
 */
std::string dir_name(const std::string &path);

/**
 * Strips a target character from the start and end of a string
 *
 * @param s String to strip
 * @param target Target character to strip
 * @returns Stripped string
 */
std::string strip(const std::string &s, const std::string &target = " ");

/**
 * Strips a target character from the end of a string
 *
 * @param s String to strip
 * @param target Target character to strip
 * @returns Stripped string
 */
std::string strip_end(const std::string &s, const std::string &target = " ");

/**
 * Create directory
 *
 * @param path Path to directory
 * @returns 0 for success, -1 for failure
 */
int create_dir(const std::string &path);

/**
 * Remove directory
 *
 * @param path Path to directory
 * @returns 0 for success, -1 for failure
 */
int remove_dir(const std::string &path);

/**
 * Remove file extension
 *
 * @param path Path to directory
 * @returns File path without extension
 */
std::string remove_ext(const std::string &path);

/**
 * List directory
 *
 * @param path Path to directory
 * @param results List of files and directories
 * @returns 0 for success, -1 for failure
 */
int list_dir(const std::string &path, std::vector<std::string> &results);

/**
 * Split path into a number of elements
 *
 * @param path Path
 * @returns List of path elements
 */
std::vector<std::string> path_split(const std::string path);

/**
 * Combine `path1` and `path2`
 *
 * @param path1 Path 1
 * @param path2 Path 22
 * @returns Combined path
 */
std::string paths_combine(const std::string path1, const std::string path2);

/******************************************************************************
 * Algebra
 *****************************************************************************/

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
 * Calculate binomial coefficient
 *
 * @param[in] n
 * @param[in] k
 * @returns Binomial coefficient
 */
double binomial(const double n, const double k);

/**
 * Return evenly spaced numbers over a specified interval.
 */
template <typename T>
std::vector<T> linspace(const T start,
                        const T end,
                        const int num) {
  std::vector<T> linspaced;

  if (num == 0) {
    return linspaced;
  }
  if (num == 1) {
    linspaced.push_back(start);
    return linspaced;
  }

  const double diff = static_cast<double>(end - start);
  const double delta = diff / static_cast<double>(num - 1);
  for (int i = 0; i < num - 1; ++i) {
    linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);
  return linspaced;
}

/******************************************************************************
 * Geometry
 *****************************************************************************/

/**
 * Sinc function.
 */
double sinc(const double x);

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
 * Create a circle point of radius `r` at angle `theta` radians.
 */
vec2_t circle(const double r, const double theta);

/**
 * Create the sphere point with sphere radius `rho` at longitude `theta`
 * [radians] and latitude `phi` [radians].
 */
vec3_t sphere(const double rho,
              const double theta,
              const double phi);

/**
 * Create look at matrix.
 */
mat4_t lookat(const vec3_t &cam_pos,
              const vec3_t &target,
              const vec3_t &up_axis = vec3_t{0.0, -1.0, 0.0});

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
 * Linear interpolation between two points.
 *
 * @param[in] a First point
 * @param[in] b Second point
 * @param[in] t Unit number
 * @returns Linear interpolation
 */
template <typename T>
T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

#define EARTH_RADIUS_M 6378137.0

/**
 * Calculate new latitude and logitude coordinates with an offset in North and
 * East direction.
 *
 * IMPORTANT NOTE: This function is only an approximation. As such do not rely
 * on this function for precise latitude, longitude offsets.
 *
 * @param lat_ref Latitude of origin (decimal format)
 * @param lon_ref Longitude of origin (decimal format)
 * @param offset_N Offset in North direction (meters)
 * @param offset_E Offset in East direction (meters)
 * @param lat_new New latitude (decimal format)
 * @param lon_new New longitude (decimal format)
 */
void latlon_offset(double lat_ref,
                   double lon_ref,
                   double offset_N,
                   double offset_E,
                   double *lat_new,
                   double *lon_new);

/**
 * Calculate difference in distance in North and East from two GPS coordinates
 *
 * IMPORTANT NOTE: This function is only an approximation. As such do not rely
 * on this function for precise latitude, longitude diffs.
 *
 * @param lat_ref Latitude of origin (decimal format)
 * @param lon_ref Longitude of origin (decimal format)
 * @param lat Latitude of point of interest (decimal format)
 * @param lon Longitude of point of interest (decimal format)
 * @param dist_N Distance of point of interest in North axis [m]
 * @param dist_E Distance of point of interest in East axis [m]
 */
void latlon_diff(double lat_ref,
                 double lon_ref,
                 double lat,
                 double lon,
                 double *dist_N,
                 double *dist_E);

/**
 * Calculate Euclidean distance between two GPS coordintes
 *
 * IMPORTANT NOTE: This function is only an approximation. As such do not rely
 * on this function for precise latitude, longitude distance.
 *
 * @param lat_ref Latitude of origin (decimal format)
 * @param lon_ref Longitude of origin (decimal format)
 * @param lat Latitude of point of interest (decimal format)
 * @param lon Longitude of point of interest (decimal format)
 *
 * @returns Euclidean distance between two GPS coordinates [m]
 */
double latlon_dist(double lat_ref, double lon_ref, double lat, double lon);

/******************************************************************************
 * Linear Algebra
 *****************************************************************************/

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

/******************************************************************************
 * Statistics
 *****************************************************************************/

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
 * Calculate median given an array of numbers
 *
 * @param[in] v Array of numbers
 * @return Median of given array
 */
double median(const std::vector<double> &v);

/**
 * Mean vector
 *
 * @param[in] x List of vectors
 * @return Mean vector
 */
vec3_t mean(const vec3s_t &x);

/**
 * Shannon Entropy of a given covariance matrix `covar`.
 */
double shannon_entropy(const matx_t &covar);

/**
 * Multivariate normal.
 */
vec3_t mvn(std::default_random_engine &engine,
           const vec3_t &mu = vec3_t{0.0, 0.0, 0.0},
           const vec3_t &stdev = vec3_t{1.0, 1.0, 1.0});

/**
 * Gassian normal.
 * http://c-faq.com/lib/gaussian.html
 */
double gauss_normal();

/*****************************************************************************
 * Transform
 *****************************************************************************/

/**
 * Extract rotation from transform
 */
inline mat3_t tf_rot(const mat4_t &tf) { return tf.block<3, 3>(0, 0); }

/**
 * Extract rotation and convert to quaternion from transform
 */
inline quat_t tf_quat(const mat4_t &tf) { return quat_t{tf.block<3, 3>(0, 0)}; }

/**
 * Extract translation from transform
 */
inline vec3_t tf_trans(const mat4_t &tf) { return tf.block<3, 1>(0, 3); }

/**
 * Form a 4x4 homogeneous transformation matrix from a
 * rotation matrix `C` and translation vector `r`.
 */
template <typename T>
Eigen::Matrix<T, 4, 4> tf(const Eigen::Matrix<T, 3, 3> &C,
                          const Eigen::Matrix<T, 3, 1> &r) {
  Eigen::Matrix<T, 4, 4> transform = Eigen::Matrix<T, 4, 4>::Identity();
  transform.block(0, 0, 3, 3) = C;
  transform.block(0, 3, 3, 1) = r;
  return transform;
}

/**
 * Form a 4x4 homogeneous transformation matrix from a
 * rotation matrix `C` and translation vector `r`.
 */
mat4_t tf(const mat3_t &C, const vec3_t &r);

/**
 * Form a 4x4 homogeneous transformation matrix from a
 * Hamiltonian quaternion `q` and translation vector `r`.
 */
mat4_t tf(const quat_t &q, const vec3_t &r);

/**
 * Perturb the rotation element in the tranform `T` by `step_size` at index
 * `i`. Where i = 0 for x-axis, i = 1 for y-axis, and i = 2 for z-axis.
 */
mat4_t tf_perturb_rot(const mat4_t &T, double step_size, const int i);

/**
 * Perturb the translation element in the tranform `T` by `step_size` at index
 * `i`. Where i = 0 for x-axis, i = 1 for y-axis, and i = 2 for z-axis.
 */
mat4_t tf_perturb_trans(const mat4_t &T, double step_size, const int i);

/**
 * Rotation matrix around x-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
mat3_t rotx(const double theta);

/**
 * Rotation matrix around y-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
mat3_t roty(const double theta);

/**
 * Rotation matrix around z-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
mat3_t rotz(const double theta);

/**
 * Convert euler sequence 123 to rotation matrix R
 * This function assumes we are performing a body fixed intrinsic rotation.
 *
 * Source:
 *
 *     Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
 *     Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
 *     Princeton University Press, 1999. Print.
 *
 *     Page 86.
 *
 * @returns Rotation matrix
 */
mat3_t euler123(const vec3_t &euler);

/**
 * Convert euler sequence 321 to rotation matrix R
 * This function assumes we are performing a body fixed intrinsic rotation.
 *
 * Source:
 *
 *     Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
 *     Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
 *     Princeton University Press, 1999. Print.
 *
 *     Page 86.
 *
 * @returns Rotation matrix
 */
mat3_t euler321(const vec3_t &euler);

/**
 * Convert roll, pitch and yaw to quaternion.
 */
quat_t euler2quat(const vec3_t &euler);

/**
 * Convert rotation vectors to rotation matrix using measured acceleration
 * `a_m` from an IMU and gravity vector `g`.
 */
mat3_t vecs2rot(const vec3_t &a_m, const vec3_t &g);

/**
 * Convert rotation vector `rvec` to rotation matrix.
 */
mat3_t rvec2rot(const vec3_t &rvec, const double eps=1e-5);

/**
 * Convert quaternion to euler angles.
 */
vec3_t quat2euler(const quat_t &q);

/**
 * Initialize attitude using IMU gyroscope `w_m` and accelerometer `a_m`
 * measurements. The calculated attitude outputted into to `C_WS`. Note: this
 * function does not calculate initial yaw angle in the world frame. Only the
 * roll, and pitch are inferred from IMU measurements.
 */
void imu_init_attitude(const vec3s_t w_m,
                       const vec3s_t a_m,
                       mat3_t &C_WS,
                       const size_t buffer_size = 50);

/*****************************************************************************
 * Time
 *****************************************************************************/

typedef uint64_t timestamp_t;
typedef std::vector<timestamp_t> timestamps_t;

/**
 * Print timestamp.
 */
void timestamp_print(const timestamp_t &ts, const std::string &prefix = "");

/**
 * Convert ts to second.
 */
double ts2sec(const timestamp_t &ts);

/**
 * Convert nano-second to second.
 */
double ns2sec(const uint64_t ns);

/**
 * Start timer.
 */
struct timespec tic();

/**
 * Stop timer and return number of seconds.
 */
float toc(struct timespec *tic);

/**
 * Stop timer and return miliseconds elasped.
 */
float mtoc(struct timespec *tic);

/**
 * Get time now in milliseconds since epoch
 */
double time_now();

/******************************************************************************
 * DATA
 *****************************************************************************/

/**
 * Convert bytes to signed 8bit number
 */
int8_t int8(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to unsigned 8bit number
 */
uint8_t uint8(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to signed 16bit number
 */
int16_t int16(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to unsigned 16bit number
 */
uint16_t uint16(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to signed 32bit number
 */
int32_t int32(const uint8_t *data, const size_t offset);

/**
 * Convert bytes to unsigned 32bit number
 */
uint32_t uint32(const uint8_t *data, const size_t offset);

/**
 * Get number of rows in CSV file.
 * @returns Number of rows in CSV file else -1 for failure.
 */
int csvrows(const std::string &file_path);

/**
 * Get number of columns in CSV file.
 * @returns Number of columns in CSV file else -1 for failure.
 */
int csvcols(const std::string &file_path);

/**
 * Convert CSV file to matrix.
 * @returns 0 for success, -1 for failure
 */
int csv2mat(const std::string &file_path, const bool header, matx_t &data);

/**
 * Convert matrix to csv file.
 * @returns 0 for success, -1 for failure
 */
int mat2csv(const std::string &file_path, const matx_t &data);

/**
 * Convert vector to csv file.
 * @returns 0 for success, -1 for failure
 */
int vec2csv(const std::string &file_path, const std::deque<vec3_t> &data);

/**
 * Convert timestamps to csv file.
 * @returns 0 for success, -1 for failure
 */
int ts2csv(const std::string &file_path, const std::deque<timestamp_t> &data);

/**
 * Print progress to screen
 */
void print_progress(const double percentage);

/**
 * Slerp
 */
quat_t slerp(const quat_t &q_start, const quat_t &q_end, const double alpha);

/**
 * Interpolate between two poses `p0` and `p1` with parameter `alpha`.
 */
mat4_t interp_pose(const mat4_t &p0, const mat4_t &p1, const double alpha);

/**
 * Interpolate `poses` where each pose has a timestamp in `timestamps` and the
 * interpolation points in time are in `interp_ts`. The results are recorded
 * in `interp_poses`.
 * @returns 0 for success, -1 for failure
 */
void interp_poses(const timestamps_t &timestamps,
                  const mat4s_t &poses,
                  const timestamps_t &interp_ts,
                  mat4s_t &interped_poses,
                  const double threshold = 0.001);

/**
 * Get the closest pose in `poses` where each pose has a timestamp in
 * `timestamps` and the target points in time are in `target_ts`. The results
 * are recorded in `result`.
 * @returns 0 for success, -1 for failure
 */
void closest_poses(const timestamps_t &timestamps,
                   const mat4s_t &poses,
                   const timestamps_t &interp_ts,
                   mat4s_t &result);

/**
 * Check if vector `x` is all true.
 */
bool all_true(const std::vector<bool> x);

/**
 * Pop front of an `std::vector`.
 */
template <typename T>
void pop_front(std::vector<T> &vec) {
  assert(!vec.empty());
  vec.front() = std::move(vec.back());
  vec.pop_back();
}

/**
 * Pop front of an `std::vector`.
 */
template <typename T1, typename T2>
void pop_front(std::vector<T1, T2> &vec) {
  assert(!vec.empty());
  vec.front() = std::move(vec.back());
  vec.pop_back();
}

/**
 * Extend `std::vector`.
 */
template <typename T>
void extend(std::vector<T> &x, std::vector<T> &add) {
  x.reserve(x.size() + add.size());
  x.insert(x.end(), add.begin(), add.end());
}

/**
 * Extend `std::vector`.
 */
template <typename T1, typename T2>
void extend(std::vector<T1, T2> &x, std::vector<T1, T2> &add) {
  x.reserve(x.size() + add.size());
  x.insert(x.end(), add.begin(), add.end());
}

/**
 * Get raw pointer of a value in a `std::map`.
 */
template< typename K, typename V>
const V* lookup(const std::map<K, V> & map, K key) {
  typename std::map<K, V>::const_iterator iter = map.find(key);
  if (iter != map.end()) {
    return &iter->second;
  } else {
    return nullptr;
  }
}

/**
 * Get raw pointer of a value in a `std::map`.
 */
template< typename K, typename V>
V* lookup(std::map<K, V> &map, K key) {
  return const_cast<V*>(lookup(const_cast<const std::map<K, V> &>(map), key));
}

/**
 * Get raw pointer of a value in a `std::map`.
 */
template< typename K, typename V>
const V* lookup(const std::unordered_map<K, V> & map, K key) {
  typename std::unordered_map<K, V>::const_iterator iter = map.find(key);
  if (iter != map.end()) {
    return &iter->second;
  } else {
    return nullptr;
  }
}

/**
 * Get raw pointer of a value in a `std::map`.
 */
template< typename K, typename V>
V* lookup(std::unordered_map<K, V> &map, K key) {
  return const_cast<V*>(
    lookup(const_cast<const std::unordered_map<K, V> &>(map), key));
}

/**
 * Union between set `a` and set `b`.
 */
template <typename T>
T set_union(const T &s1, const T &s2) {
  T result = s1;
  result.insert(s2.begin(), s2.end());
  return result;
}

/**
 * Difference between `a` and set `b`.
 */
template <typename T>
T set_diff(const T &a, const T &b) {
  T results;
  std::set_difference(a.begin(),
                      a.end(),
                      b.begin(),
                      b.end(),
                      std::inserter(results, results.end()));
  return results;
}

/**
 * Symmetric difference between `a` and `b`.
 */
template <typename T>
T set_symmetric_diff(const T &a, const T &b) {
  T results;
  std::set_symmetric_difference(a.begin(),
                                a.end(),
                                b.begin(),
                                b.end(),
                                std::back_inserter(results));
  return results;
}

/**
 * Intersection between std::vectors `vecs`.
 * @returns Number of common elements
 */
template <typename T>
std::set<T> intersection(const std::list<std::vector<T>> &vecs) {
  // Obtain element count across all vectors
  std::unordered_map<T, size_t> counter;
  for (const auto &vec : vecs) { // Loop over all vectors
    for (const auto &p : vec) {  // Loop over elements in vector
      counter[p] += 1;
    }
  }

  // Build intersection result
  std::set<T> retval;
  for (const auto &el : counter) {
    if (el.second == vecs.size()) {
      retval.insert(el.first);
    }
  }

  return retval;
}

/******************************************************************************
 * CONFIG
 *****************************************************************************/

struct config_t {
  std::string file_path;
  YAML::Node root;
  bool ok = false;

  config_t();
  config_t(const std::string &file_path_);
  ~config_t();
};

/**
 * Load YAML file.
 * @returns 0 for success or -1 for failure.
 */
int yaml_load_file(const std::string file_path, YAML::Node &root);

/**
 * Get YAML node containing the parameter value.
 * @returns 0 for success or -1 for failure.
 */
int yaml_get_node(const config_t &config,
                  const std::string &key,
                  const bool optional,
                  YAML::Node &node);

/**
 * Check if yaml file has `key`.
 * @returns 0 for success or -1 for failure.
 */
int yaml_has_key(const config_t &config, const std::string &key);

/**
 * Check if yaml file has `key`.
 * @returns 0 for success or -1 for failure.
 */
int yaml_has_key(const std::string &file_path, const std::string &key);

/**
 * Check size of vector in config file and returns the size.
 */
template <typename T>
size_t yaml_check_vector(const YAML::Node &node,
                         const std::string &key,
                         const bool optional);

/**
 * Check matrix fields.
 */
void yaml_check_matrix_fields(const YAML::Node &node,
                              const std::string &key,
                              size_t &rows,
                              size_t &cols);

/**
 * Check matrix to make sure that the parameter has the data field "rows",
 * "cols" and "data". It also checks to make sure the number of values is the
 * same size as the matrix.
 */
template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional,
                       size_t &rows,
                       size_t &cols);

template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional);

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          T &out,
          const bool optional=false);

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          std::vector<T> &out,
          const bool optional);

int parse(const config_t &config,
          const std::string &key,
          vec2_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          vec3_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          vec4_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          vecx_t &vec,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          mat2_t &mat,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          mat3_t &mat,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          mat4_t &mat,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          matx_t &mat,
          const bool optional = false);

int parse(const config_t &config,
          const std::string &key,
          cv::Mat &mat,
          const bool optional = false);

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          const int rows,
          const int cols,
          T &mat,
          const bool optional = false);

////////// CONFIG IMPLEMENTATION

template <typename T>
size_t yaml_check_vector(const YAML::Node &node,
                         const std::string &key,
                         const bool optional) {
  UNUSED(optional);
  assert(node);

  // Get expected vector size
  size_t vector_size = 0;
  if (std::is_same<T, vec2_t>::value) {
    vector_size = 2;
  } else if (std::is_same<T, vec3_t>::value) {
    vector_size = 3;
  } else if (std::is_same<T, vec4_t>::value) {
    vector_size = 4;
  } else if (std::is_same<T, vec5_t>::value) {
    vector_size = 5;
  } else if (std::is_same<T, vecx_t>::value) {
    vector_size = node.size();
    return vector_size; // Don't bother, it could be anything
  } else {
    FATAL("Unsportted vector type!");
  }

  // Check number of values in the param
  if (node.size() == 0 && node.size() != vector_size) {
    FATAL("Vector [%s] should have %d values but config has %d!",
          key.c_str(),
          static_cast<int>(vector_size),
          static_cast<int>(node.size()));
  }

  return vector_size;
}

template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional,
                       size_t &rows,
                       size_t &cols) {
  UNUSED(optional);
  assert(node);
  yaml_check_matrix_fields(node, key, rows, cols);

  // Check number of elements
  size_t nb_elements = 0;
  if (std::is_same<T, mat2_t>::value) {
    nb_elements = 4;
  } else if (std::is_same<T, mat3_t>::value) {
    nb_elements = 9;
  } else if (std::is_same<T, mat4_t>::value) {
    nb_elements = 16;
  } else if (std::is_same<T, matx_t>::value) {
    nb_elements = node["data"].size();

  } else if (std::is_same<T, cv::Mat>::value) {
    nb_elements = node["data"].size();
  } else {
    FATAL("Unsportted matrix type!");
  }
  if (node["data"].size() != nb_elements) {
    FATAL("Matrix [%s] rows and cols do not match number of values!",
          key.c_str());
  }
}

template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional) {
  size_t rows;
  size_t cols;
  yaml_check_matrix<T>(node, key, optional, rows, cols);
}

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          T &out,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  out = node.as<T>();
	return 0;
}

template <typename T>
int parse(const config_t &config,
          const std::string &key,
          std::vector<T> &out,
          const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return -1;
  }

  // Parse
  std::vector<T> array;
  for (auto n : node) {
    out.push_back(n.as<T>());
  }

	return 0;
}

/******************************************************************************
 * SPLINE
 *****************************************************************************/

typedef Eigen::Spline<double, 1> Spline1D;
typedef Eigen::Spline<double, 2> Spline2D;
typedef Eigen::Spline<double, 3> Spline3D;

#define SPLINE1D(X, Y, DEG) \
  Eigen::SplineFitting<Spline1D>::Interpolate(X, DEG, Y)

#define SPLINE2D(X, Y, DEG) \
  Eigen::SplineFitting<Spline2D>::Interpolate(X, DEG, Y)

#define SPLINE3D(X, Y, DEG) \
  Eigen::SplineFitting<Spline3D>::Interpolate(X, DEG, Y)

/**
 * Continuous trajectory generator
 */
struct ctraj_t {
  const timestamps_t timestamps;
  const vec3s_t positions;
  const quats_t orientations;

  const double ts_s_start;
  const double ts_s_end;
  const double ts_s_gap;

  Spline3D pos_spline;
  Spline3D rvec_spline;

  ctraj_t(const timestamps_t &timestamps,
          const vec3s_t &positions,
          const quats_t &orientations);
};

/**
 * Container for multiple continuous trajectories
 */
typedef std::vector<ctraj_t> ctrajs_t;

/**
 * Initialize continuous trajectory.
 */
void ctraj_init(ctraj_t &ctraj);

/**
 * Calculate pose `T_WB` at timestamp `ts`.
 */
mat4_t ctraj_get_pose(const ctraj_t &ctraj, const timestamp_t ts);

/**
 * Calculate velocity `v_WB` at timestamp `ts`.
 */
vec3_t ctraj_get_velocity(const ctraj_t &ctraj, const timestamp_t ts);

/**
 * Calculate acceleration `a_WB` at timestamp `ts`.
 */
vec3_t ctraj_get_acceleration(const ctraj_t &ctraj, const timestamp_t ts);

/**
 * Calculate angular velocity `w_WB` at timestamp `ts`.
 */
vec3_t ctraj_get_angular_velocity(const ctraj_t &ctraj, const timestamp_t ts);

/**
 * Save trajectory to file
 */
int ctraj_save(const ctraj_t &ctraj, const std::string &save_path);

/**
 * SIM IMU
 */
struct sim_imu_t {
  // IMU parameters
  double rate = 0.0;        // IMU rate [Hz]
  double tau_a = 0.0;       // Reversion time constant for accel [s]
  double tau_g = 0.0;       // Reversion time constant for gyro [s]
  double sigma_g_c = 0.0;   // Gyro noise density [rad/s/sqrt(Hz)]
  double sigma_a_c = 0.0;   // Accel noise density [m/s^s/sqrt(Hz)]
  double sigma_gw_c = 0.0;  // Gyro drift noise density [rad/s^s/sqrt(Hz)]
  double sigma_aw_c = 0.0;  // Accel drift noise density [m/s^2/sqrt(Hz)]
  double g = 0.0;           // Gravity vector [ms-2]

  // IMU flags and biases
  bool started = false;
  vec3_t b_g = zeros(3, 1);
  vec3_t b_a = zeros(3, 1);
  timestamp_t ts_prev = 0;
};

/**
 * Reset IMU
 */
void sim_imu_reset(sim_imu_t &imu);

/**
 * Simulate IMU measurement
 */
void sim_imu_measurement(
    sim_imu_t &imu,
    std::default_random_engine &rndeng,
    const timestamp_t &ts,
    const mat4_t &T_WS_W,
    const vec3_t &w_WS_W,
    const vec3_t &a_WS_W,
    vec3_t &a_WS_S,
    vec3_t &w_WS_S);

/******************************************************************************
 * Measurements
 *****************************************************************************/

struct meas_t {
  timestamp_t ts = 0;

  meas_t() {}
  meas_t(const timestamp_t &ts_) : ts{ts_} {}
  virtual ~meas_t() {}
};

struct imu_data_t : meas_t {
  vec3_t gyro;
  vec3_t accel;

  imu_data_t() {}
  imu_data_t(const vec3_t &gyro_, const vec3_t &accel_) :
    gyro{gyro_}, accel{accel_} {}
  virtual ~imu_data_t() {}
};

struct image_t : meas_t {
  timestamp_t ts = 0;
  int width = 0;
  int height = 0;
  float *data = nullptr;

  image_t() {}
  image_t(const timestamp_t ts_, const int width_, const int height_)
      : ts{ts_}, width{width_}, height{height_} {
    data = new float[width * height];
  }
  image_t(const timestamp_t ts_,
                  const int width_,
                  const int height_,
                  float *data_)
      : ts{ts_}, width{width_}, height{height_}, data{data_} {}
  virtual ~image_t() {
    if (data) {
      free(data);
    }
  }
};

} //  namespace proto
#endif // PROTO_CORE_HPP
