#ifndef PROTO_CORE_HPP
#define PROTO_CORE_HPP

/*****************************************************************************
 * This file is huge, it contains everything from parsing yaml files, linear
 * algebra functions to networking code used for robotics.
 *
 * Contents:
 * - Macros
 * - Filesystem
 * - Algebra
 * - Linear Algebra
 * - Geometry
 * - Differential Geometry
 * - Statistics
 * - Transform
 * - Time
 * - Factor Graph
 * - Data
 * - Configuration
 * - Interpolation
 * - Spline
 * - Control
 * - Measurements
 * - Models
 * - Networking
 *
 ****************************************************************************/

#define ENABLE_MACROS 1

/* PRECISION TYPE */
#define PRECISION SINGLE
// #define PRECISION DOUBLE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>
#include <dirent.h>
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <random>
#include <set>
#include <list>
#include <deque>
#include <vector>
#include <unordered_map>
#include <type_traits>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/Splines>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/******************************************************************************
 *                                MACROS
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

#endif // ENABLE_MACROS ------------------------------------------------------

namespace proto {

/******************************************************************************
 *                                FILESYSTEM
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
std::string parse_fext(const std::string &path);

/**
 * Return basename
 */
std::string parse_fname(const std::string &path);

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
 *                                  ALGEBRA
 *****************************************************************************/

#if PRECISION == SINGLE
  #define real_t float
#elif PRECISION == DOUBLE
  #define real_t double
#else
  #define real_t double
#endif

/**
 * Sign of number
 *
 * @param[in] x Number to check sign
 * @return
 *    - 0: Number is zero
 *    - 1: Positive number
 *    - -1: Negative number
 */
int sign(const real_t x);

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
int fltcmp(const real_t f1, const real_t f2);

/**
 * Calculate binomial coefficient
 *
 * @param[in] n
 * @param[in] k
 * @returns Binomial coefficient
 */
real_t binomial(const real_t n, const real_t k);

/**
 * Return evenly spaced numbers over a specified interval.
 */
template <typename T>
std::vector<T> linspace(const T start, const T end, const int num) {
  std::vector<T> linspaced;

  if (num == 0) {
    return linspaced;
  }
  if (num == 1) {
    linspaced.push_back(start);
    return linspaced;
  }

  const real_t diff = static_cast<real_t>(end - start);
  const real_t delta = diff / static_cast<real_t>(num - 1);
  for (int i = 0; i < num - 1; ++i) {
    linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);
  return linspaced;
}

/******************************************************************************
 *                              LINEAR ALGEBRA
 *****************************************************************************/

#define col_major_t Eigen::ColMajor
#define row_major_t Eigen::RowMajor

typedef Eigen::Matrix<real_t, 2, 1> vec2_t;
typedef Eigen::Matrix<real_t, 3, 1> vec3_t;
typedef Eigen::Matrix<real_t, 4, 1> vec4_t;
typedef Eigen::Matrix<real_t, 5, 1> vec5_t;
typedef Eigen::Matrix<real_t, 6, 1> vec6_t;
typedef Eigen::Matrix<real_t, Eigen::Dynamic, 1> vecx_t;
typedef Eigen::Matrix<real_t, 2, 2> mat2_t;
typedef Eigen::Matrix<real_t, 3, 3> mat3_t;
typedef Eigen::Matrix<real_t, 4, 4> mat4_t;
typedef Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic> matx_t;
typedef Eigen::Matrix<real_t, 3, 4> mat34_t;
typedef Eigen::Quaternion<real_t> quat_t;
typedef Eigen::AngleAxis<real_t> angle_axis_t;
typedef Eigen::Matrix<real_t, 1, Eigen::Dynamic> row_vector_t;
typedef Eigen::Matrix<real_t, Eigen::Dynamic, 1> col_vector_t;
typedef Eigen::Array<real_t, Eigen::Dynamic, 1> arrayx_t;

typedef std::vector<vec2_t, Eigen::aligned_allocator<vec2_t>> vec2s_t;
typedef std::vector<vec3_t, Eigen::aligned_allocator<vec3_t>> vec3s_t;
typedef std::vector<vec4_t, Eigen::aligned_allocator<vec4_t>> vec4s_t;
typedef std::vector<vec5_t, Eigen::aligned_allocator<vec5_t>> vec5s_t;
typedef std::vector<vec6_t, Eigen::aligned_allocator<vec6_t>> vec6s_t;
typedef std::vector<vecx_t> vecxs_t;
typedef std::vector<mat2_t, Eigen::aligned_allocator<mat2_t>> mat2s_t;
typedef std::vector<mat3_t, Eigen::aligned_allocator<mat3_t>> mat3s_t;
typedef std::vector<mat4_t, Eigen::aligned_allocator<mat4_t>> mat4s_t;
typedef std::vector<matx_t, Eigen::aligned_allocator<matx_t>> matxs_t;
typedef std::vector<quat_t, Eigen::aligned_allocator<quat_t>> quats_t;

template <int LENGTH, Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using vec_t = Eigen::Matrix<real_t, LENGTH, 1, STRIDE_TYPE>;

template <int ROWS,
          int COLS,
          Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using mat_t = Eigen::Matrix<real_t, ROWS, COLS, STRIDE_TYPE>;

template <int ROWS,
          int COLS,
          Eigen::StorageOptions STRIDE_TYPE = Eigen::ColMajor>
using map_mat_t = Eigen::Map<Eigen::Matrix<real_t, ROWS, COLS, STRIDE_TYPE>>;

template <int ROWS>
using map_vec_t = Eigen::Map<Eigen::Matrix<real_t, ROWS, 1>>;

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
                 const real_t *array,
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
std::string array2str(const real_t *array, const size_t size);

/**
 * Convert real_t array to Eigen::Vector
 *
 * @param[in] x Input array
 * @param[in] size Size of input array
 * @param[out] y Output vector
 */
void array2vec(const real_t *x, const size_t size, vecx_t &y);

/**
 * Vector to array
 *
 * @param[in] v Vector
 * @returns Array
 */
real_t *vec2array(const vecx_t &v);

/**
 * Matrix to array
 *
 * @param[in] m Matrix
 * @returns Array
 */
real_t *mat2array(const matx_t &m);

/**
 * Quaternion to array
 *
 * *VERY IMPORTANT*: The returned array is (x, y, z, w).
 *
 * @param[in] q Quaternion
 * @returns Array
 */
real_t *quat2array(const quat_t &q);

/**
 * Vector to array
 *
 * @param[in] v Vector
 * @param[out] out Output array
 */
void vec2array(const vecx_t &v, real_t *out);

/**
 * Matrix to array
 *
 * @param[in] m Matrix
 * @param[in] out Output array
 */
void mat2array(const matx_t &m, real_t *out);

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
std::string arr2str(const real_t *arr, const size_t len, bool brackets = true);

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
 * Load std::vector of real_ts to an Eigen::Matrix
 *
 * @param[in] x Matrix values
 * @param[in] rows Number of matrix rows
 * @param[in] cols Number of matrix colums
 * @param[out] y Output matrix
 */
void load_matrix(const std::vector<real_t> &x,
                 const int rows,
                 const int cols,
                 matx_t &y);

/**
 * Load an Eigen::Matrix into a std::vector of real_ts
 *
 * @param[in] A Matrix
 * @param[out] x Output vector of matrix values
 */
void load_matrix(const matx_t A, std::vector<real_t> &x);

/******************************************************************************
 *                                 Geometry
 *****************************************************************************/

/**
 * Sinc function.
 */
real_t sinc(const real_t x);

/**
 * Degrees to radians
 *
 * @param[in] d Degree to be converted
 * @return Degree in radians
 */
real_t deg2rad(const real_t d);

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
real_t rad2deg(const real_t r);

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
real_t wrap180(const real_t d);

/**
 * Wrap angle in degrees to 360
 *
 * @param[in] d Degrees
 * @return Angle wraped to 360
 */
real_t wrap360(const real_t d);

/**
 * Wrap angle in radians to PI
 *
 * @param[in] r Radians
 * @return Angle wraped to PI
 */
real_t wrapPi(const real_t r);

/**
 * Wrap angle in radians to 2 PI
 *
 * @param[in] r Radians
 * @return Angle wraped to 2 PI
 */
real_t wrap2Pi(const real_t r);

/**
 * Create a circle point of radius `r` at angle `theta` radians.
 */
vec2_t circle(const real_t r, const real_t theta);

/**
 * Create the sphere point with sphere radius `rho` at longitude `theta`
 * [radians] and latitude `phi` [radians].
 */
vec3_t sphere(const real_t rho, const real_t theta, const real_t phi);

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
real_t cross_track_error(const vec2_t &p1, const vec2_t &p2, const vec2_t &pos);

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
real_t closest_point(const vec2_t &p1,
                     const vec2_t &p2,
                     const vec2_t &p3,
                     vec2_t &closest);

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
void latlon_offset(real_t lat_ref,
                   real_t lon_ref,
                   real_t offset_N,
                   real_t offset_E,
                   real_t *lat_new,
                   real_t *lon_new);

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
void latlon_diff(real_t lat_ref,
                 real_t lon_ref,
                 real_t lat,
                 real_t lon,
                 real_t *dist_N,
                 real_t *dist_E);

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
real_t latlon_dist(real_t lat_ref, real_t lon_ref, real_t lat, real_t lon);

/*****************************************************************************
 *                         DIFFERENTIAL GEOMETRY
 *****************************************************************************/

namespace lie {

mat3_t Exp(const vec3_t &phi);
vec3_t Log(const mat3_t &C);
mat3_t Jr(const vec3_t &psi);

} // namespace lie


/******************************************************************************
 *                                STATISTICS
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
 * Create random real_t
 *
 * @param[in] ub Upper bound
 * @param[in] lb Lower bound
 * @return Random floating point
 */
real_t randf(const real_t ub, const real_t lb);

/**
 * Calculate median given an array of numbers
 *
 * @param[in] v Array of numbers
 * @return Median of given array
 */
real_t median(const std::vector<real_t> &v);

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
real_t shannon_entropy(const matx_t &covar);

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
real_t gauss_normal();

/*****************************************************************************
 *                               TRANSFORM
 *****************************************************************************/

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
 * Form a 4x4 homogeneous transformation matrix from a pointer to real_t array
 * containing (quaternion + translation) 7 elements: (qw, qx, qy, qz, x, y, z)
 */
mat4_t tf(const real_t *params);

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
 * Perturb the rotation element in the tranform `T` by `step_size` at index
 * `i`. Where i = 0 for x-axis, i = 1 for y-axis, and i = 2 for z-axis.
 */
mat4_t tf_perturb_rot(const mat4_t &T, real_t step_size, const int i);

/**
 * Perturb the translation element in the tranform `T` by `step_size` at index
 * `i`. Where i = 0 for x-axis, i = 1 for y-axis, and i = 2 for z-axis.
 */
mat4_t tf_perturb_trans(const mat4_t &T, real_t step_size, const int i);

/**
 * Transform point `p` with transform `T`.
 */
vec3_t tf_point(const mat4_t &T, const vec3_t &p);

/**
 * Rotation matrix around x-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
mat3_t rotx(const real_t theta);

/**
 * Rotation matrix around y-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
mat3_t roty(const real_t theta);

/**
 * Rotation matrix around z-axis (counter-clockwise, right-handed).
 * @returns Rotation matrix
 */
mat3_t rotz(const real_t theta);

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
mat3_t rvec2rot(const vec3_t &rvec, const real_t eps = 1e-5);

/**
 * Convert quaternion to euler angles.
 */
vec3_t quat2euler(const quat_t &q);

/**
 * Convert quaternion to rotation matrix.
 */
mat3_t quat2rot(const quat_t &q);

/**
 * Convert small angle euler angle to quaternion.
 */
quat_t quat_delta(const vec3_t &dalpha);

/**
 * Left quaternion product matrix.
 */
mat4_t quat_lmul(const quat_t &q);

/**
 * Right quaternion product matrix.
 */
mat4_t quat_rmul(const quat_t &q);

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
 *                                    TIME
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
real_t ts2sec(const timestamp_t &ts);

/**
 * Convert nano-second to second.
 */
real_t ns2sec(const uint64_t ns);

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
real_t time_now();

/*****************************************************************************
 *                              FACTOR GRAPH
 *****************************************************************************/

#define POSE 0
#define INTRINSIC 1
#define EXTRINSIC 2
#define LANDMARK 3

struct param_t {
  size_t id = 0;
  timestamp_t ts = 0;
  size_t local_size = 0;

  param_t() {}

  param_t(const size_t id_, const size_t local_size_)
      : id{id_}, local_size{local_size_} {}

  param_t(const size_t id_, const timestamp_t &ts_, const size_t local_size_)
      : id{id_}, ts{ts_}, local_size{local_size_} {}

  virtual ~param_t() {}

  virtual real_t *data() = 0;
  virtual void plus(const vecx_t &) = 0;
};

struct pose_t : param_t {
  real_t param[7] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  pose_t();
  pose_t(const real_t *data);
  pose_t(const mat4_t &tf_);
  pose_t(const quat_t &q_, const vec3_t &r_);
  pose_t(const size_t id, const timestamp_t &ts_,
         const mat4_t &tf);

  quat_t rot() const;
  vec3_t trans() const;
  mat4_t tf() const;

  quat_t rot();
  vec3_t trans();
  mat4_t tf();

  vec3_t vec();
  real_t *data();
  void set_trans(const vec3_t &r);
  void set_rot(const quat_t &q);
  void set_rot(const mat3_t &C);
  void plus(const vecx_t &dx);
};

struct landmark_t : param_t {
  real_t param[3] = {0.0, 0.0, 0.0};

  landmark_t(const vec3_t &p_W_);
  landmark_t(const size_t id_, const vec3_t &p_W_);

  vec3_t vec();
  real_t *data();
  void plus(const vecx_t &dx);
};

struct camera_param_t : param_t {
  int cam_index = 0;
  real_t param[4] = {0.0, 0.0, 0.0, 0.0};

  camera_param_t(const size_t id_, const int cam_index_, const vec4_t &param_);

  vec4_t vec();
  real_t *data();
  void plus(const vecx_t &dx);
};

struct dist_param_t : param_t {
  int cam_index = 0;
  real_t param[4] = {0.0, 0.0, 0.0, 0.0};

  dist_param_t(const vec4_t &param_);
  dist_param_t(const size_t id_, const int cam_index_, const vec4_t &param_);

  vec4_t vec();
  real_t *data();
  void plus(const vecx_t &dx);
};

struct sb_param_t : param_t {
  real_t param[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  sb_param_t(const size_t id_,
             const timestamp_t &ts_,
             const vec3_t &v_,
             const vec3_t &ba_,
             const vec3_t &bg_);

  vec_t<9> vec();
  real_t *data();
  void plus(const vecx_t &dx);
};

typedef std::vector<pose_t> poses_t;
typedef std::vector<landmark_t> landmarks_t;
typedef std::vector<vec2_t> keypoints_t;

void pose_print(const std::string &prefix, const pose_t &pose);
poses_t load_poses(const std::string &csv_path);

std::vector<keypoints_t> load_keypoints(const std::string &data_path);
void keypoints_print(const keypoints_t &keypoints);

/******************************************************************************
 *                                  DATA
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
 * Allocate memory for a C-style string
 */
char *malloc_string(const char *s);

/**
 * Get number of rows in CSV file.
 * @returns Number of rows in CSV file else -1 for failure.
 */
int csv_rows(const char *fp);

/**
 * Get number of cols in CSV file.
 * @returns Number of cols in CSV file else -1 for failure.
 */
int csv_cols(const char *fp);

/**
 * Return csv fields as strings and number of fields in csv file.
 */
char **csv_fields(const char *fp, int *nb_fields);

/**
 * Load data in csv file `fp`. Assumming the data are real_ts. Also returns
 * number of rows and cols in `nb_rows` and `nb_cols` respectively.
 */
real_t **csv_data(const char *fp, int *nb_rows, int *nb_cols);

/**
 * Load integer arrays in csv file located at `csv_path`. The number of arrays
 * is returned in `nb_arrays`.
 */
int **load_iarrays(const char *csv_path, int *nb_arrays);

/**
 * Load real_t arrays in csv file located at `csv_path`. The number of arrays
 * is returned in `nb_arrays`.
 */
real_t **load_darrays(const char *csv_path, int *nb_arrays);

/**
 * Get number of rows in CSV file.
 * @returns Number of rows in CSV file else -1 for failure.
 */
int csv_rows(const std::string &file_path);

/**
 * Get number of columns in CSV file.
 * @returns Number of columns in CSV file else -1 for failure.
 */
int csv_cols(const std::string &file_path);

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
void print_progress(const real_t percentage);

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
template <typename K, typename V>
const V *lookup(const std::map<K, V> &map, K key) {
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
template <typename K, typename V>
V *lookup(std::map<K, V> &map, K key) {
  return const_cast<V *>(lookup(const_cast<const std::map<K, V> &>(map), key));
}

/**
 * Get raw pointer of a value in a `std::map`.
 */
template <typename K, typename V>
const V *lookup(const std::unordered_map<K, V> &map, K key) {
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
template <typename K, typename V>
V *lookup(std::unordered_map<K, V> &map, K key) {
  return const_cast<V *>(
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

int check_jacobian(const std::string &jac_name,
                   const matx_t &fdiff,
                   const matx_t &jac,
                   const real_t threshold,
                   const bool print = false);


/******************************************************************************
 *                                  CONFIG
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
          const bool optional = false);

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
 *                              INTERPOLATION
 *****************************************************************************/

/**
 * Linear interpolation between two points.
 *
 * @param[in] a First point
 * @param[in] b Second point
 * @param[in] t Unit number
 * @returns Linear interpolation
 */
template <typename T>
T lerp(const T &a, const T &b, const real_t t) {
  return a * (1.0 - t) + b * t;
}

/**
 * Slerp
 */
quat_t slerp(const quat_t &q_start, const quat_t &q_end, const real_t alpha);

/**
 * Interpolate between two poses `p0` and `p1` with parameter `alpha`.
 */
mat4_t interp_pose(const mat4_t &p0, const mat4_t &p1, const real_t alpha);

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
                  const real_t threshold = 0.001);

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
 * Let `t0` and `t1` be timestamps from two signals. If one of them is measured
 * at a higher rate, the goal is to interpolate the lower rate signal so that
 * it aligns with the higher rate one.
 *
 * This function will determine which timestamp deque will become the reference
 * signal and the other will become the target signal. Based on this the
 * interpolation points will be based on the reference signal.
 *
 * Additionally, this function ensures the interpolation timestamps are
 * achievable by:
 *
 * - interp start > target start
 * - interp end < target end
 *
 * **Note**: This function will not include timestamps from the target
 * (lower-rate) signal. The returned interpolation timestamps only returns
 * **interpolation points** to match the reference signal (higher-rate).
 *
 * @returns Interpolation timestamps from two timestamp deques `t0` and `t1`.
 */
std::deque<timestamp_t> lerp_timestamps(const std::deque<timestamp_t> &t0,
                                        const std::deque<timestamp_t> &t1);

/**
 * Given the interpolation timestamps `lerp_ts`, target timestamps
 * `target_ts` and target data `target_data`. This function will:
 *
 * 1: Interpolate the `target_data` at the interpolation points defined by
 *    `target_ts`.
 * 2: Disgard data that are not in the target timestamp
 */
void lerp_data(const std::deque<timestamp_t> &lerp_ts,
               std::deque<timestamp_t> &target_ts,
               std::deque<vec3_t> &target_data,
               const bool keep_old = false);

/**
 * Given two data signals with timestamps `ts0`, `vs0`, `ts1`, and `vs1`, this
 * function determines which data signal is at a lower rate and performs linear
 * interpolation inorder to synchronize against the higher rate data signal.
 *
 * The outcome of this function is that both data signals will have:
 *
 * - Same number of timestamps.
 * - Lower-rate data will be interpolated against the higher rate data.
 *
 * **Note**: This function will drop values from the start and end of both
 * signals inorder to synchronize them.
 */
void lerp_data(std::deque<timestamp_t> &ts0,
               std::deque<vec3_t> &vs0,
               std::deque<timestamp_t> &ts1,
               std::deque<vec3_t> &vs1);

/******************************************************************************
 *                                 SPLINE
 *****************************************************************************/

typedef Eigen::Spline<real_t, 1> Spline1D;
typedef Eigen::Spline<real_t, 2> Spline2D;
typedef Eigen::Spline<real_t, 3> Spline3D;

#define SPLINE1D(X, Y, DEG)                                                    \
  Eigen::SplineFitting<Spline1D>::Interpolate(X, DEG, Y)

#define SPLINE2D(X, Y, DEG)                                                    \
  Eigen::SplineFitting<Spline2D>::Interpolate(X, DEG, Y)

#define SPLINE3D(X, Y, DEG)                                                    \
  Eigen::SplineFitting<Spline3D>::Interpolate(X, DEG, Y)

/**
 * Continuous trajectory generator
 */
struct ctraj_t {
  const timestamps_t timestamps;
  const vec3s_t positions;
  const quats_t orientations;

  const real_t ts_s_start;
  const real_t ts_s_end;
  const real_t ts_s_gap;

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
  real_t rate = 0.0;        // IMU rate [Hz]
  real_t tau_a = 0.0;       // Reversion time constant for accel [s]
  real_t tau_g = 0.0;       // Reversion time constant for gyro [s]
  real_t sigma_g_c = 0.0;   // Gyro noise density [rad/s/sqrt(Hz)]
  real_t sigma_a_c = 0.0;   // Accel noise density [m/s^s/sqrt(Hz)]
  real_t sigma_gw_c = 0.0;  // Gyro drift noise density [rad/s^s/sqrt(Hz)]
  real_t sigma_aw_c = 0.0;  // Accel drift noise density [m/s^2/sqrt(Hz)]
  real_t g = 9.81;          // Gravity vector [ms-2]

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
void sim_imu_measurement(sim_imu_t &imu,
                         std::default_random_engine &rndeng,
                         const timestamp_t &ts,
                         const mat4_t &T_WS_W,
                         const vec3_t &w_WS_W,
                         const vec3_t &a_WS_W,
                         vec3_t &a_WS_S,
                         vec3_t &w_WS_S);

/*****************************************************************************
 *                                CONTROL
 *****************************************************************************/

/**
 * PID Controller
 */
struct pid_t {
  real_t error_prev = 0.0;
  real_t error_sum = 0.0;

  real_t error_p = 0.0;
  real_t error_i = 0.0;
  real_t error_d = 0.0;

  real_t k_p = 0.0;
  real_t k_i = 0.0;
  real_t k_d = 0.0;

  pid_t();
  pid_t(const real_t k_p, const real_t k_i, const real_t k_d);
  ~pid_t();
};

/**
 * `pid_t` to output stream
 */
std::ostream &operator<<(std::ostream &os, const pid_t &pid);

/**
 * Update controller
 *
 * @returns Controller command
 */
real_t pid_update(pid_t &p,
                  const real_t setpoint,
                  const real_t actual,
                  const real_t dt);

/**
 * Update controller
 *
 * @returns Controller command
 */
real_t pid_update(pid_t &p, const real_t error, const real_t dt);

/**
 * Reset controller
 */
void pid_reset(pid_t &p);

/**
 * Carrot control
 */
struct carrot_ctrl_t {
  vec3s_t waypoints;
  vec3_t wp_start = vec3_t::Zero();
  vec3_t wp_end = vec3_t::Zero();
  size_t wp_index = 0;
  real_t look_ahead_dist = 0.0;

  carrot_ctrl_t();
  ~carrot_ctrl_t();
};

/**
 * Configure carrot control using a list of position `waypoints` (x, y, z), and
 * a `look_ahead` distance in [m].
 *
 * @returns 0 for success, -1 for failure
 */
int carrot_ctrl_configure(carrot_ctrl_t &cc,
                          const vec3s_t &waypoints,
                          const real_t look_ahead_dist);

/**
 * Calculate closest point along current trajectory using current position
 * `pos`, and outputs the closest point in `result`.
 *
 * @returns A number to denote progress along the waypoint, if -1 then the
 * position is before `wp_start`, 0 if the position is between `wp_start` and
 * `wp_end`, and finally 1 if the position is after `wp_end`.
 */
int carrot_ctrl_closest_point(const carrot_ctrl_t &cc,
                              const vec3_t &pos,
                              vec3_t &result);

/**
 * Calculate carrot point using current position `pos`, and outputs the carrot
 * point in `result`.
 *
 * @returns A number to denote progress along the waypoint, if -1 then the
 * position is before `wp_start`, 0 if the position is between `wp_start` and
 * `wp_end`, and finally 1 if the position is after `wp_end`.
 */
int carrot_ctrl_carrot_point(const carrot_ctrl_t &cc,
                             const vec3_t &pos,
                             vec3_t &result);

/**
 * Update carrot controller using current position `pos` and outputs the carrot
 * point in `result`.
 *
 * @returns 0 for success, 1 for all waypoints reached and -1 for failure
 */
int carrot_ctrl_update(carrot_ctrl_t &cc, const vec3_t &pos, vec3_t &carrot_pt);

/******************************************************************************
 *                               MEASUREMENTS
 *****************************************************************************/

struct meas_t {
  timestamp_t ts = 0;

  meas_t() {}
  meas_t(const timestamp_t &ts_) : ts{ts_} {}
  virtual ~meas_t() {}
};

struct imu_data_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  timestamps_t ts;
  vec3s_t gyro;
  vec3s_t accel;
};

struct image_t : meas_t {
  int width = 0;
  int height = 0;
  float *data = nullptr;

  image_t() {}

  image_t(const timestamp_t ts_, const int width_, const int height_)
      : meas_t{ts_}, width{width_}, height{height_} {
    data = new float[width * height];
  }

  image_t(const timestamp_t ts_,
          const int width_,
          const int height_,
          float *data_)
      : meas_t{ts_}, width{width_}, height{height_}, data{data_} {}

  virtual ~image_t() {
    if (data) {
      free(data);
    }
  }
};

/*****************************************************************************
 *                                MODELS
 ****************************************************************************/

/**
 * Create DH transform from link n to link n-1 (end to front)
 *
 * @param[in] theta
 * @param[in] d
 * @param[in] a
 * @param[in] alpha
 *
 * @returns DH transform
 */
mat4_t dh_transform(const real_t theta,
                    const real_t d,
                    const real_t a,
                    const real_t alpha);

/**
 * 2-DOF Gimbal Model
 */
struct gimbal_model_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // Parameter vector of transform from
  // static camera to base-mechanism
  vecx_t tau_s = zeros(6, 1);

  // Parameter vector of transform from
  // end-effector to dynamic camera
  vecx_t tau_d = zeros(6, 1);

  // First gibmal-joint
  real_t Lambda1 = 0.0;
  vec3_t w1 = zeros(3, 1);

  // Second gibmal-joint
  real_t Lambda2 = 0.0;
  vec3_t w2 = zeros(3, 1);

  real_t theta1_offset = 0.0;
  real_t theta2_offset = 0.0;

  gimbal_model_t();
  gimbal_model_t(const vec6_t &tau_s,
                 const vec6_t &tau_d,
                 const real_t Lambda1,
                 const vec3_t w1,
                 const real_t Lambda2,
                 const vec3_t w2,
                 const real_t theta1_offset = 0.0,
                 const real_t theta2_offset = 0.0);
  virtual ~gimbal_model_t();
};

/**
 * Set gimbal attitude
 *
 * @param[in,out] model Model
 * @param[in] roll Roll [rads]
 * @param[in] pitch Pitch [rads]
 */
void gimbal_model_set_attitude(gimbal_model_t &model,
                               const real_t roll,
                               const real_t pitch);

/**
 * Get gimbal joint angle
 *
 * @param[in] model Model
 * @returns Gimbal joint angles
 */
vec2_t gimbal_model_get_joint_angles(const gimbal_model_t &model);

/**
 * Returns transform from static camera to base mechanism
 *
 * @param[in] model Model
 * @returns Transform
 */
mat4_t gimbal_model_T_BS(const gimbal_model_t &model);

/**
 * Returns transform from base mechanism to end-effector
 *
 * @param[in] model Model
 * @returns Transform
 */
mat4_t gimbal_model_T_EB(const gimbal_model_t &model);

/**
 * Returns transform from end-effector to dynamic camera
 *
 * @param[in] model Model
 * @returns Transform
 */
mat4_t gimbal_model_T_DE(const gimbal_model_t &model);

/**
 * Returns transform from static to dynamic camera
 *
 * @param[in] model Model
 * @returns Transform
 */
mat4_t gimbal_model_T_DS(const gimbal_model_t &model);

/**
 * Returns transform from static to dynamic camera
 *
 * @param[in,out] model Model
 * @param[in] theta Gimbal roll and pitch [radians]
 * @returns Transform from static to dynamic camera
 */
mat4_t gimbal_model_T_DS(gimbal_model_t &model, const vec2_t &theta);

/**
 * gimbal_model_t to output stream
 */
std::ostream &operator<<(std::ostream &os, const gimbal_model_t &gimbal);

/**
 * Calculate target angular velocity and time taken to traverse a desired
 * circle * trajectory of radius r and velocity v
 *
 * @param[in] r Desired circle radius
 * @param[in] v Desired trajectory velocity
 * @param[in] w Target angular velocity
 * @param[in] time Target time taken to complete circle trajectory
 **/
void circle_trajectory(const real_t r, const real_t v, real_t *w, real_t *time);

/**
 * Two wheel robot
 */
struct two_wheel_t {
  vec3_t p_G = vec3_t::Zero();
  vec3_t v_G = vec3_t::Zero();
  vec3_t a_G = vec3_t::Zero();
  vec3_t rpy_G = vec3_t::Zero();
  vec3_t w_G = vec3_t::Zero();

  real_t vx_desired = 0.0;
  real_t yaw_desired = 0.0;

  pid_t vx_controller{0.1, 0.0, 0.1};
  pid_t yaw_controller{0.1, 0.0, 0.1};

  vec3_t a_B = vec3_t::Zero();
  vec3_t v_B = vec3_t::Zero();
  vec3_t w_B = vec3_t::Zero();

  two_wheel_t() {}

  two_wheel_t(const vec3_t &p_G_, const vec3_t &v_G_, const vec3_t &rpy_G_)
      : p_G{p_G_}, v_G{v_G_}, rpy_G{rpy_G_} {}

  ~two_wheel_t() {}
};

/**
 * Update
 *
 * @param[in,out] tm Model
 * @param[in] dt Time difference (s)
 */
void two_wheel_update(two_wheel_t &tm, const real_t dt);

/**
 * MAV model
 */
struct mav_model_t {
  vec3_t attitude{0.0, 0.0, 0.0};         ///< Attitude in global frame
  vec3_t angular_velocity{0.0, 0.0, 0.0}; ///< Angular velocity in global frame
  vec3_t position{0.0, 0.0, 0.0};         ///< Position in global frame
  vec3_t linear_velocity{0.0, 0.0, 0.0};  ///< Linear velocity in global frame

  real_t Ix = 0.0963; ///< Moment of inertia in x-axis
  real_t Iy = 0.0963; ///< Moment of inertia in y-axis
  real_t Iz = 0.1927; ///< Moment of inertia in z-axis

  real_t kr = 0.1; ///< Rotation drag constant
  real_t kt = 0.2; ///< Translation drag constant

  real_t l = 0.9; ///< MAV arm length
  real_t d = 1.0; ///< drag constant

  real_t m = 1.0;  ///< Mass
  real_t g = 9.81; ///< Gravity
};

/**
 * Update
 *
 * @param[in,out] qm Model
 * @param[in] motor_inputs Motor inputs (m1, m2, m3, m4)
 * @param[in] dt Time difference (s)
 * @returns 0 for success, -1 for failure
 */
int mav_model_update(mav_model_t &qm,
                     const vec4_t &motor_inputs,
                     const real_t dt);

/*****************************************************************************
 *                               NETWORKING
 ****************************************************************************/

/**
 * Return IP and Port info from socket file descriptor `sockfd` to `ip` and
 * `port`. Returns `0` for success and `-1` for failure.
 */
int ip_port_info(const int sockfd, char *ip, int *port);

/**
 * Return IP and Port info from socket file descriptor `sockfd` to `ip` and
 * `port`. Returns `0` for success and `-1` for failure.
 */
int ip_port_info(const int sockfd, std::string &ip, int &port);

/**
 * TCP server
 */
struct tcp_server_t {
  int port = 8080;
  int sockfd = -1;
  std::vector<int> conns;
  void *(*conn_thread)(void *) = nullptr;

  tcp_server_t(int port_ = 8080);
};

/**
 * TCP client
 */
struct tcp_client_t {
  std::string server_ip;
  int server_port = 8080;
  int sockfd = -1;
  int (*loop_cb)(tcp_client_t &) = nullptr;

  tcp_client_t(const std::string &server_ip_ = "127.0.0.1",
               int server_port_ = 8080);
};

/**
 * Configure TCP server
 */
int tcp_server_config(tcp_server_t &server);

/**
 * Loop TCP server
 */
int tcp_server_loop(tcp_server_t &server);

/**
 * Configure TCP client
 */
int tcp_client_config(tcp_client_t &client);

/**
 * Loop TCP client
 */
int tcp_client_loop(tcp_client_t &client);

} //  namespace proto
#endif // PROTO_CORE_HPP
