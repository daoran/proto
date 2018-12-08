#ifndef PROTOTYPE_CORE_DATA_HPP
#define PROTOTYPE_CORE_DATA_HPP

#include <stdlib.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <set>

#include "prototype/core/math.hpp"
#include "prototype/core/linalg.hpp"
#include "prototype/core/tf.hpp"

namespace prototype {

/**
 * Get number of rows in file.
 * @returns Number of rows in file else -1 for failure.
 */
int filerows(const std::string &file_path);

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
 * Print progress to screen
 */
void print_progress(const double percentage);

/**
 * Lerp
 */
template <typename T>
T lerp(const T &start, const T &end, const double &alpha) {
  return start + alpha * (end - start);
}

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
void interp_poses(const std::vector<long> &timestamps,
                  const mat4s_t &poses,
                  const std::vector<long> &interp_ts,
                  mat4s_t &interped_poses,
                  const double threshold=0.01);

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
  std::set_difference(
    a.begin(), a.end(),
    b.begin(), b.end(),
    std::inserter(results, results.end())
  );
  return results;
}

/**
 * Symmetric difference between `a` and `b`.
 */
template <typename T>
T set_symmetric_diff(const T &a, const T &b) {
  T results;
  std::set_symmetric_difference(
      a.begin(), a.end(),
      b.begin(), b.end(),
      std::back_inserter(results));
  return results;
}

} //  namespace prototype
#endif // PROTOTYPE_CORE_DATA_HPP
