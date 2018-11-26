#ifndef PROTOTYPE_CORE_DATA_HPP
#define PROTOTYPE_CORE_DATA_HPP

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <set>

#include "prototype/core/math.hpp"

namespace prototype {

// CSV ERROR MESSAGES
#define E_CSV_DATA_LOAD "Error! failed to load data [%s]!!\n"
#define E_CSV_DATA_OPEN "Error! failed to open file for output [%s]!!\n"

/**
 * Get number of rows in CSV file.
 * @returns number of rows in CSV file
 */
int csvrows(const std::string &file_path);

/**
 * Get number of columns in CSV file.
 * @returns number of columns in CSV file
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
std::set<T> set_union(const std::set<T> &s1, const std::set<T> &s2) {
  std::set<T> result = s1;
  result.insert(s2.begin(), s2.end());
  return result;
}

/**
 * Difference between set `a` and set `b`.
 */
template <typename T>
std::set<T> set_diff(const std::set<T> &s1, const std::set<T> &s2) {
  std::set<T> result;
  std::set_difference(
    s1.begin(), s1.end(),
    s2.begin(), s2.end(),
    std::inserter(result, result.end())
  );
  return result;
}

} //  namespace prototype
#endif // PROTOTYPE_CORE_DATA_HPP
