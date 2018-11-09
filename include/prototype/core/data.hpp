#ifndef PROTOTYPE_CORE_DATA_HPP
#define PROTOTYPE_CORE_DATA_HPP

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

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
 * Extend `std::vector`.
 */
template <typename T>
void extend(std::vector<T> &x, std::vector<T> &add) {
  x.reserve(x.size() + add.size());
  x.insert(x.end(), add.begin(), add.end());
}

} //  namespace prototype
#endif // PROTOTYPE_CORE_DATA_HPP
