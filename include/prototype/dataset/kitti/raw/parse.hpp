/**
 * @file
 * @defgroup kitti kitti
 * @ingroup dataset
 */
#ifndef PROTOTYPE_DATASET_KITTI_RAW_PARSE_HPP
#define PROTOTYPE_DATASET_KITTI_RAW_PARSE_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup kitti
 * @{
 */

/**
 * Parse string
 *
 * @param[in] line Line containing a string
 * @returns A string
 */
std::string parse_string(const std::string &line);

/**
 * Parse double
 *
 * @param[in] line Line containing a double
 * @returns A double
 */
double parse_double(const std::string &line);

/**
 * Parse array
 *
 * @param[in] line Line containing an array
 * @returns Array
 */
std::vector<double> parse_array(const std::string &line);

/**
 * Parse vector of size 2
 *
 * @param[in] line Line containing a vector of size 2
 * @returns Vector of size 2
 */
vec2_t parse_vec2(const std::string &line);

/**
 * Parse vector of size 3
 *
 * @param[in] line Line containing a vector of size 3
 * @returns Vector of size 3
 */
vec3_t parse_vec3(const std::string &line);

/**
 * Parse vector
 *
 * @param[in] line Line containing a vector
 * @returns Vector
 */
vecx_t parse_vecx(const std::string &line);

/**
 * Parse 3x3 matrix
 *
 * @param[in] line Line containing a 3x3 matrix
 * @returns 3x3 matrix
 */
mat3_t parse_mat3(const std::string &line);

/**
 * Parse 3x4 matrix
 *
 * @param[in] line Line containing a 3x4 matrix
 * @returns 3x4 matrix
 */
mat34_t parse_mat34(const std::string &line);

/**
 * Parse timestamp
 *
 * @param[in] line Line containing a timestamp
 * @param[out] s Output string
 * @returns 0 or -1 for success or failure
 */
int parse_timestamp(const std::string &line, long *s);

/** @} group kitti */
} //  namespace prototype
#endif // PROTOTYPE_DATASET_KITTI_RAW_PARSE_HPP
