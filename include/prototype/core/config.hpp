/**
 * @file
 * @defgroup config config
 * @ingroup util
 */
#ifndef PROTOTYPE_CORE_CONFIG_HPP
#define PROTOTYPE_CORE_CONFIG_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "prototype/core/file.hpp"
#include "prototype/core/log.hpp"
#include "prototype/core/math.hpp"

namespace prototype {
/**
 * @addtogroup config
 * @{
 */

/**
 * Configuration file
 */
struct config_t {
  std::string file_path;
  YAML::Node root;
  bool ok = false;

  config_t() {}
  config_t(const std::string &file_path_);
};

/**
 * Load YAML file
 */
int yaml_load_file(const std::string file_path,
                   YAML::Node &root);

/**
  * Get YAML node containing the parameter value
  */
int yaml_get_node(const config_t &config,
                  const std::string &key,
                  const bool optional,
                  YAML::Node &node);

/**
  * Check length of vector in config file
  */
template <typename T>
size_t yaml_check_vector(const YAML::Node &node,
                         const std::string &key,
                         const bool optional);

/**
  * Check matrix
  *
  * Makes sure that the parameter has the data field "rows", "cols" and
  * "data". It also checks to make sure the number of values is the same size
  * as the matrix.
  */
template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional,
                       size_t &rows,
                       size_t &cols);

/**
  * Check matrix
  *
  * Makes sure that the parameter has the data field "rows", "cols" and
  * "data". It also checks to make sure the number of values is the same size
  * as the matrix.
  */
template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       const bool optional);

template <typename T>
void parse(const config_t &config, const std::string &key, T &out, const bool optional=false);

template <typename T>
void parse(const config_t &config,
           const std::string &key,
           std::vector<T> &out,
           const bool optional=false);

void parse(const config_t &config, const std::string &key, vec2_t &vec, const bool optional=false);
void parse(const config_t &config, const std::string &key, vec3_t &vec, const bool optional=false);
void parse(const config_t &config, const std::string &key, vec4_t &vec, const bool optional=false);
void parse(const config_t &config, const std::string &key, vecx_t &vec, const bool optional=false);
void parse(const config_t &config, const std::string &key, mat2_t &mat, const bool optional=false);
void parse(const config_t &config, const std::string &key, mat3_t &mat, const bool optional=false);
void parse(const config_t &config, const std::string &key, mat4_t &mat, const bool optional=false);
void parse(const config_t &config, const std::string &key, matx_t &mat, const bool optional=false);
void parse(const config_t &config, const std::string &key, cv::Mat &mat, const bool optional=false);

/** @} group config */
} //  namespace prototype
#include "prototype/core/config_impl.hpp"
#endif // PROTOTYPE_CORE_CONFIG_HPP
