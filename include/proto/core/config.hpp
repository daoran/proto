#ifndef PROTO_CORE_CONFIG_HPP
#define PROTO_CORE_CONFIG_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "proto/core/file.hpp"
#include "proto/core/log.hpp"
#include "proto/core/math.hpp"
#include "proto/core/macros.hpp"

namespace proto {

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
void parse(const config_t &config,
           const std::string &key,
           T &out,
           const bool optional=false);

template <typename T>
void parse(const config_t &config,
           const std::string &key,
           std::vector<T> &out,
           const bool optional);

void parse(const config_t &config,
           const std::string &key,
           vec2_t &vec,
           const bool optional = false);

void parse(const config_t &config,
           const std::string &key,
           vec3_t &vec,
           const bool optional = false);

void parse(const config_t &config,
           const std::string &key,
           vec4_t &vec,
           const bool optional = false);

void parse(const config_t &config,
           const std::string &key,
           vecx_t &vec,
           const bool optional = false);

void parse(const config_t &config,
           const std::string &key,
           mat2_t &mat,
           const bool optional = false);

void parse(const config_t &config,
           const std::string &key,
           mat3_t &mat,
           const bool optional = false);

void parse(const config_t &config,
           const std::string &key,
           mat4_t &mat,
           const bool optional = false);

void parse(const config_t &config,
           const std::string &key,
           matx_t &mat,
           const bool optional = false);

void parse(const config_t &config,
           const std::string &key,
           cv::Mat &mat,
           const bool optional = false);

template <typename T>
void parse(const config_t &config,
           const std::string &key,
           const int rows,
           const int cols,
           T &mat,
           const bool optional = false);

/******************************************************************************
 * IMPLEMENTATION
 *****************************************************************************/

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
void parse(const config_t &config,
           const std::string &key,
           T &out,
           const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return;
  }

  // Parse
  out = node.as<T>();
}

template <typename T>
void parse(const config_t &config,
           const std::string &key,
           std::vector<T> &out,
           const bool optional) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return;
  }

  // Parse
  std::vector<T> array;
  for (auto n : node) {
    out.push_back(n.as<T>());
  }
}

} //  namespace proto
#endif // PROTO_CORE_CONFIG_HPP
