/**
 * @file
 * @defgroup config config
 * @ingroup util
 */
#ifndef PROTOTYPE_CORE_CONFIG_IMPL_HPP
#define PROTOTYPE_CORE_CONFIG_IMPL_HPP

#include "prototype/core/config.hpp"

namespace prototype {
/**
 * @addtogroup config
 * @{
 */

template <typename T>
size_t yaml_check_vector(const YAML::Node &node,
                         const std::string &key,
                         const bool optional) {
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
  assert(node);

  // Check fields
  const std::string targets[3] = {"rows", "cols", "data"};
  for (int i = 0; i < 3; i++) {
    if (!node[targets[i]]) {
      FATAL("Key [%s] is missing for matrix [%s]!",
            targets[i].c_str(),
            key.c_str());
    }
  }
  rows = node["rows"].as<int>();
  cols = node["cols"].as<int>();

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

/** @} group config */
} //  namespace prototype
#endif // PROTOTYPE_CORE_CONFIG_IMPL_HPP
