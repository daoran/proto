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
size_t config_parse::checkVector() const {
  ASSERT(config_.ok == true, "Config file is not loaded!");

  // Check what the vector size should be
  YAML::Node node = getNode();
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
          key_.c_str(),
          static_cast<int>(vector_size),
          static_cast<int>(node.size()));
  }

  return vector_size;
}

template <typename T>
void config_parse::checkMatrix(size_t &rows, size_t &cols) const {
  ASSERT(config.ok == true, "Config file is not loaded!");

  // Check fields
  YAML::Node node = getNode();
  const std::string targets[3] = {"rows", "cols", "data"};
  for (int i = 0; i < 3; i++) {
    if (!node[targets[i]]) {
      FATAL("Key [%s] is missing for matrix [%s]!",
            targets[i].c_str(),
            key_.c_str());
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
      FATAL("Matrix [%s] rows and cols do not match actual number of values!",
            key_.c_str());
  }
}

template <typename T>
void config_parse::checkMatrix() const {
  size_t rows;
  size_t cols;
  checkMatrix<T>(rows, cols);
}

template<typename T>
config_parse::operator T() const {
  YAML::Node node = getNode();
  return node.as<T>();
}

template<typename T>
config_parse::operator std::vector<T>() const {
  YAML::Node node = getNode();

  std::vector<T> array;
  for (auto n : node) {
    array.push_back(n.as<T>());
  }

  return array;
}

/** @} group config */
} //  namespace prototype
#endif // PROTOTYPE_CORE_CONFIG_IMPL_HPP
