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
  } if (std::is_same<T, vec3_t>::value) {
    vector_size = 3;
  } if (std::is_same<T, vec4_t>::value) {
    vector_size = 4;
  } if (std::is_same<T, vec5_t>::value) {
    vector_size = 5;
  } if (std::is_same<T, vecx_t>::value) {
    vector_size = node.size();
    return vector_size; // Don't bother, it could be anything
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
