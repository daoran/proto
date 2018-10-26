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
 * Functor used to parse config parameters
 */
class config_parse{
private:
	const config_t config_;
	const std::string key_;
	const bool optional_;

  /**
   * Get YAML node containing the parameter value
   */
  YAML::Node getNode() const;

  /**
   * Check length of vector in config file
   */
  template <typename T>
  size_t checkVector() const;

  /**
   * Check matrix
   *
   * Makes sure that the parameter has the data field "rows", "cols" and
   * "data". It also checks to make sure the number of values is the same size
   * as the matrix.
   */
  void checkMatrix(size_t &rows, size_t &cols) const;

  /**
   * Check matrix
   *
   * Makes sure that the parameter has the data field "rows", "cols" and
   * "data". It also checks to make sure the number of values is the same size
   * as the matrix.
   */
  void checkMatrix() const;

public:
	config_parse(const config_t &config,
	             const std::string &key,
	             const bool optional=false) :
	  config_{config},
	  key_{key},
	  optional_{optional} {}

  /**
   * Functor to parse a config parameter
   */
	template<typename T>
	operator T() const;

	template<typename T>
	operator std::vector<T>() const;

  operator vec2_t() const;
  operator vec3_t() const;
  operator vec4_t() const;
  operator vecx_t() const;
  operator mat2_t() const;
  operator mat3_t() const;
  operator mat4_t() const;
  operator matx_t() const;
  operator cv::Mat() const;
};

/** @} group config */
} //  namespace prototype
#include "prototype/core/config_impl.hpp"
#endif // PROTOTYPE_CORE_CONFIG_HPP
