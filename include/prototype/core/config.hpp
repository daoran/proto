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

enum config_status {
  SUCCESS = 0,
  CONFIG_NOT_FOUND = -1,
  CONFIG_NOT_LOADED = 1,
  PARSER_NOT_CONFIGURED = 2,
  KEY_NOT_FOUND = 3,
  OPTIONAL_KEY_NOT_FOUND = 4,
  INVALID_VECTOR = 5,
  INVALID_MATRIX = 6,
  INVALID_TYPE = 7
};

/**
 * An enum used by `config_param_t` to denote the yaml value type.
 *
 *  Currently we support parsing of the following data types:
 *  - **Primitives**: `bool`, `int`, `float`, `double`, `std::string`
 *  - **Arrays**: `std::vector<bool>`, `std::vector<int>`,
 * `std::vector<float>`,
 * `std::vector<double>`, `std::vector<std::string>`
 *  - **Vectors**: `Eigen::Vector2d`, `Eigen::Vector3d`, `Eigen::Vector4d`,
 * `Eigen::VectorXd`
 *  - **Matrices**: `Eigen::Matrix2d`, `Eigen::Matrix3d`, `Eigen::Matrix4d`,
 * `Eigen::MatrixXd`
 */
enum param_type {
  TYPE_NOT_SET = 0,
  // PRIMITIVES
  BOOL = 1,
  INT = 2,
  FLOAT = 3,
  DOUBLE = 4,
  STRING = 5,
  // ARRAY
  BOOL_ARRAY = 11,
  INT_ARRAY = 12,
  FLOAT_ARRAY = 13,
  DOUBLE_ARRAY = 14,
  STRING_ARRAY = 15,
  // VECTOR
  VEC2 = 21,
  VEC3 = 22,
  VEC4 = 23,
  VECX = 24,
  // MATRIX
  MAT2 = 31,
  MAT3 = 32,
  MAT4 = 33,
  MATX = 34,
  CVMAT = 35
};

/** Represents a parameter to be parsed in the yaml file */
struct config_param_t {
  enum param_type type = TYPE_NOT_SET; //!< parameter type (e.g `INT`)
  std::string key;                         //!< yaml key to parse from
  void *data = nullptr; //!< pointer to variable to store the parameter value
  bool optional = false;

  config_param_t() {}
  config_param_t(param_type type, std::string key, void *out, bool optional)
      : type{type}, key{key}, data{out}, optional{optional} {};
};

/**
 * Parses yaml files for parameters of different types.
 *
 * Currently `config_parser_t` supports parsing the following data types:
 * - **Primitives**: `bool`, `int`, `float`, `double`, `std::string`
 * - **Arrays**: `std::vector<bool>`, `std::vector<int>`, *
 * `std::vector<float>`,
 * `std::vector<double>`, `std::vector<std::string>`
 * - **Vectors**: `Eigen::Vector2d`, `Eigen::Vector3d`, `Eigen::Vector4d`,
 * `Eigen::VectorXd`
 * - **Matrices**: `Eigen::Matrix2d`, `Eigen::Matrix3d`, `Eigen::Matrix4d`,
 * `Eigen::MatrixXd`
 *
 * @note For a **matrix** we require the yaml to have three nested keys in
 * order
 * for `config_parser_t` to operate properly. They are `rows`, `cols` and `data`.
 * For example:
 *
 * ```yaml
 * some_matrix:
 *   rows: 3
 *   cols: 3
 *   data: [1.1, 2.2, 3.3,
 *          4.4, 5.5, 6.6,
 *          7.7, 8.8, 9.9]
 * ```
 */
struct config_parser_t {
  bool config_loaded = false;

  std::string file_path;
  YAML::Node root;
  std::vector<config_param_t> params;

  config_parser_t() {}
};

/**
  * Use the variations of `addParam` to add parameters you would like to
  * parse from the yaml file, where `key` is the yaml key, `out` is
  * dependent on the type of parameter you want to parse to and an
  * `optional` parameter to define whether `config_parser_t` should fail if the
  * parameter is not found.
  *
  * @param[in,out] parser Config parser
  * @param[in] key Parameter key
  * @param[out] out Parameter value
  * @param[in] optional Denotes whether parameter is optional or not
  */
// clang-format off
void config_parser_add(config_parser_t &parser, const std::string &key, bool *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, int *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, float *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, double *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, std::string *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, std::vector<bool> *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, std::vector<int> *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, std::vector<float> *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, std::vector<double> *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, std::vector<std::string> *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, vec2_t *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, vec3_t *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, vec4_t *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, vecx_t *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, mat2_t *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, mat3_t *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, mat4_t *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, matx_t *out, const bool optional = false);
void config_parser_add(config_parser_t &parser, const std::string &key, cv::Mat *out, const bool optional = false);
// clang-format on

/**
 * Get yaml node given yaml `key`. The result is assigned to `node` if
 * `key` matches anything in the config file, else `node` is set to `NULL`.
 *
 * @param[in] parser Config parser
 * @param[in] key Parameter key
 * @param[in] optional Optional
 * @param[out] node YAML node
 *
 * @returns 0 or -1 to denote success or failure
 */
int config_parser_get_node(const config_parser_t &parser,
                           const std::string &key,
                           const bool optional,
                           YAML::Node &node);

/**
 * Check vector
 *
 * @param[in] parser Config parser
 * @param[in] key Parameter key
 * @param[in] type Parameter type
 * @param[in] optional Optional
 * @param[in] node YAML node
 */
int config_parser_check_vector(const config_parser_t &parser,
                               const std::string &key,
                               const enum param_type &type,
                               const bool optional,
                               YAML::Node &node);

/**
 * Check matrix
 *
 * @param[in] parser Config parser
 * @param[in] key Parameter key
 * @param[in] optional Optional
 * @param[in] node YAML node
 */
int config_parser_check_matrix(const config_parser_t &parser,
                               const std::string &key,
                               const bool optional,
                               YAML::Node &node);

/**
  * Load yaml param primitive, array, vector or matrix.
  *
  * @return
  * - `0`: On success
  * - `-1`: Config file is not loaded
  * - `-2`: `key` not found in yaml file, parameter is not optional
  * - `-3`: `key` not found in yaml file, parameter is optional
  * - `-4`: Invalid vector (wrong size)
  * - `-5`: Invalid matrix (missing yaml key 'rows', 'cols' or 'data' for
  * matrix)
  * - `-6`: Invalid param type
  */
int config_parser_load_primitive(config_parser_t &parser, config_param_t &param);
int config_parser_load_array(config_parser_t &parser, config_param_t &param);
int config_parser_load_vector(config_parser_t &parser, config_param_t &param);
int config_parser_load_matrix(config_parser_t &parser, config_param_t &param);

/**
 * Load yaml file at `config_file`.
 *
 * @return
 * - `0`: On success
 * - `1`: File not found
 * - `-1`: Config file is not loaded
 * - `-2`: `key` not found in yaml file
 * - `-4`: Invalid vector (wrong size)
 * - `-5`: Invalid matrix (missing yaml key 'rows', 'cols' or 'data' for
 * matrix)
 * - `-6`: Invalid param type
 */
int config_parser_load(config_parser_t &parser, const std::string &config_file);

/** @} group config */
} //  namespace prototype
#endif // PROTOTYPE_CORE_CONFIG_HPP
