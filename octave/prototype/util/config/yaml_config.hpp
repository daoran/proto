#ifndef CONFIG_COMMON_HPP
#define CONFIG_COMMON_HPP

#include <cstdio>
#include <string>
#include <vector>

#include <mex.h>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__
// clang-format off
typedef Eigen::Vector2d vec2_t;
typedef Eigen::Vector3d vec3_t;
typedef Eigen::Vector4d vec4_t;
typedef Eigen::Matrix<double, 5, 1> vec5_t;
typedef Eigen::Matrix<double, 6, 1> vec6_t;
typedef Eigen::VectorXd vecx_t;

typedef Eigen::Matrix2d mat2_t;
typedef Eigen::Matrix3d mat3_t;
typedef Eigen::Matrix4d mat4_t;
typedef Eigen::MatrixXd matx_t;
typedef Eigen::Matrix<double, 3, 4> mat34_t;
// clang-format on
#endif

#define ASSERT(X, Y) \
  if (!(X)) { \
    mexErrMsgIdAndTxt("prototype:error", #Y); \
  }

#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

/**
 * Check if file exists
 */
bool file_exists(const std::string &path) {
  FILE *file;

  file = fopen(path.c_str(), "r");
  if (file != NULL) {
    fclose(file);
    return true;
  } else {
    return false;
  }
}

/**
 * Config file
 */
struct config_t {
  std::string file_path;
  YAML::Node root;
  bool ok = false;

  config_t() {}
  config_t(const std::string &file_path_) : file_path{file_path_} {
    if (file_exists(file_path) == false) {
      mexErrMsgIdAndTxt("prototype:error",
                        "File not found: %s", file_path.c_str());
      return;
    }

    // Load and parse file
    root = YAML::LoadFile(file_path);
    ok = true;
  }
  ~config_t() {}
};

/**
 * Check number of arguments
 */
inline void nargchk(bool cond) {
	if (!cond) {
		mexErrMsgIdAndTxt("prototype:error", "Wrong number of arguments!");
	}
}

/**
 * Parse args
 */
void parse_args(int nlhs, mxArray *plhs[],
                int nrhs, const mxArray *prhs[],
                std::string &yaml_path, std::string &yaml_key) {
  UNUSED(plhs);

  // Parse args
	nargchk(nlhs >= 0 && nrhs == 2);
	// -- YAML file path
  std::shared_ptr<char> arg(mxArrayToString(prhs[0]), &free);
  yaml_path = std::string{arg.get()};
	// -- YAML key
  std::shared_ptr<char> key(mxArrayToString(prhs[1]), &free);
  yaml_key = std::string{key.get()};
}

int yaml_get_node(const config_t &config,
                  const std::string &key,
                  const bool optional,
                  YAML::Node &node) {
  ASSERT(config.ok == true, "Config file is not loaded!");

  // Recurse down config key
  std::vector<YAML::Node> traversal;
  traversal.push_back(config.root);

  std::istringstream iss(key);
  std::string element;

  while (std::getline(iss, element, '.')) {
    traversal.push_back(traversal.back()[element]);
  }
  node = traversal.back();
  // Note:
  //
  //    yaml_node = yaml_node["some_level_deeper"];
  //
  // YAML::Node is mutable, by doing the above it destroys the parsed yaml
  // tree/graph, to avoid this problem we store the visited YAML::Node into
  // a std::vector and return the last visited YAML::Node

  // Check key
  if (node.IsDefined() == false && optional == false) {
    mexErrMsgIdAndTxt("prototype:error",
                      "Opps [%s] missing in yaml file [%s]!",
                      key.c_str(),
                      config.file_path.c_str());
    return -1;
  } else if (node.IsDefined() == false && optional == true) {
    return -1;
  }

  return 0;
}

template <typename T>
size_t yaml_check_vector(const YAML::Node &node,
                         const std::string &key) {
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
    mexErrMsgIdAndTxt("prototype:error", "Unsportted vector type!");
  }

  // Check number of values in the param
  if (node.size() == 0 && node.size() != vector_size) {
    mexErrMsgIdAndTxt(
          "prototype:error"
          "Vector [%s] should have %d values but config has %d!",
          key.c_str(),
          static_cast<int>(vector_size),
          static_cast<int>(node.size()));
  }

  return vector_size;
}

template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key,
                       size_t &rows,
                       size_t &cols) {
  assert(node);

  // Check fields
  const std::string targets[3] = {"rows", "cols", "data"};
  for (int i = 0; i < 3; i++) {
    if (!node[targets[i]]) {
      mexErrMsgIdAndTxt(
            "prototype:error"
            "Key [%s] is missing for matrix [%s]!",
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
  } else {
    mexErrMsgIdAndTxt("prototype:error", "Unsportted matrix type!");
  }
  if (node["data"].size() != nb_elements) {
    mexErrMsgIdAndTxt(
          "prototype:error"
          "Matrix [%s] rows and cols do not match number of values!",
          key.c_str());
  }
}

template <typename T>
void yaml_check_matrix(const YAML::Node &node,
                       const std::string &key) {
  size_t rows;
  size_t cols;
  yaml_check_matrix<T>(node, key, rows, cols);
}

template <typename T>
void parse(const config_t &config,
           const std::string &key,
           T &out,
           const bool optional=false) {
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
           const bool optional=false) {
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

void parse(const config_t &config,
           const std::string &key,
           vec2_t &vec,
           const bool optional=false) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return;
  }

  // Parse
  yaml_check_vector<vec2_t>(node, key);
  vec = vec2_t{node[0].as<double>(), node[1].as<double>()};
}

void parse(const config_t &config,
           const std::string &key,
           vec3_t &vec,
           const bool optional=false) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return;
  }

  // Parse
  yaml_check_vector<vec3_t>(node, key);
  vec =
      vec3_t{node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

void parse(const config_t &config,
           const std::string &key,
           vec4_t &vec,
           const bool optional=false) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return;
  }

  // Parse
  yaml_check_vector<vec4_t>(node, key);
  vec = vec4_t{node[0].as<double>(),
               node[1].as<double>(),
               node[2].as<double>(),
               node[3].as<double>()};
}

void parse(const config_t &config,
           const std::string &key,
           vecx_t &vec,
           const bool optional=false) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return;
  }

  // Parse
  const size_t vector_size = yaml_check_vector<vecx_t>(node, key);
  vec = vecx_t::Zero(vector_size, 1);
  for (size_t i = 0; i < node.size(); i++) {
    vec(i) = node[i].as<double>();
  }
}

void parse(const config_t &config,
           const std::string &key,
           mat2_t &mat,
           const bool optional=false) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return;
  }

  // Parse
  yaml_check_matrix<mat2_t>(node, key);
  mat(0, 0) = node["data"][0].as<double>();
  mat(0, 1) = node["data"][1].as<double>();
  mat(1, 0) = node["data"][2].as<double>();
  mat(1, 1) = node["data"][3].as<double>();
}

void parse(const config_t &config,
           const std::string &key,
           mat3_t &mat,
           const bool optional=false) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return;
  }

  // Parse
  yaml_check_matrix<mat3_t>(node, key);
  // -- Col 1
  mat(0, 0) = node["data"][0].as<double>();
  mat(0, 1) = node["data"][1].as<double>();
  mat(0, 2) = node["data"][2].as<double>();
  // -- Col 2
  mat(1, 0) = node["data"][3].as<double>();
  mat(1, 1) = node["data"][4].as<double>();
  mat(1, 2) = node["data"][5].as<double>();
  // -- Col 3
  mat(2, 0) = node["data"][6].as<double>();
  mat(2, 1) = node["data"][7].as<double>();
  mat(2, 2) = node["data"][8].as<double>();
}

void parse(const config_t &config,
           const std::string &key,
           mat4_t &mat,
           const bool optional=false) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return;
  }

  // Parse
  yaml_check_matrix<mat4_t>(node, key);
  size_t index = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      mat(i, j) = node["data"][index].as<double>();
      index++;
    }
  }
}

void parse(const config_t &config,
           const std::string &key,
           matx_t &mat,
           const bool optional=false) {
  // Get node
  YAML::Node node;
  if (yaml_get_node(config, key, optional, node) != 0) {
    return;
  }

  // Parse
  size_t rows = 0;
  size_t cols = 0;
  yaml_check_matrix<matx_t>(node, key, rows, cols);

  mat.resize(rows, cols);
  size_t index = 0;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      mat(i, j) = node["data"][index].as<double>();
      index++;
    }
  }
}

#endif // CONFIG_COMMON_HPP
