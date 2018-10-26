#include "prototype/core/config.hpp"

namespace prototype {

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       bool *out,
                       const bool optional) {
  parser.params.emplace_back(BOOL, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       int *out,
                       const bool optional) {
  parser.params.emplace_back(INT, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       float *out,
                       const bool optional) {
  parser.params.emplace_back(FLOAT, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       double *out,
                       const bool optional) {
  parser.params.emplace_back(DOUBLE, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       std::string *out,
                       const bool optional) {
  parser.params.emplace_back(STRING, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       std::vector<bool> *out,
                       const bool optional) {
  parser.params.emplace_back(BOOL_ARRAY, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       std::vector<int> *out,
                       const bool optional) {
  parser.params.emplace_back(INT_ARRAY, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       std::vector<float> *out,
                       const bool optional) {
  parser.params.emplace_back(FLOAT_ARRAY, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       std::vector<double> *out,
                       const bool optional) {
  parser.params.emplace_back(DOUBLE_ARRAY, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       std::vector<std::string> *out,
                       const bool optional) {
  parser.params.emplace_back(STRING_ARRAY, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       vec2_t *out,
                       const bool optional) {
  parser.params.emplace_back(VEC2, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       vec3_t *out,
                       const bool optional) {
  parser.params.emplace_back(VEC3, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       vec4_t *out,
                       const bool optional) {
  parser.params.emplace_back(VEC4, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       vecx_t *out,
                       const bool optional) {
  parser.params.emplace_back(VECX, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       mat2_t *out,
                       const bool optional) {
  parser.params.emplace_back(MAT2, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       mat3_t *out,
                       const bool optional) {
  parser.params.emplace_back(MAT3, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       mat4_t *out,
                       const bool optional) {
  parser.params.emplace_back(MAT4, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       matx_t *out,
                       const bool optional) {
  parser.params.emplace_back(MATX, key, out, optional);
}

void config_parser_add(config_parser_t &parser,
                       const std::string &key,
                       cv::Mat *out,
                       const bool optional) {
  parser.params.emplace_back(CVMAT, key, out, optional);
}

int config_parser_get_node(const config_parser_t &parser,
                           const std::string &key,
                           const bool optional,
                           YAML::Node &node) {
  ASSERT(parser.config_loaded == true, "Config file is not loaded!");
  std::string element;
  std::istringstream iss(key);
  std::vector<YAML::Node> traversal;

  // Recurse down config key
  traversal.push_back(parser.root);
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

  // check key
  if (!node && optional == false) {
    LOG_ERROR("Opps [%s] missing in yaml file [%s]!",
              key.c_str(),
              parser.file_path.c_str());
    return KEY_NOT_FOUND;
  } else if (!node && optional == true) {
    return OPTIONAL_KEY_NOT_FOUND;
  }

  return SUCCESS;
}

int config_parser_check_vector(const config_parser_t &parser,
                               const std::string &key,
                               const enum param_type &type,
                               const bool optional,
                               YAML::Node &node) {
  ASSERT(parser.config_loaded == true, "Config file is not loaded!");

  // Check key
  int retval = config_parser_get_node(parser, key, optional, node);
  if (retval != SUCCESS) {
    return retval;
  }

  int vector_size;
  switch (type) {
    case VEC2: vector_size = 2; break;
    case VEC3: vector_size = 3; break;
    case VEC4: vector_size = 4; break;
    default: return SUCCESS;
  }

  // Check number of values
  if (node.size() != static_cast<size_t>(vector_size)) {
    LOG_ERROR("Vector [%s] should have %d values but config has %d!",
              key.c_str(),
              vector_size,
              static_cast<int>(node.size()));
    return INVALID_VECTOR;
  }

  return 0;
}

int config_parser_check_matrix(const config_parser_t &parser,
                               const std::string &key,
                               const bool optional,
                               YAML::Node &node) {
  ASSERT(parser.config_loaded == true, "Config file is not loaded!");

  // Check key
  int retval = config_parser_get_node(parser, key, optional, node);
  if (retval != SUCCESS) {
    return retval;
  }

  // Check fields
  const std::string targets[3] = {"rows", "cols", "data"};
  for (int i = 0; i < 3; i++) {
    if (!node[targets[i]]) {
      LOG_ERROR("Key [%s] is missing for matrix [%s]!",
                targets[i].c_str(),
                key.c_str());
      return INVALID_MATRIX;
    }
  }

  return SUCCESS;
}

int config_parser_load_primitive(config_parser_t &parser,
                                 config_param_t &param) {
  ASSERT(this->config_loaded == true, "Config file is not loaded!");

  // Pre-check
  YAML::Node node;
  int retval = config_parser_get_node(parser, param.key, param.optional, node);
  if (retval != SUCCESS) {
    return retval;
  }

  // parse
  switch (param.type) {
    case BOOL: *static_cast<bool *>(param.data) = node.as<bool>(); break;
    case INT: *static_cast<int *>(param.data) = node.as<int>(); break;
    case FLOAT: *static_cast<float *>(param.data) = node.as<float>(); break;
    case DOUBLE: *static_cast<double *>(param.data) = node.as<double>(); break;
    case STRING:
      *static_cast<std::string *>(param.data) = node.as<std::string>();
      break;
    default: return INVALID_TYPE;
  }

  return SUCCESS;
}

int config_parser_load_array(config_parser_t &parser,
                             config_param_t &param) {
  ASSERT(this->config_loaded == true, "Config file is not loaded!");

  // Pre-check
  YAML::Node node;
  int retval = config_parser_get_node(parser, param.key, param.optional, node);
  if (retval != SUCCESS) {
    return retval;
  }

  // Parse
  switch (param.type) {
    case BOOL_ARRAY:
      for (auto n : node) {
        static_cast<std::vector<bool> *>(param.data)->push_back(n.as<bool>());
      }
      break;
    case INT_ARRAY:
      for (auto n : node) {
        static_cast<std::vector<int> *>(param.data)->push_back(n.as<int>());
      }
      break;
    case FLOAT_ARRAY:
      for (auto n : node) {
        static_cast<std::vector<float> *>(param.data)->push_back(n.as<float>());
      }
      break;
    case DOUBLE_ARRAY:
      for (auto n : node) {
        static_cast<std::vector<double> *>(param.data)
            ->push_back(n.as<double>());
      }
      break;
    case STRING_ARRAY:
      for (auto n : node) {
        static_cast<std::vector<std::string> *>(param.data)
            ->push_back(n.as<std::string>());
      }
      break;
    default: return INVALID_TYPE;
  }

  return 0;
}

int config_parser_load_vector(config_parser_t &parser, config_param_t &param) {
  ASSERT(this->config_loaded == true, "Config file is not loaded!");

  // Check parameter
  YAML::Node node;
  int retval = config_parser_check_vector(parser, param.key, param.type, param.optional, node);
  if (retval != SUCCESS) {
    return retval;
  }

  // parse
  switch (param.type) {
    case VEC2:
      *static_cast<vec2_t *>(param.data) << node[0].as<double>(),
          node[1].as<double>();
      break;

    case VEC3:
      *static_cast<vec3_t *>(param.data) << node[0].as<double>(),
          node[1].as<double>(), node[2].as<double>();
      break;

    case VEC4:
      *static_cast<vec4_t *>(param.data) << node[0].as<double>(),
          node[1].as<double>(), node[2].as<double>(), node[3].as<double>();
      break;

    case VECX: {
      vecx_t &vecx = *static_cast<vecx_t *>(param.data);
      vecx = vecx_t((int) node.size());
      for (size_t i = 0; i < node.size(); i++) {
        vecx(i) = node[i].as<double>();
      }
    } break;

    default: return INVALID_TYPE;
  }

  return SUCCESS;
}

int config_parser_load_matrix(config_parser_t &parser, config_param_t &param) {
  ASSERT(this->config_loaded == true, "Config file is not loaded!");

  // Check parameter
  YAML::Node node;
  int retval = config_parser_check_matrix(parser, param.key, param.optional, node);
  if (retval != SUCCESS) {
    return retval;
  }

  // parse
  int index = 0;
  int rows = node["rows"].as<int>();
  int cols = node["cols"].as<int>();

  switch (param.type) {
    case MAT2: {
      mat2_t &mat2 = *static_cast<mat2_t *>(param.data);
      mat2(0, 0) = node["data"][0].as<double>();
      mat2(0, 1) = node["data"][1].as<double>();

      mat2(1, 0) = node["data"][2].as<double>();
      mat2(1, 1) = node["data"][3].as<double>();
    } break;

    case MAT3: {
      mat3_t &mat3 = *static_cast<mat3_t *>(param.data);
      mat3(0, 0) = node["data"][0].as<double>();
      mat3(0, 1) = node["data"][1].as<double>();
      mat3(0, 2) = node["data"][2].as<double>();

      mat3(1, 0) = node["data"][3].as<double>();
      mat3(1, 1) = node["data"][4].as<double>();
      mat3(1, 2) = node["data"][5].as<double>();

      mat3(2, 0) = node["data"][6].as<double>();
      mat3(2, 1) = node["data"][7].as<double>();
      mat3(2, 2) = node["data"][8].as<double>();
    } break;

    case MAT4: {
      mat4_t &mat4 = *static_cast<mat4_t *>(param.data);
      for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
          mat4(i, j) = node["data"][index].as<double>();
          index++;
        }
      }
    } break;

    case MATX: {
      matx_t &matx = *static_cast<matx_t *>(param.data);
      matx.resize(rows, cols);
      for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
          matx(i, j) = node["data"][index].as<double>();
          index++;
        }
      }
    } break;

    case CVMAT: {
      cv::Mat &cvmat = *static_cast<cv::Mat *>(param.data);
      cvmat = cv::Mat(rows, cols, CV_64F);
      for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
          cvmat.at<double>(i, j) = node["data"][index].as<double>();
          index++;
        }
      }
    } break;
    default: return INVALID_TYPE;
  }

  return SUCCESS;
}

int config_parser_load(config_parser_t &parser,
                       const std::string &config_file) {
  // Pre-check
  if (file_exists(config_file) == false) {
    LOG_ERROR("File not found: %s", config_file.c_str());
    return CONFIG_NOT_FOUND;
  }

  // load and parse file
  parser.file_path = config_file;
  parser.root = YAML::LoadFile(config_file);
  parser.config_loaded = true;

  int retval;
  for (size_t i = 0; i < parser.params.size(); i++) {
    switch (parser.params[i].type) {
      // PRIMITIVE
      case BOOL:
      case INT:
      case FLOAT:
      case DOUBLE:
      case STRING:
        retval = config_parser_load_primitive(parser, parser.params[i]);
        break;
      // ARRAY
      case BOOL_ARRAY:
      case INT_ARRAY:
      case FLOAT_ARRAY:
      case DOUBLE_ARRAY:
      case STRING_ARRAY:
        retval = config_parser_load_array(parser, parser.params[i]);
        break;
      // VECTOR
      case VEC2:
      case VEC3:
      case VEC4:
      case VECX:
        retval = config_parser_load_vector(parser, parser.params[i]);
        break;
      // MAT
      case MAT2:
      case MAT3:
      case MAT4:
      case MATX:
      case CVMAT: retval = config_parser_load_matrix(parser, parser.params[i]); break;
      default: return INVALID_TYPE;
    }

    if (retval != SUCCESS && retval != OPTIONAL_KEY_NOT_FOUND) {
      return retval;
    }
  }

  return SUCCESS;
}

} //  namespace prototype
