#include "prototype/core/config.hpp"

namespace prototype {

config_t::config_t(const std::string &file_path_) :
  file_path{file_path_} {
  // Pre-check
  if (file_exists(file_path) == false) {
    LOG_ERROR("File not found: %s", file_path.c_str());
    ok = false;
  }

  // Load and parse file
  root = YAML::LoadFile(file_path);
  ok = true;
}

YAML::Node config_parse::getNode() const {
  ASSERT(config_.ok == true, "Config file is not loaded!");

  // Recurse down config key
  std::vector<YAML::Node> traversal;
  traversal.push_back(config_.root);

  std::istringstream iss(key_);
  std::string element;

  while (std::getline(iss, element, '.')) {
    traversal.push_back(traversal.back()[element]);
  }
  YAML::Node node = traversal.back();
  // Note:
  //
  //    yaml_node = yaml_node["some_level_deeper"];
  //
  // YAML::Node is mutable, by doing the above it destroys the parsed yaml
  // tree/graph, to avoid this problem we store the visited YAML::Node into
  // a std::vector and return the last visited YAML::Node

  // Check key
  if (!node && optional_ == false) {
    FATAL("Opps [%s] missing in yaml file [%s]!",
          key_.c_str(),
          config_.file_path.c_str());
  }

  return node;
}

void config_parse::checkMatrix(size_t &rows, size_t &cols) const {
  ASSERT(config.ok == true, "Config file is not loaded!");

  // Check key
  YAML::Node node = getNode();

  // Check fields
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
}

void config_parse::checkMatrix() const {
  size_t rows;
  size_t cols;
  checkMatrix(rows, cols);
}

config_parse::operator vec2_t() const {
  YAML::Node node = getNode();
  checkVector<vec2_t>();
  vec2_t vec{node[0].as<double>(), node[1].as<double>()};
  return vec;
}

config_parse::operator vec3_t() const {
  YAML::Node node = getNode();
  checkVector<vec3_t>();
  vec3_t vec{node[0].as<double>(),
              node[1].as<double>(),
              node[2].as<double>()};
  return vec;
}

config_parse::operator vec4_t() const {
  YAML::Node node = getNode();
  checkVector<vec4_t>();
  vec4_t vec{node[0].as<double>(),
              node[1].as<double>(),
              node[2].as<double>(),
              node[3].as<double>()};
  return vec;
}

config_parse::operator vecx_t() const {
  YAML::Node node = getNode();
  const size_t vector_size = checkVector<vecx_t>();
  vecx_t vec = vecx_t::Zero(vector_size, 1);
  for (size_t i = 0; i < node.size(); i++) {
    vec(i) = node[i].as<double>();
  }
  return vec;
}

config_parse::operator mat2_t() const {
  YAML::Node node = getNode();
  checkMatrix();

  mat2_t mat;
  mat(0, 0) = node["data"][0].as<double>();
  mat(0, 1) = node["data"][1].as<double>();

  mat(1, 0) = node["data"][2].as<double>();
  mat(1, 1) = node["data"][3].as<double>();

  return mat;
}

config_parse::operator mat3_t() const {
  YAML::Node node = getNode();
  checkMatrix();

  mat3_t mat;
  mat(0, 0) = node["data"][0].as<double>();
  mat(0, 1) = node["data"][1].as<double>();
  mat(0, 2) = node["data"][2].as<double>();

  mat(1, 0) = node["data"][3].as<double>();
  mat(1, 1) = node["data"][4].as<double>();
  mat(1, 2) = node["data"][5].as<double>();

  mat(2, 0) = node["data"][6].as<double>();
  mat(2, 1) = node["data"][7].as<double>();
  mat(2, 2) = node["data"][8].as<double>();

  return mat;
}

config_parse::operator mat4_t() const {
  YAML::Node node = getNode();
  checkMatrix();

  mat4_t mat;
  size_t index = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      mat(i, j) = node["data"][index].as<double>();
      index++;
    }
  }

  return mat;
}

config_parse::operator matx_t() const {
  YAML::Node node = getNode();
  size_t rows = 0;
  size_t cols = 0;
  checkMatrix(rows, cols);

  matx_t matx;
  matx.resize(rows, cols);
  size_t index = 0;
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      matx(i, j) = node["data"][index].as<double>();
      index++;
    }
  }

  return matx;
}

config_parse::operator cv::Mat() const {
  YAML::Node node = getNode();
  size_t rows = 0;
  size_t cols = 0;
  checkMatrix(rows, cols);

  cv::Mat mat(rows, cols, CV_64F);
  size_t index = 0;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      mat.at<double>(i, j) = node["data"][index].as<double>();
      index++;
    }
  }

  return mat;
}

} //  namespace prototype
