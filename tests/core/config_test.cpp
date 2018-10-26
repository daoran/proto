#include "prototype/munit.hpp"
#include "prototype/core/data.hpp"
#include "prototype/core/config.hpp"

#define TEST_CONFIG "test_configs/config/config.yaml"

namespace prototype {

int test_config_constructor() {
  config_t config{TEST_CONFIG};

  MU_CHECK(config.ok == true);
  MU_CHECK(config.file_path == TEST_CONFIG);

  return 0;
}

int test_config_parse_primitive() {
  config_t config{TEST_CONFIG};

  // INTEGER
  const int i = config_parse(config, "int");
  MU_CHECK_EQ(1, i);

  // FLOAT
  const float f = config_parse(config, "float");
  MU_CHECK_FLOAT(2.2, f);

  // DOUBLE
  const double d = config_parse(config, "double");
  MU_CHECK_FLOAT(3.3, d);

  // STRING
  const std::string s = config_parse(config, "string");
  MU_CHECK_EQ("hello world!", s);

  return 0;
}

int test_config_parse_array() {
  config_t config{TEST_CONFIG};

  // BOOL ARRAY
  const std::vector<bool> b_array = config_parse(config, "bool_array");
  MU_CHECK(b_array[0]);
  MU_FALSE(b_array[1]);
  MU_CHECK(b_array[2]);
  MU_FALSE(b_array[3]);

  // INTEGER
  const std::vector<int> i_array = config_parse(config, "int_array");
  for (int i = 0; i < 4; i++) {
    MU_CHECK_EQ(i + 1, i_array[i]);
  }

  // FLOAT
  const std::vector<float> f_array = config_parse(config, "float_array");
  for (int i = 0; i < 4; i++) {
    MU_CHECK_FLOAT((i + 1) * 1.1, f_array[i]);
  }

  // DOUBLE
  const std::vector<double> d_array = config_parse(config, "double_array");
  for (int i = 0; i < 4; i++) {
    MU_CHECK_FLOAT((i + 1) * 1.1, d_array[i]);
  }

  // STRING
  const std::vector<std::string> s_array = config_parse(config, "string_array");
  MU_CHECK_EQ("1.1", s_array[0]);
  MU_CHECK_EQ("2.2", s_array[1]);
  MU_CHECK_EQ("3.3", s_array[2]);
  MU_CHECK_EQ("4.4", s_array[3]);

  return 0;
}

int test_config_parse_vector() {
  config_t config{TEST_CONFIG};

  // VECTOR 2
  const vec2_t vec2 = config_parse(config, "vector2");
  std::cout << vec2.transpose() << std::endl;
  MU_CHECK_FLOAT(1.1, vec2(0));
  MU_CHECK_FLOAT(2.2, vec2(1));

  // VECTOR 3
  const vec3_t vec3 = config_parse(config, "vector3");
  std::cout << vec3.transpose() << std::endl;
  MU_CHECK_FLOAT(1.1, vec3(0));
  MU_CHECK_FLOAT(2.2, vec3(1));
  MU_CHECK_FLOAT(3.3, vec3(2));

  // VECTOR 4
  const vec4_t vec4 = config_parse(config, "vector4");
  std::cout << vec4.transpose() << std::endl;
  MU_CHECK_FLOAT(1.1, vec4(0));
  MU_CHECK_FLOAT(2.2, vec4(1));
  MU_CHECK_FLOAT(3.3, vec4(2));
  MU_CHECK_FLOAT(4.4, vec4(3));

  // VECTOR X
  const vecx_t vecx = config_parse(config, "vector");
  std::cout << vecx.transpose() << std::endl;
  for (int i = 0; i < 9; i++) {
    MU_CHECK_FLOAT((i + 1) * 1.1, vecx(i));
  }

  return 0;
}

int test_config_parse_matrix() {
  config_t config{TEST_CONFIG};

  // MATRIX 2
  const mat2_t mat2 = config_parse(config, "matrix2");
  std::cout << mat2 << std::endl;

  MU_CHECK_FLOAT(1.1, mat2(0, 0));
  MU_CHECK_FLOAT(2.2, mat2(0, 1));
  MU_CHECK_FLOAT(3.3, mat2(1, 0));
  MU_CHECK_FLOAT(4.4, mat2(1, 1));

  // MATRIX 3
  const mat3_t mat3 = config_parse(config, "matrix3");
  std::cout << mat3 << std::endl;

  int index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, mat3(i, j));
      index++;
    }
  }

  // MATRIX 4
  const mat4_t mat4 = config_parse(config, "matrix4");
  std::cout << mat4 << std::endl;

  index = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, mat4(i, j));
      index++;
    }
  }

  // MATRIX X
  const matx_t matx = config_parse(config, "matrix");
  std::cout << matx << std::endl;

  index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, matx(i, j));
      index++;
    }
  }

  // CV MATRIX
  const cv::Mat cvmat = config_parse(config, "matrix");
  std::cout << cvmat << std::endl;

  index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, cvmat.at<double>(i, j));
      index++;
    }
  }

  return 0;
}

int test_config_parser_full_example() {
  config_t config{TEST_CONFIG};

  bool b = config_parse(config, "bool");
  int i = config_parse(config, "int");
  float f = config_parse(config, "float");
  double d = config_parse(config, "double");
  std::string s = config_parse(config, "string");

  std::vector<bool> b_array = config_parse(config, "bool_array");
  std::vector<int> i_array = config_parse(config, "int_array");
  std::vector<float> f_array = config_parse(config, "float_array");
  std::vector<double> d_array = config_parse(config, "double_array");
  std::vector<std::string> s_array = config_parse(config, "string_array");

  vec2_t vec2 = config_parse(config, "vector2");
  vec3_t vec3 = config_parse(config, "vector3");
  vec4_t vec4 = config_parse(config, "vector4");
  vecx_t vecx = config_parse(config, "vector");

  mat2_t mat2 = config_parse(config, "matrix2");
  mat3_t mat3 = config_parse(config, "matrix3");
  mat4_t mat4 = config_parse(config, "matrix4");
  matx_t matx = config_parse(config, "matrix");
  cv::Mat cvmat = config_parse(config, "matrix");
  config_parse(config, "non_existant_key", true);

  std::cout << "bool: " << b << std::endl;
  std::cout << "int: " << i << std::endl;
  std::cout << "float: " << f << std::endl;
  std::cout << "double: " << d << std::endl;
  std::cout << "string: " << s << std::endl;
  std::cout << std::endl;

  std::cout << "vector2: " << vec2.transpose() << std::endl;
  std::cout << "vector3: " << vec3.transpose() << std::endl;
  std::cout << "vector4: " << vec4.transpose() << std::endl;
  std::cout << "vector: " << vecx.transpose() << std::endl;
  std::cout << std::endl;

  std::cout << "matrix2: \n" << mat2 << std::endl;
  std::cout << "matrix3: \n" << mat3 << std::endl;
  std::cout << "matrix4: \n" << mat4 << std::endl;
  std::cout << "matrix: \n" << matx << std::endl;
  std::cout << "cvmatrix: \n" << cvmat << std::endl;
  std::cout << std::endl;

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_config_constructor);
  MU_ADD_TEST(test_config_parse_primitive);
  MU_ADD_TEST(test_config_parse_array);
  MU_ADD_TEST(test_config_parse_vector);
  MU_ADD_TEST(test_config_parse_matrix);
  MU_ADD_TEST(test_config_parser_full_example);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
