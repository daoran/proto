#include "prototype/munit.hpp"
#include "prototype/core/data.hpp"
#include "prototype/core/config.hpp"

#define TEST_CONFIG "test_configs/config/config.yaml"

namespace prototype {

int test_config_param_constructor() {
  config_param_t param;

  MU_CHECK_EQ(TYPE_NOT_SET, param.type);
  MU_CHECK_EQ("", param.key);
  MU_FALSE(param.optional);
  MU_CHECK_EQ(NULL, param.data);

  return 0;
}

int test_config_parser_constructor() {
  config_parser_t parser;

  MU_FALSE(parser.config_loaded);

  return 0;
}

int test_config_parser_add() {
  bool b;
  int i;
  float f;
  double d;
  std::string s;

  std::vector<bool> b_array;
  std::vector<int> i_array;
  std::vector<float> f_array;
  std::vector<double> d_array;
  std::vector<std::string> s_array;

  vec2_t vec2;
  vec3_t vec3;
  vec4_t vec4;
  vecx_t vecx;

  mat2_t mat2;
  mat3_t mat3;
  mat4_t mat4;
  matx_t matx;
  cv::Mat cvmat;

  config_parser_t parser;

  config_parser_add(parser, "bool", &b);
  config_parser_add(parser, "int", &i);
  config_parser_add(parser, "float", &f);
  config_parser_add(parser, "double", &d);
  config_parser_add(parser, "string", &s);

  config_parser_add(parser, "bool_array", &b_array);
  config_parser_add(parser, "int_array", &i_array);
  config_parser_add(parser, "float_array", &f_array);
  config_parser_add(parser, "double_array", &d_array);
  config_parser_add(parser, "string_array", &s_array);

  config_parser_add(parser, "vector2", &vec2);
  config_parser_add(parser, "vector3", &vec3);
  config_parser_add(parser, "vector4", &vec4);
  config_parser_add(parser, "vector", &vecx);

  config_parser_add(parser, "matrix2", &mat2);
  config_parser_add(parser, "matrix3", &mat3);
  config_parser_add(parser, "matrix4", &mat4);
  config_parser_add(parser, "matrix", &matx);
  config_parser_add(parser, "matrix", &cvmat);

  MU_CHECK_EQ(19, (int) parser.params.size());
  MU_CHECK_EQ(BOOL, parser.params[0].type);
  MU_CHECK_EQ("bool", parser.params[0].key);
  MU_CHECK(parser.params[0].data != NULL);

  return 0;
}

int test_config_parser_get_node() {
  YAML::Node node1, node2, node3;
  config_parser_t parser;

  parser.params.clear();
  config_parser_load(parser, TEST_CONFIG);

  int retval;
  retval = config_parser_get_node(parser, "level3.a.b.c", false, node1);
  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(3, node1.as<int>());

  retval = config_parser_get_node(parser, "level3.a.b.d", false, node2);
  MU_CHECK_EQ(0, retval);
  // if (node2.IsSequence()) {
  //   std::cout << node2.size() << std::endl;
  //   std::cout << node2[0].as<double>() << std::endl;
  //   std::cout << node2[1].as<double>() << std::endl;
  //   std::cout << node2[2].as<double>() << std::endl;
  // }

  config_parser_get_node(parser, "float", false, node3);
  MU_CHECK_FLOAT(2.2, node3.as<float>());

  return 0;
}

int test_config_parser_load_primitive() {
  int i;
  float f;
  double d;
  std::string s;
  config_parser_t parser;
  config_param_t param;

  // setup
  parser.root = YAML::LoadFile(TEST_CONFIG);
  parser.config_loaded = true;

  // INTEGER
  param.optional = false;
  param.type = INT;
  param.key = "int";
  param.data = &i;
  config_parser_load_primitive(parser, param);
  MU_CHECK_EQ(1, i);

  // FLOAT
  param.optional = false;
  param.type = FLOAT;
  param.key = "float";
  param.data = &f;
  config_parser_load_primitive(parser, param);
  MU_CHECK_FLOAT(2.2, f);

  // DOUBLE
  param.optional = false;
  param.type = DOUBLE;
  param.key = "double";
  param.data = &d;
  config_parser_load_primitive(parser, param);
  MU_CHECK_FLOAT(3.3, d);

  // STRING
  param.optional = false;
  param.type = STRING;
  param.key = "string";
  param.data = &s;
  config_parser_load_primitive(parser, param);
  MU_CHECK_EQ("hello world!", s);

  return 0;
}

int test_config_parser_load_array() {
  std::vector<bool> b_array;
  std::vector<int> i_array;
  std::vector<float> f_array;
  std::vector<double> d_array;
  std::vector<std::string> s_array;
  config_parser_t parser;
  config_param_t param;

  // setup
  parser.root = YAML::LoadFile(TEST_CONFIG);
  parser.config_loaded = true;

  // BOOL ARRAY
  param.optional = false;
  param.type = BOOL_ARRAY;
  param.key = "bool_array";
  param.data = &b_array;
  config_parser_load_array(parser, param);

  MU_CHECK(b_array[0]);
  MU_FALSE(b_array[1]);
  MU_CHECK(b_array[2]);
  MU_FALSE(b_array[3]);

  // INTEGER
  param.optional = false;
  param.type = INT_ARRAY;
  param.key = "int_array";
  param.data = &i_array;
  config_parser_load_array(parser, param);

  for (int i = 0; i < 4; i++) {
    MU_CHECK_EQ(i + 1, i_array[i]);
  }

  // FLOAT
  param.optional = false;
  param.type = FLOAT_ARRAY;
  param.key = "float_array";
  param.data = &f_array;
  config_parser_load_array(parser, param);

  for (int i = 0; i < 4; i++) {
    MU_CHECK_FLOAT((i + 1) * 1.1, f_array[i]);
  }

  // DOUBLE
  param.optional = false;
  param.type = DOUBLE_ARRAY;
  param.key = "double_array";
  param.data = &d_array;
  config_parser_load_array(parser, param);

  for (int i = 0; i < 4; i++) {
    MU_CHECK_FLOAT((i + 1) * 1.1, d_array[i]);
  }

  // STRING
  param.optional = false;
  param.type = STRING_ARRAY;
  param.key = "string_array";
  param.data = &s_array;
  config_parser_load_array(parser, param);

  MU_CHECK_EQ("1.1", s_array[0]);
  MU_CHECK_EQ("2.2", s_array[1]);
  MU_CHECK_EQ("3.3", s_array[2]);
  MU_CHECK_EQ("4.4", s_array[3]);

  return 0;
}

int test_config_parser_load_vector() {
  vec2_t vec2;
  vec3_t vec3;
  vec4_t vec4;
  vecx_t vecx;
  config_parser_t parser;
  config_param_t param;

  // Setup
  parser.root = YAML::LoadFile(TEST_CONFIG);
  parser.config_loaded = true;

  // VECTOR 2
  param.optional = false;
  param.type = VEC2;
  param.key = "vector2";
  param.data = &vec2;
  config_parser_load_vector(parser, param);
  std::cout << vec2.transpose() << std::endl;

  MU_CHECK_FLOAT(1.1, vec2(0));
  MU_CHECK_FLOAT(2.2, vec2(1));

  // VECTOR 3
  param.optional = false;
  param.type = VEC3;
  param.key = "vector3";
  param.data = &vec3;
  config_parser_load_vector(parser, param);
  std::cout << vec3.transpose() << std::endl;

  MU_CHECK_FLOAT(1.1, vec3(0));
  MU_CHECK_FLOAT(2.2, vec3(1));
  MU_CHECK_FLOAT(3.3, vec3(2));

  // VECTOR 4
  param.optional = false;
  param.type = VEC4;
  param.key = "vector4";
  param.data = &vec4;
  config_parser_load_vector(parser, param);
  std::cout << vec4.transpose() << std::endl;

  MU_CHECK_FLOAT(1.1, vec4(0));
  MU_CHECK_FLOAT(2.2, vec4(1));
  MU_CHECK_FLOAT(3.3, vec4(2));
  MU_CHECK_FLOAT(4.4, vec4(3));

  // VECTOR X
  param.optional = false;
  param.type = VECX;
  param.key = "vector";
  param.data = &vecx;
  config_parser_load_vector(parser, param);
  std::cout << vecx.transpose() << std::endl;

  for (int i = 0; i < 9; i++) {
    MU_CHECK_FLOAT((i + 1) * 1.1, vecx(i));
  }

  return 0;
}

int test_config_parser_load_matrix() {
  int index;
  mat2_t mat2;
  mat3_t mat3;
  mat4_t mat4;
  matx_t matx;
  cv::Mat cvmat;
  config_parser_t parser;
  config_param_t param;

  // setup
  parser.root = YAML::LoadFile(TEST_CONFIG);
  parser.config_loaded = true;

  // MATRIX 2
  param.optional = false;
  param.type = MAT2;
  param.key = "matrix2";
  param.data = &mat2;
  config_parser_load_matrix(parser, param);
  std::cout << mat2 << std::endl;

  MU_CHECK_FLOAT(1.1, mat2(0, 0));
  MU_CHECK_FLOAT(2.2, mat2(0, 1));
  MU_CHECK_FLOAT(3.3, mat2(1, 0));
  MU_CHECK_FLOAT(4.4, mat2(1, 1));

  // MATRIX 3
  param.optional = false;
  param.type = MAT3;
  param.key = "matrix3";
  param.data = &mat3;
  config_parser_load_matrix(parser, param);
  std::cout << mat3 << std::endl;

  index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, mat3(i, j));
      index++;
    }
  }

  // MATRIX 4
  param.optional = false;
  param.type = MAT4;
  param.key = "matrix4";
  param.data = &mat4;
  config_parser_load_matrix(parser, param);
  std::cout << mat4 << std::endl;

  index = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, mat4(i, j));
      index++;
    }
  }

  // MATRIX X
  param.optional = false;
  param.type = MATX;
  param.key = "matrix";
  param.data = &matx;
  config_parser_load_matrix(parser, param);
  std::cout << matx << std::endl;

  index = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      MU_CHECK_FLOAT((index + 1) * 1.1, matx(i, j));
      index++;
    }
  }

  // CV MATRIX
  param.optional = false;
  param.type = CVMAT;
  param.key = "matrix";
  param.data = &cvmat;
  config_parser_load_matrix(parser, param);
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

int test_config_parser_load() {
  int retval;
  bool b;
  int i;
  float f;
  double d;
  std::string s;

  std::vector<bool> b_array;
  std::vector<int> i_array;
  std::vector<float> f_array;
  std::vector<double> d_array;
  std::vector<std::string> s_array;

  vec2_t vec2;
  vec3_t vec3;
  vec4_t vec4;
  vecx_t vecx;

  mat2_t mat2;
  mat3_t mat3;
  mat4_t mat4;
  matx_t matx;
  cv::Mat cvmat;

  config_parser_t parser;

  config_parser_add(parser, "bool", &b);
  config_parser_add(parser, "int", &i);
  config_parser_add(parser, "float", &f);
  config_parser_add(parser, "double", &d);
  config_parser_add(parser, "string", &s);

  config_parser_add(parser, "bool_array", &b_array);
  config_parser_add(parser, "int_array", &i_array);
  config_parser_add(parser, "float_array", &f_array);
  config_parser_add(parser, "double_array", &d_array);
  config_parser_add(parser, "string_array", &s_array);

  config_parser_add(parser, "vector2", &vec2);
  config_parser_add(parser, "vector3", &vec3);
  config_parser_add(parser, "vector4", &vec4);
  config_parser_add(parser, "vector", &vecx);

  config_parser_add(parser, "matrix2", &mat2);
  config_parser_add(parser, "matrix3", &mat3);
  config_parser_add(parser, "matrix4", &mat4);
  config_parser_add(parser, "matrix", &matx);
  config_parser_add(parser, "matrix", &cvmat);
  config_parser_add(parser, "non_existant_key", &cvmat, true);

  retval = config_parser_load(parser, TEST_CONFIG);
  if (retval != 0) {
    return -1;
  }

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
  MU_ADD_TEST(test_config_param_constructor);
  MU_ADD_TEST(test_config_parser_constructor);
  MU_ADD_TEST(test_config_parser_add);
  MU_ADD_TEST(test_config_parser_get_node);
  MU_ADD_TEST(test_config_parser_load_primitive);
  MU_ADD_TEST(test_config_parser_load_array);
  MU_ADD_TEST(test_config_parser_load_vector);
  MU_ADD_TEST(test_config_parser_load_matrix);
  MU_ADD_TEST(test_config_parser_load);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
