#include "prototype/munit.hpp"
#include "prototype/dataset/kitti/raw/parse.hpp"

namespace prototype {

int test_parse_string() {
  std::string value = parse_string("X: Hello World");

  MU_CHECK_EQ("Hello World", value);

  return 0;
}

int test_parse_double() {
  double value = parse_double("X: 1.23");

  MU_CHECK_FLOAT(1.23, value);

  return 0;
}

int test_parse_array() {
  std::vector<double> value = parse_array("X: 1.0 2.0 3.0");

  MU_CHECK_EQ(3, (int) value.size());
  MU_CHECK_FLOAT(1.0, value[0]);
  MU_CHECK_FLOAT(2.0, value[1]);
  MU_CHECK_FLOAT(3.0, value[2]);

  return 0;
}

int test_parse_vec2() {
  vec2_t value = parse_vec2("X: 1.0 2.0");

  MU_CHECK_FLOAT(1.0, value(0));
  MU_CHECK_FLOAT(2.0, value(1));

  return 0;
}

int test_parse_vec3() {
  vec3_t value = parse_vec3("X: 1.0 2.0 3.0");

  MU_CHECK_FLOAT(1.0, value(0));
  MU_CHECK_FLOAT(2.0, value(1));
  MU_CHECK_FLOAT(3.0, value(2));

  return 0;
}

int test_parse_vecx() {
  vecx_t value = parse_vecx("X: 1.0 2.0 3.0 4.0 5.0 6.0");

  MU_CHECK_FLOAT(1.0, value(0));
  MU_CHECK_FLOAT(2.0, value(1));
  MU_CHECK_FLOAT(3.0, value(2));
  MU_CHECK_FLOAT(4.0, value(3));
  MU_CHECK_FLOAT(5.0, value(4));
  MU_CHECK_FLOAT(6.0, value(5));

  return 0;
}

int test_parse_mat3() {
  mat3_t value = parse_mat3("X: 1 2 3 4 5 6 7 8 9");

  MU_CHECK_FLOAT(1.0, value(0, 0));
  MU_CHECK_FLOAT(2.0, value(0, 1));
  MU_CHECK_FLOAT(3.0, value(0, 2));
  MU_CHECK_FLOAT(4.0, value(1, 0));
  MU_CHECK_FLOAT(5.0, value(1, 1));
  MU_CHECK_FLOAT(6.0, value(1, 2));
  MU_CHECK_FLOAT(7.0, value(2, 0));
  MU_CHECK_FLOAT(8.0, value(2, 1));
  MU_CHECK_FLOAT(9.0, value(2, 2));

  return 0;
}

int test_parse_mat34() {
  mat34_t value = parse_mat34("X: 1 2 3 4 5 6 7 8 9 10 11 12");

  MU_CHECK_FLOAT(1.0, value(0, 0));
  MU_CHECK_FLOAT(2.0, value(0, 1));
  MU_CHECK_FLOAT(3.0, value(0, 2));
  MU_CHECK_FLOAT(4.0, value(0, 3));

  MU_CHECK_FLOAT(5.0, value(1, 0));
  MU_CHECK_FLOAT(6.0, value(1, 1));
  MU_CHECK_FLOAT(7.0, value(1, 2));
  MU_CHECK_FLOAT(8.0, value(1, 3));

  MU_CHECK_FLOAT(9.0, value(2, 0));
  MU_CHECK_FLOAT(10.0, value(2, 1));
  MU_CHECK_FLOAT(11.0, value(2, 2));
  MU_CHECK_FLOAT(12.0, value(2, 3));

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_parse_string);
  MU_ADD_TEST(test_parse_double);
  MU_ADD_TEST(test_parse_array);
  MU_ADD_TEST(test_parse_vec2);
  MU_ADD_TEST(test_parse_vec3);
  MU_ADD_TEST(test_parse_vecx);
  MU_ADD_TEST(test_parse_mat3);
  MU_ADD_TEST(test_parse_mat34);
}

} // namespace prototype

MU_RUN_TESTS(prototype::test_suite);
