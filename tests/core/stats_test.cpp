#include "prototype/core/stats.hpp"
#include "prototype/munit.hpp"

namespace proto {

int test_linreg() {
  vec2_t p;
  vec2s_t points;
  double m;
  double c;
  double r;

  p << 1, 4;
  points.push_back(p);
  p << 2, 6;
  points.push_back(p);
  p << 4, 12;
  points.push_back(p);
  p << 5, 15;
  points.push_back(p);
  p << 10, 34;
  points.push_back(p);
  p << 20, 68;
  points.push_back(p);

  linreg(points, &m, &c, &r);
  std::cout << "m: " << m << std::endl;
  std::cout << "c: " << c << std::endl;
  std::cout << "r: " << r << std::endl;

  return 0;
}

void test_suite() { MU_ADD_TEST(test_linreg); }

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
