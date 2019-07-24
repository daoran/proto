#include "play/draw/object.hpp"

globj_t::globj_t(const char *vs, const char *fs) : program{vs, fs} {}

void globj_t::pos(const glm::vec3 &pos) {
  T_SM = glm::translate(T_SM, pos);
}

glm::vec3 globj_t::pos() {
  return glm::vec3(T_SM[3]);
}

glm::mat3 globj_t::rot() {
  glm::vec3 col_0(T_SM[0]);
  glm::vec3 col_1(T_SM[1]);
  glm::vec3 col_2(T_SM[2]);
  glm::mat3 R(col_0, col_1, col_2);
  return R;
}
