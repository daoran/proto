#ifndef PROTOTOYE_VIZ_PLAY_COMMON_HPP
#define PROTOTOYE_VIZ_PLAY_COMMON_HPP

#include <iostream>

#include <glm/glm.hpp>

#include "proto/core/stb_image.h"
#include "proto/core/stb_image_resize.h"


void print_vec3(const std::string &title, const glm::vec3 &v) {
  printf("%s: ", title.c_str());
  printf("%f, %f, %f\n", v.x, v.y, v.z);
}

void print_vec4(const std::string &title, const glm::vec4 &v) {
  printf("%s: " , title.c_str());
  printf("%f, %f, %f, %f\n", v.x, v.y, v.z, v.w);
}

void print_mat3(const std::string &title, const glm::mat3 &m) {
  const glm::vec3 c1 = m[0];
  const glm::vec3 c2 = m[1];
  const glm::vec3 c3 = m[2];

  printf("%s:\n", title.c_str());
  printf("%f, %f, %f\n", c1.x, c2.x, c3.x);
  printf("%f, %f, %f\n", c1.y, c2.y, c3.y);
  printf("%f, %f, %f\n", c1.z, c2.z, c3.z);
}

void print_mat4(const std::string &title, const glm::mat4 &m) {
  const glm::vec4 c1 = m[0];
  const glm::vec4 c2 = m[1];
  const glm::vec4 c3 = m[2];
  const glm::vec4 c4 = m[3];

  printf("%s:\n", title.c_str());
  printf("%f, %f, %f, %f\n", c1.x, c2.x, c3.x, c4.x);
  printf("%f, %f, %f, %f\n", c1.y, c2.y, c3.y, c4.y);
  printf("%f, %f, %f, %f\n", c1.z, c2.z, c3.z, c4.z);
  printf("%f, %f, %f, %f\n", c1.w, c2.w, c3.w, c4.w);
}

#endif // PROTOTOYE_VIZ_PLAY_COMMON_HPP
