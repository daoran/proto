#ifndef DRAW_OBJECT_HPP
#define DRAW_OBJECT_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "play/camera.hpp"
#include "play/shader.hpp"

struct globj_t {
  glprog_t program;
  unsigned int VAO;
  unsigned int VBO;
  unsigned int EBO;
  glm::mat4 T_SM = glm::mat4(1.0f);

  globj_t(const char *vs, const char *fs);

  void pos(const glm::vec3 &pos);
  glm::vec3 pos();
  glm::mat3 rot();
};

#endif // DRAW_OBJECT_HPP
