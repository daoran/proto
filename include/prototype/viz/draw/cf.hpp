#ifndef PROTOTYPE_VIZ_DRAW_CF_HPP
#define PROTOTYPE_VIZ_DRAW_CF_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "prototype/viz/camera.hpp"
#include "prototype/viz/shader.hpp"
#include "prototype/viz/draw/object.hpp"

namespace proto {
namespace shaders {

const char* glcf_vs = R"glsl(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
  gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)glsl";

const char* glcf_fs = R"glsl(
#version 150 core

out vec4 FragColor;

void main() {
  FragColor = vec4(1.0f, 1.0f, 1.0f, 1.0f);
}
)glsl";

} // namespace shaders

struct glcf_t : globj_t {
  float fov = glm::radians(60.0f);
  float scale = 1.0;
  float line_width = 2.0f;
  glcf_t();
};

void glcf_init(glcf_t &cf);
void glcf_draw(const glcf_t &cf, const glcamera_t &camera);

} // namespace proto
#endif // PROTOTYPE_VIZ_DRAW_CF_HPP
