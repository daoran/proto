#ifndef DRAW_CUBE_HPP
#define DRAW_CUBE_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "play/camera.hpp"
#include "play/shader.hpp"
#include "play/draw/object.hpp"


namespace shaders {

const char* glcube_vs = R"glsl(
#version 330 core
layout (location = 0) in vec3 in_pos;
layout (location = 1) in vec3 in_color;
out vec3 color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
  gl_Position = projection * view * model * vec4(in_pos, 1.0);
  color = in_color;
}
)glsl";

const char* glcube_fs = R"glsl(
#version 150 core
in vec3 color;
out vec4 frag_color;

void main() {
  frag_color = vec4(color, 1.0f);
}
)glsl";

} // namespace shaders

struct glcube_t : globj_t {
  float cube_size = 0.5;
  glm::vec3 color{0.9, 0.4, 0.2};
  glcube_t();
};

void glcube_init(glcube_t &cube);
void glcube_draw(const glcube_t &cube, const glcamera_t &camera);

#endif // DRAW_CUBE_HPP
