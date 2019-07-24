#ifndef DRAW_FRAME_HPP
#define DRAW_FRAME_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "play/camera.hpp"
#include "play/shader.hpp"
#include "play/draw/object.hpp"

namespace shaders {

const char* glframe_vs = R"glsl(
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

const char* glframe_fs = R"glsl(
#version 150 core
in vec3 color;
out vec4 frag_color;

void main() {
  frag_color = vec4(color, 1.0f);
}
)glsl";

} // namespace shaders

struct glframe_t : globj_t {
  float line_width = 5.0f;
  glframe_t();
};

void glframe_init(glframe_t &grid);
void glframe_draw(const glframe_t &grid, const glcamera_t &camera);

#endif // DRAW_FRAME_HPP
