#ifndef DRAW_GRID_HPP
#define DRAW_GRID_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "play/camera.hpp"
#include "play/shader.hpp"
#include "play/draw/object.hpp"

namespace shaders {

const char* glgrid_vs = R"glsl(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
  gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)glsl";

const char* glgrid_fs = R"glsl(
#version 150 core

out vec4 FragColor;

void main() {
  FragColor = vec4(0.8f, 0.8f, 0.8f, 1.0f);
}
)glsl";

} // namespace shaders

struct glgrid_t : globj_t {
  int grid_size = 10;
  glgrid_t();
};

void glgrid_init(glgrid_t &grid);
void glgrid_draw(const glgrid_t &grid, const glcamera_t &camera);

#endif // DRAW_GRID_HPP
