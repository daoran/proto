#ifndef PROTO_DRAW_DRAW_HPP
#define PROTO_DRAW_DRAW_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "proto/core/core.hpp"
#include "proto/viz/camera.hpp"
#include "proto/viz/shader.hpp"
#include "proto/viz/texture.hpp"

namespace proto {
namespace shaders {

const char *glcf_vs = R"glsl(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
  gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)glsl";

const char *glcf_fs = R"glsl(
#version 150 core

out vec4 FragColor;

void main() {
  FragColor = vec4(1.0f, 1.0f, 1.0f, 1.0f);
}
)glsl";

const char *glcube_vs = R"glsl(
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

const char *glcube_fs = R"glsl(
#version 150 core
in vec3 color;
out vec4 frag_color;

void main() {
  frag_color = vec4(color, 1.0f);
}
)glsl";

const char *glframe_vs = R"glsl(
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

const char *glframe_fs = R"glsl(
#version 150 core
in vec3 color;
out vec4 frag_color;

void main() {
  frag_color = vec4(color, 1.0f);
}
)glsl";

const char *glgrid_vs = R"glsl(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
  gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)glsl";

const char *glgrid_fs = R"glsl(
#version 150 core

out vec4 FragColor;

void main() {
  FragColor = vec4(0.8f, 0.8f, 0.8f, 1.0f);
}
)glsl";

const char *glplane_vs = R"glsl(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
layout (location = 2) in vec2 aTexCoord;

out vec3 ourColor;
out vec2 TexCoord;

void main() {
	gl_Position = vec4(aPos, 1.0);
	ourColor = aColor;
	TexCoord = vec2(aTexCoord.x, aTexCoord.y);
}
)glsl";

const char *glplane_fs = R"glsl(
#version 330 core

in vec3 ourColor;
in vec2 TexCoord;
out vec4 FragColor;

uniform sampler2D texture1;

void main() {
	FragColor = texture(texture1, TexCoord);
}
)glsl";

} // namespace shaders

class globj_t {
public:
  glprog_t program_;
  unsigned int VAO_;
  unsigned int VBO_;
  unsigned int EBO_;
  glm::mat4 T_SM_ = glm::mat4(1.0f);

  globj_t(const char *vs, const char *fs);

  void pos(const glm::vec3 &pos);
  glm::vec3 pos();
  glm::mat3 rot();
};

class glcf_t : globj_t {
public:
  float fov_ = glm::radians(60.0f);
  float scale_ = 1.0;
  float line_width_ = 2.0f;

  glcf_t();
  ~glcf_t();
  void draw(const glcamera_t &camera);
};

class glcube_t : globj_t {
public:
  float cube_size_ = 0.5;
  glm::vec3 color_{0.9, 0.4, 0.2};

  glcube_t();
  ~glcube_t();
  void draw(const glcamera_t &camera);
};

class glframe_t : globj_t {
public:
  float line_width_ = 5.0f;

  glframe_t();
  ~glframe_t();
  void draw(const glcamera_t &camera);
};

class glgrid_t : globj_t {
public:
  int grid_size_ = 10;

  glgrid_t();
  ~glgrid_t();
  void draw(const glcamera_t &camera);
};

class glplane_t : globj_t {
public:
  std::string image_path_;
  float width_ = 0.5;
  float height_ = 0.5;
  glm::vec3 color_{0.9, 0.4, 0.2};

  int img_width_ = 0;
  int img_height_ = 0;
  int img_channels_ = 0;

  unsigned int texture_;
  GLuint FBO_;

  glplane_t(const std::string &image_path);
  void draw(const glcamera_t &camera);
};

} // namespace proto
#endif // PROTO_VIZ_DRAW_HPP
