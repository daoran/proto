#ifndef DRAW_IMG_HPP
#define DRAW_IMG_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "play/common.hpp"
#include "play/camera.hpp"
#include "play/shader.hpp"
#include "play/texture.hpp"
#include "play/draw/object.hpp"


namespace shaders {

const char* glimg_vs = R"glsl(
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

const char* glimg_fs = R"glsl(
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

struct glimg_t : globj_t {
  std::string image_path;
  float width = 0.5;
  float height = 0.5;
  glm::vec3 color{0.9, 0.4, 0.2};

	int img_width = 0;
	int img_height = 0;
	int img_channels = 0;

  unsigned int texture;
	GLuint FBO;

  glimg_t(const std::string &image_path);
};

void glimg_init(glimg_t &cube);
void glimg_draw(const glimg_t &img);
void glimg_draw(const glimg_t &img, const glcamera_t &camera);

#endif // DRAW_IMG_HPP
