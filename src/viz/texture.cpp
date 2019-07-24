#include "prototype/viz/texture.hpp"

namespace proto {

unsigned int load_texture(int img_width,
													int img_height,
													int img_channels,
													const unsigned char *data) {
	unsigned int texture_id;

	glGenTextures(1, &texture_id);

  GLenum format;
  switch (img_channels) {
  case 1: format = GL_RED; break;
  case 3: format = GL_RGB; break;
  case 4: format = GL_RGBA; break;
  }

  glBindTexture(GL_TEXTURE_2D, texture_id);
  glTexImage2D(GL_TEXTURE_2D, 0, format, img_width, img_height, 0, format, GL_UNSIGNED_BYTE, data);
  glGenerateMipmap(GL_TEXTURE_2D);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	return texture_id;
}

unsigned int load_texture(const std::string &texture_file,
													int &img_width,
													int &img_height,
													int &img_channels) {
  unsigned char *data = stbi_load(texture_file.c_str(),
                                  &img_width,
                                  &img_height,
                                  &img_channels,
                                  0);
	if (!data) {
		printf("Failed to load texture at path [%s]!\n", texture_file.c_str());
	}

	unsigned int texture_id = load_texture(img_width, img_height, img_channels, data);
	stbi_image_free(data);

  return texture_id;
}

unsigned int load_texture(const std::string &texture_file) {
  int img_width = 0;
  int img_height = 0;
  int img_channels = 0;
  return load_texture(texture_file, img_width, img_height, img_channels);
}

} // namespace proto
