#ifndef PROTO_VIZ_TEXTURE_HPP
#define PROTO_VIZ_TEXTURE_HPP

#include <string>

#include <glad/glad.h>

#include "proto/core/stb_image.h"

namespace proto {

struct gltexture_t {
  unsigned int id;
  std::string type;
  std::string path;
};

unsigned int load_texture(int img_width,
                          int img_height,
                          int img_channels,
                          const unsigned char *data);

unsigned int load_texture(const std::string &texture_file,
                          int &img_width,
                          int &img_height,
                          int &img_channels);

unsigned int load_texture(const std::string &texture_file);

} // namespace proto
#endif // TEXTURE_HPP
