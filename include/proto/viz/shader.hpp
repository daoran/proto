#ifndef PROTO_VIZ_SHADER_HPP
#define PROTO_VIZ_SHADER_HPP

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

namespace proto {

int shader_compile(const char *shader_src, const int type);
int shader_compile(const std::string &shader_path, const int type);
int shaders_link(const int vertex_shader,
                 const int fragment_shader,
                 const int geometry_shader = -1);

struct glprog_t {
  unsigned int program_id;

  glprog_t(const std::string &vs_path, const std::string &fs_path);
  glprog_t(const std::string &vs_path,
           const std::string &fs_path,
           const std::string &gs_path);
  glprog_t(const char *vs_src, const char *fs_var);

  void use() const;
  int setBool(const std::string &key, const bool value) const;
  int setInt(const std::string &key, const int value) const;
  int setFloat(const std::string &key, const float value) const;
  int setVec2(const std::string &key, const glm::vec2 &value) const;
  int setVec2(const std::string &key, const float x, const float y) const;
  int setVec3(const std::string &key, const glm::vec3 &value) const;
  int setVec3(const std::string &key,
              const float x,
              const float y,
              const float z) const;
  int setVec4(const std::string &key, const glm::vec4 &value) const;
  int setVec4(const std::string &key,
              const float x,
              const float y,
              const float z,
              const float w) const;
  int setMat2(const std::string &key, const glm::mat2 &mat) const;
  int setMat3(const std::string &key, const glm::mat3 &mat) const;
  int setMat4(const std::string &key, const glm::mat4 &mat) const;
};

} // namespace proto
#endif // PROTO_VIZ_SHADER_HPP
