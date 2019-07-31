#ifndef PROTOTYPE_VIZ_MESH_HPP
#define PROTOTYPE_VIZ_MESH_HPP

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "prototype/viz/shader.hpp"
#include "prototype/viz/texture.hpp"

namespace proto {

struct glvertex_t {
  glm::vec3 position;
  glm::vec3 normal;
  glm::vec2 texcoords;
  glm::vec3 tangent;
  glm::vec3 bitangent;
};

struct glmesh_t {
  std::vector<glvertex_t> vertices;
  std::vector<unsigned int> indices;
  std::vector<gltexture_t> textures;
  unsigned int VAO;
  unsigned int VBO;
  unsigned int EBO;

  glmesh_t(const std::vector<glvertex_t> &vertices_,
         const std::vector<unsigned int> &indices_,
         const std::vector<gltexture_t> &textures_);
};

void glmesh_init(glmesh_t &mesh);
void glmesh_draw(const glmesh_t &mesh, const glprog_t &program);

} // namespace proto
#endif // PROTOTYPE_VIZ_MESH_HPP