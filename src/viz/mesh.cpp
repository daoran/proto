#include "proto/viz/mesh.hpp"

namespace proto {

glmesh_t::glmesh_t(const std::vector<glvertex_t> &vertices_,
                   const std::vector<unsigned int> &indices_,
                   const std::vector<gltexture_t> &textures_) {
  vertices = vertices_;
  indices = indices_;
  textures = textures_;
  glmesh_init(*this);
}

void glmesh_init(glmesh_t &mesh) {
  // Load data into vertex buffers
  // -- VAO
  glGenVertexArrays(1, &mesh.VAO);
  glBindVertexArray(mesh.VAO);

  // -- VBO
  glGenBuffers(1, &mesh.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, mesh.VBO);
  // A great thing about structs is that their memory layout is sequential for
  // all its items.  The effect is that we can simply pass a pointer to the
  // struct and it translates perfectly to a glm::vec3/2 array which again
  // translates to 3/2 floats which translates to a byte array.
  glBufferData(GL_ARRAY_BUFFER,
               mesh.vertices.size() * sizeof(glvertex_t),
               &mesh.vertices[0],
               GL_STATIC_DRAW);

  // -- EBO
  glGenBuffers(1, &mesh.EBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               mesh.indices.size() * sizeof(unsigned int),
               &mesh.indices[0],
               GL_STATIC_DRAW);

  // Set vertex attribute pointers
  // Vertex positions
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        sizeof(glvertex_t),
                        (void *) 0);

  // Vertex normals
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        sizeof(glvertex_t),
                        (void *) offsetof(glvertex_t, normal));

  // Vertex texture coords
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2,
                        2,
                        GL_FLOAT,
                        GL_FALSE,
                        sizeof(glvertex_t),
                        (void *) offsetof(glvertex_t, texcoords));

  // Vertex tangent
  glEnableVertexAttribArray(3);
  glVertexAttribPointer(3,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        sizeof(glvertex_t),
                        (void *) offsetof(glvertex_t, tangent));

  // Vertex bitangent
  glEnableVertexAttribArray(4);
  glVertexAttribPointer(4,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        sizeof(glvertex_t),
                        (void *) offsetof(glvertex_t, bitangent));

  glBindVertexArray(0);
}

void glmesh_draw(const glmesh_t &mesh, const glprog_t &program) {
  // bind appropriate textures
  unsigned int diffuse_counter = 1;
  unsigned int specular_counter = 1;
  unsigned int normal_counter = 1;
  unsigned int height_counter = 1;

  for (unsigned int i = 0; i < mesh.textures.size(); i++) {
    // Acitivate proper texture unit before binding
    glActiveTexture(GL_TEXTURE0 + i);

    // Retrieve texture number (the N in diffuse_textureN)
    std::string number;
    std::string name = mesh.textures[i].type;
    if (name == "texture_diffuse") {
      number = std::to_string(diffuse_counter++);
    } else if (name == "texture_specular") {
      // Transfer unsigned int to stream
      number = std::to_string(specular_counter++);
    } else if (name == "texture_normal") {
      // Transfer unsigned int to stream
      number = std::to_string(normal_counter++);
    } else if (name == "texture_height") {
      // Transfer unsigned int to stream
      number = std::to_string(height_counter++);
    }

    // Set the sampler to the correct texture unit and bind texture
    program.setInt((name + number), i);
    // glUniform1i(glGetUniformLocation(program.program_id, (name +
    // number).c_str()), i);
    glBindTexture(GL_TEXTURE_2D, mesh.textures[i].id);
  }

  // Draw mesh
  glBindVertexArray(mesh.VAO);
  glDrawElements(GL_TRIANGLES, mesh.indices.size(), GL_UNSIGNED_INT, 0);
  glBindVertexArray(0);

  // Set everything back to defaults once configured
  glActiveTexture(GL_TEXTURE0);
}

} // namespace proto
