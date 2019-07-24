#include "play/draw/grid.hpp"

glgrid_t::glgrid_t() : globj_t{shaders::glgrid_vs, shaders::glgrid_fs} {
  glgrid_init(*this);
}

static GLfloat *glgrid_create_vertices(int grid_size) {
  // Allocate memory for vertices
  int nb_lines = (grid_size + 1) * 2;
  int nb_vertices = nb_lines * 2;
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;
  GLfloat *vertices = (GLfloat *) malloc(buffer_size);

  // Setup
  const float min_x = -1.0f * (float) grid_size / 2.0f;
  const float max_x = (float) grid_size / 2.0f;
  const float min_z = -1.0f * (float) grid_size / 2.0f;
  const float max_z = (float) grid_size / 2.0f;

  // Row vertices
  float z = min_z;
  int vert_idx = 0;
  for (int i = 0; i < ((grid_size + 1) * 2); i++) {
    if ((i % 2) == 0) {
      vertices[(vert_idx * 3)] = min_x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = z;
    } else {
      vertices[(vert_idx * 3)] = max_x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = z;
      z += 1.0f;
    }
    vert_idx++;
  }

  // Column vertices
  float x = min_x;
  for (int j = 0; j < ((grid_size + 1) * 2); j++) {
    if ((j % 2) == 0) {
      vertices[(vert_idx * 3)] = x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = min_z;
    } else {
      vertices[(vert_idx * 3)] = x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = max_z;
      x += 1.0f;
    }
    vert_idx++;
  }

  return vertices;
}

void glgrid_init(glgrid_t &grid) {
  // Create vertices
  const int nb_lines = (grid.grid_size + 1) * 2;
  const int nb_vertices = nb_lines * 2;
  GLfloat *vertices = glgrid_create_vertices(grid.grid_size);
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;

  // VAO
  glGenVertexArrays(1, &grid.VAO);
  glBindVertexArray(grid.VAO);

  // VBO
  glGenBuffers(1, &grid.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, grid.VBO);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0);  // Unbind VBO
  glBindVertexArray(0);  // Unbind VAO
  free(vertices);
}

void glgrid_draw(const glgrid_t &grid, const glcamera_t &camera) {
  grid.program.use();
  grid.program.setMat4("projection", glcamera_projection(camera));
  grid.program.setMat4("view", glcamera_view_matrix(camera));
  grid.program.setMat4("model", grid.T_SM);

  const int nb_lines = (grid.grid_size + 1) * 2;
  const int nb_vertices = nb_lines * 2;

  glBindVertexArray(grid.VAO);
  glDrawArrays(GL_LINES, 0, nb_vertices);
  glBindVertexArray(0); // Unbind VAO
}
