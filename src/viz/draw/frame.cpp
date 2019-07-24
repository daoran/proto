#include "prototype/viz/draw/frame.hpp"

namespace proto {

glframe_t::glframe_t() : globj_t{shaders::glframe_vs, shaders::glframe_fs} {
  glframe_init(*this);
}

void glframe_init(glframe_t &grid) {
  // Vertices
  // clang-format off
  static const GLfloat vertices[] = {
    // Line 1 : x-axis + color
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    // Line 2 : y-axis + color
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    // Line 3 : z-axis + color
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f
  };
  const size_t nb_vertices = 6;
  const size_t buffer_size = sizeof(GLfloat) * 6 * nb_vertices;
  // clang-format on

  // VAO
  glGenVertexArrays(1, &grid.VAO);
  glBindVertexArray(grid.VAO);

  // VBO
  glGenBuffers(1, &grid.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, grid.VBO);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  // -- Position attribute
  size_t vertex_size = 6 * sizeof(float);
  void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, pos_offset);
  glEnableVertexAttribArray(0);
  // -- Color attribute
  void *color_offset = (void *)(3 * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, vertex_size, color_offset);
  glEnableVertexAttribArray(1);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0);  // Unbind VBO
  glBindVertexArray(0);  // Unbind VAO
}

void glframe_draw(const glframe_t &grid, const glcamera_t &camera) {
  grid.program.use();
  grid.program.setMat4("projection", glcamera_projection(camera));
  grid.program.setMat4("view", glcamera_view_matrix(camera));
  grid.program.setMat4("model", grid.T_SM);

  // Store original line width
  float original_line_width = 0.0f;
  glGetFloatv(GL_LINE_WIDTH, &original_line_width);

  // Set line width
  glLineWidth(grid.line_width);

  // Draw frame
  glBindVertexArray(grid.VAO);
  glDrawArrays(GL_LINES, 0, 6);
  glBindVertexArray(0); // Unbind VAO

  // Restore original line width
  glLineWidth(original_line_width);
}

} // namespace proto
