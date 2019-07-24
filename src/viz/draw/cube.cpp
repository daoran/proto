#include "play/draw/cube.hpp"

glcube_t::glcube_t() : globj_t{shaders::glcube_vs, shaders::glcube_fs} {
  glcube_init(*this);
}

void glcube_init(glcube_t &cube) {
  // Vertices
  // clang-format off
  const float cube_size = cube.cube_size;
  const float r = cube.color.x;
  const float g = cube.color.y;
  const float b = cube.color.z;
  static const GLfloat vertices[] = {
    // Triangle 1 : begin
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    // Triangle 1 : end
    // Triangle 2 : begin
    cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 2 : end
    // Triangle 3 : begin
    cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 3 : end
    // Triangle 4 : begin
    cube_size, cube_size, -cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 4 : end
    // Triangle 5 : begin
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 5 : end
    // Triangle 6 : begin
    cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 6 : end
    // Triangle 7 : begin
    -cube_size, cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b,
    // Triangle 7 : end
    // Triangle 8 : begin
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 8 : end
    // Triangle 9 : begin
    cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b,
    // Triangle 9 : end
    // Triangle 10 : begin
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 10 : end
    // Triangle 11 : begin
    cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    // Triangle 11 : end
    // Triangle 12 : begin
    cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b
    // Triangle 12 : end
  };
  const size_t nb_triangles = 12;
  const size_t vertices_per_triangle = 3;
  const size_t nb_vertices = vertices_per_triangle * nb_triangles;
  const size_t vertex_buffer_size = sizeof(float) * 6 * nb_vertices;
  // clang-format on

  // VAO
  glGenVertexArrays(1, &cube.VAO);
  glBindVertexArray(cube.VAO);

  // VBO
  glGenBuffers(1, &cube.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, cube.VBO);
  glBufferData(GL_ARRAY_BUFFER, vertex_buffer_size, vertices, GL_STATIC_DRAW);
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

void glcube_draw(const glcube_t &cube, const glcamera_t &camera) {
  cube.program.use();
  cube.program.setMat4("projection", glcamera_projection(camera));
  cube.program.setMat4("view", glcamera_view_matrix(camera));
  cube.program.setMat4("model", cube.T_SM);

  // 12 x 3 indices starting at 0 -> 12 triangles -> 6 squares
  glBindVertexArray(cube.VAO);
  glDrawArrays(GL_TRIANGLES, 0, 36);
  glBindVertexArray(0); // Unbind VAO
}
