#include "proto/viz/draw.hpp"

namespace proto {

globj_t::globj_t(const char *vs, const char *fs) : program_{vs, fs} {}

void globj_t::pos(const glm::vec3 &pos) { T_SM_ = glm::translate(T_SM_, pos); }

glm::vec3 globj_t::pos() { return glm::vec3(T_SM_[3]); }

glm::mat3 globj_t::rot() {
  glm::vec3 col_0(T_SM_[0]);
  glm::vec3 col_1(T_SM_[1]);
  glm::vec3 col_2(T_SM_[2]);
  glm::mat3 R(col_0, col_1, col_2);
  return R;
}

glcf_t::glcf_t() : globj_t{shaders::glcf_vs, shaders::glcf_fs} {
  // Form the camera fov frame
  float hfov = fov_ / 2.0f;
  float z = scale_;
  float hwidth = z * glm::tan(hfov);
  const glm::vec3 lb{-hwidth, hwidth, z};  // Left bottom
  const glm::vec3 lt{-hwidth, -hwidth, z}; // Left top
  const glm::vec3 rt{hwidth, -hwidth, z};  // Right top
  const glm::vec3 rb{hwidth, hwidth, z};   // Right bottom

  // Rectangle frame
  // clang-format off
  static const GLfloat vertices[] = {
    // -- Left bottom to left top
    lb.x, lb.y, lb.z, lt.x, lt.y, lt.z,
    // -- Left top to right top
    lt.x, lt.y, lt.z, rt.x, rt.y, rt.z,
    // -- Right top to right bottom
    rt.x, rt.y, rt.z, rb.x, rb.y, rb.z,
    // -- Right bottom to left bottom
    rb.x, rb.y, rb.z, lb.x, lb.y, lb.z,
    // Rectangle frame to origin
    // -- Origin to left bottom
    0.0f, 0.0f, 0.0f, lb.x, lb.y, lb.z,
    // -- Origin to left top
    0.0f, 0.0f, 0.0f, lt.x, lt.y, lt.z,
    // -- Origin to right top
    0.0f, 0.0f, 0.0f, rt.x, rt.y, rt.z,
    // -- Origin to right bottom
    0.0f, 0.0f, 0.0f, rb.x, rb.y, rb.z
  };
  // clang-format on
  const size_t nb_lines = 8;
  const size_t nb_vertices = nb_lines * 2;
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;

  // VAO
  glGenVertexArrays(1, &VAO_);
  glBindVertexArray(VAO_);

  // VBO
  glGenBuffers(1, &VBO_);
  glBindBuffer(GL_ARRAY_BUFFER, VBO_);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  glVertexAttribPointer(0,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        3 * sizeof(float),
                        (void *) 0);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
}

glcf_t::~glcf_t() {
  glDeleteVertexArrays(1, &VAO_);
  glDeleteBuffers(1, &VBO_);
}

void glcf_t::draw(const glcamera_t &camera) {
  program_.use();
  program_.setMat4("projection", glcamera_projection(camera));
  program_.setMat4("view", glcamera_view_matrix(camera));
  program_.setMat4("model", T_SM_);

  // Store original line width
  float original_line_width = 0.0f;
  glGetFloatv(GL_LINE_WIDTH, &original_line_width);

  // Set line width
  glLineWidth(line_width_);

  // Draw frame
  const size_t nb_lines = 8;
  const size_t nb_vertices = nb_lines * 2;
  glBindVertexArray(VAO_);
  glDrawArrays(GL_LINES, 0, nb_vertices);
  glBindVertexArray(0); // Unbind VAO

  // Restore original line width
  glLineWidth(original_line_width);
}

glcube_t::glcube_t() : globj_t{shaders::glcube_vs, shaders::glcube_fs} {
  // Vertices
  // clang-format off
  const float cube_size = cube_size_;
  const float r = color_.x;
  const float g = color_.y;
  const float b = color_.z;
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
  glGenVertexArrays(1, &VAO_);
  glBindVertexArray(VAO_);

  // VBO
  glGenBuffers(1, &VBO_);
  glBindBuffer(GL_ARRAY_BUFFER, VBO_);
  glBufferData(GL_ARRAY_BUFFER, vertex_buffer_size, vertices, GL_STATIC_DRAW);
  // -- Position attribute
  size_t vertex_size = 6 * sizeof(float);
  void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, pos_offset);
  glEnableVertexAttribArray(0);
  // -- Color attribute
  void *color_offset = (void *) (3 * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, vertex_size, color_offset);
  glEnableVertexAttribArray(1);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
}

glcube_t::~glcube_t() {
  glDeleteVertexArrays(1, &VAO_);
  glDeleteBuffers(1, &VBO_);
}

void glcube_t::draw(const glcamera_t &camera) {
  program_.use();
  program_.setMat4("projection", glcamera_projection(camera));
  program_.setMat4("view", glcamera_view_matrix(camera));
  program_.setMat4("model", T_SM_);

  // 12 x 3 indices starting at 0 -> 12 triangles -> 6 squares
  glBindVertexArray(VAO_);
  glDrawArrays(GL_TRIANGLES, 0, 36);
  glBindVertexArray(0); // Unbind VAO
}

glframe_t::glframe_t() : globj_t{shaders::glframe_vs, shaders::glframe_fs} {
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
  glGenVertexArrays(1, &VAO_);
  glBindVertexArray(VAO_);

  // VBO
  glGenBuffers(1, &VBO_);
  glBindBuffer(GL_ARRAY_BUFFER, VBO_);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  // -- Position attribute
  size_t vertex_size = 6 * sizeof(float);
  void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, pos_offset);
  glEnableVertexAttribArray(0);
  // -- Color attribute
  void *color_offset = (void *) (3 * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, vertex_size, color_offset);
  glEnableVertexAttribArray(1);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
}

glframe_t::~glframe_t() {
  glDeleteVertexArrays(1, &VAO_);
  glDeleteBuffers(1, &VBO_);
}

void glframe_t::draw(const glcamera_t &camera) {
  program_.use();
  program_.setMat4("projection", glcamera_projection(camera));
  program_.setMat4("view", glcamera_view_matrix(camera));
  program_.setMat4("model", T_SM_);

  // Store original line width
  float original_line_width = 0.0f;
  glGetFloatv(GL_LINE_WIDTH, &original_line_width);

  // Set line width
  glLineWidth(line_width_);

  // Draw frame
  glBindVertexArray(VAO_);
  glDrawArrays(GL_LINES, 0, 6);
  glBindVertexArray(0); // Unbind VAO

  // Restore original line width
  glLineWidth(original_line_width);
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

glgrid_t::glgrid_t() : globj_t{shaders::glgrid_vs, shaders::glgrid_fs} {
  // Create vertices
  const int nb_lines = (grid_size_ + 1) * 2;
  const int nb_vertices = nb_lines * 2;
  GLfloat *vertices = glgrid_create_vertices(grid_size_);
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;

  // VAO
  glGenVertexArrays(1, &VAO_);
  glBindVertexArray(VAO_);

  // VBO
  glGenBuffers(1, &VBO_);
  glBindBuffer(GL_ARRAY_BUFFER, VBO_);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  glVertexAttribPointer(0,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        3 * sizeof(float),
                        (void *) 0);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
  free(vertices);
}

glgrid_t::~glgrid_t() {
  glDeleteVertexArrays(1, &VAO_);
  glDeleteBuffers(1, &VBO_);
}

void glgrid_t::draw(const glcamera_t &camera) {
  program_.use();
  program_.setMat4("projection", glcamera_projection(camera));
  program_.setMat4("view", glcamera_view_matrix(camera));
  program_.setMat4("model", T_SM_);

  const int nb_lines = (grid_size_ + 1) * 2;
  const int nb_vertices = nb_lines * 2;

  glBindVertexArray(VAO_);
  glDrawArrays(GL_LINES, 0, nb_vertices);
  glBindVertexArray(0); // Unbind VAO
}

glplane_t::glplane_t(const std::string &image_path)
    : globj_t{shaders::glplane_vs, shaders::glplane_fs}, image_path_{
                                                             image_path} {
  // clang-format off
  const float vertices[] = {
    // Positions         // Colors           // Texture coords
    0.5f,  0.5f,  0.0f,  1.0f, 1.0f, 1.0f,   1.0f, 1.0f,  // Top right
    0.5f, -0.5f,  0.0f,  1.0f, 1.0f, 1.0f,   1.0f, 0.0f,  // Bottom right
    -0.5f, -0.5f, 0.0f,  1.0f, 1.0f, 1.0f,   0.0f, 0.0f,  // Bottom left
    -0.5f,  0.5f, 0.0f,  1.0f, 1.0f, 1.0f,   0.0f, 1.0f   // Top left
  };
  const unsigned int indices[] = {
    0, 1, 3, // first triangle
    1, 2, 3  // second triangle
  };
  // clang-format on

  // VAO
  glGenVertexArrays(1, &VAO_);
  glBindVertexArray(VAO_);

  // VBO
  glGenBuffers(1, &VBO_);
  glBindBuffer(GL_ARRAY_BUFFER, VBO_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  // -- Position attribute
  const void *pos_offset = (void *) 0;
  glVertexAttribPointer(0,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        8 * sizeof(float),
                        pos_offset);
  glEnableVertexAttribArray(0);
  // -- Color attribute
  void *color_offset = (void *) (3 * sizeof(float));
  glVertexAttribPointer(1,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        8 * sizeof(float),
                        color_offset);
  glEnableVertexAttribArray(1);
  // -- Texture coord attribute
  void *texture_offset = (void *) (6 * sizeof(float));
  glVertexAttribPointer(2,
                        2,
                        GL_FLOAT,
                        GL_FALSE,
                        8 * sizeof(float),
                        texture_offset);
  glEnableVertexAttribArray(2);

  // EBO
  glGenBuffers(1, &EBO_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               sizeof(indices),
               indices,
               GL_STATIC_DRAW);

  // FBO
  glGenFramebuffers(1, &FBO_);
  glBindFramebuffer(GL_FRAMEBUFFER, FBO_);

  // Load and create a texture
  texture_ = load_texture(image_path_, img_width_, img_height_, img_channels_);
  glFramebufferTexture2D(GL_FRAMEBUFFER,
                         GL_COLOR_ATTACHMENT0,
                         GL_TEXTURE_2D,
                         texture_,
                         0);

  // Render buffer
  unsigned int rbo;
  glGenRenderbuffers(1, &rbo);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo);
  glRenderbufferStorage(GL_RENDERBUFFER,
                        GL_DEPTH24_STENCIL8,
                        img_width_,
                        img_height_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER,
                            GL_DEPTH_STENCIL_ATTACHMENT,
                            GL_RENDERBUFFER,
                            rbo);

  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    printf("Framebuffer is not complete!\n");
  }

  // Clean up
  glBindFramebuffer(GL_FRAMEBUFFER, 0); // Unbind FBO
  glBindBuffer(GL_ARRAY_BUFFER, 0);     // Unbind VBO
  glBindVertexArray(0);                 // Unbind VAO
}

void glplane_t::draw(const glcamera_t &camera) {
  UNUSED(camera);
  // glBindFramebuffer(GL_FRAMEBUFFER, FBO_);
  // glEnable(GL_DEPTH_TEST);
  // glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  program_.use();
  // program.setMat4("projection", glcamera_projection(camera));
  // program.setMat4("view", glcamera_view_matrix(camera));
  // program.setMat4("model", T_SM);

  glActiveTexture(GL_TEXTURE0);
  glBindVertexArray(VAO_);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
  glBindVertexArray(0); // Unbind VAO

  // glDisable(GL_DEPTH_TEST);
  // glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

} // namespace proto
