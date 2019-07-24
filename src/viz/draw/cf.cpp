#include "prototype/viz/draw/cf.hpp"

namespace proto {

glcf_t::glcf_t() : globj_t{shaders::glcf_vs, shaders::glcf_fs} {
  glcf_init(*this);
}

void glcf_init(glcf_t &cf) {
  // Form the camera fov frame
	float hfov = cf.fov / 2.0f;
  float z = cf.scale;
	float hwidth = z * glm::tan(hfov);
  const glm::vec3 lb{-hwidth, hwidth, z};   // Left bottom
  const glm::vec3 lt{-hwidth, -hwidth, z};  // Left top
  const glm::vec3 rt{hwidth, -hwidth, z};		// Right top
  const glm::vec3 rb{hwidth, hwidth, z};		// Right bottom

  static const GLfloat vertices[] = {
		// Rectangle frame
		// -- Left bottom to left top
		lb.x, lb.y, lb.z,
		lt.x, lt.y, lt.z,
		// -- Left top to right top
		lt.x, lt.y, lt.z,
		rt.x, rt.y, rt.z,
		// -- Right top to right bottom
		rt.x, rt.y, rt.z,
		rb.x, rb.y, rb.z,
		// -- Right bottom to left bottom
		rb.x, rb.y, rb.z,
		lb.x, lb.y, lb.z,
		// Rectangle frame to origin
		// -- Origin to left bottom
		0.0f, 0.0f, 0.0f,
		lb.x, lb.y, lb.z,
		// -- Origin to left top
		0.0f, 0.0f, 0.0f,
		lt.x, lt.y, lt.z,
		// -- Origin to right top
		0.0f, 0.0f, 0.0f,
		rt.x, rt.y, rt.z,
		// -- Origin to right bottom
		0.0f, 0.0f, 0.0f,
		rb.x, rb.y, rb.z
  };
	const size_t nb_lines = 8;
	const size_t nb_vertices = nb_lines * 2;
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;

  // VAO
  glGenVertexArrays(1, &cf.VAO);
  glBindVertexArray(cf.VAO);

  // VBO
  glGenBuffers(1, &cf.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, cf.VBO);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0);  // Unbind VBO
  glBindVertexArray(0);  // Unbind VAO
}

void glcf_draw(const glcf_t &grid, const glcamera_t &camera) {
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
	const size_t nb_lines = 8;
	const size_t nb_vertices = nb_lines * 2;
  glBindVertexArray(grid.VAO);
  glDrawArrays(GL_LINES, 0, nb_vertices);
  glBindVertexArray(0); // Unbind VAO

  // Restore original line width
  glLineWidth(original_line_width);
}

} // namespace proto
