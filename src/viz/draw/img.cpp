#include "play/draw/img.hpp"

glimg_t::glimg_t(const std::string &image_path_)
    : globj_t{shaders::glimg_vs, shaders::glimg_fs}, image_path{image_path_} {
  glimg_init(*this);
}

void glimg_init(glimg_t &img) {
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
  glGenVertexArrays(1, &img.VAO);
  glBindVertexArray(img.VAO);

  // VBO
  glGenBuffers(1, &img.VBO);
  glBindBuffer(GL_ARRAY_BUFFER, img.VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  // -- Position attribute
  const void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), pos_offset);
  glEnableVertexAttribArray(0);
  // -- Color attribute
  void *color_offset = (void *)(3 * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), color_offset);
  glEnableVertexAttribArray(1);
  // -- Texture coord attribute
  void *texture_offset = (void *)(6 * sizeof(float));
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), texture_offset);
  glEnableVertexAttribArray(2);

  // EBO
  glGenBuffers(1, &img.EBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, img.EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	// FBO
	glGenFramebuffers(1, &img.FBO);
  glBindFramebuffer(GL_FRAMEBUFFER, img.FBO);

	// Load and create a texture
	img.texture = load_texture(img.image_path,
														 img.img_width,
														 img.img_height,
														 img.img_channels);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, img.texture, 0);

	// Render buffer
	unsigned int rbo;
	glGenRenderbuffers(1, &rbo);
	glBindRenderbuffer(GL_RENDERBUFFER, rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, img.img_width, img.img_height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		printf("Framebuffer is not complete!\n");
	}

  // Clean up
	glBindFramebuffer(GL_FRAMEBUFFER, 0);  // Unbind FBO
  glBindBuffer(GL_ARRAY_BUFFER, 0);  // Unbind VBO
  glBindVertexArray(0);  // Unbind VAO
}

void glimg_draw(const glimg_t &img) {
	// bind to framebuffer and draw scene as we normally would to color texture
	// glBindFramebuffer(GL_FRAMEBUFFER, img.FBO);
	// glEnable(GL_DEPTH_TEST); // enable depth testing (is disabled for rendering screen-space quad)

	// // make sure we clear the framebuffer's content
	// glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	// glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  img.program.use();
	glActiveTexture(GL_TEXTURE0);
	glBindVertexArray(img.VAO);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
  glBindVertexArray(0); // Unbind VAO

  // glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void glimg_draw(const glimg_t &img, const glcamera_t &camera) {
	// glBindFramebuffer(GL_FRAMEBUFFER, img.FBO);
	// glEnable(GL_DEPTH_TEST);
	// glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	// glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  img.program.use();
  // img.program.setMat4("projection", glcamera_projection(camera));
  // img.program.setMat4("view", glcamera_view_matrix(camera));
  // img.program.setMat4("model", img.T_SM);

	glActiveTexture(GL_TEXTURE0);
	glBindVertexArray(img.VAO);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
  glBindVertexArray(0); // Unbind VAO

	// glDisable(GL_DEPTH_TEST);
  // glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
