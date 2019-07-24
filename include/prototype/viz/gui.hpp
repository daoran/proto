#ifndef PROTOTOYE_VIZ_GUI_HPP
#define PROTOTOYE_VIZ_GUI_HPP

#include <stdio.h>
#include <string>

#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "prototype/viz/texture.hpp"

namespace proto {

struct gui_imshow_t {
  std::string title;
  GLuint FBO;
  GLuint RBO;

  std::string img_path;
  int img_width = 0;
  int img_height = 0;
  int img_channels = 0;
  GLuint img_id;

  gui_imshow_t(const std::string &title_, const std::string &img_path_)
      : title{title_}, img_path{img_path_} {
    // FBO
    glGenFramebuffers(1, &FBO);
    glBindFramebuffer(GL_FRAMEBUFFER, FBO);

    // Load and create a texture
    img_id = load_texture(img_path, img_width, img_height, img_channels);
    glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D,
                           img_id,
                           0);

    // Render buffer
    glGenRenderbuffers(1, &RBO);
    glBindRenderbuffer(GL_RENDERBUFFER, RBO);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, img_width, img_height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, RBO);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      printf("Framebuffer is not complete!\n");
    }

    // Clean up
    glBindFramebuffer(GL_FRAMEBUFFER, 0);  // Unbind FBO
  }

  gui_imshow_t(const std::string &title_,
               const int img_width_,
               const int img_height_,
               const int img_channels_,
               const unsigned char *data_)
      : title{title_},
        img_width{img_width_},
        img_height{img_height_},
        img_channels{img_channels_} {
    // FBO
    glGenFramebuffers(1, &FBO);
    glBindFramebuffer(GL_FRAMEBUFFER, FBO);

    // Load and create a texture
    img_id = load_texture(img_width, img_height, img_channels, data_);
    glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D,
                           img_id,
                           0);

    // Render buffer
    glGenRenderbuffers(1, &RBO);
    glBindRenderbuffer(GL_RENDERBUFFER, RBO);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, img_width, img_height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, RBO);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      printf("Framebuffer is not complete!\n");
    }

    // Clean up
    glBindFramebuffer(GL_FRAMEBUFFER, 0);  // Unbind FBO
  }

  void update(void *pixels) {
    GLenum img_format;
    switch (img_channels) {
    case 1: img_format = GL_RED; break;
    case 3: img_format = GL_RGB; break;
    case 4: img_format = GL_RGBA; break;
    }

    glBindTexture(GL_TEXTURE_2D, img_id);
    glTexSubImage2D(
      GL_TEXTURE_2D,
      0,
      0,
      0,
      img_width,
      img_height,
      img_format,
      GL_UNSIGNED_BYTE,
      pixels
    );
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  void show() {
		// Set window alpha
    float alpha = 2.0f;
    ImGui::SetNextWindowBgAlpha(alpha);

		// Set window size
    ImVec2 win_size(img_width + 15, img_height + 35);
    ImGui::SetNextWindowSize(win_size);

		// Begin window
    bool open = false;
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize;
    ImGui::Begin(title.c_str(), &open, flags);

		// Add image
    const auto start = ImGui::GetCursorScreenPos();
    const auto end_x = start.x + img_width;
    const auto end_y = start.y + img_height;
    const auto end = ImVec2(end_x, end_y);
    ImGui::GetWindowDrawList()->AddImage(
      (void*)(intptr_t) img_id,
      ImVec2(start.x, start.y),
      end
    );

		// End window
    ImGui::End();
  }
};

GLFWwindow *gui_init();

bool gui_is_ok(GLFWwindow *window);

void gui_poll();

void gui_render(GLFWwindow *window,
                const ImVec4 clear_color = ImVec4{0.45f, 0.55f, 0.60f, 1.00f});

void gui_close(GLFWwindow *window);

} // namespace proto
#endif  // PROTOTYPE_VIZ_GUI_HPP
