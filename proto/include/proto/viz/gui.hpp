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

#include "proto/core/core.hpp"
#include "proto/viz/texture.hpp"

namespace proto {

class gui_t {
public:
  GLFWwindow *gui_;
  const std::string title_;
  int width_;
  int height_;
  float time_last_;
  ImVec4 clear_color_{0.45f, 0.55f, 0.60f, 1.00f};

  gui_t(const std::string &title, const int width, const int height);
  gui_t(const std::string &title);
  ~gui_t();

  static void error_callback(int error, const char *description);
  static void window_callback(GLFWwindow *window,
                              const int width,
                              const int height);
  static void cursor_callback(GLFWwindow *window,
                              const double xpos,
                              const double ypos);

  float dt();
  bool ok();
  void poll();
  void clear();
  void render(const bool clear_gui = false);
  void close();
};

class gui_imshow_t {
public:
  bool ok_ = false;

  std::string title_;
  GLuint FBO_;
  GLuint RBO_;

  std::string img_path_;
  int img_width_ = 0;
  int img_height_ = 0;
  int img_channels_ = 0;
  GLuint img_id_;

  gui_imshow_t(const std::string &title);
  gui_imshow_t(const std::string &title, const std::string &img_path);
  gui_imshow_t(const std::string &title,
               const int img_width,
               const int img_height,
               const int img_channels,
               const unsigned char *data);

  void init(const std::string &title,
            const int img_width,
            const int img_height,
            const int img_channels,
            const unsigned char *data);

  bool ok();
  void update(void *pixels);
  void show();
  void show(const int img_width,
            const int img_height,
            const int img_channels,
            const unsigned char *data);
};

} // namespace proto
#endif // PROTO_VIZ_GUI_HPP
