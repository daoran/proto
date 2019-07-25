#include "prototype/viz/gui.hpp"

namespace proto {

static void glfw_cursor_cb(GLFWwindow *window, double xpos, double ypos) {
  UNUSED(window);
  // cursor_x = xpos;
  // cursor_y = ypos;
}

// static void glfw_scroll_cb(GLFWwindow *window, double xoffset, double yoffset) {
//   glcamera_scroll_handler(camera, yoffset);
// }

gui_t::gui_t(const std::string &title,
             const int width,
             const int height)
  : title_{title}, width_{width}, height_{height} {
  // Setup window
  if (!glfwInit()) {
    FATAL("Failed to set GLFW error callback!");
  }

  // Decide GL+GLSL versions
  // GL 3.0 + GLSL 130
  const char* glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
  //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only

  // Create window with graphics context
  gui_ = glfwCreateWindow(
    width_,
    height_,
    title_.c_str(),
    NULL,
    NULL
  );
  if (gui_ == NULL) {
    FATAL("Failed to create a GLFW window!");
  }
  glfwSetWindowAspectRatio(gui_, width_, height_);
  glfwMakeContextCurrent(gui_);
  glfwSwapInterval(1); // Enable vsync

  // Event handlers
  // -- Error
  glfwSetErrorCallback(error_callback);
  // -- Keyboard
  // glfwSetKeyCallback(gui_, keyboard_callback);
  // -- Mouse
  // glfwSetCursorPosCallback(gui_, cursor_callback);
  // glfwSetScrollCallback(gui_, scroll_callback);
  // -- Window
  glfwSetFramebufferSizeCallback(gui_, window_callback);

  // Initialize OpenGL loader
  bool err = gladLoadGL() == 0;
  if (err) {
    FATAL("Failed to initialize OpenGL loader!");
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  // ImGuiIO& io = ImGui::GetIO();

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsClassic();

  // Setup Platform/Renderer bindings
  ImGui_ImplGlfw_InitForOpenGL(gui_, true);
  ImGui_ImplOpenGL3_Init(glsl_version);
}

gui_t::gui_t(const std::string &title)
  : gui_t{title, 1280, 720} {}

gui_t::~gui_t() {
  glfwTerminate();
}

void gui_t::error_callback(int error, const char *description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void gui_t::window_callback(GLFWwindow *window,
                            const int width,
                            const int height) {
  UNUSED(window);
  glViewport(0, 0, width, height);
}

float gui_t::dt() {
  float time_now = glfwGetTime();
  float dt = time_now - time_last_;
  time_last_ = time_now;
  return dt;
}

bool gui_t::ok() {
  return !glfwWindowShouldClose(gui_);
}

void gui_t::poll() {
  glfwPollEvents();
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  glfwGetWindowSize(gui_, &width_, &height_);
}

void gui_t::clear() {
  glClearColor(clear_color_.x, clear_color_.y, clear_color_.z, clear_color_.w);
  glClear(GL_COLOR_BUFFER_BIT);
}

void gui_t::render(const bool clear_gui) {
  if (clear_gui) {
    clear();
  }

  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  // glfwMakeContextCurrent(gui_);
  glfwSwapBuffers(gui_);
}

void gui_t::close() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(gui_);
  glfwTerminate();
}

gui_imshow_t::gui_imshow_t(const std::string &title,
                           const std::string &img_path)
      : title_{title}, img_path_{img_path} {
  // Frame buffer
  glGenFramebuffers(1, &FBO_);
  glBindFramebuffer(GL_FRAMEBUFFER, FBO_);

  // Load and create a texture
  img_id_ = load_texture(img_path, img_width_, img_height_, img_channels_);
  glFramebufferTexture2D(GL_FRAMEBUFFER,
                          GL_COLOR_ATTACHMENT0,
                          GL_TEXTURE_2D,
                          img_id_,
                          0);

  // Render buffer
  glGenRenderbuffers(1, &RBO_);
  glBindRenderbuffer(GL_RENDERBUFFER, RBO_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, img_width_, img_height_);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, RBO_);

  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    printf("Framebuffer is not complete!\n");
  }

  // Clean up
  glBindFramebuffer(GL_FRAMEBUFFER, 0);  // Unbind FBO
}

gui_imshow_t::gui_imshow_t(const std::string &title,
                           const int img_width,
                           const int img_height,
                           const int img_channels,
                           const unsigned char *data)
    : title_{title},
      img_width_{img_width},
      img_height_{img_height},
      img_channels_{img_channels} {
  // FBO
  glGenFramebuffers(1, &FBO_);
  glBindFramebuffer(GL_FRAMEBUFFER, FBO_);

  // Load and create a texture
  img_id_ = load_texture(img_width_, img_height_, img_channels_, data);
  glFramebufferTexture2D(GL_FRAMEBUFFER,
                          GL_COLOR_ATTACHMENT0,
                          GL_TEXTURE_2D,
                          img_id_,
                          0);

  // Render buffer
  glGenRenderbuffers(1, &RBO_);
  glBindRenderbuffer(GL_RENDERBUFFER, RBO_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, img_width, img_height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, RBO_);

  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    printf("Framebuffer is not complete!\n");
  }

  // Clean up
  glBindFramebuffer(GL_FRAMEBUFFER, 0);  // Unbind FBO
}

void gui_imshow_t::update(void *pixels) {
  GLenum img_format;
  switch (img_channels_) {
  case 1: img_format = GL_RED; break;
  case 3: img_format = GL_RGB; break;
  case 4: img_format = GL_RGBA; break;
  }

  glBindTexture(GL_TEXTURE_2D, img_id_);
  glTexSubImage2D(
    GL_TEXTURE_2D,
    0,
    0,
    0,
    img_width_,
    img_height_,
    img_format,
    GL_UNSIGNED_BYTE,
    pixels
  );
  glBindTexture(GL_TEXTURE_2D, 0);
}

void gui_imshow_t::show() {
  // Set window alpha
  float alpha = 2.0f;
  ImGui::SetNextWindowBgAlpha(alpha);

  // Set window size
  ImVec2 win_size(img_width_ + 15, img_height_ + 35);
  ImGui::SetNextWindowSize(win_size);

  // Begin window
  bool open = false;
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize;
  ImGui::Begin(title_.c_str(), &open, flags);

  // Add image
  const auto start = ImGui::GetCursorScreenPos();
  const auto end_x = start.x + img_width_;
  const auto end_y = start.y + img_height_;
  const auto end = ImVec2(end_x, end_y);
  ImGui::GetWindowDrawList()->AddImage(
    (void*)(intptr_t) img_id_,
    ImVec2(start.x, start.y),
    end
  );

  // End window
  ImGui::End();
}

} // namespace proto
