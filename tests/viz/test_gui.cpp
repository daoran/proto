#include "proto/munit.hpp"
#include "proto/viz/gui.hpp"
#include "proto/viz/model.hpp"
#include "proto/viz/camera.hpp"
#include "proto/viz/draw.hpp"

#include "opencv2/opencv.hpp"

using namespace proto;

class demo_window_t {
public:
  bool show_demo_window = false;
  bool show_another_window = false;
  float f = 0.0f;
  ImVec4 clear_color = ImVec4{0.45f, 0.55f, 0.60f, 1.00f};

  demo_window_t() {
    float alpha = 2.0f;
    ImGui::SetNextWindowBgAlpha(alpha);
    ImGui::SetNextWindowSize(ImVec2(400, 400));

    bool open = false;
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize;
    ImGui::Begin("Hello, world!", &open, flags);

    ImGui::Text("This is some useful text.");
    ImGui::Checkbox("Demo Window", &show_demo_window);
    ImGui::Checkbox("Another Window", &show_another_window);
    ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
    ImGui::ColorEdit3("clear color", (float *) &clear_color);

    ImGui::End();
  }
};

// Has some memory leak problems
class scene_window_t {
public:
  std::string title_ = "Scene";
  int width_ = 800;
  int height_ = 800;

  float alpha_ = 2.0f;
  bool open_ = false;
  ImGuiWindowFlags flags_ = ImGuiWindowFlags_NoResize;

  GLuint FBO_ = 0;
  GLuint texture_id_ = 0;

  glcamera_t camera_;
  glgrid_t grid_;

  scene_window_t() : camera_{width_, height_, glm::vec3(0.0f, 5.0f, 30.0f)} {
    // scene_window_t() {
    ImGui::SetNextWindowBgAlpha(alpha_);
    ImGui::SetNextWindowSize(ImVec2(width_, height_));
    ImGui::Begin(title_.c_str(), &open_, flags_);

    // Frame buffer
    glGenFramebuffers(1, &FBO_);
    glBindFramebuffer(GL_FRAMEBUFFER, FBO_);

    // Texture
    glGenTextures(1, &texture_id_);
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGB,
                 width_,
                 height_,
                 0,
                 GL_RGB,
                 GL_UNSIGNED_BYTE,
                 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // Add texture to frame buffer
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture_id_, 0);

    // Set the list of draw buffers.
    GLenum draw_buffer[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, draw_buffer);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      FATAL("Frame buffer is not ready!");
    }

    // Attach frame buffer
    glBindFramebuffer(GL_FRAMEBUFFER, FBO_);
    glViewport(0, 0, width_, height_);

    // Draw scene
    glEnable(GL_CULL_FACE);
    grid_.draw(camera_);
    // glmodel_draw(model, camera);
    glDisable(GL_CULL_FACE);

    // Detach frame buffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Draw ImGui window

    const auto start = ImGui::GetCursorScreenPos();
    const auto end_x = start.x + width_;
    const auto end_y = start.y + height_;
    const auto end = ImVec2(end_x, end_y);
    ImGui::GetWindowDrawList()->AddImage((void *) (intptr_t) texture_id_,
                                         ImVec2(start.x, start.y),
                                         end,
                                         ImVec2(0, 1),
                                         ImVec2(1, 0));

    ImGui::End();
  }

  ~scene_window_t() {
    glDeleteTextures(1, &texture_id_);
    glDeleteFramebuffers(1, &FBO_);
  }

  void clear() {
    glDeleteTextures(1, &texture_id_);
    glDeleteFramebuffers(1, &FBO_);
  }
};

int test_gui() {
  gui_t gui{"Play"};
  glgrid_t grid;
  glcamera_t camera{gui.width_, gui.height_, glm::vec3(0.0f, 5.0f, 30.0f)};

  while (gui.ok()) {
    gui.poll();

    // demo_window_t demo;
    // scene_window_t scene;
    // gui.render(true);

    gui.clear();
    glEnable(GL_CULL_FACE);
    grid.draw(camera);
    glDisable(GL_CULL_FACE);
    gui.render();
  }

  gui.close();
  return 0;
}

int test_gui_imshow() {
  gui_t gui{"Play"};
  gui_imshow_t imshow{"Image", "test_data/viz/container.jpg"};

  while (gui.ok()) {
    gui.poll();
    imshow.show();
    gui.clear();
    gui.render();
  }

  gui.close();
  return 0;
}

int test_gui_imshow_update() {
  gui_t gui{"Play"};

  // Open webcam
  cv::VideoCapture capture(0);
  if (!capture.isOpened()) {
    printf("Failed to open camera!\n");
    return -1;
  }

  // Create imshow gui
  gui_imshow_t imshow{"Camera"};

  // Show GUI
  cv::Mat frame;
  while (gui.ok()) {
    gui.poll();

    // Get new frame and update the window
    capture >> frame;
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    const int img_w = 320;
    const int img_h = 240;
    cv::resize(frame, frame, cv::Size(img_w, img_h));
    imshow.show(img_w, img_h, frame.channels(), frame.data);

    gui.clear();
    gui.render();
  }
  gui.close();

  return 0;
}

void test_suite() {
  // MU_ADD_TEST(test_gui);
  // MU_ADD_TEST(test_gui_imshow);
  MU_ADD_TEST(test_gui_imshow_update);
}

MU_RUN_TESTS(test_suite);
