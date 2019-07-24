#include "prototype/viz/gui.hpp"

namespace proto {

static void glfw_error_callback(int error, const char* description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void fbsize_callback(GLFWwindow *window,
                          const int width,
                          const int height) {
  // Make sure the viewport matches the new window dimensions
  glViewport(0, 0, width, height);
}

GLFWwindow *gui_init() {
  // Setup window
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit()) {
      return nullptr;
  }

  // Decide GL+GLSL versions
  // GL 3.0 + GLSL 130
  const char* glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
  //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only

  // Create window with graphics context
  const int window_width = 1280;
  const int window_height = 720;
  const std::string title = "Play";
  GLFWwindow* window = glfwCreateWindow(
    window_width,
    window_height,
    title.c_str(),
    NULL,
    NULL
  );
  if (window == NULL) {
    return nullptr;
  }
  glfwSetWindowAspectRatio(window, window_width, window_height);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync

  // Events handler
  glfwSetFramebufferSizeCallback(window, fbsize_callback);
  // glfwSetKeyCallback(window, play_keyboard_callback);

  // Initialize OpenGL loader
  bool err = gladLoadGL() == 0;
  if (err) {
    fprintf(stderr, "Failed to initialize OpenGL loader!\n");
    return nullptr;
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsClassic();

  // Setup Platform/Renderer bindings
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  return window;
}

bool gui_is_ok(GLFWwindow *window) {
  return !glfwWindowShouldClose(window);
}

void gui_poll() {
  glfwPollEvents();
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void gui_render(GLFWwindow *window, const ImVec4 clear_color) {
  ImGui::Render();
  glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  glfwMakeContextCurrent(window);
  glfwSwapBuffers(window);
}

void gui_close(GLFWwindow *window) {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
}

} // namespace proto
