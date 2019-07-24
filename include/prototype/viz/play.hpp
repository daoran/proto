#ifndef PROTOTYPE_VIZ_PLAY_HPP
#define PROTOTYPE_VIZ_PLAY_HPP

#include <iostream>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/euler_angles.hpp>

#include "prototype/viz/mesh.hpp"
#include "prototype/viz/shader.hpp"
#include "prototype/viz/model.hpp"
#include "prototype/viz/camera.hpp"
#include "prototype/viz/draw/draw.hpp"

namespace proto {

// GLOBAL VARIABLES
#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 720
#define WINDOW_TITLE "Play"
std::string nav_mode;
int cursor_x = 0.0;
int cursor_y = 0.0;
float frame_last = 0.0f;
bool first_mouse = true;
glcamera_t camera(SCREEN_WIDTH, SCREEN_HEIGHT,
                  glm::vec3(0.0f, 5.0f, 30.0f));

float focal_length(int image_width, float fov) {
  return (image_width / 2.0) / glm::tan(glm::radians(fov) / 2.0);
}

int line_plane_intersection(const glm::vec3 &p0,
                            const glm::vec3 &p1,
                            const glm::vec3 &n,
                            glm::vec3 &p) {
  const float a = n.x;
  const float b = n.y;
  const float c = n.z;
  // const float d = (a * p1.x + b * p1.y + c * p1.z);
  const float d = 0.0f;

  const glm::vec3 v{p1 - p0};
  std::cout << "dot: " << glm::dot(v, n) << std::endl;
  if (glm::dot(v, n) < 0.0) {
    return -1;
  }

  const float t = -(glm::dot(p0, n) + d) / glm::dot(v, n);
  p = p0 + t * v;

  return 0;
}

void print_vec3(const std::string &title, const glm::vec3 &v) {
  printf("%s: ", title.c_str());
  printf("%f, %f, %f\n", v.x, v.y, v.z);
}

void print_vec4(const std::string &title, const glm::vec4 &v) {
  printf("%s: " , title.c_str());
  printf("%f, %f, %f, %f\n", v.x, v.y, v.z, v.w);
}

void print_mat3(const std::string &title, const glm::mat3 &m) {
  const glm::vec3 c1 = m[0];
  const glm::vec3 c2 = m[1];
  const glm::vec3 c3 = m[2];

  printf("%s:\n", title.c_str());
  printf("%f, %f, %f\n", c1.x, c2.x, c3.x);
  printf("%f, %f, %f\n", c1.y, c2.y, c3.y);
  printf("%f, %f, %f\n", c1.z, c2.z, c3.z);
}

void print_mat4(const std::string &title, const glm::mat4 &m) {
  const glm::vec4 c1 = m[0];
  const glm::vec4 c2 = m[1];
  const glm::vec4 c3 = m[2];
  const glm::vec4 c4 = m[3];

  printf("%s:\n", title.c_str());
  printf("%f, %f, %f, %f\n", c1.x, c2.x, c3.x, c4.x);
  printf("%f, %f, %f, %f\n", c1.y, c2.y, c3.y, c4.y);
  printf("%f, %f, %f, %f\n", c1.z, c2.z, c3.z, c4.z);
  printf("%f, %f, %f, %f\n", c1.w, c2.w, c3.w, c4.w);
}

void play_keyboard_callback(GLFWwindow *window,
                            int key, int scancode,
                            int action, int mods) {
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
}

bool cursor_last_set = false;
int cursor_last_x = 0;
int cursor_last_y = 0;

void play_event_handler(GLFWwindow *window, glcamera_t &camera, const float dt) {
  // Forward, backward, strafe left, strafe right
  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
    glcamera_keyboard_handler(camera, FORWARD, dt);
  }
  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
    glcamera_keyboard_handler(camera, BACKWARD, dt);
  }
  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
    glcamera_keyboard_handler(camera, LEFT, dt);
  }
  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
    glcamera_keyboard_handler(camera, RIGHT, dt);
  }

  // Up, down, rotate left, rotate right
  if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
    glcamera_keyboard_handler(camera, UP, dt);
  }
  if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
    glcamera_keyboard_handler(camera, DOWN, dt);
  }
  if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
    glcamera_keyboard_handler(camera, YAW_LEFT, dt);
  }
  if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
    glcamera_keyboard_handler(camera, YAW_RIGHT, dt);
  }

  // Pan
  {
    int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    if (state == GLFW_PRESS && nav_mode == "" && cursor_last_set == false) {
        nav_mode = "PAN_MODE";
        cursor_last_x = cursor_x;
        cursor_last_y = cursor_y;
        cursor_last_set = true;

    } else if (state == GLFW_PRESS && nav_mode == "PAN_MODE") {
      float dx = (cursor_x - cursor_last_x) * camera.mouse_sensitivity;
      float dy = (cursor_y - cursor_last_y) * camera.mouse_sensitivity;

      camera.position -= camera.right * dx;
      camera.position += camera.up * dy;
      cursor_last_x = cursor_x;
      cursor_last_y = cursor_y;

    } else if (state == GLFW_RELEASE && nav_mode == "PAN_MODE") {
      cursor_last_x = 0;
      cursor_last_y = 0;
      cursor_last_set = false;
      nav_mode = "";
    }
  }

  // Rotate
  {
    int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
    if (state == GLFW_PRESS && nav_mode == "" && cursor_last_set == false) {
      nav_mode = "ROTATE_MODE";
      cursor_last_x = cursor_x;
      cursor_last_y = cursor_y;
      cursor_last_set = true;

    } else if (state == GLFW_PRESS && nav_mode == "ROTATE_MODE") {
      float dx = (cursor_x - cursor_last_x) * camera.mouse_sensitivity;
      float dy = (cursor_y - cursor_last_y) * camera.mouse_sensitivity;

      // Rotate yaw and pitch
      camera.yaw += dx;
      camera.pitch += dy;

      // Constrain pitch
      if (camera.pitch > 89.0f) {
        camera.pitch = 89.0f;
      }
      if (camera.pitch < -89.0f) {
        camera.pitch = -89.0f;
      }

      // Update camera
      glcamera_update(camera);

      cursor_last_x = cursor_x;
      cursor_last_y = cursor_y;

    } else if (state == GLFW_RELEASE && nav_mode == "ROTATE_MODE") {
      cursor_last_x = 0;
      cursor_last_y = 0;
      cursor_last_set = false;
      nav_mode = "";
    }
  }
}

void play_fbsize_callback(GLFWwindow *window,
                          const int width,
                          const int height) {
  // Make sure the viewport matches the new window dimensions
  glViewport(0, 0, width, height);
}

void play_mouse_cursor_callback(GLFWwindow *window, double xpos, double ypos) {
  cursor_x = xpos;
  cursor_y = ypos;
}

void play_scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
  glcamera_scroll_handler(camera, yoffset);
}

float play_get_dt() {
  float frame_now = glfwGetTime();
  float dt = frame_now - frame_last;
  frame_last = frame_now;
  return dt;
}

void play_clear_screen() {
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  // glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  // glClear(GL_COLOR_BUFFER_BIT);
}

void play_poll(GLFWwindow *window) {
  // Swap buffers and poll IO events (keyboard, mouse)
  glfwSwapBuffers(window);
  glfwPollEvents();
}

GLFWwindow *play_init() {
  // Initialize
  if (!glfwInit()) {
    printf("Failed to initialize GLFW!\n");
    return nullptr;
  }

  // Create a windowed mode window and its OpenGL context
  glfwWindowHint(GLFW_SAMPLES, 4);
  GLFWwindow *window = glfwCreateWindow(SCREEN_WIDTH,
                                        SCREEN_HEIGHT,
                                        WINDOW_TITLE,
                                        NULL, NULL);
  if (!window) {
    glfwTerminate();
    printf("Failed to create window!\n");
    return nullptr;
  }
  glfwMakeContextCurrent(window);

  // Event handlers
  // -- Keyboard
  glfwSetKeyCallback(window, play_keyboard_callback);
  // -- Mouse
  glfwSetCursorPosCallback(window, play_mouse_cursor_callback);
  glfwSetScrollCallback(window, play_scroll_callback);
  // -- Window
  glfwSetFramebufferSizeCallback(window, play_fbsize_callback);

  // Glad: load all OpenGL function pointers
  if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
    printf("Failed to initialize GLAD!\n");
    return nullptr;
  }
  // glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_CULL_FACE);

  return window;
}

void play_terminate() {
  glfwTerminate();
}

bool play_loop(GLFWwindow *window) {
  return (glfwWindowShouldClose(window)) ? false : true;
}

} // namespace proto
#endif // PROTOTYPE_VIZ_PLAY_HPP
