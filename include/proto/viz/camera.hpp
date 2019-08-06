#ifndef PROTOTOYE_VIZ_CAMERA_HPP
#define PROTOTOYE_VIZ_CAMERA_HPP

#include <stdio.h>
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace proto {

enum glcamera_movement_t {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  UP,
  DOWN,
  YAW_LEFT,
  YAW_RIGHT,
  PAN
};

class glcamera_t {
public:
  glm::vec3 world_up = glm::vec3(0.0f, 1.0f, 0.0f);

  // State
  glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f);
  glm::vec3 right = glm::vec3(-1.0f, 0.0f, 0.0f);
  glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
  glm::vec3 front = glm::vec3(0.0f, 0.0f, -1.0f);
  float yaw = -180.0f;
  float pitch = 0.0f;

  // Settings
  float movement_speed = 10.0f;
  float mouse_sensitivity = 0.05f;
  float fov = 45.0f;
  float near = 0.1f;
  float far = 100.0f;
  int &screen_width;
  int &screen_height;

  glcamera_t(int &screen_width_,
             int &screen_height_,
             const glm::vec3 position_);
};

glm::mat4 glcamera_projection(const glcamera_t &camera);
glm::mat4 glcamera_view_matrix(const glcamera_t &camera);
void glcamera_update(glcamera_t &camera);
void glcamera_keyboard_handler(glcamera_t &camera,
                               const glcamera_movement_t &direction,
                               const float dt);
void glcamera_mouse_handler(glcamera_t &camera, const float dx, const float dy);
void glcamera_scroll_handler(glcamera_t &camera, const float dy);

} // namespace proto
#endif
