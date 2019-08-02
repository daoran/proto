#include "proto/viz/camera.hpp"

namespace proto {

glcamera_t::glcamera_t(int &screen_width,
                       int &screen_height,
                       const glm::vec3 position)
  : position{position},
    screen_width{screen_width},
    screen_height{screen_height} {
  glcamera_update(*this);
}

glm::mat4 glcamera_projection(const glcamera_t &camera) {
  const float fov = glm::radians(camera.fov);
  const float ratio = (float) camera.screen_width / (float) camera.screen_height;
  const float near = camera.near;
  const float far = camera.far;
  glm::mat4 projection = glm::perspective(fov, ratio, near, far);
  return projection;
}

glm::mat4 glcamera_view_matrix(const glcamera_t &camera) {
  return glm::lookAt(camera.position,
                     camera.position + camera.front,
                     camera.world_up);
}

void glcamera_update(glcamera_t &camera) {
  // Calculate the new front vector
  glm::vec3 front;
  front.x = sin(glm::radians(camera.yaw)) * cos(glm::radians(camera.pitch));
  front.y = sin(glm::radians(camera.pitch));
  front.z = cos(glm::radians(camera.yaw)) * cos(glm::radians(camera.pitch));
  camera.front = glm::normalize(front);

  // Also re-calculate the right and Up vector
  camera.right = glm::normalize(glm::cross(camera.front, camera.world_up));

  // Normalize the vectors, because their length gets closer to 0 the more
  // you look up or down which results in slower movement.
  camera.up = glm::normalize(glm::cross(camera.right, camera.front));
}

void glcamera_keyboard_handler(glcamera_t &camera,
                               const glcamera_movement_t &direction,
                               const float dt) {
  float velocity = camera.movement_speed * dt;
  if (direction == FORWARD) {
    camera.position += camera.front * velocity;
  }
  if (direction == BACKWARD) {
    camera.position -= camera.front * velocity;
  }
  if (direction == LEFT) {
    camera.position -= camera.right * velocity;
  }
  if (direction == RIGHT) {
    camera.position += camera.right * velocity;
  }
  if (direction == UP) {
    camera.position += camera.up * velocity;
  }
  if (direction == DOWN) {
    camera.position -= camera.up * velocity;
  }
}

void glcamera_mouse_handler(glcamera_t &camera,
                            const float dx,
                            const float dy) {
  camera.yaw += dx * camera.mouse_sensitivity;
  camera.pitch += dy * camera.mouse_sensitivity;

  // Constrain pitch
  if (camera.pitch > 89.0f) {
    camera.pitch = 89.0f;
  }
  if (camera.pitch < -89.0f) {
    camera.pitch = -89.0f;
  }

  // Update camera attitude
  glcamera_update(camera);
}

void glcamera_scroll_handler(glcamera_t &camera, const float dy) {
  if (camera.fov >= 1.0f && camera.fov <= 45.0f) {
    camera.fov -= dy;
  }
  if (camera.fov <= 1.0f) {
    camera.fov = 1.0f;
  }
  if (camera.fov >= 45.0f) {
    camera.fov = 45.0f;
  }
}

} // namespace proto
