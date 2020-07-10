#include "proto/munit.hpp"
#include "proto/dataset/kitti.hpp"
#include "proto/viz/viz.hpp"

namespace proto {

#define TEST_DATA_BASEPATH "/data/kitti"

int test_raytracing() {
  gui_t gui{"Play"};
  glmodel_t model("test_data/viz/nanosuit/nanosuit.obj");
  glgrid_t grid;
  glcf_t cf;

  glvoxels_t voxels{0.05, 10000};
  // for (int i = 0; i < 1000; i++) {
  //   voxels.add({randf(-5.0, 5.0), randf(-5.0, 5.0), randf(-5.0, 5.0)});
  // }

  // std::vector<glcube_t *> voxels;
  // for (int i = 0; i < 1000; i++) {
  //   glcube_t *cube = new glcube_t(0.01);
  //   cube->pos({randf(-5.0, 5.0), randf(-5.0, 5.0), randf(-5.0, 5.0)});
  //   voxels.push_back(cube);
  // }

  while (gui.ok()) {
    gui.poll();
    gui.clear();

    grid.draw(gui.camera);
    glmodel_draw(model, gui.camera);
    // cf.draw(gui.camera);

    // for (auto &voxel : voxels) {
    //   voxel->draw(gui.camera);
    // }

    // Keyboard handler
    if (glfwGetKey(gui.gui_, GLFW_KEY_W) == GLFW_PRESS) {
      glcamera_keyboard_handler(gui.camera, FORWARD, gui.dt_);
    } else if (glfwGetKey(gui.gui_, GLFW_KEY_A) == GLFW_PRESS) {
      glcamera_keyboard_handler(gui.camera, LEFT, gui.dt_);
    } else if (glfwGetKey(gui.gui_, GLFW_KEY_S) == GLFW_PRESS) {
      glcamera_keyboard_handler(gui.camera, BACKWARD, gui.dt_);
    } else if (glfwGetKey(gui.gui_, GLFW_KEY_D) == GLFW_PRESS) {
      glcamera_keyboard_handler(gui.camera, RIGHT, gui.dt_);
    } else if (glfwGetKey(gui.gui_, GLFW_KEY_Q) == GLFW_PRESS) {
      break;
    }

    gui.render();
  }

  return 0;
}

void test_suite() { MU_ADD_TEST(test_raytracing); }

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
