#include "proto/viz/viz.hpp"

int main(void) {
  // Setup
  proto::gui_t gui{"Play"};
  proto::glcamera_t camera{gui.width_,
                           gui.height_,
                           glm::vec3(0.0f, 5.0f, 30.0f)};

  // proto::glplane_t plane{"test_data/viz/container.jpg"};
  proto::glplane_t plane{"test_data/viz/awesomeface.png"};
  proto::glgrid_t grid;

  while (gui.ok()) {
    gui.poll();

    gui.clear();
    plane.draw(camera);
    grid.draw(camera);
    gui.render();
  }

  gui.close();
  return 0;
}
