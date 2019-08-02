#include "proto/viz/gui.hpp"
#include "proto/viz/draw.hpp"

int main(void) {
  // Setup
  proto::gui_t gui{"Play"};

  // glimg_t img{"../assets/textures/awesomeface.png"};
  proto::glplane_t plane{"../assets/textures/container.jpg"};

  while (gui.ok()) {
    gui.poll();

    gui.clear();
		gui.render();
  }

  gui.close();
  return 0;
}
