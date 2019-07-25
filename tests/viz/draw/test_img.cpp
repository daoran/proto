#include "prototype/viz/play.hpp"
#include "prototype/viz/draw/draw.hpp"

int main(void) {
  // Setup
  GLFWwindow *window = play_init();
  if (window == nullptr) {
    printf("Failed to initialize play!");
    return -1;
  }
  // glimg_t img{"../assets/textures/awesomeface.png"};
  glimg_t img{"../assets/textures/container.jpg"};

  // Loop until the user closes the window
  while (play_loop(window)) {
    play_event_handler(window, camera, play_get_dt());
    play_clear_screen();
		glimg_draw(img, camera);
    play_poll(window);
  }

  play_terminate();
  return 0;
}
