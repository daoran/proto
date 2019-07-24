#include "play/play.hpp"
#include "play/draw/draw.hpp"

int main(void) {
  // Setup
  GLFWwindow *window = play_init();
  if (window == nullptr) {
    printf("Failed to initialize play!");
    return -1;
  }

  // Cube
  glgrid_t grid;
  glcube_t cube;
  glframe_t frame;
  glcf_t cf;
  glmodel_t model("../assets/nanosuit/nanosuit.obj",
                  "../assets/shaders/model.vs",
                  "../assets/shaders/model.fs");

  // Loop until the user closes the window
  while (play_loop(window)) {
    play_event_handler(window, camera, play_get_dt());
    play_clear_screen();

    // glframe_draw(frame, camera);
    // glcube_draw(cube, camera);
    glgrid_draw(grid, camera);
    glmodel_draw(model, camera);
    // glcf_draw(cf, camera);

    play_poll(window);
  }

  play_terminate();
  return 0;
}
