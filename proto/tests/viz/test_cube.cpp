#include "proto/munit.hpp"
#include "proto/viz/viz.hpp"

int test_cube() {
  proto::gui_t gui{"Play"};

  while (gui.ok()) {
    gui.poll();

    gui.clear();
    gui.render();
  }

  return 0;
}

void test_suite() { MU_ADD_TEST(test_cube); }

MU_RUN_TESTS(test_suite);
