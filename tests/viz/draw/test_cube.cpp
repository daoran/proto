#include "prototype/munit.hpp"
#include "prototype/viz/gui.hpp"
#include "prototype/viz/draw.hpp"

int test_cube() {
  gui_t gui{"Play"};

  while (gui.ok()) {
    gui.poll();

    gui.clear();
		gui.render();
  }

  gui.close();
  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_cube);
}

MU_RUN_TESTS(test_suite);
