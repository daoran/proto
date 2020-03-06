#include "proto/munit.hpp"
#include "proto/dataset/kitti.hpp"
#include "proto/vision/vision.hpp"
#include "proto/viz/viz.hpp"

namespace proto {

#define TEST_DATA_BASEPATH "/data/kitti"

int test_raytracing() {
  gui_t gui{"Play"};
  glmodel_t model("test_data/viz/nanosuit/nanosuit.obj");
	glgrid_t grid;
	glcube_t cube;
	glcf_t cf;

  while (gui.ok()) {
    gui.poll();
    gui.clear();

		grid.draw(gui.camera);
		glmodel_draw(model, gui.camera);
		cube.draw(gui.camera);
		// cf.draw(gui.camera);

    gui.render();
  }

  return 0;
}

void test_suite() { MU_ADD_TEST(test_raytracing); }

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
