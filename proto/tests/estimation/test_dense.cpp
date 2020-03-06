#include "proto/munit.hpp"
#include "proto/dataset/kitti.hpp"
#include "proto/vision/vision.hpp"
#include "proto/viz/viz.hpp"

namespace proto {

#define TEST_DATA_BASEPATH "/data/kitti"

int test_raytracing() {
  gui_t gui{"Play"};
	glgrid_t grid;
	glcf_t cf;

	std::vector<glcube_t> voxels;
	for (int i = 0; i < 1000; i++) {
		glcube_t cube{0.1};

		const double x = randf(-5.0, 5.0);
		const double y = randf(-5.0, 5.0);
		const double z = randf(-5.0, 5.0);
		cube.pos(glm::vec3{x, y, z});

		voxels.push_back(cube);
	}

  while (gui.ok()) {
    gui.poll();
    gui.clear();

		for (auto &voxel : voxels) {
			voxel.draw(gui.camera);
		}
		grid.draw(gui.camera);
		cf.draw(gui.camera);

    gui.render();
  }

  return 0;
}

void test_suite() { MU_ADD_TEST(test_raytracing); }

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
