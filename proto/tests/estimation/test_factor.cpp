#include "proto/munit.hpp"
#include "proto/estimation/factor.hpp"

namespace proto {

int test_variable() {
  landmark_t landmark{0, 0, vec3_t{1.0, 2.0, 3.0}};
  MU_CHECK(landmark.id == 0);
  return 0;
}

int test_graph() {
  graph_t graph;

  return 0;
}

int test_graph_set_sensor_camera_extrinsics() {
  graph_t graph;

  int cam_idx = 0;
  mat4_t T_SC = I(4);
  graph_set_sensor_camera_extrinsic(graph, cam_idx, T_SC);

  return 0;
}

int test_graph_add_camera_factor() {
  graph_t graph;

  timestamp_t ts = 0;
  int cam_idx = 0;
  vec2_t z;
  vec3_t p_W;
  mat4_t T_WS = I(4);
  graph_add_camera_factor(graph, ts, cam_idx, z, p_W, T_WS);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_variable);
  MU_ADD_TEST(test_graph);
  MU_ADD_TEST(test_graph_set_sensor_camera_extrinsics);
  MU_ADD_TEST(test_graph_add_camera_factor);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
