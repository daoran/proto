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

  int cam_idx = 0;
  mat4_t T_SC = I(4);
  graph_set_sensor_camera_extrinsic(graph, cam_idx, T_SC);

  timestamp_t ts = 0;
  vec2s_t z;
  vec3s_t p_W;
  mat4_t T_WS = I(4);
  graph_add_cam_error(graph, ts, cam_idx, z, p_W, T_WS);

  return 0;
}

int test_graph_delete() {
  graph_t graph;

  int cam_idx = 0;
  mat4_t T_SC = I(4);
  graph_set_sensor_camera_extrinsic(graph, cam_idx, T_SC);

  timestamp_t ts = 0;
  vec2s_t z;
  vec3s_t p_W;
  mat4_t T_WS = I(4);
  graph_add_cam_error(graph, ts, cam_idx, z, p_W, T_WS);
  graph_add_cam_error(graph, ts, cam_idx, z, p_W, T_WS);
  graph_add_cam_error(graph, ts, cam_idx, z, p_W, T_WS);
  graph_add_cam_error(graph, ts, cam_idx, z, p_W, T_WS);

  printf("landmarks size: %zu\n", graph.landmarks.size());
  printf("T_WS size: %zu\n", graph.T_WS.size());
  printf("T_SC size: %zu\n", graph.T_SC.size());
  printf("cam_errors size: %zu\n", graph.cam_errors.size());
  printf("cam_errors[0] size: %zu\n", graph.cam_errors[0].size());

  graph_delete(graph);
  MU_CHECK(graph.landmarks.size() == 0);
  MU_CHECK(graph.T_WS.size() == 0);
  MU_CHECK(graph.T_SC.size() == 0);
  MU_CHECK(graph.cam_errors.size() == 0);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_variable);
  MU_ADD_TEST(test_graph);
  MU_ADD_TEST(test_graph_delete);
}

} // namespace proto

MU_RUN_TESTS(proto::test_suite);
