#include "munit.h"
#include "xyz.h"
#include "xyz_octree.h"

int test_octree_node(void) {
  const double center[3] = {0.0, 0.0, 0.0};
  const double size = 1.0;
  const int depth = 0;
  const int max_depth = 10;

  octree_node_t *node = octree_node_malloc(center, size, depth, max_depth);
  octree_node_free(node);

  return 0;
}

int test_octree(void) {
  // Setup
  const double center[3] = {0.0, 0.0, 0.0};
  const double size = 0.1;
  const int depth = 0;
  const int max_depth = 10;

  const int N = 200000;
  double *points = malloc(sizeof(double) * 3 * N);
  for (int i = 0; i < N; ++i) {
    const float x = randf(-1.0, 1.0);
    const float y = randf(-1.0, 1.0);
    const float z = randf(-1.0, 1.0);
    points[i * 3 + 0] = x;
    points[i * 3 + 1] = y;
    points[i * 3 + 2] = z;
  }

  // Build octree
  struct timespec t = tic();
  octree_t *octree = octree_malloc(center, size, depth, max_depth, points, N);
  printf("[octree malloc] time taken: %.4f [s]\n", toc(&t));

  // Clean up
  free(points);
  octree_free(octree);

  return 0;
}

void test_suite(void) {
  MU_ADD_TEST(test_octree_node);
  MU_ADD_TEST(test_octree);
}
MU_RUN_TESTS(test_suite)
