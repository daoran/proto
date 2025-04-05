#include "munit.h"
#include "xyz.h"
#include "xyz_kdtree.h"

int test_kdtree_node(void) {
  const float p[3] = {0.0, 0.0, 0.0};
  const int k = 1;
  kdtree_node_t *node = kdtree_node_malloc(p, k);
  kdtree_node_free(node);

  return 0;
}

int test_kdtree(void) {
  // Setup
  const int N = 200000;
  float *points = malloc(sizeof(float) * 3 * N);
  for (int i = 0; i < N; ++i) {
    const float x = randf(-1.0, 1.0);
    const float y = randf(-1.0, 1.0);
    const float z = randf(-1.0, 1.0);
    points[i * 3 + 0] = x;
    points[i * 3 + 1] = y;
    points[i * 3 + 2] = z;
  }

  // Build kdtree
  struct timespec t = tic();
  int kdim = 3;
  kdtree_node_t *root = kdtree_build(points, 0, N - 1, 0);
  printf("[kdtree malloc] time taken: %.4f [s]\n", toc(&t));

  // Clean up
  // free(points);
  // kdtree_node_free(root);

  return 0;
}

void test_suite(void) {
  MU_ADD_TEST(test_kdtree_node);
  MU_ADD_TEST(test_kdtree);
}
MU_RUN_TESTS(test_suite)
