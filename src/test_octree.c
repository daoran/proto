#include "munit.h"
#include "xyz.h"
#include "xyz_octree.h"

int test_octree_node(void) {
  const float center[3] = {0.0, 0.0, 0.0};
  const float size = 1.0;
  const int depth = 0;
  const int max_depth = 10;
  const int max_points = 100;

  octree_node_t *n =
      octree_node_malloc(center, size, depth, max_depth, max_points);
  octree_node_free(n);

  return 0;
}

int test_octree(void) {
  // Setup
  const float center[3] = {0.0, 0.0, 0.0};
  const float size = 0.1;
  const int max_depth = 10;
  const int max_points = 1e6;

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

  // Build octree
  octree_t *octree =
      octree_malloc(center, size, max_depth, max_points, points, N);

  // Clean up
  free(points);
  octree_free(octree);

  return 0;
}

int test_octree_points(void) {
  // Setup
  const float center[3] = {0.0, 0.0, 0.0};
  const float size = 0.1;
  const int max_depth = 10;
  const int max_points = 1e6;

  const int N = 20000;
  float *points = malloc(sizeof(float) * 3 * N);
  for (int i = 0; i < N; ++i) {
    const float x = randf(-1.0, 1.0);
    const float y = randf(-1.0, 1.0);
    const float z = randf(-1.0, 1.0);
    points[i * 3 + 0] = x;
    points[i * 3 + 1] = y;
    points[i * 3 + 2] = z;
  }

  // Build octree
  octree_t *octree =
      octree_malloc(center, size, max_depth, max_points, points, N);

  // Get points
  octree_data_t data = {0};
  data.points = malloc(sizeof(float) * 3 * 100);
  data.num_points = 0;
  data.capacity = 100;
  octree_points(octree->root, &data);

  // Assert
  MU_ASSERT(data.num_points == N);

  // Clean up
  free(points);
  octree_free(octree);
  free(data.points);

  return 0;
}

int test_octree_downsample(void) {
  // Setup
  const int n = 200;
  float *points = malloc(sizeof(float) * 3 * n);
  for (int i = 0; i < n; ++i) {
    const float x = randf(-1.0, 1.0);
    const float y = randf(-1.0, 1.0);
    const float z = randf(-1.0, 1.0);
    points[i * 3 + 0] = x;
    points[i * 3 + 1] = y;
    points[i * 3 + 2] = z;
  }

  // Downsample
  float voxel_size = 0.25;
  size_t voxel_limit = 1;
  size_t n_out = 0;
  float *points_out =
      octree_downsample(points, N, voxel_size, voxel_limit, &n_out);

  // Assert
  MU_ASSERT(n_out < N);

  // Clean up
  free(points);
  free(points_out);

  return 0;
}

void test_suite(void) {
  MU_ADD_TEST(test_octree_node);
  MU_ADD_TEST(test_octree);
  MU_ADD_TEST(test_octree_points);
  MU_ADD_TEST(test_octree_downsample);
}
MU_RUN_TESTS(test_suite)
