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
  const float octree_center[3] = {0.0, 0.0, 0.0};
  const float octree_size = 1.0;
  const int octree_max_depth = 10;
  const int voxel_max_points = 1e6;

  const int n = 200000;
  float *octree_points = malloc(sizeof(float) * 3 * n);
  for (int i = 0; i < n; ++i) {
    const float x = randf(-1.0, 1.0);
    const float y = randf(-1.0, 1.0);
    const float z = randf(-1.0, 1.0);
    octree_points[i * 3 + 0] = x;
    octree_points[i * 3 + 1] = y;
    octree_points[i * 3 + 2] = z;
  }

  // Build octree
  octree_t *octree = octree_malloc(octree_center,
                                   octree_size,
                                   octree_max_depth,
                                   voxel_max_points,
                                   octree_points,
                                   n);

  // Clean up
  free(octree_points);
  octree_free(octree);

  return 0;
}

int test_octree_points(void) {
  // Setup
  const float octree_center[3] = {0.0, 0.0, 0.0};
  const float octree_size = 1.0;
  const int voxel_max_points = 100;
  const int octree_max_depth = 8;

  const int n = 20000;
  float *octree_data = malloc(sizeof(float) * 3 * n);
  for (int i = 0; i < n; ++i) {
    const float x = randf(-1.0, 1.0);
    const float y = randf(-1.0, 1.0);
    const float z = randf(-1.0, 1.0);
    octree_data[i * 3 + 0] = x;
    octree_data[i * 3 + 1] = y;
    octree_data[i * 3 + 2] = z;
  }

  // Build octree
  octree_t *octree = octree_malloc(octree_center,
                                   octree_size,
                                   octree_max_depth,
                                   voxel_max_points,
                                   octree_data,
                                   n);

  // Get points
  octree_data_t data = {0};
  data.points = malloc(sizeof(float) * 3 * n);
  data.num_points = 0;
  data.capacity = n;
  octree_points(octree->root, &data);

  // Assert
  MU_ASSERT(data.num_points == n);

  // Clean up
  free(octree_data);
  octree_free(octree);
  free(data.points);

  return 0;
}

int test_octree_downsample(void) {
  // Setup
  const int n = 20000;
  float offset[3] = {10.0f, 20.0f, 30.0f};
  float *points = malloc(sizeof(float) * 3 * n);
  for (int i = 0; i < n; ++i) {
    const float x = randf(-1.0 + offset[0], 1.0 + offset[0]);
    const float y = randf(-1.0 + offset[1], 1.0 + offset[1]);
    const float z = randf(-1.0 + offset[2], 1.0 + offset[2]);
    points[i * 3 + 0] = x;
    points[i * 3 + 1] = y;
    points[i * 3 + 2] = z;
  }

  // Downsample
  float voxel_size = 0.5;
  size_t voxel_limit = 100;
  size_t n_out = 0;
  float *points_out =
      octree_downsample(points, n, voxel_size, voxel_limit, &n_out);

  // Assert
  MU_ASSERT(n_out < n);

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
