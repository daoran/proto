#include "munit.h"
#include "xyz.h"
#include "xyz_kdtree.h"

static int vec3_equals(const float src[3], const float target[3]) {
  const float eps = 1e-3;
  for (int i = 0; i < 3; ++i) {
    if (fabs(src[i] - target[i]) >= eps) {
      return 0;
    }
  }
  return 1;
}

static int point_cmp(const void *a, const void *b, void *arg) {
  const float *vecA = (const float *) a;
  const float *vecB = (const float *) b;
  const int k = *(int *) arg;
  const float valA = vecA[k];
  const float valB = vecB[k];
  if (valA < valB)
    return -1;
  if (valA > valB)
    return 1;
  return 0;
}

static int vec3_cmp(const void *a, const void *b, void *arg) {
  const float *v0 = (const float *) a;
  const float *v1 = (const float *) b;
  if (v0[0] < v1[0])
    return -1;
  if (v0[0] > v1[0])
    return 1;
  if (v0[1] < v1[1])
    return -1;
  if (v0[1] > v1[1])
    return 1;
  if (v0[2] < v1[2])
    return -1;
  if (v0[2] > v1[2])
    return 1;
  return 0;
}

int test_sort(void) {
  // Setup
  const int N = 10;
  float *points = malloc(sizeof(float) * 3 * N);
  float *points_ = malloc(sizeof(float) * 3 * N);
  for (int i = 0; i < N; ++i) {
    const float x = randf(-1.0, 1.0);
    const float y = randf(-1.0, 1.0);
    const float z = randf(-1.0, 1.0);

    points[i * 3 + 0] = x;
    points[i * 3 + 1] = y;
    points[i * 3 + 2] = z;

    points_[i * 3 + 0] = points[i * 3 + 0];
    points_[i * 3 + 1] = points[i * 3 + 1];
    points_[i * 3 + 2] = points[i * 3 + 2];
  }

  printf("original:\n");
  for (int i = 0; i < N; ++i) {
    printf("(%.2f, %.2f, %.2f)\n",
           points_[i * 3 + 0],
           points_[i * 3 + 1],
           points_[i * 3 + 2]);
  }
  printf("\n");

  int start = 0;
  int end = N - 1;
  int k = 0;
  qsort_r(points + start, end - start + 1, sizeof(float) * 3, point_cmp, &k);

  printf("sorted:\n");
  for (int i = 0; i < N; ++i) {
    printf("(%.2f, %.2f, %.2f)\n",
           points[i * 3 + 0],
           points[i * 3 + 1],
           points[i * 3 + 2]);
  }
  printf("\n");

  return 0;
}

int test_kdtree_node(void) {
  const float p[3] = {0.0, 0.0, 0.0};
  const int k = 1;
  kdtree_node_t *node = kdtree_node_malloc(p, k);
  kdtree_node_free(node);

  return 0;
}

int test_kdtree(void) {
  // Generate random 3d points
  const int N = 20000;
  float *points = malloc(sizeof(float) * N * 3);
  float *points_gnd = malloc(sizeof(float) * N * 3);
  for (int i = 0; i < N; ++i) {
    const float x = randf(-10000.0, 10000.0);
    const float y = randf(-10000.0, 10000.0);
    const float z = randf(-10000.0, 10000.0);
    points[i * 3 + 0] = x;
    points[i * 3 + 1] = y;
    points[i * 3 + 2] = z;
    points_gnd[i * 3 + 0] = x;
    points_gnd[i * 3 + 1] = y;
    points_gnd[i * 3 + 2] = z;
  }

  // Build kdtree
  size_t n_ = 0;
  float *points_est = malloc(sizeof(float) * 3 * N);
  kdtree_node_t *root = kdtree_build(points, 0, N - 1, 0);
  kdtree_points(root, points_est, &n_);

  // Assert
  size_t checked = 0;
  int k = 0;
  qsort_r(points_est, n_, sizeof(float) * 3, vec3_cmp, &k);
  qsort_r(points_gnd, N, sizeof(float) * 3, vec3_cmp, &k);
  for (int i = 0; i < N; ++i) {
    if (vec3_equals(&points_gnd[i * 3], &points_est[i * 3])) {
      checked++;
      continue;
    }
  }
  MU_ASSERT(checked == N);

  // Clean up
  free(points);
  free(points_gnd);
  free(points_est);
  kdtree_node_free(root);

  return 0;
}

int test_kdtree_nn(void) {
  // Generate random 3d points
  const int N = 200000;
  float *points = malloc(sizeof(float) * N * 3);
  float *points_gnd = malloc(sizeof(float) * N * 3);
  for (int i = 0; i < N; ++i) {
    const float x = randf(-10000.0, 10000.0);
    const float y = randf(-10000.0, 10000.0);
    const float z = randf(-10000.0, 10000.0);
    points[i * 3 + 0] = x;
    points[i * 3 + 1] = y;
    points[i * 3 + 2] = z;
    points_gnd[i * 3 + 0] = x;
    points_gnd[i * 3 + 1] = y;
    points_gnd[i * 3 + 2] = z;
  }

  // Build kdtree
  kdtree_node_t *root = kdtree_build(points, 0, N - 1, 0);

  // Search closest point
  float p[3] = {5.0, 3.0, 0.0};
  float best_point[3] = {0};
  float best_dist = INFINITY;
  kdtree_nn(root, p, best_point, &best_dist);

  // Check
  // -- Brute force get closest point
  float assert_point[3] = {0};
  float assert_dist = INFINITY;
  for (int i = 0; i < N; ++i) {
    float sq_dist = 0.0f;
    sq_dist += (points_gnd[i * 3 + 0] - p[0]) * (points_gnd[i * 3 + 0] - p[0]);
    sq_dist += (points_gnd[i * 3 + 1] - p[1]) * (points_gnd[i * 3 + 1] - p[1]);
    sq_dist += (points_gnd[i * 3 + 2] - p[2]) * (points_gnd[i * 3 + 2] - p[2]);
    if (sq_dist <= assert_dist) {
      assert_point[0] = points_gnd[i * 3 + 0];
      assert_point[1] = points_gnd[i * 3 + 1];
      assert_point[2] = points_gnd[i * 3 + 2];
      assert_dist = sq_dist;
    }
  }
  // -- Assert
  MU_ASSERT(fabs(best_point[0] - assert_point[0]) < 1e-3);
  MU_ASSERT(fabs(best_point[1] - assert_point[1]) < 1e-3);
  MU_ASSERT(fabs(best_point[2] - assert_point[2]) < 1e-3);
  MU_ASSERT(fabs(best_dist - assert_dist) < 1e-3);

  // Clean up
  kdtree_node_free(root);
  free(points);
  free(points_gnd);

  return 0;
}

void test_suite(void) {
  // MU_ADD_TEST(test_sort);
  MU_ADD_TEST(test_kdtree_node);
  MU_ADD_TEST(test_kdtree);
  MU_ADD_TEST(test_kdtree_nn);
}
MU_RUN_TESTS(test_suite)
