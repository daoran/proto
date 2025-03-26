#include <time.h>

#include "munit.h"
#include "xyz_voxel.h"

/**
 * Generate random number between a and b from a uniform distribution.
 * @returns Random number
 */
static float randf(const float a, const float b) {
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

/**
 * Tic, start timer.
 * @returns A timespec encapsulating the time instance when tic() is called
 */
struct timespec tic(void) {
  struct timespec time_start;
  clock_gettime(CLOCK_MONOTONIC, &time_start);
  return time_start;
}

/**
 * Toc, stop timer.
 * @returns Time elapsed in seconds
 */
float toc(struct timespec *tic) {
  assert(tic != NULL);
  struct timespec toc;
  float time_elasped;

  clock_gettime(CLOCK_MONOTONIC, &toc);
  time_elasped = (toc.tv_sec - tic->tv_sec);
  time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

  return time_elasped;
}

/******************************************************************************
 * VOXEL
 *****************************************************************************/

int test_voxel_setup(void) {
  voxel_t voxel;
  int32_t key[3] = {1, 2, 3};
  voxel_setup(&voxel, key);

  MU_ASSERT(voxel.key[0] == key[0]);
  MU_ASSERT(voxel.key[1] == key[1]);
  MU_ASSERT(voxel.key[2] == key[2]);
  MU_ASSERT(voxel.length == 0);

  return 0;
}

int test_voxel_reset(void) {
  voxel_t voxel;
  int32_t key[3] = {-1, -1, -1};
  voxel_reset(&voxel);

  MU_ASSERT(voxel.key[0] == key[0]);
  MU_ASSERT(voxel.key[1] == key[1]);
  MU_ASSERT(voxel.key[2] == key[2]);
  MU_ASSERT(voxel.length == 0);

  return 0;
}

int test_voxel_copy(void) {
  voxel_t src;
  int32_t key[3] = {1, 2, 3};
  voxel_setup(&src, key);

  voxel_t dst;
  voxel_copy(&src, &dst);

  MU_ASSERT(src.key[0] == dst.key[0]);
  MU_ASSERT(src.key[1] == dst.key[1]);
  MU_ASSERT(src.key[2] == dst.key[2]);
  MU_ASSERT(src.length == dst.length);

  return 0;
}

int test_voxel_add(void) {
  voxel_t voxel;
  int32_t key[3] = {1, 2, 3};
  voxel_setup(&voxel, key);

  size_t N = 100;
  for (int i = 0; i < N; ++i) {
    float p[3] = {0.1 * i, 0.1 * i, 0.1 * i};
    voxel_add(&voxel, p);
  }

  MU_ASSERT(voxel.key[0] == key[0]);
  MU_ASSERT(voxel.key[1] == key[1]);
  MU_ASSERT(voxel.key[2] == key[2]);
  MU_ASSERT(voxel.length == N);
  for (int i = 0; i < N; ++i) {
    float *p = voxel.points + (3 * i);
    MU_ASSERT(fabs(p[0] - (0.1 * i)) < 1e-3);
    MU_ASSERT(fabs(p[1] - (0.1 * i)) < 1e-3);
    MU_ASSERT(fabs(p[2] - (0.1 * i)) < 1e-3);
  }

  return 0;
}

/******************************************************************************
 * VOXEL-HASH
 *****************************************************************************/

int test_voxel_hash_malloc_and_free(void) {
  const size_t capacity = 10000;
  voxel_hash_t *voxel_hash = voxel_hash_malloc(capacity);
  voxel_hash_free(voxel_hash);
  return 0;
}

int test_voxel_hash_expand(void) {
  const size_t capacity = 100000;
  voxel_hash_t *voxel_hash = voxel_hash_malloc(capacity);

  // Create random points
  const int N = 8000;
  float *points = malloc(sizeof(float) * N * 3);
  for (int i = 0; i < N; ++i) {
    points[i * 3 + 0] = randf(-100.0, 100.0);
    points[i * 3 + 1] = randf(-100.0, 100.0);
    points[i * 3 + 2] = randf(-100.0, 100.0);
    voxel_hash_add(voxel_hash, points + i * 3);
  }
  free(points);

  voxel_hash_expand(voxel_hash);
  MU_ASSERT(voxel_hash->capacity == capacity * 2);

  voxel_hash_free(voxel_hash);
  return 0;
}

int test_voxel_hash_add(void) {
  const size_t capacity = pow(2, 17);
  voxel_hash_t *voxel_hash = voxel_hash_malloc(capacity);

  // Create random points
  const int N = 80000;
  float *points = malloc(sizeof(float) * N * 3);
  for (int i = 0; i < N; ++i) {
    points[i * 3 + 0] = randf(-100.0, 100.0);
    points[i * 3 + 1] = randf(-100.0, 100.0);
    points[i * 3 + 2] = randf(-100.0, 100.0);
  }

  // Insert random points into voxel hash
  struct timespec t = tic();
  for (int i = 0; i < N; ++i) {
    voxel_hash_add(voxel_hash, &points[i * 3]);
  }
  printf("time taken: %.4f [s]\n", toc(&t));

  // Iterate through voxel hash
  size_t num_points = 0;
  voxel_hash_iter_t it = voxel_hash_iterator(voxel_hash);
  while (voxel_hash_next(&it)) {
    voxel_t *voxel = it.value;
    num_points += voxel->length;
  }
  printf("N: %d\n", N);
  printf("num_points: %ld\n", num_points);
  MU_ASSERT(num_points == N);

  voxel_hash_free(voxel_hash);
  free(points);

  return 0;
}

void test_suite(void) {
  MU_ADD_TEST(test_voxel_setup);
  MU_ADD_TEST(test_voxel_reset);
  MU_ADD_TEST(test_voxel_copy);
  MU_ADD_TEST(test_voxel_add);

  MU_ADD_TEST(test_voxel_hash_malloc_and_free);
  MU_ADD_TEST(test_voxel_hash_expand);
  MU_ADD_TEST(test_voxel_hash_add);
}
MU_RUN_TESTS(test_suite)
