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

int test_voxel_malloc_and_free(void) {
  int32_t key[3] = {1, 2, 3};
  voxel_t *voxel = voxel_malloc(key);

  MU_ASSERT(voxel->key[0] == key[0]);
  MU_ASSERT(voxel->key[1] == key[1]);
  MU_ASSERT(voxel->key[2] == key[2]);
  MU_ASSERT(voxel->length == 0);

  voxel_free(voxel);

  return 0;
}

int test_voxel_reset(void) {
  int32_t key[3] = {-1, -1, -1};
  voxel_t *voxel = voxel_malloc(key);
  voxel_reset(voxel);

  MU_ASSERT(voxel->key[0] == key[0]);
  MU_ASSERT(voxel->key[1] == key[1]);
  MU_ASSERT(voxel->key[2] == key[2]);
  MU_ASSERT(voxel->length == 0);

  voxel_free(voxel);

  return 0;
}

int test_voxel_copy(void) {
  int32_t key_src[3] = {1, 2, 3};
  voxel_t *src = voxel_malloc(key_src);

  int32_t key_dst[3] = {1, 2, 3};
  voxel_t *dst = voxel_malloc(key_dst);
  voxel_copy(src, dst);

  MU_ASSERT(src->key[0] == dst->key[0]);
  MU_ASSERT(src->key[1] == dst->key[1]);
  MU_ASSERT(src->key[2] == dst->key[2]);
  MU_ASSERT(src->length == dst->length);

  voxel_free(src);
  voxel_free(dst);

  return 0;
}

int test_voxel_add(void) {
  int32_t key[3] = {1, 2, 3};
  voxel_t *voxel = voxel_malloc(key);

  size_t N = 100;
  for (int i = 0; i < N; ++i) {
    float p[3] = {0.1 * i, 0.1 * i, 0.1 * i};
    voxel_add(voxel, p);
  }

  MU_ASSERT(voxel->key[0] == key[0]);
  MU_ASSERT(voxel->key[1] == key[1]);
  MU_ASSERT(voxel->key[2] == key[2]);
  MU_ASSERT(voxel->length == N);
  for (int i = 0; i < N; ++i) {
    float *p = voxel->points + (3 * i);
    MU_ASSERT(fabs(p[0] - (0.1 * i)) < 1e-3);
    MU_ASSERT(fabs(p[1] - (0.1 * i)) < 1e-3);
    MU_ASSERT(fabs(p[2] - (0.1 * i)) < 1e-3);
  }

  voxel_free(voxel);

  return 0;
}

/******************************************************************************
 * HASH-TABLE
 *****************************************************************************/

int test_ht_malloc_and_free(void) {
  const size_t capacity = 100;
  ht_t *ht = ht_malloc(capacity);

  for (int i = 0; i < capacity; ++i) {
    MU_ASSERT(ht->entries[i].key == 0);
    MU_ASSERT(ht->entries[i].value == 0);
  }
  MU_ASSERT(ht->capacity == capacity);
  MU_ASSERT(ht->length == 0);

  ht_free(ht);
  return 0;
}

int test_ht_set_and_get(void) {
  // Setup
  const size_t capacity = 100;
  ht_t *ht = ht_malloc(capacity);

  // Create voxel keys
  const float voxel_size = 0.1;
  const float p0[3] = {randf(-1.0, 1.0), randf(-1.0, 1.0), randf(-1.0, 1.0)};
  const float p1[3] = {randf(-1.0, 1.0), randf(-1.0, 1.0), randf(-1.0, 1.0)};
  POINT2VOXEL(p0, voxel_size, voxel0_key);
  POINT2VOXEL(p1, voxel_size, voxel1_key);

  // Set voxels
  voxel_t voxel0;
  voxel_t voxel1;
  MU_ASSERT(ht_set(ht, voxel0_key, &voxel0) == 0);
  MU_ASSERT(ht_set(ht, voxel1_key, &voxel1) == 0);

  // Get voxels
  MU_ASSERT(ht->length == 2);
  MU_ASSERT(ht_get(ht, voxel0_key) == &voxel0);
  MU_ASSERT(ht_get(ht, voxel1_key) == &voxel1);

  // Clean up
  ht_free(ht);

  return 0;
}

int test_ht_expand(void) {
  // Setup
  const size_t capacity = 100000;
  ht_t *ht = ht_malloc(capacity);

  // Create random voxels
  const float voxel_size = 0.1;
  const int N = 8000;
  voxel_t *voxels = calloc(N, sizeof(voxel_t));
  int32_t *voxel_keys = calloc(N * 3, sizeof(voxel_t));
  for (int i = 0; i < N; ++i) {
    const float p[3] = {i, i, i};
    point2voxel(p, voxel_size, &voxel_keys[i * 3]);
    ht_set(ht, &voxel_keys[i * 3], &voxels[i]);
  }

  // Expand
  ht_expand(ht);
  MU_ASSERT(ht->capacity == capacity * 2);

  // Clean up
  ht_free(ht);
  free(voxels);
  free(voxel_keys);

  return 0;
}

int test_ht_iterate(void) {
  // Setup
  const size_t capacity = 100000;
  ht_t *ht = ht_malloc(capacity);

  // Create 1 random point per voxel
  const float voxel_size = 1;
  const int N = 200000;
  int32_t *voxel_keys = calloc(N * 3, sizeof(int32_t));
  voxel_t *voxels = calloc(N, sizeof(int32_t));
  float *points = malloc(sizeof(float) * N * 3);
  for (int i = 0; i < N; ++i) {
    // const float p[3] = {randf(-1000.0, 1000.0), randf(-1000.0, 1000.0), randf(-1000.0, 1000.0)};
    // point2voxel(p, voxel_size, &voxel_keys[i * 3]);
    points[i * 3 + 0] = randf(-1000.0, 1000.0);
    points[i * 3 + 1] = randf(-1000.0, 1000.0);
    points[i * 3 + 2] = randf(-1000.0, 1000.0);
  }
  free(points);

  // Insert random points into voxel hash
  // {
  //   struct timespec t = tic();
  //   for (int i = 0; i < N; ++i) {
  //     ht_set(ht, &voxel_keys[i * 3], &voxels[i]);
  //   }
  //   printf("[voxel hash add points] time taken: %.4f [s]\n", toc(&t));
  // }

  // Iterate through voxel hash
  // {
  //   struct timespec t = tic();
  //   float *points_ = malloc(sizeof(float) * N * 3);
  //   size_t num_points = 0;
  //   ht_iter_t it = ht_iterator(ht);
  //   while (ht_next(&it)) {
  //     voxel_t *voxel = it.value;
  //
  //     for (size_t i = 0; i < voxel->length; ++i) {
  //       points_[num_points * 3 + 0] = voxel->points[i * 3 + 0];
  //       points_[num_points * 3 + 1] = voxel->points[i * 3 + 1];
  //       points_[num_points * 3 + 2] = voxel->points[i * 3 + 2];
  //       num_points++;
  //     }
  //   }
  //   free(points_);
  //
  //   printf("[voxel hash get points] time taken: %.4f [s]\n", toc(&t));
  //   printf("num_points inserted:  %d\n", N);
  //   printf("num_points retrieved: %ld\n", num_points);
  //   MU_ASSERT(num_points == N);
  // }

  // Clean up
  ht_free(ht);
  free(voxel_keys);
  free(voxels);

  return 0;
}

/******************************************************************************
 * VOXEL-HASH
 *****************************************************************************/

// int test_voxel_hash_malloc_and_free(void) {
//   const size_t capacity = 10000;
//   voxel_hash_t *voxel_hash = voxel_hash_malloc(capacity);
//   voxel_hash_free(voxel_hash);
//   return 0;
// }


void test_suite(void) {
  // MU_ADD_TEST(test_voxel_malloc_and_free);
  // MU_ADD_TEST(test_voxel_reset);
  // MU_ADD_TEST(test_voxel_copy);
  // MU_ADD_TEST(test_voxel_add);

  // MU_ADD_TEST(test_ht_malloc_and_free);
  // MU_ADD_TEST(test_ht_set_and_get);
  // MU_ADD_TEST(test_ht_expand);
  MU_ADD_TEST(test_ht_iterate);

  // MU_ADD_TEST(test_voxel_hash_malloc_and_free);
  // MU_ADD_TEST(test_voxel_hash_expand);
  // MU_ADD_TEST(test_voxel_hash_add);
}
MU_RUN_TESTS(test_suite)
