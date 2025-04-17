#include "xyz_voxel.h"

/******************************************************************************
 * VOXEL
 *****************************************************************************/

void voxel_setup(voxel_t *voxel, const int32_t key[3]) {
  assert(voxel);
  assert(key);

  voxel->key[0] = key[0];
  voxel->key[1] = key[1];
  voxel->key[2] = key[2];

  for (int i = 0; i < VOXEL_MAX_POINTS; ++i) {
    voxel->points[i * 3 + 0] = 0.0f;
    voxel->points[i * 3 + 1] = 0.0f;
    voxel->points[i * 3 + 2] = 0.0f;
  }

  voxel->length = 0;
}

voxel_t *voxel_malloc(const int32_t key[3]) {
  voxel_t *voxel = malloc(sizeof(voxel_t));
  voxel->points = calloc(VOXEL_MAX_POINTS * 3, sizeof(float));
  voxel_setup(voxel, key);
  return voxel;
}

void voxel_free(voxel_t *voxel) {
  assert(voxel);
  free(voxel->points);
  free(voxel);
}

void voxel_print(voxel_t *voxel) {
  assert(voxel);

  printf("key: [%d, %d, %d]\n", voxel->key[0], voxel->key[1], voxel->key[2]);
  printf("length: %ld\n", voxel->length);
  for (int i = 0; i < voxel->length; ++i) {
    const float x = voxel->points[i * 3 + 0];
    const float y = voxel->points[i * 3 + 1];
    const float z = voxel->points[i * 3 + 2];
    printf("%d: [%.2f, %.2f, %.2f]\n", i, x, y, z);
  }
}

void voxel_reset(voxel_t *voxel) {
  assert(voxel);
  assert(voxel->points != NULL);

  voxel->key[0] = -1;
  voxel->key[1] = -1;
  voxel->key[2] = -1;

  for (int i = 0; i < VOXEL_MAX_POINTS; ++i) {
    voxel->points[i * 3 + 0] = 0.0f;
    voxel->points[i * 3 + 1] = 0.0f;
    voxel->points[i * 3 + 2] = 0.0f;
  }

  voxel->length = 0;
}

void voxel_copy(const voxel_t *src, voxel_t *dst) {
  assert(src && dst);
  assert(src->points != NULL);
  assert(dst->points != NULL);

  dst->key[0] = src->key[0];
  dst->key[1] = src->key[1];
  dst->key[2] = src->key[2];
  for (int i = 0; i < src->length; ++i) {
    dst->points[i] = src->points[i];
  }
  dst->length = src->length;
}

void voxel_add(voxel_t *voxel, const float p[3]) {
  if (voxel->length >= VOXEL_MAX_POINTS) {
    return;
  }
  voxel->points[voxel->length * 3 + 0] = p[0];
  voxel->points[voxel->length * 3 + 1] = p[1];
  voxel->points[voxel->length * 3 + 2] = p[2];
  voxel->length++;
}

/******************************************************************************
 * VOXEL-HASH
 *****************************************************************************/

void point2voxel(const float p[3], const float voxel_size, int32_t voxel[3]) {
  voxel[0] = floor(p[0] / voxel_size);
  voxel[1] = floor(p[1] / voxel_size);
  voxel[2] = floor(p[2] / voxel_size);
}
