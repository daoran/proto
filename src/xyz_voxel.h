#pragma once
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <math.h>

/******************************************************************************
 * VOXEL
 *****************************************************************************/

#define VOXEL_MAX_POINTS 100

typedef struct voxel_t {
  int32_t key[3];
  float points[VOXEL_MAX_POINTS * 3];
  size_t length;
} voxel_t;

void voxel_setup(voxel_t *voxel, const int32_t key[3]);
void voxel_reset(voxel_t *voxel);
void voxel_print(voxel_t *voxel);
void voxel_copy(const voxel_t *src, voxel_t *dst);
void voxel_add(voxel_t *voxel, const float p[3]);

/******************************************************************************
 * VOXEL-HASH
 *****************************************************************************/

typedef struct voxel_hash_t {
  voxel_t *voxels;
  size_t capacity;
  size_t length;
} voxel_hash_t;

typedef struct voxel_hash_iter_t {
  int32_t *key;
  voxel_t *value;
  voxel_hash_t *_voxel_hash;
  size_t _index;
} voxel_hash_iter_t;

void point2voxel(const float p[3], const double voxel_size, int32_t voxel[3]);
voxel_hash_t *voxel_hash_malloc(const size_t capacity);
void voxel_hash_free(voxel_hash_t *voxel_hash);
int voxel_hash_expand(voxel_hash_t *voxel_hash);
int voxel_hash_add(voxel_hash_t *voxel_hash, const float p[3]);
voxel_t *voxel_hash_get(voxel_hash_t *voxel_hash, const int32_t *key);
size_t voxel_hash_length(const voxel_hash_t *voxel_hash);
voxel_hash_iter_t voxel_hash_iterator(voxel_hash_t *voxel_hash);
int voxel_hash_next(voxel_hash_iter_t *it);
