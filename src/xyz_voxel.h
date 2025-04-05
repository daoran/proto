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
  float *points;
  size_t length;
} voxel_t;

void voxel_setup(voxel_t *voxel, const int32_t key[3]);
voxel_t *voxel_malloc(const int32_t key[3]);
void voxel_free(voxel_t *voxel);
void voxel_reset(voxel_t *voxel);
void voxel_print(voxel_t *voxel);
void voxel_copy(const voxel_t *src, voxel_t *dst);
void voxel_add(voxel_t *voxel, const float p[3]);

/******************************************************************************
 * HASH-TABLE
 *****************************************************************************/

typedef struct ht_entry_t {
  int32_t *key;
  voxel_t *value;
} ht_entry_t;

typedef struct ht_t {
  ht_entry_t *entries;
  size_t capacity;
  size_t length;
} ht_t;

typedef struct ht_iter_t {
  int32_t *key;
  voxel_t *value;
  ht_t *_ht;
  size_t _index;
} ht_iter_t;

ht_t *ht_malloc(const size_t capacity);
void ht_free(ht_t *ht);
voxel_t *ht_get(const ht_t *ht, const int32_t key[3]);
int ht_expand(ht_t *ht);
int ht_set(ht_t *ht, int32_t *key, voxel_t *value);
ht_iter_t ht_iterator(ht_t *ht);
int ht_next(ht_iter_t *it);

/******************************************************************************
 * VOXEL-HASH
 *****************************************************************************/

#define POINT2VOXEL(POINT, VOXEL_SIZE, VOXEL)                                  \
  int32_t VOXEL[3] = {0};                                                      \
  VOXEL[0] = floor(POINT[0] / VOXEL_SIZE);                                     \
  VOXEL[1] = floor(POINT[1] / VOXEL_SIZE);                                     \
  VOXEL[2] = floor(POINT[2] / VOXEL_SIZE);

void point2voxel(const float p[3], const float voxel_size, int32_t voxel[3]);

// typedef struct voxel_hash_t {
//   voxel_t *voxels;
//   size_t capacity;
//   size_t length;
// } voxel_hash_t;
//
// typedef struct voxel_hash_iter_t {
//   int32_t *key;
//   voxel_t *value;
//   voxel_hash_t *_voxel_hash;
//   size_t _index;
// } voxel_hash_iter_t;

// void point2voxel(const float p[3], const double voxel_size, int32_t voxel[3]);
// voxel_hash_t *voxel_hash_malloc(const size_t capacity);
// void voxel_hash_free(voxel_hash_t *voxel_hash);
// int voxel_hash_expand(voxel_hash_t *voxel_hash);
// int voxel_hash_add(voxel_hash_t *voxel_hash, const float p[3]);
// voxel_t *voxel_hash_get(voxel_hash_t *voxel_hash, const int32_t *key);
// size_t voxel_hash_length(const voxel_hash_t *voxel_hash);
// voxel_hash_iter_t voxel_hash_iterator(voxel_hash_t *voxel_hash);
// int voxel_hash_next(voxel_hash_iter_t *it);
