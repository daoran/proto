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
 * VOXEL-HASH
 *****************************************************************************/

#define POINT2VOXEL(POINT, VOXEL_SIZE, VOXEL)                                  \
  int32_t VOXEL[3] = {0};                                                      \
  VOXEL[0] = floor(POINT[0] / VOXEL_SIZE);                                     \
  VOXEL[1] = floor(POINT[1] / VOXEL_SIZE);                                     \
  VOXEL[2] = floor(POINT[2] / VOXEL_SIZE);

void point2voxel(const float p[3], const float voxel_size, int32_t voxel[3]);
