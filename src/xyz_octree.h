#pragma once
#include <stdlib.h>
#include <assert.h>

/*****************************************************************************
 * OCTREE NODE
 ****************************************************************************/

typedef struct octree_node_t {
  float center[3];
  float size;
  int depth;
  int max_depth;
  int max_points;

  struct octree_node_t *children[8];
  float *points;
  size_t num_points;
  size_t capacity;
} octree_node_t;

octree_node_t *octree_node_malloc(const float center[3],
                                  const float size,
                                  const int depth,
                                  const int max_depth,
                                  const int max_points);
void octree_node_free(octree_node_t *node);

/*****************************************************************************
 * OCTREE
 ****************************************************************************/

typedef struct octree_data_t {
  float *points;
  size_t num_points;
  size_t capacity;
} octree_data_t;

typedef struct octree_t {
  float center[3];
  float size;
  octree_node_t *root;
} octree_t;

octree_t *octree_malloc(const float center[3],
                        const float voxel_size,
                        const int max_depth,
                        const int max_points,
                        const float *points,
                        const size_t num_points);
void octree_free(octree_t *octree);
void octree_add_point(octree_node_t *node, const float point[3]);
void octree_points(const octree_node_t *node, octree_data_t *data);
float *octree_downsample(const float *points,
                         const size_t n,
                         const float voxel_size,
                         const size_t voxel_limit,
                         size_t *n_out);
