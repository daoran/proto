#pragma once
#include <stdlib.h>

#ifndef OCTREE_MAX_POINTS
#define OCTREE_MAX_POINTS 100
#endif

/* Octree Node */
typedef struct octree_node_t {
  double center[3];
  double size;
  int depth;
  int max_depth;

  struct octree_node_t *children[8];
  double points[OCTREE_MAX_POINTS * 3];
  size_t num_points;
} octree_node_t;

/* Octree */
typedef struct octree_t {
  double center[3];
  double size;
  octree_node_t *root;
} octree_t;

octree_node_t *octree_node_malloc(const double center[3],
                                  const double size,
                                  const int depth,
                                  const int max_depth);
void octree_node_free(octree_node_t *node);

octree_t *octree_malloc(const double center[3],
                        const double size,
                        const int depth,
                        const int max_depth,
                        const double *points,
                        const size_t num_points);
void octree_free(octree_t *octree);
void octree_add_point(octree_node_t *node, const double point[3]);
