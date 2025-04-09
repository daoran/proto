#pragma once
#include <stdlib.h>

#define KDTREE_KDIM 3

typedef struct kdtree_node_t {
  float p[3];
  int k;
  struct kdtree_node_t *left;
  struct kdtree_node_t *right;
} kdtree_node_t;

kdtree_node_t *kdtree_node_malloc(const float p[3], const int k);
void kdtree_node_free(kdtree_node_t *node);
kdtree_node_t *kdtree_insert(kdtree_node_t *root,
                             const float p[3],
                             const int depth);
kdtree_node_t *
kdtree_build(float *points, const int start, const int end, const int depth);
void kdtree_points(const kdtree_node_t *root, float *points, size_t *n);
void kdtree_nn(const kdtree_node_t *root,
               const float target[3],
               float *best_point,
               float *best_dist);
