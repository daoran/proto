#pragma once
#include <assert.h>
#include <stdlib.h>

#define KDTREE_KDIM 3

/*****************************************************************************
 * KD-TREE NODE
 ****************************************************************************/

typedef struct kdtree_node_t {
  float p[3];
  int k;
  struct kdtree_node_t *left;
  struct kdtree_node_t *right;
} kdtree_node_t;

kdtree_node_t *kdtree_node_malloc(const float p[3], const int k);
void kdtree_node_free(kdtree_node_t *node);

/*****************************************************************************
 * KD-TREE
 ****************************************************************************/

typedef struct kdtree_data_t {
  float *points;
  size_t num_points;
  size_t capacity;
} kdtree_data_t;

typedef struct kdtree_t {
  kdtree_node_t *root;
} kdtree_t;

kdtree_node_t *kdtree_insert(kdtree_node_t *node,
                             const float p[3],
                             const int depth);

kdtree_t *kdtree_malloc(float *points, size_t num_points);
void kdtree_free(kdtree_t *kdtree);
void kdtree_points(const kdtree_t *kdtree, kdtree_data_t *data);
void kdtree_nn(const kdtree_t *kdtree,
               const float target[3],
               float *best_point,
               float *best_dist);
