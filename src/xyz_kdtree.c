#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "xyz_kdtree.h"

/*****************************************************************************
 * KD-TREE NODE
 ****************************************************************************/

kdtree_node_t *kdtree_node_malloc(const float p[3], const int k) {
  assert(p);
  assert(k >= 0 && k <= 2);
  kdtree_node_t *node = malloc(sizeof(kdtree_node_t));

  node->p[0] = p[0];
  node->p[1] = p[1];
  node->p[2] = p[2];
  node->k = k;
  node->left = NULL;
  node->right = NULL;

  return node;
}

void kdtree_node_free(kdtree_node_t *node) {
  if (node == NULL) {
    return;
  }
  kdtree_node_free(node->left);
  kdtree_node_free(node->right);
  free(node);
}

/*****************************************************************************
 * KD-TREE
 ****************************************************************************/

int point_cmp(const void *a, const void *b, void *k) {
  return (((float *) a)[*(int *) k] < ((float *) b)[*(int *) k]) ? -1 : 1;
}

kdtree_node_t *kdtree_insert(kdtree_node_t *node,
                             const float p[3],
                             const int depth) {
  const int k = depth % KDTREE_KDIM;
  if (node == NULL) {
    return kdtree_node_malloc(p, k);
  }

  if (p[k] < node->p[k]) {
    node->left = kdtree_insert(node->left, p, depth + 1);
  } else {
    node->right = kdtree_insert(node->right, p, depth + 1);
  }

  return node;
}

static kdtree_node_t *
_kdtree_build(float *points, const int start, const int end, const int depth) {
  if (start > end) {
    return NULL;
  }

  int k = depth % KDTREE_KDIM;
  const int mid = (start + end + 1) / 2;
  qsort_r(points + start * 3,
          end - start + 1,
          sizeof(float) * 3,
          point_cmp,
          &k);

  kdtree_node_t *root = kdtree_node_malloc(points + mid * 3, k);
  root->left = _kdtree_build(points, start, mid - 1, depth + 1);
  root->right = _kdtree_build(points, mid + 1, end, depth + 1);

  return root;
}

kdtree_t *kdtree_malloc(float *points, size_t num_points) {
  kdtree_t *kdtree = malloc(sizeof(kdtree_t));
  kdtree->root = _kdtree_build(points, 0, num_points - 1, 0);
  return kdtree;
}

void kdtree_free(kdtree_t *kdtree) {
  kdtree_node_free(kdtree->root);
  free(kdtree);
}

static void _kdtree_points(const kdtree_node_t *node, kdtree_data_t *data) {
  assert(data);
  if (node == NULL) {
    return;
  }

  data->points[data->num_points * 3 + 0] = node->p[0];
  data->points[data->num_points * 3 + 1] = node->p[1];
  data->points[data->num_points * 3 + 2] = node->p[2];
  data->num_points++;
  if (data->num_points >= data->capacity) {
    data->capacity = data->capacity * 2;
    data->points = realloc(data->points, sizeof(float) * 3 * data->capacity);
  }

  _kdtree_points(node->left, data);
  _kdtree_points(node->right, data);
}

void kdtree_points(const kdtree_t *kdtree, kdtree_data_t *data) {
  assert(kdtree && kdtree->root);
  assert(data && data->points && data->capacity > 0);
  _kdtree_points(kdtree->root, data);
}

void _kdtree_nn(const kdtree_node_t *node,
                const float target[3],
                float *best_dist,
                float *best_point,
                int depth) {
  // Pre-check
  if (node == NULL) {
    return;
  }

  // Calculate distance and keep track of best
  float sq_dist = 0.0f;
  sq_dist += (node->p[0] - target[0]) * (node->p[0] - target[0]);
  sq_dist += (node->p[1] - target[1]) * (node->p[1] - target[1]);
  sq_dist += (node->p[2] - target[2]) * (node->p[2] - target[2]);
  if (sq_dist <= *best_dist) {
    best_point[0] = node->p[0];
    best_point[1] = node->p[1];
    best_point[2] = node->p[2];
    *best_dist = sq_dist;
  }

  // Determine which side to search first
  const int axis = node->k;
  const float diff = target[axis] - node->p[axis];

  // Search the closer subtree first
  const kdtree_node_t *closer = (diff <= 0) ? node->left : node->right;
  const kdtree_node_t *farther = (diff <= 0) ? node->right : node->left;
  _kdtree_nn(closer, target, best_dist, best_point, depth + 1);

  // Search the farther subtree
  if (fabs(diff) < *best_dist) {
    _kdtree_nn(farther, target, best_dist, best_point, depth + 1);
  }
}

void kdtree_nn(const kdtree_t *kdtree,
               const float target[3],
               float *best_point,
               float *best_dist) {
  *best_dist = INFINITY;
  best_point[0] = target[0];
  best_point[1] = target[1];
  best_point[2] = target[2];
  _kdtree_nn(kdtree->root, target, best_dist, best_point, 0);
}
