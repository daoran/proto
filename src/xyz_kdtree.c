#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "xyz_kdtree.h"

kdtree_node_t *kdtree_node_malloc(const float p[3], const int k) {
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

kdtree_node_t *kdtree_insert(kdtree_node_t *root,
                             const float p[3],
                             const int depth) {
  const int k = depth % KDTREE_KDIM;
  if (root == NULL) {
    return kdtree_node_malloc(p, k);
  }

  if (p[k] < root->p[k]) {
    root->left = kdtree_insert(root->left, p, depth + 1);
  } else {
    root->right = kdtree_insert(root->right, p, depth + 1);
  }

  return root;
}

int point_cmp(const void *a, const void *b, void *k) {
  return (((float *) a)[*(int *) k] < ((float *) b)[*(int *) k]) ? -1 : 1;
}

kdtree_node_t *
kdtree_build(float *points, const int start, const int end, const int depth) {
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
  root->left = kdtree_build(points, start, mid - 1, depth + 1);
  root->right = kdtree_build(points, mid + 1, end, depth + 1);

  return root;
}

void kdtree_points(const kdtree_node_t *root, float *points, size_t *n) {
  if (root == NULL) {
    return;
  }

  points[*n * 3 + 0] = root->p[0];
  points[*n * 3 + 1] = root->p[1];
  points[*n * 3 + 2] = root->p[2];
  (*n)++;

  kdtree_points(root->left, points, n);
  kdtree_points(root->right, points, n);
}

void _kdtree_search(const kdtree_node_t *node,
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
  kdtree_node_t *closer = NULL;
  kdtree_node_t *farther = NULL;
  if (diff <= 0) {
    closer = node->left;
    farther = node->right;
  } else {
    closer = node->right;
    farther = node->left;
  }
  _kdtree_search(closer, target, best_dist, best_point, depth + 1);

  // Search the farther subtree
  if (fabs(diff) < *best_dist) {
    _kdtree_search(farther, target, best_dist, best_point, depth + 1);
  }
}

void kdtree_nn(const kdtree_node_t *root,
               const float target[3],
               float *best_point,
               float *best_dist) {
  *best_dist = INFINITY;
  best_point[0] = target[0];
  best_point[1] = target[1];
  best_point[2] = target[2];
  _kdtree_search(root, target, best_dist, best_point, 0);
}
