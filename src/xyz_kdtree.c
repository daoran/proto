#include <stdio.h>
#include <stdlib.h>

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

// kdtree_node_t *kdtree_build(const float *points,
//                             const size_t num_points,
//                             const int kdim) {
//   if (points == NULL || num_points == 0) {
//     return NULL;
//   }
//
//   kdtree_node_t *root = NULL;
//   for (size_t i = 0; i < num_points; i++) {
//     root = kdtree_insert(root, &points[i], kdim, 0);
//   }
//
//   return root;
// }

int point_cmp(const void *a, const void *b, void *k) {
  return ((float *) a)[*(int *) k] - ((float *) b)[*(int *) k];
}

// Function to construct a balanced k-d tree
kdtree_node_t *
kdtree_build(float *points, const int start, const int end, const int depth) {
  if (start > end) {
    return NULL;
  }

  int k = depth % KDTREE_KDIM;
  const int mid = (start + end) / 2;
  qsort_r(points + start, end - start + 1, sizeof(float), point_cmp, &k);

  kdtree_node_t *root = kdtree_node_malloc(&points[mid], k);
  root->left = kdtree_build(points, start, mid - 1, depth + 1);
  root->right = kdtree_build(points, mid + 1, end, depth + 1);

  return root;
}

void kdtree_inorder_traversal(kdtree_node_t *root) {
  if (root == NULL) {
    return;
  }
  kdtree_inorder_traversal(root->left);
  printf("(%.2f, %.2f, %.2f)\n", root->p[0], root->p[1], root->p[2]);
  kdtree_inorder_traversal(root->right);
}
