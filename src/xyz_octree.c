#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "xyz_octree.h"

octree_node_t *octree_node_malloc(const double center[3],
                                  const double size,
                                  const int depth,
                                  const int max_depth) {
  octree_node_t *node = malloc(sizeof(octree_node_t));

  node->center[0] = center[0];
  node->center[1] = center[1];
  node->center[2] = center[2];
  node->size = size;
  node->depth = depth;
  node->max_depth = max_depth;

  for (int i = 0; i < 8; ++i) {
    node->children[i] = NULL;
  }
  memset(node->points, 0, sizeof(node->points));
  node->num_points = 0;

  return node;
}

void octree_node_free(octree_node_t *node) {
  if (node == NULL) {
    return;
  }

  for (int i = 0; i < 8; ++i) {
    octree_node_free(node->children[i]);
  }
  free(node);
}

octree_t *octree_malloc(const double center[3],
                        const double size,
                        const int depth,
                        const int max_depth,
                        const double *points,
                        const size_t num_points) {
  octree_t *octree = malloc(sizeof(octree_t));

  octree->center[0] = center[0];
  octree->center[1] = center[1];
  octree->center[2] = center[2];
  octree->size = size;
  octree->root = octree_node_malloc(center, size, depth, max_depth);

  for (size_t i = 0; i < num_points; i++) {
    octree_add_point(octree->root, &points[i * 3]);
  }

  return octree;
}

void octree_free(octree_t *octree) {
  if (octree == NULL) {
    return;
  }
  octree_node_free(octree->root);
  free(octree);
}

void octree_add_point(octree_node_t *node, const double point[3]) {
  // Max depth reached? Add the point
  if (node->depth == node->max_depth) {
    if (node->num_points >= OCTREE_MAX_POINTS) {
      return;
    }
    node->points[node->num_points * 3 + 0] = point[0];
    node->points[node->num_points * 3 + 1] = point[1];
    node->points[node->num_points * 3 + 2] = point[2];
    node->num_points++;
    return;
  }

  // Calculate node index
  int index = 0;
  for (int i = 0; i < 3; ++i) {
    if (point[i] < node->center[i]) {
      index |= (1 << i);
    }
  }

  // Create new child node if it doesn't exist already
  octree_node_t *child = node->children[index];
  if (child == NULL) {
    const double offset_x = -1 * ((index >> 0) & 1) * node->size / 4.0;
    const double offset_y = -1 * ((index >> 1) & 1) * node->size / 4.0;
    const double offset_z = -1 * ((index >> 2) & 1) * node->size / 4.0;

    const double center[3] = {
        node->center[0] + offset_x,
        node->center[1] + offset_y,
        node->center[2] + offset_z,
    };
    const double size = node->size / 2.0;
    const int depth = node->depth + 1;
    const int max_depth = node->max_depth;
    child = octree_node_malloc(center, size, depth, max_depth);

    node->children[index] = child;
  }

  // Recurse down the octree
  octree_add_point(child, point);
}
