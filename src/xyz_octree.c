#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "xyz_octree.h"

/*****************************************************************************
 * Octree Node
 ****************************************************************************/

octree_node_t *octree_node_malloc(const float center[3],
                                  const float size,
                                  const int depth,
                                  const int max_depth,
                                  const int max_points) {
  octree_node_t *node = malloc(sizeof(octree_node_t));

  node->center[0] = center[0];
  node->center[1] = center[1];
  node->center[2] = center[2];
  node->size = size;
  node->depth = depth;
  node->max_depth = max_depth;
  node->max_points = max_points;

  for (int i = 0; i < 8; ++i) {
    node->children[i] = NULL;
  }
  node->points = malloc(sizeof(float) * 3 * max_points);
  node->num_points = 0;
  node->capacity = max_points;

  return node;
}

void octree_node_free(octree_node_t *node) {
  if (node == NULL) {
    return;
  }

  for (int i = 0; i < 8; ++i) {
    octree_node_free(node->children[i]);
  }
  free(node->points);
  free(node);
}

/*****************************************************************************
 * Octree
 ****************************************************************************/

octree_t *octree_malloc(const float center[3],
                        const float size,
                        const int depth,
                        const int max_depth,
                        const int max_points,
                        const float *points,
                        const size_t num_points) {
  assert(center);
  octree_t *octree = malloc(sizeof(octree_t));

  octree->center[0] = center[0];
  octree->center[1] = center[1];
  octree->center[2] = center[2];
  octree->size = size;
  octree->root = octree_node_malloc(center, size, depth, max_depth, max_points);

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

void octree_add_point(octree_node_t *node, const float point[3]) {
  assert(node);
  assert(point);

  // Max depth reached? Add the point
  if (node->depth == node->max_depth) {
    if (node->num_points >= node->max_points) {
      return;
    }
    node->points[node->num_points * 3 + 0] = point[0];
    node->points[node->num_points * 3 + 1] = point[1];
    node->points[node->num_points * 3 + 2] = point[2];
    node->num_points++;
    if (node->num_points >= node->capacity) {
      node->capacity = node->capacity * 2;
      node->points = realloc(node->points, sizeof(float) * 3 * node->capacity);
    }
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
    const float offset_x = -1 * ((index >> 0) & 1) * node->size / 4.0;
    const float offset_y = -1 * ((index >> 1) & 1) * node->size / 4.0;
    const float offset_z = -1 * ((index >> 2) & 1) * node->size / 4.0;

    const float center[3] = {
        node->center[0] + offset_x,
        node->center[1] + offset_y,
        node->center[2] + offset_z,
    };
    const float size = node->size / 2.0;
    const int depth = node->depth + 1;
    const int max_depth = node->max_depth;
    const int max_points = node->max_points;
    child = octree_node_malloc(center, size, depth, max_depth, max_points);

    node->children[index] = child;
  }

  // Recurse down the octree
  octree_add_point(child, point);
}

void octree_points(const octree_node_t *node, octree_data_t *data) {
  assert(node);
  assert(data && data->points && data->capacity > 0);

  if (node->num_points) {
    for (size_t i = 0; i < node->num_points; ++i) {
      data->points[data->num_points * 3 + 0] = node->points[i * 3 + 0];
      data->points[data->num_points * 3 + 1] = node->points[i * 3 + 1];
      data->points[data->num_points * 3 + 2] = node->points[i * 3 + 2];
      data->num_points++;
      if (data->num_points >= data->capacity) {
        data->capacity = data->capacity * 2;
        data->points = realloc(data->points, sizeof(float) * 3 * data->capacity);
      }
    }
  }

  for (size_t i = 0; i < 8; ++i) {
    if (node->children[i]) {
      octree_points(node->children[i], data);
    }
  }
}
