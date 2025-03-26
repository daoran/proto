#include "xyz_voxel.h"

/******************************************************************************
 * VOXEL
 *****************************************************************************/

void voxel_setup(voxel_t *voxel, const int32_t key[3]) {
  voxel->key[0] = key[0];
  voxel->key[1] = key[1];
  voxel->key[2] = key[2];
  for (int i = 0; i < VOXEL_MAX_POINTS; ++i) {
    voxel->points[i * 3 + 0] = 0.0f;
    voxel->points[i * 3 + 1] = 0.0f;
    voxel->points[i * 3 + 2] = 0.0f;
  }
  voxel->length = 0;
}

void voxel_print(voxel_t *voxel) {
  printf("key: [%d, %d, %d]\n", voxel->key[0], voxel->key[1], voxel->key[2]);
  printf("length: %ld\n", voxel->length);
  for (int i = 0; i < voxel->length; ++i) {
    const float x = voxel->points[i * 3 + 0];
    const float y = voxel->points[i * 3 + 1];
    const float z = voxel->points[i * 3 + 2];
    printf("%d: [%.2f, %.2f, %.2f]\n", i, x, y, z);
  }
}

void voxel_reset(voxel_t *voxel) {
  int32_t key[3] = {-1, -1, -1};
  voxel_setup(voxel, key);
}

void voxel_copy(const voxel_t *src, voxel_t *dst) {
  dst->key[0] = src->key[0];
  dst->key[1] = src->key[1];
  dst->key[2] = src->key[2];
  for (int i = 0; i < (VOXEL_MAX_POINTS * 3); ++i) {
    dst->points[i] = src->points[i];
  }
  dst->length = src->length;
}

void voxel_add(voxel_t *voxel, const float p[3]) {
  if (voxel->length >= VOXEL_MAX_POINTS) {
    return;
  }
  voxel->points[voxel->length * 3 + 0] = p[0];
  voxel->points[voxel->length * 3 + 1] = p[1];
  voxel->points[voxel->length * 3 + 2] = p[2];
  voxel->length++;
}

/******************************************************************************
 * VOXEL-HASH
 *****************************************************************************/

#define POINT2VOXEL(POINT, VOXEL_SIZE, VOXEL)                                  \
  int32_t VOXEL[3] = {0};                                                      \
  VOXEL[0] = floor(POINT[0] / VOXEL_SIZE);                                     \
  VOXEL[1] = floor(POINT[1] / VOXEL_SIZE);                                     \
  VOXEL[2] = floor(POINT[2] / VOXEL_SIZE);

#define HASH_KEY(x, KEY)                                                       \
  const uint64_t v[3] = {(uint64_t) x[0], (uint64_t) x[1], (uint64_t) x[2]};   \
  uint64_t KEY = (v[0] * 73856093 ^ v[1] * 19349669 ^ v[2] * 83492791);

#define KEY_EQUALS(X, Y) (X[0] == Y[0]) && (X[1] == Y[1]) && (X[2] == Y[2])

voxel_hash_t *voxel_hash_malloc(const size_t capacity) {
  voxel_hash_t *voxel_hash = malloc(sizeof(voxel_hash_t));
  if (voxel_hash == NULL) {
    return NULL;
  }

  voxel_hash->length = 0;
  voxel_hash->capacity = capacity;
  voxel_hash->voxels = calloc(capacity, sizeof(voxel_t));
  if (voxel_hash->voxels == NULL) {
    free(voxel_hash);
    return NULL;
  }
  for (size_t i = 0; i < voxel_hash->capacity; ++i) {
    voxel_reset(&voxel_hash->voxels[i]);
  }

  return voxel_hash;
}

void voxel_hash_free(voxel_hash_t *voxel_hash) {
  if (voxel_hash == NULL) {
    return;
  }
  free(voxel_hash->voxels);
  free(voxel_hash);
}

int voxel_hash_expand(voxel_hash_t *voxel_hash) {
  // Allocate new voxels array.
  const size_t new_capacity = voxel_hash->capacity * 2;
  voxel_t *new_voxels = calloc(new_capacity, sizeof(voxel_t));
  if (new_voxels == NULL) {
    return -1;
  }
  for (size_t i = 0; i < new_capacity; ++i) {
    voxel_reset(&new_voxels[i]);
  }

  // Iterate voxels, move all non-empty ones to new voxels.
  for (size_t i = 0; i < voxel_hash->capacity; ++i) {
    voxel_t *voxel = &voxel_hash->voxels[i];
    if (voxel->length == 0) {
      continue;
    }

    // Hash key
    HASH_KEY(voxel->key, hash);
    size_t index = (size_t) (hash & (uint64_t) (new_capacity - 1));

    // Linear probing
    while (new_voxels[index].length != 0) {
      if (KEY_EQUALS(voxel->key, new_voxels[index].key)) {
        break;
      }
      index = ((index + 1) >= new_capacity) ? 0 : index + 1;
    }

    // Copy
    new_voxels[index].key[0] = voxel->key[0];
    new_voxels[index].key[1] = voxel->key[1];
    new_voxels[index].key[2] = voxel->key[2];
    for (int j = 0; j < voxel->length; ++j) {
      if (new_voxels[index].length >= VOXEL_MAX_POINTS) {
        break;
      }
      new_voxels[index].points[j * 3 + 0] = voxel->points[j * 3 + 0];
      new_voxels[index].points[j * 3 + 1] = voxel->points[j * 3 + 1];
      new_voxels[index].points[j * 3 + 2] = voxel->points[j * 3 + 2];
      new_voxels[index].length++;
    }
  }

  // Update
  free(voxel_hash->voxels);
  voxel_hash->voxels = new_voxels;
  voxel_hash->capacity = new_capacity;

  return 0;
}

int voxel_hash_add(voxel_hash_t *voxel_hash, const float p[3]) {
  // Expand?
  if (voxel_hash->length >= voxel_hash->capacity / 2) {
    if (voxel_hash_expand(voxel_hash) == -1) {
      return -1;
    }
  }

  // Hash key
  POINT2VOXEL(p, 0.1, key);
  HASH_KEY(key, hash);
  size_t index = (size_t) (hash & (uint64_t) (voxel_hash->capacity - 1));

  // Linear probing
  while (voxel_hash->voxels[index].length) {
    if (KEY_EQUALS(key, voxel_hash->voxels[index].key)) {
      break;
    }
    index = ((index + 1) >= voxel_hash->capacity) ? 0 : index + 1;
  }

  // Add point to voxel
  voxel_t *voxel = &voxel_hash->voxels[index];
  voxel_hash->length += (voxel_hash->voxels[index].length == 0) ? 1 : 0;
  voxel->key[0] = key[0];
  voxel->key[1] = key[1];
  voxel->key[2] = key[2];
  voxel_add(voxel, p);

  return 0;
}

voxel_t *voxel_hash_get(voxel_hash_t *voxel_hash, const int32_t *key) {
  // Hash key
  HASH_KEY(key, hash);
  size_t index = (size_t) (hash & (uint64_t) (voxel_hash->capacity - 1));

  // Linear probing
  while (voxel_hash->voxels[index].length == 0) {
    if (KEY_EQUALS(key, voxel_hash->voxels[index].key)) {
      return &voxel_hash->voxels[index];
    }
    index = ((index + 1) >= voxel_hash->capacity) ? 0 : index + 1;
  }

  return NULL;
}

size_t voxel_hash_length(const voxel_hash_t *voxel_hash) {
  // Length
  return voxel_hash->length;
}

voxel_hash_iter_t voxel_hash_iterator(voxel_hash_t *voxel_hash) {
  voxel_hash_iter_t it;
  it._voxel_hash = voxel_hash;
  it._index = 0;
  return it;
}

int voxel_hash_next(voxel_hash_iter_t *it) {
  voxel_hash_t *voxel_hash = it->_voxel_hash;
  while (it->_index < voxel_hash->capacity) {
    size_t i = it->_index;
    it->_index++;

    if (voxel_hash->voxels[i].length) {
      it->key = voxel_hash->voxels[i].key;
      it->value = &voxel_hash->voxels[i];
      return 1;
    }
  }

  return 0;
}
