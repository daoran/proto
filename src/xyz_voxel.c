#include "xyz_voxel.h"

/******************************************************************************
 * VOXEL
 *****************************************************************************/

void voxel_setup(voxel_t *voxel, const int32_t key[3]) {
  assert(voxel);
  assert(key);

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

voxel_t *voxel_malloc(const int32_t key[3]) {
  voxel_t *voxel = malloc(sizeof(voxel_t));
  voxel->points = calloc(VOXEL_MAX_POINTS * 3, sizeof(float));
  voxel_setup(voxel, key);
  return voxel;
}

void voxel_free(voxel_t *voxel) {
  assert(voxel);
  free(voxel->points);
  free(voxel);
}

void voxel_print(voxel_t *voxel) {
  assert(voxel);

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
  assert(voxel);
  assert(voxel->points != NULL);

  voxel->key[0] = -1;
  voxel->key[1] = -1;
  voxel->key[2] = -1;

  for (int i = 0; i < VOXEL_MAX_POINTS; ++i) {
    voxel->points[i * 3 + 0] = 0.0f;
    voxel->points[i * 3 + 1] = 0.0f;
    voxel->points[i * 3 + 2] = 0.0f;
  }

  voxel->length = 0;
}

void voxel_copy(const voxel_t *src, voxel_t *dst) {
  assert(src && dst);
  assert(src->points != NULL);
  assert(dst->points != NULL);

  dst->key[0] = src->key[0];
  dst->key[1] = src->key[1];
  dst->key[2] = src->key[2];
  for (int i = 0; i < src->length; ++i) {
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

#define HASH_KEY(x, KEY)                                                       \
  const uint32_t v[3] = {(uint32_t) x[0], (uint32_t) x[1], (uint32_t) x[2]};   \
  uint32_t KEY = (v[0] * 73856093 ^ v[1] * 19349669 ^ v[2] * 83492791);

#define KEY_EQUALS(X, Y) (X[0] == Y[0]) && (X[1] == Y[1]) && (X[2] == Y[2])

ht_t *ht_malloc(const size_t capacity) {
  ht_t *ht = malloc(sizeof(ht_t));
  ht->entries = calloc(capacity, sizeof(ht_entry_t));
  ht->capacity = capacity;
  ht->length = 0;
  return ht;
}

void ht_free(ht_t *ht) {
  free(ht->entries);
  free(ht);
}

voxel_t *ht_get(const ht_t *ht, const int32_t key[3]) {
  // Hash key
  HASH_KEY(key, hash);
  size_t index = (size_t) (hash & (uint32_t) (ht->capacity - 1));

  // Linear probing
  while (ht->entries[index].key != NULL) {
    if (KEY_EQUALS(key, ht->entries[index].key)) {
      return ht->entries[index].value;
    }
    index = ((index + 1) >= ht->capacity) ? 0 : index + 1;
  }

  return NULL;
}

int ht_expand(ht_t *ht) {
  printf("expand!\n");
  // Allocate new voxels array.
  const size_t new_capacity = ht->capacity * 2;
  ht_entry_t *new_entries = calloc(new_capacity, sizeof(ht_entry_t));
  if (new_entries == NULL) {
    return -1;
  }

  // Iterate voxels, move all non-empty ones to new voxels.
  for (size_t i = 0; i < ht->capacity; ++i) {
    ht_entry_t *entry = &ht->entries[i];
    if (entry->key == NULL) {
      continue;
    }

    // Hash key
    HASH_KEY(entry->key, hash);
    size_t index = (size_t) (hash & (uint32_t) (new_capacity - 1));

    // Linear probing
    while (new_entries[index].key != NULL) {
      if (KEY_EQUALS(entry->key, new_entries[index].key)) {
        break;
      }
      index = ((index + 1) >= new_capacity) ? 0 : index + 1;
    }

    // Copy
    new_entries[index].key = entry->key;
    new_entries[index].value = entry->value;
  }

  // Update
  free(ht->entries);
  ht->entries = new_entries;
  ht->capacity = new_capacity;

  return 0;
}

int ht_set(ht_t *ht, int32_t *key, voxel_t *value) {
  // Expand?
  if (ht->length >= ht->capacity / 2) {
    if (ht_expand(ht) == -1) {
      return -1;
    }
  }

  // Hash key
  HASH_KEY(key, hash);
  size_t index = (size_t) (hash & (uint32_t) (ht->capacity - 1));

  // Linear probing
  int is_new = 1;
  while (ht->entries[index].key != NULL) {
    if (KEY_EQUALS(key, ht->entries[index].key)) {
      is_new = 0;
      break;
    }
    index = ((index + 1) >= ht->capacity) ? 0 : index + 1;
  }

  // Add value
  ht_entry_t *entry = &ht->entries[index];
  entry->key = key;
  entry->value = value;
  ht->length += is_new;

  return 0;
}

ht_iter_t ht_iterator(ht_t *ht) {
  ht_iter_t it;
  it._ht = ht;
  it._index = 0;
  return it;
}

int ht_next(ht_iter_t *it) {
  ht_t *ht = it->_ht;
  while (it->_index < ht->capacity) {
    size_t i = it->_index;
    it->_index++;

    if (ht->entries[i].key) {
      it->key = ht->entries[i].key;
      it->value = ht->entries[i].value;
      return 1;
    }
  }

  return 0;
}

/******************************************************************************
 * VOXEL-HASH
 *****************************************************************************/

void point2voxel(const float p[3], const float voxel_size, int32_t voxel[3]) {
  voxel[0] = floor(p[0] / voxel_size);
  voxel[1] = floor(p[1] / voxel_size);
  voxel[2] = floor(p[2] / voxel_size);
}

// voxel_hash_t *voxel_hash_malloc(const size_t capacity) {
//   voxel_hash_t *voxel_hash = malloc(sizeof(voxel_hash_t));
//   if (voxel_hash == NULL) {
//     return NULL;
//   }
//
//   voxel_hash->length = 0;
//   voxel_hash->capacity = capacity;
//   voxel_hash->voxels = calloc(capacity, sizeof(voxel_t));
//   if (voxel_hash->voxels == NULL) {
//     free(voxel_hash);
//     return NULL;
//   }
//   for (size_t i = 0; i < voxel_hash->capacity; ++i) {
//     voxel_hash->voxels[i].key[0] = -1;
//     voxel_hash->voxels[i].key[1] = -1;
//     voxel_hash->voxels[i].key[2] = -1;
//     voxel_hash->voxels[i].points = calloc(VOXEL_MAX_POINTS * 3, sizeof(float));
//     voxel_hash->voxels[i].length = 0;
//   }
//
//   return voxel_hash;
// }
//
// void voxel_hash_free(voxel_hash_t *voxel_hash) {
//   if (voxel_hash == NULL) {
//     return;
//   }
//
//   for (size_t i = 0; i < voxel_hash->capacity; ++i) {
//     free(voxel_hash->voxels[i].points);
//   }
//   free(voxel_hash->voxels);
//   free(voxel_hash);
// }
//
// int voxel_hash_expand(voxel_hash_t *voxel_hash) {
//   // Allocate new voxels array.
//   const size_t new_capacity = voxel_hash->capacity * 2;
//   voxel_t *new_voxels = calloc(new_capacity, sizeof(voxel_t));
//   if (new_voxels == NULL) {
//     return -1;
//   }
//   for (size_t i = 0; i < new_capacity; ++i) {
//     new_voxels[i].key[0] = -1;
//     new_voxels[i].key[1] = -1;
//     new_voxels[i].key[2] = -1;
//     new_voxels[i].points = NULL;
//     new_voxels[i].length = 0;
//   }
//
//   // Iterate voxels, move all non-empty ones to new voxels.
//   for (size_t i = 0; i < voxel_hash->capacity; ++i) {
//     voxel_t *voxel = &voxel_hash->voxels[i];
//     if (voxel->length == 0) {
//       continue;
//     }
//
//     // Hash key
//     HASH_KEY(voxel->key, hash);
//     size_t index = (size_t) (hash & (uint32_t) (new_capacity - 1));
//
//     // Linear probing
//     while (new_voxels[index].length != 0) {
//       if (KEY_EQUALS(voxel->key, new_voxels[index].key)) {
//         break;
//       }
//       index = ((index + 1) >= new_capacity) ? 0 : index + 1;
//     }
//
//     // Copy
//     new_voxels[index].key[0] = voxel->key[0];
//     new_voxels[index].key[1] = voxel->key[1];
//     new_voxels[index].key[2] = voxel->key[2];
//     new_voxels[index].points = voxel->points;
//     new_voxels[index].length = voxel->length;
//     voxel->points = NULL;
//   }
//
//   // Update
//   free(voxel_hash->voxels);
//   voxel_hash->voxels = new_voxels;
//   voxel_hash->capacity = new_capacity;
//
//   return 0;
// }
//
// int voxel_hash_add(voxel_hash_t *voxel_hash, const float p[3]) {
//   // Expand?
//   if (voxel_hash->length >= voxel_hash->capacity / 2) {
//     if (voxel_hash_expand(voxel_hash) == -1) {
//       return -1;
//     }
//   }
//
//   // Hash key
//   POINT2VOXEL(p, 0.1, key);
//   HASH_KEY(key, hash);
//   size_t index = (size_t) (hash & (uint32_t) (voxel_hash->capacity - 1));
//
//   // Linear probing
//   while (voxel_hash->voxels[index].length) {
//     if (KEY_EQUALS(key, voxel_hash->voxels[index].key)) {
//       break;
//     }
//     index = ((index + 1) >= voxel_hash->capacity) ? 0 : index + 1;
//   }
//
//   // Add point to voxel
//   voxel_t *voxel = &voxel_hash->voxels[index];
//   voxel_hash->length += (voxel_hash->voxels[index].length == 0) ? 1 : 0;
//   voxel->key[0] = key[0];
//   voxel->key[1] = key[1];
//   voxel->key[2] = key[2];
//   voxel_add(voxel, p);
//
//   return 0;
// }
//
// voxel_t *voxel_hash_get(voxel_hash_t *voxel_hash, const int32_t *key) {
//   // Hash key
//   HASH_KEY(key, hash);
//   size_t index = (size_t) (hash & (uint32_t) (voxel_hash->capacity - 1));
//
//   // Linear probing
//   while (voxel_hash->voxels[index].length == 0) {
//     if (KEY_EQUALS(key, voxel_hash->voxels[index].key)) {
//       return &voxel_hash->voxels[index];
//     }
//     index = ((index + 1) >= voxel_hash->capacity) ? 0 : index + 1;
//   }
//
//   return NULL;
// }
//
// size_t voxel_hash_length(const voxel_hash_t *voxel_hash) {
//   // Length
//   return voxel_hash->length;
// }
