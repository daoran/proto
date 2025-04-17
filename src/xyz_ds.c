#include "xyz_ds.h"

/*******************************************************************************
 * DARRAY
 ******************************************************************************/

darray_t *darray_new(size_t element_size, size_t initial_max) {
  assert(element_size > 0);
  assert(initial_max > 0);

  darray_t *array = malloc(sizeof(darray_t) * 1);
  if (array == NULL) {
    return NULL;
  }

  array->end = 0;
  array->max = (int) initial_max;
  array->element_size = element_size;
  array->expand_rate = DEFAULT_EXPAND_RATE;
  array->contents = calloc(initial_max, sizeof(void *));
  if (array->contents == NULL) {
    free(array);
    return NULL;
  }

  return array;
}

void darray_clear(darray_t *array) {
  assert(array != NULL);
  for (int i = 0; i < array->max; i++) {
    if (array->contents[i]) {
      free(array->contents[i]);
    }
  }
}

void darray_destroy(darray_t *array) {
  if (array) {
    if (array->contents) {
      free(array->contents);
    }
    free(array);
  }
}

void darray_clear_destroy(darray_t *array) {
  if (array) {
    darray_clear(array);
    darray_destroy(array);
  }
}

int darray_push(darray_t *array, void *el) {
  assert(array != NULL);

  // Push
  array->contents[array->end] = el;
  array->end++;

  // Expand darray if necessary
  if (array->end >= array->max) {
    return darray_expand(array);
  }

  return 0;
}

void *darray_pop(darray_t *array) {
  assert(array != NULL);

  // pop
  void *el = darray_remove(array, array->end - 1);
  array->end--;

  // contract
  int expanded = array->end > (int) array->expand_rate;
  int trailing_memory = array->end % (int) array->expand_rate;
  if (expanded && trailing_memory) {
    darray_contract(array);
  }

  return el;
}

int darray_contains(darray_t *array,
                    void *el,
                    int (*cmp)(const void *, const void *)) {
  assert(array != NULL);
  assert(el != NULL);
  assert(cmp != NULL);

  // Check first element
  void *element = darray_get(array, 0);
  if (element != NULL && cmp(element, el) == 0) {
    return 1;
  }

  // Rest of element
  for (int i = 0; i < array->end; i++) {
    element = darray_get(array, i);
    if (element != NULL && cmp(element, el) == 0) {
      return 1;
    }
  }

  return 0;
}

darray_t *darray_copy(darray_t *array) {
  assert(array != NULL);

  // Copy first element
  darray_t *array_copy = darray_new(array->element_size, (size_t) array->max);
  void *el = darray_get(array, 0);
  void *el_copy = NULL;

  if (el != NULL) {
    el_copy = darray_new_element(array_copy);
    memcpy(el_copy, el, array->element_size);
    darray_set(array_copy, 0, el_copy);
  }

  // Copy the rest of the elements
  for (int i = 1; i < array->end; i++) {
    el = darray_get(array, i);
    // el_copy = NULL;

    if (el != NULL) {
      memcpy(el_copy, el, array->element_size);
      darray_set(array_copy, i, el);
    }
  }

  return array_copy;
}

void *darray_new_element(darray_t *array) {
  assert(array != NULL);
  assert(array->element_size > 0);
  return calloc(1, array->element_size);
}

void *darray_first(darray_t *array) {
  assert(array != NULL);
  return array->contents[0];
}

void *darray_last(darray_t *array) {
  assert(array != NULL);
  return array->contents[array->end - 1];
}

void darray_set(darray_t *array, int i, void *el) {
  assert(array != NULL);
  assert(i < array->max);

  // Set
  array->contents[i] = el;

  // Update end
  if (i > array->end) {
    array->end = i;
  }
}

void *darray_get(darray_t *array, int i) {
  assert(array != NULL);
  assert(i < array->max);
  return array->contents[i];
}

void *darray_update(darray_t *array, int i, void *el) {
  assert(array != NULL);
  assert(i < array->max);
  void *old_el;

  // Update
  old_el = darray_get(array, i);
  darray_set(array, i, el);

  return old_el;
}

void *darray_remove(darray_t *array, int i) {
  assert(array != NULL);
  void *el = array->contents[i];
  array->contents[i] = NULL;
  return el;
}

static inline int darray_resize(darray_t *array, size_t new_max) {
  assert(array != NULL);

  // Calculate new max and size
  int old_max = (int) array->max;
  array->max = (int) new_max;

  // Reallocate new memory
  void *contents = realloc(array->contents, new_max * sizeof(void *));
  if (contents == NULL) {
    return -1;
  }
  array->contents = contents;

  // Initialize new memory to NULL
  for (int i = old_max; i < (int) new_max; i++) {
    array->contents[i] = NULL;
  }

  return 0;
}

int darray_expand(darray_t *array) {
  assert(array != NULL);
  assert(array->max > 0);

  size_t old_max = (size_t) array->max;
  size_t new_max = (size_t) array->max + array->expand_rate;
  int res = darray_resize(array, new_max);
  if (res != 0) {
    return -1;
  }
  memset(array->contents + old_max, 0, array->expand_rate + 1);

  return 0;
}

int darray_contract(darray_t *array) {
  assert(array != NULL);
  assert(array->max > 0);

  // Contract
  int new_size = 0;
  if (array->end < (int) array->expand_rate) {
    new_size = (int) array->expand_rate;
  } else {
    new_size = array->end;
  }

  return darray_resize(array, (size_t) new_size + 1);
}

/*******************************************************************************
 * LIST
 ******************************************************************************/

list_t *list_malloc(void) {
  list_t *list = calloc(1, sizeof(list_t));
  list->length = 0;
  list->first = NULL;
  list->last = NULL;
  return list;
}

void list_free(list_t *list) {
  assert(list != NULL);

  list_node_t *node;
  list_node_t *next_node;

  // Destroy
  node = list->first;
  while (node != NULL) {
    next_node = node->next;
    if (node) {
      free(node);
    }
    node = next_node;
  }

  free(list);
}

void list_clear(list_t *list) {
  assert(list != NULL);

  list_node_t *node;
  list_node_t *next_node;

  node = list->first;
  while (node != NULL) {
    next_node = node->next;
    free(node->value);
    node = next_node;
  }
}

void list_clear_free(list_t *list) {
  assert(list != NULL);

  list_node_t *node = list->first;
  while (node != NULL) {
    list_node_t *next_node = node->next;
    free(node->value);
    free(node);
    node = next_node;
  }
  free(list);
}

void list_push(list_t *list, void *value) {
  assert(list != NULL);
  assert(value != NULL);

  // Initialize node
  list_node_t *node = calloc(1, sizeof(list_node_t));
  if (node == NULL) {
    return;
  }
  node->value = value;

  // Push node
  if (list->last == NULL) {
    list->first = node;
    list->last = node;
  } else {
    list->last->next = node;
    node->prev = list->last;
    list->last = node;
  }

  list->length++;
}

void *list_pop(list_t *list) {
  assert(list != NULL);

  // Get last
  list_node_t *last = list->last;
  if (last == NULL) {
    return NULL;
  }
  void *value = last->value;
  list_node_t *before_last = last->prev;
  free(last);

  // Pop
  if (before_last == NULL && list->length == 1) {
    list->last = NULL;
    list->first = NULL;
  } else {
    list->last = before_last;
  }
  list->length--;

  return value;
}

void *list_pop_front(list_t *list) {
  assert(list != NULL);
  assert(list->first != NULL);

  // Pop front
  list_node_t *first_node = list->first;
  void *data = first_node->value;
  list_node_t *next_node = first_node->next;

  if (next_node != NULL) {
    list->first = next_node;
  } else {
    list->first = NULL;
  }
  list->length--;

  // Clean up
  free(first_node);

  return data;
}

void *list_shift(list_t *list) {
  assert(list != NULL);

  list_node_t *first = list->first;
  void *value = first->value;
  list_node_t *second = list->first->next;

  list->first = second;
  list->length--;
  free(first);

  return value;
}

void list_unshift(list_t *list, void *value) {
  assert(list != NULL);

  list_node_t *node = calloc(1, sizeof(list_node_t));
  if (node == NULL) {
    return;
  }
  node->value = value;

  if (list->first == NULL) {
    list->first = node;
    list->last = node;
  } else {
    node->next = list->first;
    list->first->prev = node;
    list->first = node;
  }

  list->length++;
}

void *list_remove(list_t *list,
                  void *value,
                  int (*cmp)(const void *, const void *)) {
  assert(list != NULL);
  assert(value != NULL);
  assert(cmp != NULL);

  // Iterate list
  list_node_t *node = list->first;
  while (node != NULL) {

    // Compare target with node value
    if (cmp(node->value, value) == 0) {
      value = node->value;

      if (list->length == 1) {
        // Last node in list
        list->first = NULL;
        list->last = NULL;

      } else if (node == list->first) {
        // First node in list
        list->first = node->next;
        node->next->prev = NULL;

      } else if (node == list->last) {
        // In the case of removing last node in list
        list->last = node->prev;
        node->prev->next = NULL;

      } else {
        // Remove others
        node->prev->next = node->next;
        node->next->prev = node->prev;
      }
      list->length--;
      free(node);

      return value;
    }

    node = node->next;
  }

  return NULL;
}

int list_remove_destroy(list_t *list,
                        void *value,
                        int (*cmp)(const void *, const void *),
                        void (*free_func)(void *)) {
  assert(list != NULL);
  void *result = list_remove(list, value, cmp);
  free_func(result);
  return 0;
}

/////////////
// HASHMAP //
/////////////

// FNV-1a constants (64-bit version is generally recommended for wider distribution)
#define FNV_PRIME_64 1099511628211ULL
#define FNV_OFFSET_BASIS_64 14695981039346656037ULL

size_t hm_default_hash(const void *key, const size_t key_size) {
  size_t hash = FNV_OFFSET_BASIS_64;
  const uint8_t *p = (const uint8_t *) key;
  for (size_t i = 0; i < key_size; i++) {
    hash ^= (size_t) p[i];
    hash *= FNV_PRIME_64;
  }
  return hash;
}

size_t hm_int_hash(const void *key) {
  return hm_default_hash(key, sizeof(int));
}

size_t hm_float_hash(const void *key) {
  return hm_default_hash(key, sizeof(float));
}

size_t hm_double_hash(const void *key) {
  return hm_default_hash(key, sizeof(float));
}

size_t hm_string_hash(const void *key) {
  return hm_default_hash(key, strlen((char *) key));
}

hm_t *hm_malloc(const size_t capacity,
                size_t (*hash)(const void *),
                int (*cmp)(const void *, const void *)) {
  hm_t *hm = malloc(sizeof(hm_t));
  hm->entries = calloc(capacity, sizeof(hm_entry_t));
  hm->length = 0;
  hm->capacity = capacity;
  hm->hash = hash;
  hm->cmp = cmp;
  return hm;
}

void hm_free(hm_t *hm, void (*free_key)(void *), void (*free_value)(void *)) {
  assert(hm);

  for (size_t i = 0; i < hm->capacity; ++i) {
    hm_entry_t *entry = &hm->entries[i];
    if (entry->key == NULL) {
      continue;
    }
    if (free_key) {
      free_key(entry->key);
    }
    if (free_value) {
      free_value(entry->value);
    }
  }
  free(hm->entries);
  free(hm);
}

void *hm_get(const hm_t *hm, const void *key) {
  assert(hm);
  assert(hm->hash);
  assert(key);

  // Hash key
  const size_t hash = hm->hash(key);
  size_t index = hash & (hm->capacity - 1);

  // Linear probing
  while (hm->entries[index].key != NULL) {
    if (hm->cmp(key, hm->entries[index].key) == 0) {
      return hm->entries[index].value;
    }
    index = ((index + 1) >= hm->capacity) ? 0 : index + 1;
  }

  return NULL;
}

int hm_expand(hm_t *hm) {
  assert(hm);
  assert(hm->hash);

  // Allocate new voxels array.
  const size_t new_capacity = hm->capacity * 2;
  hm_entry_t *new_entries = calloc(new_capacity, sizeof(hm_entry_t));
  if (new_entries == NULL) {
    return -1;
  }

  // Iterate and move
  for (size_t i = 0; i < hm->capacity; ++i) {
    hm_entry_t *entry = &hm->entries[i];
    if (entry->key == NULL) {
      continue;
    }

    // Hash key
    const size_t hash = hm->hash(entry->key);
    size_t index = hash & (new_capacity - 1);

    // Linear probing
    while (new_entries[index].key != NULL) {
      if (hm->cmp(entry->key, new_entries[index].key) == 0) {
        break;
      }
      index = ((index + 1) >= new_capacity) ? 0 : index + 1;
    }

    // Copy
    new_entries[index].key = entry->key;
    new_entries[index].value = entry->value;
  }

  // Update
  free(hm->entries);
  hm->entries = new_entries;
  hm->capacity = new_capacity;

  return 0;
}

int hm_set(hm_t *hm, void *key, void *value) {
  // Expand?
  if (hm->length >= hm->capacity / 2) {
    if (hm_expand(hm) == -1) {
      return -1;
    }
  }

  // Hash key
  const size_t hash = hm->hash(key);
  size_t index = hash & (hm->capacity - 1);

  // Linear probing
  int is_new = 1;
  while (hm->entries[index].key != NULL) {
    if (hm->cmp(key, hm->entries[index].key) == 0) {
      is_new = 0;
      break;
    }
    index = ((index + 1) >= hm->capacity) ? 0 : index + 1;
  }

  // Add value
  hm_entry_t *entry = &hm->entries[index];
  entry->key = key;
  entry->value = value;
  hm->length += is_new;

  return 0;
}

hm_iter_t hm_iterator(hm_t *hm) {
  hm_iter_t it;
  it._hm = hm;
  it._index = 0;
  return it;
}

int hm_next(hm_iter_t *it) {
  hm_t *hm = it->_hm;
  while (it->_index < hm->capacity) {
    size_t i = it->_index;
    it->_index++;

    if (hm->entries[i].key) {
      it->key = hm->entries[i].key;
      it->value = hm->entries[i].value;
      return 1;
    }
  }

  return 0;
}
