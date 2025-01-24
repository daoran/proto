#include "ds.h"

////////////
// DARRAY //
////////////

darray_t *darray_new(size_t element_size, size_t initial_max) {
  assert(element_size > 0);
  assert(initial_max > 0);

  darray_t *array = MALLOC(darray_t, 1);
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
    FREE_MEM(array->contents[i], free);
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

//////////
// LIST //
//////////

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
    FREE_MEM(node, free);
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

  if (before_last == NULL) {
    return NULL;
  }
  before_last->next = NULL;

  return value;
}

void *list_pop_front(list_t *list) {
  assert(list != NULL);
  assert(list->first != NULL);

  // pop front
  list_node_t *first_node = list->first;
  void *data = first_node->value;
  list_node_t *next_node = first_node->next;

  if (next_node != NULL) {
    list->first = next_node;
  } else {
    list->first = NULL;
  }
  list->length--;

  // clean up
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

///////////
// STACK //
///////////

mstack_t *stack_new(void) {
  mstack_t *s = MALLOC(mstack_t, 1);
  s->size = 0;
  s->root = NULL;
  s->end = NULL;
  return s;
}

void mstack_destroy_traverse(mstack_node_t *n, void (*free_func)(void *)) {
  if (n->next) {
    mstack_destroy_traverse(n->next, free_func);
  }
  if (free_func) {
    free_func(n->value);
  }
  free(n);
  n = NULL;
}

void mstack_clear_destroy(mstack_t *s, void (*free_func)(void *)) {
  if (s->root) {
    mstack_destroy_traverse(s->root, free_func);
  }
  free(s);
  s = NULL;
}

void mstack_destroy(mstack_t *s) {
  if (s->root) {
    mstack_destroy_traverse(s->root, NULL);
  }
  free(s);
  s = NULL;
}

int mstack_push(mstack_t *s, void *value) {
  mstack_node_t *n = MALLOC(mstack_node_t, 1);
  if (n == NULL) {
    return -1;
  }

  mstack_node_t *prev_end = s->end;
  n->value = value;
  n->next = NULL;
  n->prev = prev_end;

  if (s->size == 0) {
    s->root = n;
    s->end = n;
  } else {
    prev_end->next = n;
    s->end = n;
  }
  s->size++;

  return 0;
}

void *mstack_pop(mstack_t *s) {
  void *value = s->end->value;
  mstack_node_t *previous = s->end->prev;

  free(s->end);
  if (s->size > 1) {
    previous->next = NULL;
    s->end = previous;
  } else {
    s->root = NULL;
    s->end = NULL;
  }
  s->size--;

  return value;
}

///////////
// QUEUE //
///////////

queue_t *queue_malloc(void) {
  queue_t *q = calloc(1, sizeof(queue_t));
  q->queue = list_malloc();
  q->count = 0;
  return q;
}

void queue_free(queue_t *q) {
  assert(q != NULL);
  list_free(q->queue);
  free(q);
  q = NULL;
}

int queue_enqueue(queue_t *q, void *data) {
  assert(q != NULL);
  list_push(q->queue, data);
  q->count++;
  return 0;
}

void *queue_dequeue(queue_t *q) {
  assert(q != NULL);
  void *data = list_pop_front(q->queue);
  q->count--;

  return data;
}

int queue_empty(queue_t *q) {
  assert(q != NULL);
  return (q->count == 0) ? 1 : 0;
}

void *queue_first(queue_t *q) {
  assert(q != NULL);
  if (q->count != 0) {
    return q->queue->first->value;
  }
  return NULL;
}

void *queue_last(queue_t *q) {
  assert(q != NULL);
  if (q->count != 0) {
    return q->queue->last->value;
  }
  return NULL;
}

/////////////
// HASHMAP //
/////////////

static inline int default_cmp(void *a, void *b) { return strcmp(a, b); }

static uint32_t default_hash(void *a) {
  // Simple bob jenkins's hash algorithm
  char *k = a;
  uint32_t hash = 0;
  for (uint32_t i = 0; i < strlen(a); i++) {
    hash += (uint32_t) k[i];
    hash += (hash << 10);
    hash ^= (hash >> 6);
  }

  hash += (hash << 3);
  hash ^= (hash >> 11);
  hash += (hash << 15);

  return hash;
}

static inline void *default_key_copy(void *target) {
  return string_malloc(target);
}

static inline void *default_value_copy(void *target) {
  return string_malloc(target);
}

hashmap_t *hashmap_new(void) {
  hashmap_t *map = MALLOC(hashmap_t, 1);
  if (map == NULL) {
    return NULL;
  }

  // Create bucket
  map->buckets = darray_new(sizeof(darray_t *), DEFAULT_NUMBER_OF_BUCKETS);
  map->buckets->end = map->buckets->max; // fake out expanding it
  if (map->buckets == NULL) {
    free(map);
    return NULL;
  }

  // Set comparator and hash functions
  map->cmp = default_cmp;
  map->hash = default_hash;

  // Set key and value copy functions
  map->copy_kv = 1;
  map->k_copy = default_key_copy;
  map->v_copy = default_value_copy;
  map->k_free = free;
  map->v_free = free;

  return map;
}

static void free_bucket(darray_t *bucket) {
  assert(bucket != NULL);

  for (int i = 0; i < bucket->end; i++) {
    hashmap_node_t *n = darray_get(bucket, i);
    free(n);
  }

  darray_destroy(bucket);
}

static void clear_free_bucket(hashmap_t *map, darray_t *bucket) {
  assert(map != NULL);
  assert(bucket != NULL);

  // Clear free bucket
  for (int i = 0; i < bucket->end; i++) {
    hashmap_node_t *n = darray_get(bucket, i);
    map->k_free(n->key);
    map->k_free(n->value);
    free(n);
  }

  darray_destroy(bucket);
}

static void free_buckets(hashmap_t *map) {
  assert(map != NULL);

  // Free buckets
  for (int i = 0; i < map->buckets->end; i++) {
    darray_t *bucket = darray_get(map->buckets, i);

    if (bucket) {
      if (map->copy_kv) {
        clear_free_bucket(map, bucket);
      } else {
        free_bucket(bucket);
      }
    }
  }

  darray_destroy(map->buckets);
}

void hashmap_clear_destroy(hashmap_t *map) {
  if (map) {
    if (map->buckets) {
      free_buckets(map);
    }
    free(map);
  }
}

void hashmap_destroy(hashmap_t *map) {
  if (map) {
    if (map->buckets) {
      free_buckets(map);
    }
    free(map);
  }
}

static hashmap_node_t *hashmap_node_new(uint32_t h, void *k, void *v) {
  assert(k != NULL);
  assert(v != NULL);

  // Setup
  hashmap_node_t *node = calloc(1, sizeof(hashmap_node_t));
  if (node == NULL) {
    return NULL;
  }

  // Create hashmap node
  node->key = k;
  node->value = v;
  node->hash = h;

  return node;
}

static darray_t *
hashmap_find_bucket(hashmap_t *map, void *k, int create, uint32_t *hash_out) {
  assert(map != NULL);
  assert(k != NULL);
  assert(hash_out != NULL);

  // Pre-check
  uint32_t hash = map->hash(k);
  int bucket_n = hash % DEFAULT_NUMBER_OF_BUCKETS;
  if ((bucket_n >= 0) == 0) {
    return NULL;
  }
  *hash_out = hash; // Store it for return so caller can use it

  // Find bucket
  darray_t *bucket = darray_get(map->buckets, bucket_n);

  // Coundn't find bucket, create one instead
  if (!bucket && create) {
    // New bucket, set it up
    bucket = darray_new(sizeof(void *), DEFAULT_NUMBER_OF_BUCKETS);
    if (bucket == NULL) {
      return NULL;
    }
    darray_set(map->buckets, bucket_n, bucket);
  }

  return bucket;
}

int hashmap_set(hashmap_t *map, void *k, void *v) {
  assert(map != NULL);
  assert(map->k_copy != NULL);
  assert(map->v_copy != NULL);
  assert(k != NULL);
  assert(v != NULL);

  // Pre-check
  uint32_t hash = 0;
  darray_t *bucket = hashmap_find_bucket(map, k, 1, &hash);
  if (bucket == NULL) {
    return -1;
  }

  // Set hashmap
  hashmap_node_t *node = hashmap_node_new(hash, map->k_copy(k), map->v_copy(v));
  if (node == NULL) {
    return -1;
  }
  darray_push(bucket, node);

  return 0;
}

static inline int
hashmap_get_node(hashmap_t *map, uint32_t hash, darray_t *bucket, void *k) {
  assert(map != NULL);
  assert(bucket != NULL);
  assert(k != NULL);

  for (int i = 0; i < bucket->end; i++) {
    hashmap_node_t *node = darray_get(bucket, i);
    if (node->hash == hash && map->cmp(node->key, k) == 0) {
      return i;
    }
  }

  return -1;
}

void *hashmap_get(hashmap_t *map, void *k) {
  assert(map != NULL);
  assert(k != NULL);

  // Find bucket
  uint32_t hash = 0;
  darray_t *bucket = hashmap_find_bucket(map, k, 0, &hash);
  if (bucket == NULL) {
    return NULL;
  }

  // Find hashmap node
  int i = hashmap_get_node(map, hash, bucket, k);
  if (i == -1) {
    return NULL;
  }

  // Get value
  hashmap_node_t *node = darray_get(bucket, i);
  if (node == NULL) {
    return NULL;
  }

  return node->value;
}

int hashmap_traverse(hashmap_t *map,
                     int (*hashmap_traverse_cb)(hashmap_node_t *)) {
  assert(map != NULL);
  assert(hashmap_traverse_cb != NULL);

  // Traverse
  int rc = 0;
  for (int i = 0; i < map->buckets->end; i++) {
    darray_t *bucket = darray_get(map->buckets, i);

    if (bucket) {
      for (int j = 0; j < bucket->end; j++) {
        hashmap_node_t *node = darray_get(bucket, j);
        rc = hashmap_traverse_cb(node);

        if (rc != 0) {
          return rc;
        }
      }
    }
  }

  return 0;
}

void *hashmap_delete(hashmap_t *map, void *k) {
  assert(map != NULL);
  assert(k != NULL);

  // Find bucket containing hashmap node
  uint32_t hash = 0;
  darray_t *bucket = hashmap_find_bucket(map, k, 0, &hash);
  if (bucket == NULL) {
    return NULL;
  }

  // From bucket get hashmap node and free it
  int i = hashmap_get_node(map, hash, bucket, k);
  if (i == -1) {
    return NULL;
  }

  // Get node
  hashmap_node_t *node = darray_get(bucket, i);
  void *v = node->value;
  if (map->copy_kv) {
    map->k_free(node->key);
  }
  free(node);

  // Check to see if last element in bucket is a node
  hashmap_node_t *ending = darray_pop(bucket);
  if (ending != node) {
    // Alright looks like it's not the last one, swap it
    darray_set(bucket, i, ending);
  }

  return v;
}
