#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>

#include "macros.h"
#include "data.h"


////////////
// DARRAY //
////////////

#ifndef DEFAULT_EXPAND_RATE
#define DEFAULT_EXPAND_RATE 300
#endif

typedef struct darray_t {
  int end;
  int max;
  size_t element_size;
  size_t expand_rate;
  void **contents;
} darray_t;

darray_t *darray_new(size_t element_size, size_t initial_max);
void darray_destroy(darray_t *array);
void darray_clear(darray_t *array);
void darray_clear_destroy(darray_t *array);
int darray_push(darray_t *array, void *el);
void *darray_pop(darray_t *array);
int darray_contains(darray_t *array,
                    void *el,
                    int (*cmp)(const void *, const void *));
darray_t *darray_copy(darray_t *array);
void *darray_new_element(darray_t *array);
void *darray_first(darray_t *array);
void *darray_last(darray_t *array);
void darray_set(darray_t *array, int i, void *el);
void *darray_get(darray_t *array, int i);
void *darray_update(darray_t *array, int i, void *el);
void *darray_remove(darray_t *array, int i);
int darray_expand(darray_t *array);
int darray_contract(darray_t *array);

//////////
// LIST //
//////////

typedef struct list_node_t list_node_t;
struct list_node_t {
  list_node_t *next;
  list_node_t *prev;
  void *value;
};

typedef struct list_t {
  int length;
  list_node_t *first;
  list_node_t *last;
} list_t;

list_t *list_malloc(void);
void list_free(list_t *list);
void list_clear(list_t *list);
void list_clear_free(list_t *list);
void list_push(list_t *list, void *value);
void *list_pop(list_t *list);
void *list_pop_front(list_t *list);
void *list_shift(list_t *list);
void list_unshift(list_t *list, void *value);
void *list_remove(list_t *list,
                  void *target,
                  int (*cmp)(const void *, const void *));
int list_remove_destroy(list_t *list,
                        void *value,
                        int (*cmp)(const void *, const void *),
                        void (*free_func)(void *));

///////////
// STACK //
///////////

typedef struct mstack_node_t mstack_node_t;
struct mstack_node_t {
  void *value;
  mstack_node_t *next;
  mstack_node_t *prev;
};

typedef struct mstack_t {
  int size;
  mstack_node_t *root;
  mstack_node_t *end;
} mstack_t;

mstack_t *stack_new(void);
void mstack_destroy_traverse(mstack_node_t *n, void (*free_func)(void *));
void mstack_clear_destroy(mstack_t *s, void (*free_func)(void *));
void mstack_destroy(mstack_t *s);
int mstack_push(mstack_t *s, void *value);
void *mstack_pop(mstack_t *s);

///////////
// QUEUE //
///////////

typedef struct queue_t {
  int count;
  list_t *queue;
} queue_t;

queue_t *queue_malloc(void);
void queue_free(queue_t *q);
int queue_enqueue(queue_t *q, void *data);
void *queue_dequeue(queue_t *q);
int queue_count(queue_t *q);
int queue_empty(queue_t *q);
int queue_full(queue_t *q);
void *queue_first(queue_t *q);
void *queue_last(queue_t *q);

/////////////
// HASHMAP //
/////////////

#ifndef DEFEAULT_NUMBER_OF_BUCKETS
#define DEFAULT_NUMBER_OF_BUCKETS 10000
#endif

typedef struct hashmap_node_t {
  uint32_t hash;
  void *key;
  void *value;
} hashmap_node_t;

typedef struct hashmap_t {
  darray_t *buckets;
  int (*cmp)(void *, void *);
  uint32_t (*hash)(void *);

  int copy_kv;
  void *(*k_copy)(void *);
  void *(*v_copy)(void *);
  void (*k_free)(void *);
  void (*v_free)(void *);
} hashmap_t;

hashmap_t *hashmap_new(void);
void hashmap_clear_destroy(hashmap_t *map);
void hashmap_destroy(hashmap_t *map);
int hashmap_set(hashmap_t *map, void *key, void *data);
void *hashmap_get(hashmap_t *map, void *key);
int hashmap_traverse(hashmap_t *map,
                     int (*hashmap_traverse_cb)(hashmap_node_t *node));
void *hashmap_delete(hashmap_t *map, void *key);
