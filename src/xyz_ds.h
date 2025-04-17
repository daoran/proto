#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

#include "xyz.h"

/*******************************************************************************
 * DARRAY
 ******************************************************************************/

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

/*******************************************************************************
 * LIST
 ******************************************************************************/

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

/*******************************************************************************
 * HASHMAP
 ******************************************************************************/

typedef struct hm_entry_t {
  void *key;
  void *value;
  size_t key_size;
} hm_entry_t;

typedef struct hm_t {
  hm_entry_t *entries;
  size_t length;
  size_t capacity;
  size_t (*hash)(const void *);
  int (*cmp)(const void *, const void *);
} hm_t;

typedef struct hm_iter_t {
  void *key;
  void *value;
  hm_t *_hm;
  size_t _index;
} hm_iter_t;

size_t hm_default_hash(const void *data, const size_t size);
size_t hm_int_hash(const void *data);
size_t hm_float_hash(const void *data);
size_t hm_double_hash(const void *data);
size_t hm_string_hash(const void *data);

hm_t *hm_malloc(const size_t capacity,
                size_t (*hash)(const void *),
                int (*cmp)(const void *, const void *));
void hm_free(hm_t *hm, void (*free_key)(void *), void (*free_value)(void *));
void *hm_get(const hm_t *hm, const void *key);
int hm_expand(hm_t *hm);
int hm_set(hm_t *hm, void *key, void *value);
hm_iter_t hm_iterator(hm_t *hm);
int hm_next(hm_iter_t *it);
