#include "munit.h"
#include "xyz.h"
#include "xyz_ds.h"

/*******************************************************************************
 * DARRAY
 ******************************************************************************/

int test_darray_new_and_destroy(void) {
  darray_t *array = darray_new(sizeof(int), 100);

  MU_ASSERT(array != NULL);
  MU_ASSERT(array->contents != NULL);
  MU_ASSERT(array->end == 0);
  MU_ASSERT(array->element_size == sizeof(int));
  MU_ASSERT(array->max == 100);

  darray_destroy(array);
  return 0;
}

int test_darray_push_pop(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* test push */
  for (int i = 0; i < 1000; i++) {
    int *val = darray_new_element(test_darray);
    *val = i * 333;
    darray_push(test_darray, val);
  }
  MU_ASSERT(test_darray->max == 1300);

  /* test pop */
  for (int i = 999; i >= 0; i--) {
    int *val = darray_pop(test_darray);
    MU_ASSERT(val != NULL);
    MU_ASSERT(*val == i * 333);
    free(val);
  }

  darray_clear_destroy(test_darray);
  return 0;
}

int test_darray_contains(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* set element in array */
  int *val = darray_new_element(test_darray);
  *val = 99;
  darray_set(test_darray, 0, val);

  /* test contains */
  int res = darray_contains(test_darray, val, intcmp2);
  MU_ASSERT(res == 1);

  darray_clear_destroy(test_darray);
  return 0;
}

int test_darray_copy(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* set element in array */
  int *val = darray_new_element(test_darray);
  *val = 99;
  darray_set(test_darray, 0, val);

  /* test copy */
  darray_t *array_copy = darray_copy(test_darray);
  int *val_copy = darray_get(array_copy, 0);
  MU_ASSERT(val != val_copy);
  MU_ASSERT(intcmp2(val, val_copy) == 0);

  darray_clear_destroy(test_darray);
  darray_clear_destroy(array_copy);
  return 0;
}

int test_darray_new_element(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* test new */
  int *val1 = darray_new_element(test_darray);
  int *val2 = darray_new_element(test_darray);

  MU_ASSERT(val1 != NULL);
  MU_ASSERT(val2 != NULL);

  free(val1);
  free(val2);

  darray_clear_destroy(test_darray);
  return 0;
}

int test_darray_set_and_get(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* test set element */
  int *val1 = darray_new_element(test_darray);
  int *val2 = darray_new_element(test_darray);
  darray_set(test_darray, 0, val1);
  darray_set(test_darray, 1, val2);

  /* test get element */
  MU_ASSERT(darray_get(test_darray, 0) == val1);
  MU_ASSERT(darray_get(test_darray, 1) == val2);

  darray_clear_destroy(test_darray);
  return 0;
}

int test_darray_update(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* set element */
  int *new_val1 = darray_new_element(test_darray);
  int *new_val2 = darray_new_element(test_darray);
  *new_val1 = 123;
  *new_val2 = 987;

  /* update */
  darray_update(test_darray, 0, new_val1);
  darray_update(test_darray, 1, new_val2);

  /* assert */
  MU_ASSERT(darray_get(test_darray, 0) == new_val1);
  MU_ASSERT(darray_get(test_darray, 1) == new_val2);

  darray_clear_destroy(test_darray);
  return 0;
}

int test_darray_remove(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* set elements */
  int *val_1 = darray_new_element(test_darray);
  int *val_2 = darray_new_element(test_darray);
  *val_1 = 123;
  *val_2 = 987;
  darray_set(test_darray, 0, val_1);
  darray_set(test_darray, 1, val_2);

  /* remove element at index = 0 */
  int *result = darray_remove(test_darray, 0);
  MU_ASSERT(result != NULL);
  MU_ASSERT(*result == *val_1);
  MU_ASSERT(darray_get(test_darray, 0) == NULL);
  free(result);

  /* remove element at index = 1 */
  result = darray_remove(test_darray, 1);
  MU_ASSERT(result != NULL);
  MU_ASSERT(*result == *val_2);
  MU_ASSERT(darray_get(test_darray, 1) == NULL);
  free(result);

  darray_clear_destroy(test_darray);
  return 0;
}

int test_darray_expand_and_contract(void) {
  darray_t *test_darray = darray_new(sizeof(int), 100);

  /* test expand */
  size_t old_max = (unsigned int) test_darray->max;
  darray_expand(test_darray);
  MU_ASSERT((unsigned int) test_darray->max ==
            old_max + test_darray->expand_rate);

  /* test contract */
  darray_contract(test_darray);
  MU_ASSERT((unsigned int) test_darray->max == test_darray->expand_rate + 1);

  darray_clear_destroy(test_darray);
  return 0;
}

/*******************************************************************************
 * LIST
 ******************************************************************************/

int test_list_malloc_and_free(void) {
  list_t *list = list_malloc();
  MU_ASSERT(list != NULL);
  list_clear_free(list);
  return 0;
}

int test_list_push_pop(void) {
  /* Setup */
  list_t *list = list_malloc();
  char *t1 = string_malloc("test1 data");
  char *t2 = string_malloc("test2 data");
  char *t3 = string_malloc("test3 data");

  /* Push tests */
  list_push(list, t1);
  MU_ASSERT(strcmp(list->last->value, t1) == 0);

  list_push(list, t2);
  MU_ASSERT(strcmp(list->last->value, t2) == 0);

  list_push(list, t3);
  MU_ASSERT(strcmp(list->last->value, t3) == 0);
  MU_ASSERT(list->length == 3);

  /* Pop tests */
  char *val = list_pop(list);
  MU_ASSERT(val == t3);
  MU_ASSERT(list->first->value == t1);
  MU_ASSERT(list->last->value == t2);
  MU_ASSERT(list->length == 2);
  free(val);

  val = list_pop(list);
  MU_ASSERT(val == t2);
  MU_ASSERT(list->first->value == t1);
  MU_ASSERT(list->last->value == t1);
  MU_ASSERT(list->length == 1);
  free(val);

  val = list_pop(list);
  MU_ASSERT(val == t1);
  MU_ASSERT(list->first == NULL);
  MU_ASSERT(list->last == NULL);
  MU_ASSERT(list->length == 0);
  free(val);

  list_clear_free(list);
  return 0;
}

int test_list_shift(void) {
  /* Setup */
  list_t *list = list_malloc();
  char *t1 = string_malloc("test1 data");
  char *t2 = string_malloc("test2 data");

  /* Push elements */
  list_push(list, t1);
  list_push(list, t2);

  /* Shift */
  char *val = list_shift(list);
  MU_ASSERT(val == t1);
  MU_ASSERT(list->length == 1);
  free(val);

  val = list_shift(list);
  MU_ASSERT(val == t2);
  MU_ASSERT(list->length == 0);
  free(val);

  list_clear_free(list);
  return 0;
}

int test_list_unshift(void) {
  /* Setup */
  list_t *list = list_malloc();
  char *t1 = string_malloc("test1 data");
  char *t2 = string_malloc("test2 data");
  char *t3 = string_malloc("test3 data");

  /* Unshift */
  list_unshift(list, t1);
  MU_ASSERT(strcmp(list->first->value, t1) == 0);
  MU_ASSERT(strcmp(list->first->value, t1) == 0);
  MU_ASSERT(list->length == 1);

  list_unshift(list, t2);
  MU_ASSERT(strcmp(list->first->value, t2) == 0);
  MU_ASSERT(strcmp(list->first->value, t2) == 0);
  MU_ASSERT(list->length == 2);

  list_unshift(list, t3);
  MU_ASSERT(strcmp(list->first->value, t3) == 0);
  MU_ASSERT(strcmp(list->first->value, t3) == 0);
  MU_ASSERT(list->length == 3);
  list_clear_free(list);

  return 0;
}

int test_list_remove(void) {
  /* Push elements */
  list_t *list = list_malloc();
  char *t1 = string_malloc("test1 data");
  char *t2 = string_malloc("test2 data");
  char *t3 = string_malloc("test3 data");
  list_push(list, t1);
  list_push(list, t2);
  list_push(list, t3);

  /* Remove 2nd value */
  void *value = list_remove(list, t2, strcmp2);
  free(value);

  /* Assert */
  MU_ASSERT(list->length == 2);
  MU_ASSERT(strcmp(list->first->next->value, t3) == 0);
  MU_ASSERT(strcmp(list->first->value, t1) == 0);

  /* Remove 2nd value */
  value = list_remove(list, t3, strcmp2);
  free(value);

  /* Assert */
  MU_ASSERT(list->length == 1);
  MU_ASSERT(list->first->next == NULL);
  MU_ASSERT(strcmp(list->first->value, t1) == 0);
  list_clear_free(list);

  return 0;
}

int test_list_remove_destroy(void) {
  /* Setup */
  list_t *list = list_malloc();
  char *t1 = string_malloc("test1 data");
  char *t2 = string_malloc("test2 data");
  char *t3 = string_malloc("test3 data");

  /* Push elements */
  list_push(list, t1);
  list_push(list, t2);
  list_push(list, t3);

  /* Remove 2nd value */
  int result = list_remove_destroy(list, t2, strcmp2, free);

  /* Assert */
  MU_ASSERT(result == 0);
  MU_ASSERT(list->length == 2);
  MU_ASSERT(strcmp(list->first->next->value, t3) == 0);
  MU_ASSERT(strcmp(list->first->value, t1) == 0);

  /* Remove 2nd value */
  result = list_remove_destroy(list, t3, strcmp2, free);

  /* Assert */
  MU_ASSERT(result == 0);
  MU_ASSERT(list->length == 1);
  MU_ASSERT(list->first->next == NULL);
  MU_ASSERT(strcmp(list->first->value, t1) == 0);
  list_clear_free(list);

  return 0;
}

/*******************************************************************************
 * HASHMAP
 ******************************************************************************/

static int int_cmp(const void *x, const void *y) {
  if (*(int *) x == *(int *) y) {
    return 0;
  }
  return (*(int *) x < *(int *) y) ? -1 : 1;
}

static int float_cmp(const void *x, const void *y) {
  if (fabs(*(float *) x - *(float *) y) < 1e-18) {
    return 0;
  }
  return (*(float *) x < *(float *) y) ? -1 : 1;
}

static int string_cmp(const void *x, const void *y) {
  return strcmp((char *) x, (char *) y);
}

int test_hm_malloc_and_free(void) {
  {
    hm_t *hm = hm_malloc(100, hm_int_hash, int_cmp);
    hm_free(hm, NULL, NULL);
  }

  {
    hm_t *hm = hm_malloc(100, hm_float_hash, float_cmp);
    hm_free(hm, NULL, NULL);
  }

  {
    hm_t *hm = hm_malloc(100, hm_string_hash, string_cmp);
    hm_free(hm, NULL, NULL);
  }

  return 0;
}

int test_hm_set_and_get(void) {
  // Test integer-integer hashmap
  {
    const size_t hm_capacity = 10000;
    hm_t *hm = hm_malloc(hm_capacity, hm_int_hash, int_cmp);

    int k0 = 0;
    int v0 = 0;
    hm_set(hm, &k0, &v0);
    MU_ASSERT(hm->length == 1);
    MU_ASSERT(hm->capacity == hm_capacity);

    int k1 = 1;
    int v1 = 1;
    hm_set(hm, &k1, &v1);
    MU_ASSERT(hm->length == 2);
    MU_ASSERT(hm->capacity == hm_capacity);

    int k2 = 2;
    int v2 = 2;
    hm_set(hm, &k2, &v2);
    MU_ASSERT(hm->length == 3);
    MU_ASSERT(hm->capacity == hm_capacity);

    int k3 = 3;
    int v3 = 3;
    hm_set(hm, &k3, &v3);
    MU_ASSERT(hm->length == 4);
    MU_ASSERT(hm->capacity == hm_capacity);

    MU_ASSERT(*(int *) hm_get(hm, &k0) == v0);
    MU_ASSERT(*(int *) hm_get(hm, &k1) == v1);
    MU_ASSERT(*(int *) hm_get(hm, &k2) == v2);
    MU_ASSERT(*(int *) hm_get(hm, &k3) == v3);

    hm_free(hm, NULL, NULL);
    printf(" -- \n");
  }

  // Test float-integer hashmap
  {
    const size_t hm_capacity = 10000;
    hm_t *hm = hm_malloc(hm_capacity, hm_float_hash, float_cmp);

    float k0 = 0;
    int v0 = 0;
    hm_set(hm, &k0, &v0);
    MU_ASSERT(hm->length == 1);
    MU_ASSERT(hm->capacity == hm_capacity);

    float k1 = 1;
    int v1 = 1;
    hm_set(hm, &k1, &v1);
    MU_ASSERT(hm->length == 2);
    MU_ASSERT(hm->capacity == hm_capacity);

    float k2 = 2;
    int v2 = 2;
    hm_set(hm, &k2, &v2);
    MU_ASSERT(hm->length == 3);
    MU_ASSERT(hm->capacity == hm_capacity);

    float k3 = 3;
    int v3 = 3;
    hm_set(hm, &k3, &v3);
    MU_ASSERT(hm->length == 4);
    MU_ASSERT(hm->capacity == hm_capacity);

    MU_ASSERT(*(int *) hm_get(hm, &k0) == v0);
    MU_ASSERT(*(int *) hm_get(hm, &k1) == v1);
    MU_ASSERT(*(int *) hm_get(hm, &k2) == v2);
    MU_ASSERT(*(int *) hm_get(hm, &k3) == v3);

    hm_free(hm, NULL, NULL);
    printf(" -- \n");
  }

  // Test string-integer hashmap
  {
    const size_t hm_capacity = 10000;
    hm_t *hm = hm_malloc(hm_capacity, hm_string_hash, string_cmp);

    char* k0 = "ABC";
    int v0 = 0;
    hm_set(hm, &k0, &v0);
    MU_ASSERT(hm->length == 1);
    MU_ASSERT(hm->capacity == hm_capacity);

    char* k1 = "DEF";
    int v1 = 1;
    hm_set(hm, &k1, &v1);
    MU_ASSERT(hm->length == 2);
    MU_ASSERT(hm->capacity == hm_capacity);

    char* k2 = "GHI";
    int v2 = 2;
    hm_set(hm, &k2, &v2);
    MU_ASSERT(hm->length == 3);
    MU_ASSERT(hm->capacity == hm_capacity);

    char* k3 = "JKL";
    int v3 = 3;
    hm_set(hm, &k3, &v3);
    MU_ASSERT(hm->length == 4);
    MU_ASSERT(hm->capacity == hm_capacity);

    MU_ASSERT(*(int *) hm_get(hm, &k0) == v0);
    MU_ASSERT(*(int *) hm_get(hm, &k1) == v1);
    MU_ASSERT(*(int *) hm_get(hm, &k2) == v2);
    MU_ASSERT(*(int *) hm_get(hm, &k3) == v3);

    hm_free(hm, NULL, NULL);
  }


  return 0;
}

/*******************************************************************************
 * TEST SUITE
 ******************************************************************************/

void test_suite(void) {
  MU_ADD_TEST(test_darray_new_and_destroy);
  MU_ADD_TEST(test_darray_push_pop);
  MU_ADD_TEST(test_darray_contains);
  MU_ADD_TEST(test_darray_copy);
  MU_ADD_TEST(test_darray_new_element);
  MU_ADD_TEST(test_darray_set_and_get);
  MU_ADD_TEST(test_darray_update);
  MU_ADD_TEST(test_darray_remove);
  MU_ADD_TEST(test_darray_expand_and_contract);
  MU_ADD_TEST(test_list_malloc_and_free);
  MU_ADD_TEST(test_list_push_pop);
  MU_ADD_TEST(test_list_shift);
  MU_ADD_TEST(test_list_unshift);
  MU_ADD_TEST(test_list_remove);
  MU_ADD_TEST(test_list_remove_destroy);
  MU_ADD_TEST(test_hm_malloc_and_free);
  MU_ADD_TEST(test_hm_set_and_get);
}
MU_RUN_TESTS(test_suite)
