#include "xyz.h"
#include "munit.h"

/* TEST PARAMS */
#define TEST_DATA_PATH "./test_data/"
#define TEST_SIM_DATA TEST_DATA_PATH "sim_data"
#define TEST_SIM_GIMBAL TEST_DATA_PATH "sim_gimbal"

#define KITTI_TEST_DATA "/data/kitti_raw/2011_09_26"
#define KITTI_TEST_SEQ_NAME "2011_09_26_drive_0001_sync"
#define KITTI_TEST_SEQ KITTI_TEST_DATA "/" KITTI_TEST_SEQ_NAME


/******************************************************************************
 * MACROS
 *****************************************************************************/

int test_median_value(void) {
  real_t median = 0.0f;
  real_t buf[5] = {4.0, 1.0, 0.0, 3.0, 2.0};
  MEDIAN_VALUE(real_t, fltcmp2, buf, 5, median);
  MU_ASSERT(fltcmp(median, 2.0) == 0);

  return 0;
}

int test_mean_value(void) {
  real_t mean = 0.0f;
  real_t buf[5] = {0.0, 1.0, 2.0, 3.0, 4.0};
  MEAN_VALUE(real_t, buf, 5, mean);
  MU_ASSERT(fltcmp(mean, 2.0) == 0);

  return 0;
}

/*******************************************************************************
 * TIME
 ******************************************************************************/

int test_tic_toc(void) {
  struct timespec t_start = tic();
  usleep(1);
  MU_ASSERT(fabs(toc(&t_start) - 1e-3) < 1e-2);
  return 0;
}

int test_mtoc(void) {
  struct timespec t_start = tic();
  usleep(1);
  MU_ASSERT(fabs(mtoc(&t_start) - 1e-3) < 1);
  return 0;
}

int test_time_now(void) {
  timestamp_t t_now = time_now();
  // printf("t_now: %ld\n", t_now);
  MU_ASSERT(t_now > 0);
  return 0;
}

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
 * RED-BLACK-TREE
 ******************************************************************************/

int test_rbt_node_malloc_and_free(void) {
  rbt_node_t *node = rbt_node_malloc(RB_RED, NULL, NULL);
  MU_ASSERT(node->key == NULL);
  MU_ASSERT(node->value == NULL);
  MU_ASSERT(node->color == RB_RED);
  MU_ASSERT(node->child[0] == NULL);
  MU_ASSERT(node->child[1] == NULL);
  MU_ASSERT(node->size == 1);
  rbt_node_free(node);
  return 0;
}

int test_rbt_node_min_max(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  rbt_node_t *node_0 = rbt_node_malloc(RB_BLACK, &key_0, NULL);
  rbt_node_t *node_1 = rbt_node_malloc(RB_BLACK, &key_1, NULL);
  rbt_node_t *node_2 = rbt_node_malloc(RB_BLACK, &key_2, NULL);
  rbt_node_t *node_3 = rbt_node_malloc(RB_BLACK, &key_3, NULL);
  rbt_node_t *node_4 = rbt_node_malloc(RB_BLACK, &key_4, NULL);

  // Form BST
  //   1
  //  / \
  // 0   3
  //    / \
  //   2   4
  node_1->child[0] = node_0;
  node_1->child[1] = node_3;
  node_3->child[0] = node_2;
  node_3->child[1] = node_4;

  MU_ASSERT(rbt_node_min(node_1) == node_0);
  MU_ASSERT(rbt_node_max(node_1) == node_4);
  rbt_node_free(node_1);

  return 0;
}

int test_rbt_node_height_size(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  rbt_node_t *root = rbt_node_malloc(RB_BLACK, &key_0, NULL);
  root = rbt_node_insert(root, &key_1, NULL, int_cmp);
  root = rbt_node_insert(root, &key_2, NULL, int_cmp);
  root = rbt_node_insert(root, &key_3, NULL, int_cmp);
  root = rbt_node_insert(root, &key_4, NULL, int_cmp);

  //       3
  //      / \
  //     1   4
  //    / \
  //   0   2
  MU_ASSERT(rbt_node_height(root) == 2);
  MU_ASSERT(rbt_node_size(root) == 5);
  rbt_node_free(root);

  return 0;
}

int test_rbt_node_keys(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  rbt_node_t *root = rbt_node_malloc(RB_BLACK, &key_0, NULL);
  root = rbt_node_insert(root, &key_1, NULL, int_cmp);
  root = rbt_node_insert(root, &key_2, NULL, int_cmp);
  root = rbt_node_insert(root, &key_3, NULL, int_cmp);
  root = rbt_node_insert(root, &key_4, NULL, int_cmp);

  //       3
  //      / \
  //     1   4
  //    / \
  //   0   2
  arr_t *keys = arr_malloc(10);
  rbt_node_keys(root, &key_0, &key_4, keys, int_cmp);
  for (int i = 0; i < keys->size; ++i) {
    MU_ASSERT(*(int *) keys->data[i] == i);
  }

  // Clean up
  rbt_node_free(root);
  arr_free(keys);

  return 0;
}

int test_rbt_node_flip_colors(void) {
  // Setup
  char *key_a = "a";
  char *key_b = "b";
  char *key_c = "c";
  rbt_node_t *node_a = rbt_node_malloc(RB_RED, key_a, NULL);
  rbt_node_t *node_b = rbt_node_malloc(RB_BLACK, key_b, NULL);
  rbt_node_t *node_c = rbt_node_malloc(RB_BLACK, key_c, NULL);
  node_a->child[0] = node_b;
  node_a->child[1] = node_c;

  // Flip colors and assert
  rbt_node_flip_colors(node_a);
  MU_ASSERT(node_a->color == RB_BLACK);
  MU_ASSERT(node_b->color == RB_RED);
  MU_ASSERT(node_c->color == RB_RED);

  // Clean up
  rbt_node_free(node_a);

  return 0;
}

int test_rbt_node_rotate(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  rbt_node_t *node_0 = rbt_node_malloc(RB_BLACK, &key_0, NULL);
  rbt_node_t *node_1 = rbt_node_malloc(RB_BLACK, &key_1, NULL);
  rbt_node_t *node_2 = rbt_node_malloc(RB_BLACK, &key_2, NULL);
  rbt_node_t *node_3 = rbt_node_malloc(RB_BLACK, &key_3, NULL);
  rbt_node_t *node_4 = rbt_node_malloc(RB_BLACK, &key_4, NULL);

  // Form BST
  //   1
  //  / \
  // 0   3
  //    / \
  //   2   4
  node_1->child[0] = node_0;
  node_1->child[1] = node_3;
  node_3->child[0] = node_2;
  node_3->child[1] = node_4;

  // Test rotate left
  //   1               3
  //  / \             / \
  // 0   3    -->    1   4
  //    / \         / \
  //   2   4       0   2
  rbt_node_rotate(node_1, 0);
  MU_ASSERT(node_3->child[0] == node_1);
  MU_ASSERT(node_3->child[1] == node_4);
  MU_ASSERT(node_1->child[0] == node_0);
  MU_ASSERT(node_1->child[1] == node_2);

  // Test rotate right
  //     3             1
  //    / \           / \
  //   1   4  -->    0   3
  //  / \               / \
  // 0   2             2   4
  rbt_node_rotate(node_3, 1);
  MU_ASSERT(node_1->child[0] == node_0);
  MU_ASSERT(node_1->child[1] == node_3);
  MU_ASSERT(node_3->child[0] == node_2);
  MU_ASSERT(node_3->child[1] == node_4);

  // Clean up
  rbt_node_free(node_1);

  return 0;
}

int test_rbt_node_move_red_left(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  rbt_node_t *node_0 = rbt_node_malloc(RB_RED, &key_0, NULL);
  rbt_node_t *node_1 = rbt_node_malloc(RB_RED, &key_1, NULL);
  rbt_node_t *node_2 = rbt_node_malloc(RB_BLACK, &key_2, NULL);

  // Form BST
  //    0
  //     \
  //      2
  //     /
  //    1
  node_0->child[0] = NULL;
  node_0->child[1] = node_2;
  node_1->child[0] = NULL;
  node_1->child[1] = NULL;
  node_2->child[0] = node_1;
  node_2->child[1] = NULL;

  // Test move red left
  //   0            1
  //    \          / \
  //     2  -->   0   2
  //    /
  //   1
  rbt_node_t *root = rbt_node_move_red_left(node_0);
  MU_ASSERT(root->child[0] == node_0);
  MU_ASSERT(root->child[1] == node_2);
  MU_ASSERT(node_0->child[0] == NULL);
  MU_ASSERT(node_0->child[1] == NULL);
  MU_ASSERT(node_2->child[0] == NULL);
  MU_ASSERT(node_2->child[1] == NULL);

  // Clean up
  rbt_node_free(node_1);

  return 0;
}

int test_rbt_node_move_red_right(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  rbt_node_t *node_0 = rbt_node_malloc(RB_RED, &key_0, NULL);
  rbt_node_t *node_1 = rbt_node_malloc(RB_RED, &key_1, NULL);
  rbt_node_t *node_2 = rbt_node_malloc(RB_BLACK, &key_2, NULL);

  // Form BST
  //      2
  //     /
  //    1
  //   /
  //  0
  node_0->child[0] = NULL;
  node_0->child[1] = NULL;
  node_1->child[0] = node_0;
  node_1->child[1] = NULL;
  node_2->child[0] = node_1;
  node_2->child[1] = NULL;

  // Test move red left
  //      2
  //     /
  //    1    -->    1
  //   /           / \
  //  0           0   2
  rbt_node_t *root = rbt_node_move_red_right(node_2);
  MU_ASSERT(root->child[0] == node_0);
  MU_ASSERT(root->child[1] == node_2);
  MU_ASSERT(node_0->child[0] == NULL);
  MU_ASSERT(node_0->child[1] == NULL);
  MU_ASSERT(node_2->child[0] == NULL);
  MU_ASSERT(node_2->child[1] == NULL);

  // Clean up
  rbt_node_free(node_1);

  return 0;
}

int test_rbt_node_insert(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  rbt_node_t *root = rbt_node_malloc(RB_BLACK, &key_0, NULL);
  root = rbt_node_insert(root, &key_1, NULL, int_cmp);
  root = rbt_node_insert(root, &key_2, NULL, int_cmp);
  root = rbt_node_insert(root, &key_3, NULL, int_cmp);
  root = rbt_node_insert(root, &key_4, NULL, int_cmp);

  //       3
  //      / \
  //     1   4
  //    / \
  //   0   2
  MU_ASSERT(root->key == &key_3);
  MU_ASSERT(root->child[0]->key == &key_1);
  MU_ASSERT(root->child[1]->key == &key_4);
  MU_ASSERT(root->child[0]->child[0]->key == &key_0);
  MU_ASSERT(root->child[0]->child[1]->key == &key_2);
  MU_ASSERT(root->child[1]->child[0] == NULL);
  MU_ASSERT(root->child[1]->child[1] == NULL);

  // Clean up
  rbt_node_free(root);

  return 0;
}

int test_rbt_node_delete(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  rbt_node_t *root = rbt_node_malloc(RB_BLACK, &key_0, NULL);
  root = rbt_node_insert(root, &key_1, NULL, int_cmp);
  root = rbt_node_insert(root, &key_2, NULL, int_cmp);
  root = rbt_node_insert(root, &key_3, NULL, int_cmp);
  root = rbt_node_insert(root, &key_4, NULL, int_cmp);

  // Test deletes
  root = rbt_node_delete(root, &key_0, int_cmp);
  MU_ASSERT(rbt_node_size(root) == 4);
  MU_ASSERT(rbt_node_check(root, int_cmp) == 1);

  root = rbt_node_delete(root, &key_1, int_cmp);
  MU_ASSERT(rbt_node_size(root) == 3);
  MU_ASSERT(rbt_node_check(root, int_cmp) == 1);

  root = rbt_node_delete(root, &key_2, int_cmp);
  MU_ASSERT(rbt_node_size(root) == 2);
  MU_ASSERT(rbt_node_check(root, int_cmp) == 1);

  root = rbt_node_delete(root, &key_3, int_cmp);
  MU_ASSERT(rbt_node_size(root) == 1);
  MU_ASSERT(rbt_node_check(root, int_cmp) == 1);

  root = rbt_node_delete(root, &key_4, int_cmp);
  MU_ASSERT(rbt_node_size(root) == 0);
  MU_ASSERT(rbt_node_check(root, int_cmp) == 1);

  // Clean up
  rbt_node_free(root);

  return 0;
}

int test_rbt_malloc_and_free(void) {
  rbt_t *rbt = rbt_malloc(int_cmp);
  rbt_free(rbt);
  return 0;
}

int test_rbt_insert(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  rbt_t *rbt = rbt_malloc(int_cmp);
  rbt_insert(rbt, &key_0, NULL);
  rbt_insert(rbt, &key_1, NULL);
  rbt_insert(rbt, &key_2, NULL);
  rbt_insert(rbt, &key_3, NULL);
  rbt_insert(rbt, &key_4, NULL);

  //       3
  //      / \
  //     1   4
  //    / \
  //   0   2
  MU_ASSERT(rbt->root->key == &key_3);
  MU_ASSERT(rbt->root->child[0]->key == &key_1);
  MU_ASSERT(rbt->root->child[1]->key == &key_4);
  MU_ASSERT(rbt->root->child[0]->child[0]->key == &key_0);
  MU_ASSERT(rbt->root->child[0]->child[1]->key == &key_2);
  MU_ASSERT(rbt->root->child[1]->child[0] == NULL);
  MU_ASSERT(rbt->root->child[1]->child[1] == NULL);

  // Clean up
  rbt_free(rbt);

  return 0;
}

int test_rbt_delete(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  rbt_t *rbt = rbt_malloc(int_cmp);
  rbt_insert(rbt, &key_0, NULL);
  rbt_insert(rbt, &key_1, NULL);
  rbt_insert(rbt, &key_2, NULL);
  rbt_insert(rbt, &key_3, NULL);
  rbt_insert(rbt, &key_4, NULL);

  rbt_delete(rbt, &key_0);
  rbt_delete(rbt, &key_1);
  rbt_delete(rbt, &key_2);
  rbt_delete(rbt, &key_3);
  rbt_delete(rbt, &key_4);

  // Clean up
  rbt_free(rbt);

  return 0;
}

int test_rbt_search(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  int val_0 = 0;
  int val_1 = 1;
  int val_2 = 2;
  int val_3 = 3;
  int val_4 = 4;
  rbt_t *rbt = rbt_malloc(int_cmp);
  rbt_insert(rbt, &key_0, &val_0);
  rbt_insert(rbt, &key_1, &val_1);
  rbt_insert(rbt, &key_2, &val_2);
  rbt_insert(rbt, &key_3, &val_3);
  rbt_insert(rbt, &key_4, &val_4);

  // Search
  MU_ASSERT(rbt_search(rbt, &key_0) == &val_0);
  MU_ASSERT(rbt_search(rbt, &key_1) == &val_1);
  MU_ASSERT(rbt_search(rbt, &key_2) == &val_2);
  MU_ASSERT(rbt_search(rbt, &key_3) == &val_3);
  MU_ASSERT(rbt_search(rbt, &key_4) == &val_4);

  // Clean up
  rbt_free(rbt);

  return 0;
}

int test_rbt_contains(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  int val_0 = 0;
  int val_1 = 1;
  int val_2 = 2;
  int val_3 = 3;
  int val_4 = 4;
  rbt_t *rbt = rbt_malloc(int_cmp);
  rbt_insert(rbt, &key_0, &val_0);
  rbt_insert(rbt, &key_1, &val_1);
  rbt_insert(rbt, &key_2, &val_2);
  rbt_insert(rbt, &key_3, &val_3);
  rbt_insert(rbt, &key_4, &val_4);

  // Search
  MU_ASSERT(rbt_contains(rbt, &key_0) == true);
  MU_ASSERT(rbt_contains(rbt, &key_1) == true);
  MU_ASSERT(rbt_contains(rbt, &key_2) == true);
  MU_ASSERT(rbt_contains(rbt, &key_3) == true);
  MU_ASSERT(rbt_contains(rbt, &key_4) == true);

  // Clean up
  rbt_free(rbt);

  return 0;
}

int test_rbt_min_max(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  rbt_t *rbt = rbt_malloc(int_cmp);
  rbt_insert(rbt, &key_0, NULL);
  rbt_insert(rbt, &key_1, NULL);
  rbt_insert(rbt, &key_2, NULL);
  rbt_insert(rbt, &key_3, NULL);
  rbt_insert(rbt, &key_4, NULL);

  // Min and max
  MU_ASSERT(rbt_min(rbt)->key == &key_0);
  MU_ASSERT(rbt_max(rbt)->key == &key_4);

  // Clean up
  rbt_free(rbt);

  return 0;
}

int test_rbt_keys(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  rbt_t *rbt = rbt_malloc(int_cmp);
  rbt_insert(rbt, &key_0, NULL);
  rbt_insert(rbt, &key_1, NULL);
  rbt_insert(rbt, &key_2, NULL);
  rbt_insert(rbt, &key_3, NULL);
  rbt_insert(rbt, &key_4, NULL);

  arr_t *keys = arr_malloc(10);
  rbt_keys(rbt, keys);
  MU_ASSERT(keys->size == 5);
  MU_ASSERT(keys->data[0] == &key_0);
  MU_ASSERT(keys->data[1] == &key_1);
  MU_ASSERT(keys->data[2] == &key_2);
  MU_ASSERT(keys->data[3] == &key_3);
  MU_ASSERT(keys->data[4] == &key_4);

  // Clean up
  rbt_free(rbt);
  arr_free(keys);

  return 0;
}

int test_rbt_rank(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  rbt_t *rbt = rbt_malloc(int_cmp);
  rbt_insert(rbt, &key_0, NULL);
  rbt_insert(rbt, &key_1, NULL);
  rbt_insert(rbt, &key_2, NULL);
  rbt_insert(rbt, &key_3, NULL);
  rbt_insert(rbt, &key_4, NULL);

  // Rank
  MU_ASSERT(rbt_rank(rbt, &key_0) == 0);
  MU_ASSERT(rbt_rank(rbt, &key_1) == 1);
  MU_ASSERT(rbt_rank(rbt, &key_2) == 2);
  MU_ASSERT(rbt_rank(rbt, &key_3) == 3);
  MU_ASSERT(rbt_rank(rbt, &key_4) == 4);

  // Clean up
  rbt_free(rbt);

  return 0;
}

int test_rbt_select(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  rbt_t *rbt = rbt_malloc(int_cmp);
  rbt_insert(rbt, &key_0, NULL);
  rbt_insert(rbt, &key_1, NULL);
  rbt_insert(rbt, &key_2, NULL);
  rbt_insert(rbt, &key_3, NULL);
  rbt_insert(rbt, &key_4, NULL);

  // Rank
  MU_ASSERT(rbt_select(rbt, 0) == &key_0);
  MU_ASSERT(rbt_select(rbt, 1) == &key_1);
  MU_ASSERT(rbt_select(rbt, 2) == &key_2);
  MU_ASSERT(rbt_select(rbt, 3) == &key_3);
  MU_ASSERT(rbt_select(rbt, 4) == &key_4);

  // Clean up
  rbt_free(rbt);

  return 0;
}

int test_rbt_sandbox(void) {
  // Setup
  int key_0 = 0;
  int key_1 = 1;
  int key_2 = 2;
  int key_3 = 3;
  int key_4 = 4;
  int val_0 = 0;
  int val_1 = 1;
  int val_2 = 2;
  int val_3 = 3;
  int val_4 = 4;
  rbt_t *rbt = rbt_malloc(default_cmp);
  rbt_insert(rbt, &key_0, &val_0);
  rbt_insert(rbt, &key_1, &val_1);
  rbt_insert(rbt, &key_2, &val_2);
  rbt_insert(rbt, &key_3, &val_3);
  rbt_insert(rbt, &key_4, &val_4);

  const size_t n = rbt_size(rbt);
  arr_t *keys = arr_malloc(n);
  arr_t *vals = arr_malloc(n);
  rbt_keys_values(rbt, keys, vals);

  // Clean up
  rbt_free(rbt);
  arr_free(keys);
  arr_free(vals);

  return 0;
}

/*******************************************************************************
 * HASHMAP
 ******************************************************************************/

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
  }

  // Test string-integer hashmap
  {
    const size_t hm_capacity = 10000;
    hm_t *hm = hm_malloc(hm_capacity, hm_string_hash, string_cmp);

    char k0[10] = "ABC";
    int v0 = 0;
    hm_set(hm, &k0, &v0);
    MU_ASSERT(hm->length == 1);
    MU_ASSERT(hm->capacity == hm_capacity);

    char k1[10] = "DEF";
    int v1 = 1;
    hm_set(hm, &k1, &v1);
    MU_ASSERT(hm->length == 2);
    MU_ASSERT(hm->capacity == hm_capacity);

    char k2[10] = "GHI";
    int v2 = 2;
    hm_set(hm, &k2, &v2);
    MU_ASSERT(hm->length == 3);
    MU_ASSERT(hm->capacity == hm_capacity);

    char k3[10] = "JKL";
    int v3 = 3;
    hm_set(hm, k3, &v3);
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

/******************************************************************************
 * NETWORK
 ******************************************************************************/

int test_tcp_server_setup(void) {
  tcp_server_t server;
  const int port = 8080;
  int retval = tcp_server_setup(&server, port);
  MU_ASSERT(retval == 0);
  return 0;
}

/******************************************************************************
 * MATH
 ******************************************************************************/

int test_min(void) {
  MU_ASSERT(MIN(1, 2) == 1);
  MU_ASSERT(MIN(2, 1) == 1);
  return 0;
}

int test_max(void) {
  MU_ASSERT(MAX(1, 2) == 2);
  MU_ASSERT(MAX(2, 1) == 2);
  return 0;
}

int test_randf(void) {
  const real_t val = randf(0.0, 10.0);
  MU_ASSERT(val < 10.0);
  MU_ASSERT(val > 0.0);
  return 0;
}

int test_deg2rad(void) {
  MU_ASSERT(fltcmp(deg2rad(180.0f), M_PI) == 0);
  return 0;
}

int test_rad2deg(void) {
  MU_ASSERT(fltcmp(rad2deg(M_PI), 180.0f) == 0);
  return 0;
}

int test_wrap_180(void) {
  MU_ASSERT(fltcmp(wrap_180(181), -179) == 0);
  MU_ASSERT(fltcmp(wrap_180(90), 90) == 0);
  MU_ASSERT(fltcmp(wrap_180(-181), 179) == 0);
  return 0;
}

int test_wrap_360(void) {
  MU_ASSERT(fltcmp(wrap_360(-1), 359) == 0);
  MU_ASSERT(fltcmp(wrap_360(180), 180) == 0);
  MU_ASSERT(fltcmp(wrap_360(361), 1) == 0);
  return 0;
}

int test_wrap_pi(void) {
  MU_ASSERT(fltcmp(wrap_pi(deg2rad(181)), deg2rad(-179)) == 0);
  MU_ASSERT(fltcmp(wrap_pi(deg2rad(90)), deg2rad(90)) == 0);
  MU_ASSERT(fltcmp(wrap_pi(deg2rad(-181)), deg2rad(179)) == 0);
  return 0;
}

int test_wrap_2pi(void) {
  MU_ASSERT(fltcmp(wrap_2pi(deg2rad(-1)), deg2rad(359)) == 0);
  MU_ASSERT(fltcmp(wrap_2pi(deg2rad(180)), deg2rad(180)) == 0);
  MU_ASSERT(fltcmp(wrap_2pi(deg2rad(361)), deg2rad(1)) == 0);
  return 0;
}

int test_fltcmp(void) {
  MU_ASSERT(fltcmp(1.0, 1.0) == 0);
  MU_ASSERT(fltcmp(1.0, 1.01) != 0);
  return 0;
}

int test_fltcmp2(void) {
  const real_t x = 1.0f;
  const real_t y = 1.0f;
  const real_t z = 1.01f;
  MU_ASSERT(fltcmp2(&x, &y) == 0);
  MU_ASSERT(fltcmp2(&x, &z) != 0);
  return 0;
}

int test_cumsum(void) {
  real_t x[10] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
  real_t s[10] = {0};
  cumsum(x, 10, s);

  MU_ASSERT(flteqs(s[0], 1.0));
  MU_ASSERT(flteqs(s[1], 3.0));
  MU_ASSERT(flteqs(s[2], 6.0));
  MU_ASSERT(flteqs(s[3], 10.0));
  MU_ASSERT(flteqs(s[4], 15.0));
  MU_ASSERT(flteqs(s[5], 21.0));
  MU_ASSERT(flteqs(s[6], 28.0));
  MU_ASSERT(flteqs(s[7], 36.0));
  MU_ASSERT(flteqs(s[8], 45.0));
  MU_ASSERT(flteqs(s[9], 55.0));

  return 0;
}

int test_logspace(void) {
  real_t x[10] = {0};
  logspace(1.0, 2.0, 10, x);

  MU_ASSERT(flteqs(x[0], 10.000000));
  MU_ASSERT(flteqs(x[1], 12.915497));
  MU_ASSERT(flteqs(x[2], 16.681005));
  MU_ASSERT(flteqs(x[3], 21.544347));
  MU_ASSERT(flteqs(x[4], 27.825594));
  MU_ASSERT(flteqs(x[5], 35.938137));
  MU_ASSERT(flteqs(x[6], 46.415888));
  MU_ASSERT(flteqs(x[7], 59.948425));
  MU_ASSERT(flteqs(x[8], 77.426368));
  MU_ASSERT(flteqs(x[9], 100.00000));

  return 0;
}

int test_pythag(void) {
  MU_ASSERT(fltcmp(pythag(3.0, 4.0), 5.0) == 0);
  return 0;
}

int test_lerp(void) {
  MU_ASSERT(fltcmp(lerp(0.0, 1.0, 0.5), 0.5) == 0);
  MU_ASSERT(fltcmp(lerp(0.0, 10.0, 0.8), 8.0) == 0);
  return 0;
}

int test_lerp3(void) {
  real_t a[3] = {0.0, 1.0, 2.0};
  real_t b[3] = {1.0, 2.0, 3.0};
  real_t c[3] = {0.0, 0.0, 0.0};
  real_t t = 0.5;

  lerp3(a, b, t, c);
  MU_ASSERT(fltcmp(c[0], 0.5) == 0);
  MU_ASSERT(fltcmp(c[1], 1.5) == 0);
  MU_ASSERT(fltcmp(c[2], 2.5) == 0);

  return 0;
}

int test_sinc(void) { return 0; }

int test_mean(void) {
  real_t vals[4] = {1.0, 2.0, 3.0, 4.0};
  MU_ASSERT(fltcmp(mean(vals, 4), 2.5) == 0);

  return 0;
}

int test_median(void) {
  {
    const real_t vals[5] = {2.0, 3.0, 1.0, 4.0, 5.0};
    const real_t retval = median(vals, 5);
    MU_ASSERT(fltcmp(retval, 3.0) == 0);
  }

  {
    const real_t vals2[6] = {2.0, 3.0, 1.0, 4.0, 5.0, 6.0};
    const real_t retval = median(vals2, 6);
    MU_ASSERT(fltcmp(retval, 3.5) == 0);
  }

  return 0;
}

int test_var(void) {
  real_t vals[4] = {1.0, 2.0, 3.0, 4.0};
  MU_ASSERT(fltcmp(var(vals, 4), 1.666666667) == 0);

  return 0;
}

int test_stddev(void) {
  real_t vals[4] = {1.0, 2.0, 3.0, 4.0};
  MU_ASSERT(fltcmp(stddev(vals, 4), sqrt(1.666666667)) == 0);

  return 0;
}

/******************************************************************************
 * LINEAR ALGEBRA
 ******************************************************************************/

int test_eye(void) {
  real_t A[25] = {0.0};
  eye(A, 5, 5);

  /* print_matrix("I", A, 5, 5); */
  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      real_t expected = (i == j) ? 1.0 : 0.0;
      MU_ASSERT(fltcmp(A[idx], expected) == 0);
      idx++;
    }
  }

  return 0;
}

int test_ones(void) {
  real_t A[25] = {0.0};
  ones(A, 5, 5);

  /* print_matrix("A", A, 5, 5); */
  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      MU_ASSERT((fabs(A[idx] - 1.0) < 1e-5));
      idx++;
    }
  }

  return 0;
}

int test_zeros(void) {
  real_t A[25] = {0.0};
  zeros(A, 5, 5);

  /* print_matrix("A", A, 5, 5); */
  size_t idx = 0;
  size_t rows = 5;
  size_t cols = 5;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      MU_ASSERT((fabs(A[idx] - 0.0) < 1e-5));
      idx++;
    }
  }

  return 0;
}

int test_mat_set(void) {
  real_t A[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  mat_set(A, 3, 0, 0, 1.0);
  mat_set(A, 3, 1, 1, 1.0);
  mat_set(A, 3, 2, 2, 1.0);

  /* print_matrix("A", A, 3, 3); */
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 1, 1), 1.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 2, 2), 1.0) == 0);

  return 0;
}

int test_mat_val(void) {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

  /* print_matrix("A", A, 3, 3); */
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 1), 2.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 2), 3.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 1, 0), 4.0) == 0);

  return 0;
}

int test_mat_copy(void) {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {0};

  mat_copy(A, 3, 3, B);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(B[i], i + 1.0) == 0);
  }

  return 0;
}

int test_mat_row_set(void) {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[3] = {0.0, 0.0, 0.0};

  /* Set first row zeros */
  mat_row_set(A, 3, 0, B);
  for (int i = 0; i < 3; i++) {
    MU_ASSERT(fltcmp(A[i], 0.0) == 0);
  }

  /* Set second row zeros */
  mat_row_set(A, 3, 1, B);
  for (int i = 0; i < 6; i++) {
    MU_ASSERT(fltcmp(A[i], 0.0) == 0);
  }

  /* Set third row zeros */
  mat_row_set(A, 3, 1, B);
  for (int i = 0; i < 6; i++) {
    MU_ASSERT(fltcmp(A[i], 0.0) == 0);
  }

  return 0;
}

int test_mat_col_set(void) {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[3] = {0.0, 0.0, 0.0};

  /* Set first column zeros */
  mat_col_set(A, 3, 3, 0, B);
  for (int i = 0; i < 3; i++) {
    MU_ASSERT(fltcmp(A[i * 3], 0.0) == 0);
  }

  /* Set second column zeros */
  mat_col_set(A, 3, 3, 1, B);
  for (int i = 0; i < 3; i++) {
    MU_ASSERT(fltcmp(A[(i * 3) + 1], 0.0) == 0);
  }

  /* Set third column zeros */
  mat_col_set(A, 3, 3, 2, B);
  for (int i = 0; i < 3; i++) {
    MU_ASSERT(fltcmp(A[(i * 3) + 2], 0.0) == 0);
  }

  /* Check whether full matrix is zeros */
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(A[i], 0.0) == 0);
  }

  return 0;
}

int test_mat_block_get(void) {
  // clang-format off
  real_t A[9] = {0.0, 1.0, 2.0,
                 3.0, 4.0, 5.0,
                 6.0, 7.0, 8.0};
  real_t B[4] = {0.0};
  real_t C[4] = {0.0};
  // clang-format on
  mat_block_get(A, 3, 1, 2, 1, 2, B);
  mat_block_get(A, 3, 0, 1, 1, 2, C);

  // print_matrix("A", A, 3, 3);
  // print_matrix("B", B, 2, 2);
  // print_matrix("C", C, 2, 2);

  MU_ASSERT(fltcmp(mat_val(B, 2, 0, 0), 4.0) == 0);
  MU_ASSERT(fltcmp(mat_val(B, 2, 0, 1), 5.0) == 0);
  MU_ASSERT(fltcmp(mat_val(B, 2, 1, 0), 7.0) == 0);
  MU_ASSERT(fltcmp(mat_val(B, 2, 1, 1), 8.0) == 0);

  MU_ASSERT(fltcmp(mat_val(C, 2, 0, 0), 1.0) == 0);
  MU_ASSERT(fltcmp(mat_val(C, 2, 0, 1), 2.0) == 0);
  MU_ASSERT(fltcmp(mat_val(C, 2, 1, 0), 4.0) == 0);
  MU_ASSERT(fltcmp(mat_val(C, 2, 1, 1), 5.0) == 0);

  return 0;
}

int test_mat_block_set(void) {
  // clang-format off
  real_t A[4 * 4] = {0.0, 1.0, 2.0, 3.0,
                     4.0, 5.0, 6.0, 7.0,
                     8.0, 9.0, 10.0, 11.0,
                     12.0, 13.0, 14.0, 15.0};
  real_t B[2 * 2] = {0.0, 0.0,
                     0.0, 0.0};
  // clang-format on

  // print_matrix("A", A, 3, 3);
  // print_matrix("B", B, 2, 2);
  mat_block_set(A, 4, 1, 2, 1, 2, B);
  // print_matrix("A", A, 4, 4);
  // print_matrix("B", B, 2, 2);

  MU_ASSERT(fltcmp(mat_val(A, 4, 1, 1), 0.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 4, 1, 2), 0.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 4, 2, 1), 0.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 4, 2, 2), 0.0) == 0);

  return 0;
}

int test_mat_diag_get(void) {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t d[3] = {0.0, 0.0, 0.0};
  mat_diag_get(A, 3, 3, d);

  // print_matrix("A", A, 3, 3);
  // print_vector("d", d, 3);
  MU_ASSERT(fltcmp(d[0], 1.0) == 0);
  MU_ASSERT(fltcmp(d[1], 5.0) == 0);
  MU_ASSERT(fltcmp(d[2], 9.0) == 0);

  return 0;
}

int test_mat_diag_set(void) {
  real_t A[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  real_t d[4] = {1.0, 2.0, 3.0};
  mat_diag_set(A, 3, 3, d);

  // print_matrix("A", A, 3, 3);
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 1, 1), 2.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 2, 2), 3.0) == 0);

  return 0;
}

int test_mat_triu(void) {
  // clang-format off
  real_t A[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  real_t U[16] = {0};
  // clang-format on
  mat_triu(A, 4, U);
  // print_matrix("U", U, 4, 4);

  return 0;
}

int test_mat_tril(void) {
  // clang-format off
  real_t A[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  real_t L[16] = {0};
  // clang-format on
  mat_tril(A, 4, L);
  // print_matrix("L", L, 4, 4);

  return 0;
}

int test_mat_trace(void) {
  // clang-format off
  real_t A[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  // clang-format on
  const real_t tr = mat_trace(A, 4, 4);
  MU_ASSERT(fltcmp(tr, 1.0 + 6.0 + 11.0 + 16.0) == 0.0);

  return 0;
}

int test_mat_transpose(void) {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t At[9] = {0.0};
  real_t At_expected[9] = {1.0, 4.0, 7.0, 2.0, 5.0, 8.0, 3.0, 6.0, 9.0};
  mat_transpose(A, 3, 3, At);
  MU_ASSERT(mat_equals(At, At_expected, 3, 3, 1e-8));

  real_t B[2 * 3] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  real_t Bt[3 * 2] = {0};
  real_t Bt_expected[3 * 2] = {1.0, 4.0, 2.0, 5.0, 3.0, 6.0};
  mat_transpose(B, 2, 3, Bt);
  for (int i = 0; i < 6; i++) {
    MU_ASSERT(fltcmp(Bt[i], Bt_expected[i]) == 0);
  }

  return 0;
}

int test_mat_add(void) {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
  real_t C[9] = {0.0};
  mat_add(A, B, C, 3, 3);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], 10.0) == 0);
  }

  return 0;
}

int test_mat_sub(void) {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  mat_sub(A, B, C, 3, 3);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], 0.0) == 0);
  }

  return 0;
}

int test_mat_scale(void) {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  mat_scale(A, 3, 3, 2.0);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(A[i], 2 * (i + 1)) == 0);
  }

  return 0;
}

int test_vec_add(void) {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
  real_t C[9] = {0.0};
  vec_add(A, B, C, 9);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], 10.0) == 0);
  }

  return 0;
}

int test_vec_sub(void) {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  vec_sub(A, B, C, 9);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], 0.0) == 0);
  }

  return 0;
}

int test_dot(void) {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[3] = {1.0, 2.0, 3.0};
  real_t C[3] = {0.0};

  /* Multiply matrix A and B */
  dot(A, 3, 3, B, 3, 1, C);
  // print_matrix("A", A, 3, 3);
  // print_matrix("B", B, 3, 1);
  // print_matrix("C", C, 3, 1);

  MU_ASSERT(fltcmp(C[0], 14.0) == 0);
  MU_ASSERT(fltcmp(C[1], 32.0) == 0);
  MU_ASSERT(fltcmp(C[2], 50.0) == 0);

  return 0;
}

int test_bdiag_inv(void) {
  // int num_rows = 0;
  // int num_cols = 0;
  // real_t *H = mat_load("/tmp/H.csv", &num_rows, &num_cols);
  //
  // // Invert taking advantage of block diagonal structure
  // {
  //   real_t *H_inv = CALLOC(real_t, num_rows * num_rows);
  //
  //   // TIC(bdiag_time);
  //   bdiag_inv(H, num_rows, 6, H_inv);
  //   // printf("H: %dx%d\n", num_rows, num_cols);
  //   // printf("invert block diagonal -> time taken: %f\n", TOC(bdiag_time));
  //   MU_ASSERT(check_inv(H, H_inv, num_rows) == 0);
  //
  //   free(H_inv);
  // }
  //
  // // Invert the dumb way
  // {
  //
  //   real_t *H_inv = CALLOC(real_t, num_rows * num_rows);
  //
  //   // TIC(pinv_time);
  //   pinv(H, num_rows, num_rows, H_inv);
  //   // eig_inv(H, num_rows, num_rows, 0, H_inv);
  //   // printf("invert dumb way -> time taken: %f\n", TOC(pinv_time));
  //   MU_ASSERT(check_inv(H, H_inv, num_rows) == 0);
  //
  //   free(H_inv);
  // }
  //
  // free(H);

  return 0;
}

int test_hat(void) {
  real_t x[3] = {1.0, 2.0, 3.0};
  real_t S[3 * 3] = {0};

  hat(x, S);

  MU_ASSERT(fltcmp(S[0], 0.0) == 0);
  MU_ASSERT(fltcmp(S[1], -3.0) == 0);
  MU_ASSERT(fltcmp(S[2], 2.0) == 0);

  MU_ASSERT(fltcmp(S[3], 3.0) == 0);
  MU_ASSERT(fltcmp(S[4], 0.0) == 0);
  MU_ASSERT(fltcmp(S[5], -1.0) == 0);

  MU_ASSERT(fltcmp(S[6], -2.0) == 0);
  MU_ASSERT(fltcmp(S[7], 1.0) == 0);
  MU_ASSERT(fltcmp(S[8], 0.0) == 0);

  return 0;
}

int test_check_jacobian(void) {
  const size_t m = 2;
  const size_t n = 3;
  const real_t threshold = 1e-6;
  const int print = 0;

  // Positive test
  {
    const real_t fdiff[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    const real_t jac[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    int retval = check_jacobian("test_check_jacobian",
                                fdiff,
                                jac,
                                m,
                                n,
                                threshold,
                                print);
    MU_ASSERT(retval == 0);
  }

  // Negative test
  {
    const real_t fdiff[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    const real_t jac[6] = {0.0, 1.0, 2.0, 3.1, 4.0, 5.0};
    int retval = check_jacobian("test_check_jacobian",
                                fdiff,
                                jac,
                                m,
                                n,
                                threshold,
                                print);
    MU_ASSERT(retval == -1);
  }

  return 0;
}

int test_svd(void) {
  // Matrix A
  // clang-format off
  real_t A[6 * 4] = {
    7.52, -1.10, -7.95,  1.08,
    -0.76,  0.62,  9.34, -7.10,
     5.13,  6.62, -5.66,  0.87,
    -4.75,  8.52,  5.75,  5.30,
     1.33,  4.91, -5.49, -3.52,
    -2.40, -6.77,  2.34,  3.95
  };
  // clang-format on

  // Decompose A with SVD
  // struct timespec t = tic();
  real_t U[6 * 4] = {0};
  real_t s[4] = {0};
  real_t V[4 * 4] = {0};
  svd(A, 6, 4, U, s, V);
  // printf("time taken: [%fs]\n", toc(&t));

  // Multiply the output to see if it can form matrix A again
  // U * S * Vt
  real_t S[4 * 4] = {0};
  real_t Vt[4 * 4] = {0};
  real_t US[6 * 4] = {0};
  real_t USVt[6 * 4] = {0};
  mat_diag_set(S, 4, 4, s);
  mat_transpose(V, 4, 4, Vt);
  dot(U, 6, 4, S, 4, 4, US);
  dot(US, 6, 4, Vt, 4, 4, USVt);

  // print_matrix("U", U, 6, 4);
  // print_matrix("S", S, 4, 4);
  // print_matrix("V", V, 4, 4);
  // print_matrix("USVt", USVt, 6, 4);
  // print_matrix("A", A, 6, 4);
  MU_ASSERT(mat_equals(USVt, A, 6, 4, 1e-5));

  return 0;
}

int test_pinv(void) {
  // clang-format off
  const int m = 4;
  const int n = 4;
  real_t A[4 * 4] = {
     7.52, -1.10, -7.95,  1.08,
    -0.76,  0.62,  9.34, -7.10,
     5.13,  6.62, -5.66,  0.87,
    -4.75,  8.52,  5.75,  5.30,
  };
  // clang-format on

  // Invert matrix A using SVD
  real_t A_inv[4 * 4] = {0};
  pinv(A, m, n, A_inv);
  // print_matrix("A", A, m, n);
  // print_matrix("A_inv", A_inv, m, n);

  // Inverse check: A * A_inv = eye
  MU_ASSERT(check_inv(A, A_inv, 4) == 0);

  return 0;
}

int test_svd_det(void) {
  // clang-format off
  const int m = 4;
  const int n = 4;
  real_t A[4 * 4] = {
     7.52, -1.10, -7.95,  1.08,
    -0.76,  0.62,  9.34, -7.10,
     5.13,  6.62, -5.66,  0.87,
    -4.75,  8.52,  5.75,  5.30,
  };
  // clang-format on

  real_t det = 0.0;
  MU_ASSERT(svd_det(A, m, n, &det) == 0);

  return 0;
}

int test_chol(void) {
  // clang-format off
  const int n = 3;
  real_t A[9] = {
    4.0, 12.0, -16.0,
    12.0, 37.0, -43.0,
    -16.0, -43.0, 98.0
  };
  // clang-format on

  // struct timespec t = tic();
  real_t L[9] = {0};
  chol(A, n, L);
  // printf("time taken: [%fs]\n", toc(&t));
  // mat_save("/tmp/L.csv", L, 3, 3);

  real_t Lt[9] = {0};
  real_t LLt[9] = {0};
  mat_transpose(L, n, n, Lt);
  dot(L, n, n, Lt, n, n, LLt);

  int debug = 0;
  if (debug) {
    print_matrix("L", L, n, n);
    printf("\n");
    print_matrix("Lt", Lt, n, n);
    printf("\n");
    print_matrix("LLt", LLt, n, n);
    printf("\n");
    print_matrix("A", A, n, n);
  }
  MU_ASSERT(mat_equals(A, LLt, n, n, 1e-5));

  return 0;
}

int test_chol_solve(void) {
  // clang-format off
  const int n = 3;
  real_t A[9] = {
    2.0, -1.0, 0.0,
    -1.0, 2.0, -1.0,
    0.0, -1.0, 1.0
  };
  real_t b[3] = {1.0, 0.0, 0.0};
  real_t x[3] = {0.0, 0.0, 0.0};
  // clang-format on

  // struct timespec t = tic();
  chol_solve(A, b, x, n);
  // printf("time taken: [%fs]\n", toc(&t));
  // print_matrix("A", A, n, n);
  // print_vector("b", b, n);
  // print_vector("x", x, n);

  MU_ASSERT(fltcmp(x[0], 1.0) == 0);
  MU_ASSERT(fltcmp(x[1], 1.0) == 0);
  MU_ASSERT(fltcmp(x[2], 1.0) == 0);

  return 0;
}

int test_qr(void) {
  // clang-format off
  const int m = 5;
  const int n = 5;
  real_t A[5 * 5] = {
    17.0, 24.0,  1.0,  8.0, 15.0,
    23.0,  5.0,  7.0, 14.0, 16.0,
     4.0,  6.0, 13.0, 20.0, 22.0,
    10.0, 12.0, 19.0, 21.0,  3.0,
    11.0, 18.0, 25.0,  2.0,  9.0,
  };
  // clang-format on

  // Test
  real_t R[5 * 5] = {0};
  qr(A, m, n, R);
  // print_matrix("A", A, m, n);
  // print_matrix("R", R, m, n);

  // clang-format off
  real_t R_expected[5 * 5] = {
    -32.4808,  -26.6311,  -21.3973,  -23.7063,  -25.8615,
           0,   19.8943,   12.3234,    1.9439,    4.0856,
           0,         0,  -24.3985,  -11.6316,   -3.7415,
           0,         0,         0,  -20.0982,   -9.9739,
           0,         0,         0,         0,  -16.0005
  };
  // print_matrix("R", R, m, n);
  // print_matrix("R_expected", R_expected, m, n);
  // clang-format on
  MU_ASSERT(mat_equals(R, R_expected, 5, 5, 1e-4));

  return 0;
}

int test_eig_sym(void) {
  // clang-format off
  const int m = 5;
  const int n = 5;
  real_t A[5 * 5] = {
     1.96, -6.49, -0.47, -7.20, -0.65,
    -6.49,  3.80, -6.39,  1.50, -6.34,
    -0.47, -6.39,  4.17, -1.51,  2.67,
    -7.20,  1.50, -1.51,  5.70,  1.80,
    -0.65, -6.34,  2.67,  1.80, -7.10
  };
  // clang-format on

  // Eigen-decomposition
  real_t V[5 * 5] = {0};
  real_t w[5] = {0};
  int retval = eig_sym(A, m, n, V, w);
  MU_ASSERT(retval == 0);

  // Assert
  //
  //   A * V == lambda * V
  //
  // where:
  //
  //   A: original matrix
  //   V: Eigen-vectors
  //   lambda: Eigen-values
  //
  DOT(A, 5, 5, V, 5, 5, AV);

  for (int j = 0; j < n; j++) {
    real_t lv[5] = {0};
    mat_col_get(V, m, n, j, lv);
    vec_scale(lv, 5, w[j]);

    real_t av[5] = {0};
    mat_col_get(A, m, n, j, av);

    MU_ASSERT(vec_equals(av, lv, 5) == 0);
  }

  // print_matrix("AV", AV, 5, 5);
  // print_matrix("A", A, 5, 5);
  // print_matrix("V", V, 5, 5);
  // print_vector("w", w, 5);

  return 0;
}

int test_eig_inv(void) {
  // clang-format off
  const int m = 5;
  const int n = 5;
  real_t A[5 * 5] = {
     1.96, -6.49, -0.47, -7.20, -0.65,
    -6.49,  3.80, -6.39,  1.50, -6.34,
    -0.47, -6.39,  4.17, -1.51,  2.67,
    -7.20,  1.50, -1.51,  5.70,  1.80,
    -0.65, -6.34,  2.67,  1.80, -7.10
  };
  // clang-format on

  // Invert matrix A using SVD
  real_t A_inv[5 * 5] = {0};
  eig_inv(A, m, n, 1, A_inv);

  // Inverse check: A * A_inv = eye
  MU_ASSERT(check_inv(A, A_inv, 5) == 0);

  return 0;
}

int test_schur_complement(void) {
  // clang-format off
  real_t H[10 * 10] = {
    0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
    2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 3, 3, 3, 3, 3, 3,
  };
  real_t b[10] = {0, 0, 0, 0, 1, 1, 1, 1, 1, 1};
  // clang-format on

  // H = [Hmm, Hmr,
  //      Hrm, Hrr]
  real_t Hmm[4 * 4] = {0};
  real_t Hmr[4 * 6] = {0};
  real_t Hrm[6 * 4] = {0};
  real_t Hrr[6 * 6] = {0};
  int H_size = 10;
  int m = 4;
  int r = 6;
  mat_block_get(H, H_size, 0, m - 1, 0, m - 1, Hmm);
  mat_block_get(H, H_size, 0, m - 1, m, H_size - 1, Hmr);
  mat_block_get(H, H_size, m, H_size - 1, 0, m - 1, Hrm);
  mat_block_get(H, H_size, m, H_size - 1, m, H_size - 1, Hrr);

  // print_matrix("H", H, 10, 10);
  // print_matrix("Hmm", Hmm, m, m);
  // print_matrix("Hmr", Hmr, m, r);
  // print_matrix("Hrm", Hrm, r, m);
  // print_matrix("Hrr", Hrr, r, r);

  real_t bmm[4] = {0};
  real_t brr[6] = {0};
  vec_copy(b, m, bmm);
  vec_copy(b + m, r, brr);

  // print_vector("b", b, 10);
  // print_vector("bmm", bmm, m);
  // print_vector("brr", brr, r);

  // real_t *Hmm = malloc(sizeof(real_t) * m * m);
  // real_t *Hmr = malloc(sizeof(real_t) * m * r);
  // real_t *Hrm = malloc(sizeof(real_t) * m * r);
  // real_t *Hrr = malloc(sizeof(real_t) * r * r);
  // real_t *Hmm_inv = malloc(sizeof(real_t) * m * m);

  return 0;
}

int test_suitesparse_chol_solve(void) {
  // clang-format off
  const int n = 3;
  real_t A[9] = {
    2.0, -1.0, 0.0,
    -1.0, 2.0, -1.0,
    0.0, -1.0, 1.0
  };
  real_t b[3] = {1.0, 0.0, 0.0};
  real_t x[3] = {0.0, 0.0, 0.0};
  // clang-format on

  // struct timespec t = tic();
  cholmod_common common;
  cholmod_start(&common);
  suitesparse_chol_solve(&common, A, n, n, b, n, x);
  cholmod_finish(&common);
  // printf("time taken: [%fs]\n", toc(&t));
  // print_vector("x", x, n);

  MU_ASSERT(fltcmp(x[0], 1.0) == 0);
  MU_ASSERT(fltcmp(x[1], 1.0) == 0);
  MU_ASSERT(fltcmp(x[2], 1.0) == 0);

  return 0;
}

/******************************************************************************
 * TRANSFORMS
 ******************************************************************************/

int test_tf_rot_set(void) {
  real_t C[9];
  for (int i = 0; i < 9; i++) {
    C[i] = 1.0;
  }

  real_t T[16] = {0.0};
  tf_rot_set(T, C);
  /* print_matrix("T", T, 4, 4); */

  MU_ASSERT(fltcmp(T[0], 1.0) == 0);
  MU_ASSERT(fltcmp(T[1], 1.0) == 0);
  MU_ASSERT(fltcmp(T[2], 1.0) == 0);
  MU_ASSERT(fltcmp(T[3], 0.0) == 0);

  MU_ASSERT(fltcmp(T[4], 1.0) == 0);
  MU_ASSERT(fltcmp(T[5], 1.0) == 0);
  MU_ASSERT(fltcmp(T[6], 1.0) == 0);
  MU_ASSERT(fltcmp(T[7], 0.0) == 0);

  MU_ASSERT(fltcmp(T[8], 1.0) == 0);
  MU_ASSERT(fltcmp(T[9], 1.0) == 0);
  MU_ASSERT(fltcmp(T[10], 1.0) == 0);
  MU_ASSERT(fltcmp(T[11], 0.0) == 0);

  MU_ASSERT(fltcmp(T[12], 0.0) == 0);
  MU_ASSERT(fltcmp(T[13], 0.0) == 0);
  MU_ASSERT(fltcmp(T[14], 0.0) == 0);
  MU_ASSERT(fltcmp(T[15], 0.0) == 0);

  return 0;
}

int test_tf_trans_set(void) {
  real_t r[3] = {1.0, 2.0, 3.0};

  real_t T[16] = {0.0};
  tf_trans_set(T, r);
  /* print_matrix("T", T, 4, 4); */

  MU_ASSERT(fltcmp(T[0], 0.0) == 0);
  MU_ASSERT(fltcmp(T[1], 0.0) == 0);
  MU_ASSERT(fltcmp(T[2], 0.0) == 0);
  MU_ASSERT(fltcmp(T[3], 1.0) == 0);

  MU_ASSERT(fltcmp(T[4], 0.0) == 0);
  MU_ASSERT(fltcmp(T[5], 0.0) == 0);
  MU_ASSERT(fltcmp(T[6], 0.0) == 0);
  MU_ASSERT(fltcmp(T[7], 2.0) == 0);

  MU_ASSERT(fltcmp(T[8], 0.0) == 0);
  MU_ASSERT(fltcmp(T[9], 0.0) == 0);
  MU_ASSERT(fltcmp(T[10], 0.0) == 0);
  MU_ASSERT(fltcmp(T[11], 3.0) == 0);

  MU_ASSERT(fltcmp(T[12], 0.0) == 0);
  MU_ASSERT(fltcmp(T[13], 0.0) == 0);
  MU_ASSERT(fltcmp(T[14], 0.0) == 0);
  MU_ASSERT(fltcmp(T[15], 0.0) == 0);

  return 0;
}

int test_tf_trans_get(void) {
  // clang-format off
  real_t T[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  // clang-format on

  /* Get translation vector */
  real_t r[3];
  tf_trans_get(T, r);
  MU_ASSERT(fltcmp(r[0], 4.0) == 0);
  MU_ASSERT(fltcmp(r[1], 8.0) == 0);
  MU_ASSERT(fltcmp(r[2], 12.0) == 0);

  return 0;
}

int test_tf_rot_get(void) {
  /* Transform */
  // clang-format off
  real_t T[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  // clang-format on

  /* Get rotation matrix */
  real_t C[9];
  tf_rot_get(T, C);

  MU_ASSERT(fltcmp(C[0], 1.0) == 0);
  MU_ASSERT(fltcmp(C[1], 2.0) == 0);
  MU_ASSERT(fltcmp(C[2], 3.0) == 0);

  MU_ASSERT(fltcmp(C[3], 5.0) == 0);
  MU_ASSERT(fltcmp(C[4], 6.0) == 0);
  MU_ASSERT(fltcmp(C[5], 7.0) == 0);

  MU_ASSERT(fltcmp(C[6], 9.0) == 0);
  MU_ASSERT(fltcmp(C[7], 10.0) == 0);
  MU_ASSERT(fltcmp(C[8], 11.0) == 0);

  return 0;
}

int test_tf_quat_get(void) {
  /* Transform */
  // clang-format off
  real_t T[16] = {1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0};
  // clang-format on

  /* Create rotation matrix */
  const real_t ypr_in[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
  real_t C[9] = {0};
  euler321(ypr_in, C);
  tf_rot_set(T, C);

  /* Extract quaternion from transform */
  real_t q[4] = {0};
  tf_quat_get(T, q);

  /* Convert quaternion back to euler angles */
  real_t ypr_out[3] = {0};
  quat2euler(q, ypr_out);

  MU_ASSERT(fltcmp(rad2deg(ypr_out[0]), 10.0) == 0);
  MU_ASSERT(fltcmp(rad2deg(ypr_out[1]), 20.0) == 0);
  MU_ASSERT(fltcmp(rad2deg(ypr_out[2]), 30.0) == 0);

  return 0;
}

int test_tf_inv(void) {
  /* Create Transform */
  // clang-format off
  real_t T[16] = {1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0};
  // clang-format on
  /* -- Set rotation component */
  const real_t euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
  real_t C[9] = {0};
  euler321(euler, C);
  tf_rot_set(T, C);
  /* -- Set translation component */
  real_t r[3] = {1.0, 2.0, 3.0};
  tf_trans_set(T, r);

  /* Invert transform */
  real_t T_inv[16] = {0};
  tf_inv(T, T_inv);

  /* real_t Invert transform */
  real_t T_inv_inv[16] = {0};
  tf_inv(T_inv, T_inv_inv);

  /* Assert */
  int idx = 0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      MU_ASSERT(fltcmp(T_inv_inv[idx], T[idx]) == 0);
    }
  }

  return 0;
}

int test_tf_point(void) {
  /* Transform */
  // clang-format off
  real_t T[16] = {1.0, 0.0, 0.0, 1.0,
                  0.0, 1.0, 0.0, 2.0,
                  0.0, 0.0, 1.0, 3.0,
                  0.0, 0.0, 0.0, 1.0};
  // clang-format on

  /* Point */
  real_t p[3] = {1.0, 2.0, 3.0};

  /* Transform point */
  real_t result[3] = {0};
  tf_point(T, p, result);

  return 0;
}

int test_tf_hpoint(void) {
  /* Transform */
  // clang-format off
  real_t T[16] = {1.0, 0.0, 0.0, 1.0,
                  0.0, 1.0, 0.0, 2.0,
                  0.0, 0.0, 1.0, 3.0,
                  0.0, 0.0, 0.0, 1.0};
  // clang-format on

  /* Homogeneous point */
  real_t hp[4] = {1.0, 2.0, 3.0, 1.0};

  /* Transform homogeneous point */
  real_t result[4] = {0};
  tf_hpoint(T, hp, result);

  return 0;
}

int test_tf_perturb_rot(void) {
  /* Transform */
  // clang-format off
  real_t T[4 * 4] = {1.0, 0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0, 2.0,
                     0.0, 0.0, 1.0, 3.0,
                     0.0, 0.0, 0.0, 1.0};
  // clang-format on

  /* Perturb rotation */
  const real_t step_size = 1e-2;
  tf_perturb_rot(T, step_size, 0);

  /* Assert */
  MU_ASSERT(fltcmp(T[0], 1.0) == 0);
  MU_ASSERT(fltcmp(T[5], 1.0) != 0);
  MU_ASSERT(fltcmp(T[10], 1.0) != 0);

  return 0;
}

int test_tf_perturb_trans(void) {
  /* Transform */
  // clang-format off
  real_t T[4 * 4] = {1.0, 0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0, 2.0,
                     0.0, 0.0, 1.0, 3.0,
                     0.0, 0.0, 0.0, 1.0};
  // clang-format on

  /* Perturb translation */
  const real_t step_size = 1e-2;
  tf_perturb_trans(T, step_size, 0);

  /* Assert */
  MU_ASSERT(fltcmp(T[3], 1.01) == 0);
  MU_ASSERT(fltcmp(T[7], 2.0) == 0);
  MU_ASSERT(fltcmp(T[11], 3.0) == 0);

  return 0;
}

int test_tf_chain(void) {
  /* First transform */
  const real_t r0[3] = {0.0, 0.0, 0.1};
  const real_t euler0[3] = {deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  real_t T0[4 * 4] = {0};
  real_t C0[9] = {0};

  euler321(euler0, C0);
  tf_rot_set(T0, C0);
  tf_trans_set(T0, r0);
  T0[15] = 1.0;

  /* Second transform */
  const real_t r1[3] = {0.0, 0.0, 0.1};
  const real_t euler1[3] = {deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  real_t T1[4 * 4] = {0};
  real_t C1[9] = {0};

  euler321(euler1, C1);
  tf_rot_set(T1, C1);
  tf_trans_set(T1, r1);
  T1[15] = 1.0;

  /* Third transform */
  const real_t r2[3] = {0.0, 0.0, 0.1};
  const real_t euler2[3] = {deg2rad(0.0), deg2rad(0.0), deg2rad(0.0)};
  real_t T2[4 * 4] = {0};
  real_t C2[9] = {0};

  euler321(euler2, C2);
  tf_rot_set(T2, C2);
  tf_trans_set(T2, r2);
  T2[15] = 1.0;

  /* Chain transforms */
  const real_t *tfs[3] = {T0, T1, T2};
  const int N = 3;
  real_t T_out[4 * 4] = {0};
  tf_chain(tfs, N, T_out);

  return 0;
}

int test_euler321(void) {
  /* Euler to rotation matrix */
  const real_t euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
  real_t C[9] = {0};
  euler321(euler, C);

  /* Rotation matrix to quaternion */
  real_t q[4] = {0};
  rot2quat(C, q);

  /* Quaternion to Euler angles*/
  real_t euler2[3] = {0};
  quat2euler(q, euler2);

  MU_ASSERT(fltcmp(euler2[0], euler[0]) == 0);
  MU_ASSERT(fltcmp(euler2[1], euler[1]) == 0);
  MU_ASSERT(fltcmp(euler2[2], euler[2]) == 0);

  return 0;
}

int test_rot2quat(void) {
  /* Rotation matrix to quaternion */
  const real_t C[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  real_t q[4] = {0.0};
  rot2quat(C, q);

  MU_ASSERT(fltcmp(q[0], 1.0) == 0);
  MU_ASSERT(fltcmp(q[1], 0.0) == 0);
  MU_ASSERT(fltcmp(q[2], 0.0) == 0);
  MU_ASSERT(fltcmp(q[3], 0.0) == 0);

  return 0;
}

int test_quat2euler(void) {
  const real_t C[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  /* Rotation matrix to quaternion */
  real_t q[4] = {0.0};
  rot2quat(C, q);

  /* Quaternion to Euler angles */
  real_t ypr[3] = {0.0};
  quat2euler(q, ypr);

  MU_ASSERT(fltcmp(ypr[0], 0.0) == 0);
  MU_ASSERT(fltcmp(ypr[1], 0.0) == 0);
  MU_ASSERT(fltcmp(ypr[2], 0.0) == 0);

  return 0;
}

int test_quat2rot(void) {
  /* Euler to rotation matrix */
  const real_t euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
  real_t C[9] = {0};
  euler321(euler, C);

  /* Rotation matrix to quaternion */
  real_t q[4] = {0.0};
  rot2quat(C, q);
  /* print_vector("q", q, 4); */

  /* Quaternion to rotation matrix */
  real_t rot[9] = {0.0};
  quat2rot(q, rot);

  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], rot[i]) == 0);
  }

  return 0;
}

/******************************************************************************
 * LIE
 ******************************************************************************/

int test_lie_Exp_Log(void) {
  const real_t phi[3] = {0.1, 0.2, 0.3};
  real_t C[3 * 3] = {0};
  lie_Exp(phi, C);

  real_t rvec[3] = {0};
  lie_Log(C, rvec);

  // print_vector("phi", phi, 3);
  // printf("\n");
  // print_matrix("C", C, 3, 3);
  // print_vector("rvec", rvec, 3);

  MU_ASSERT(fltcmp(phi[0], rvec[0]) == 0);
  MU_ASSERT(fltcmp(phi[1], rvec[1]) == 0);
  MU_ASSERT(fltcmp(phi[2], rvec[2]) == 0);

  return 0;
}

/*******************************************************************************
 * GNUPLOT
 ******************************************************************************/

int test_gnuplot_xyplot(void) {
  // Start gnuplot
  FILE *gnuplot = gnuplot_init();

  // First dataset
  {
    int num_points = 5;
    real_t xvals[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    real_t yvals[5] = {5.0, 3.0, 1.0, 3.0, 5.0};
    gnuplot_send(gnuplot, "set title 'Plot 1'");
    gnuplot_send_xy(gnuplot, "$DATA1", xvals, yvals, num_points);
  }

  // Second dataset
  {
    int num_points = 5;
    real_t xvals[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    real_t yvals[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    gnuplot_send_xy(gnuplot, "$DATA2", xvals, yvals, num_points);
  }

  // Plot both datasets in same plot
  gnuplot_send(gnuplot, "plot $DATA1 with lines, $DATA2 with lines");

  // Clean up
  gnuplot_close(gnuplot);

  return 0;
}

int test_gnuplot_multiplot(void) {
  // Start gnuplot
  FILE *gnuplot = gnuplot_init();

  // Setup multiplot
  const int num_rows = 1;
  const int num_cols = 2;
  gnuplot_multiplot(gnuplot, num_rows, num_cols);

  // First plot
  {
    int num_points = 5;
    real_t xvals[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    real_t yvals[5] = {5.0, 3.0, 1.0, 3.0, 5.0};
    gnuplot_send(gnuplot, "set title 'Plot 1'");
    gnuplot_send_xy(gnuplot, "$DATA1", xvals, yvals, num_points);
    gnuplot_send(gnuplot, "plot $DATA1 title 'data1' with lines lt 1");
  }

  // Second plot
  {
    int num_points = 5;
    real_t xvals[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    real_t yvals[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    gnuplot_send(gnuplot, "set title 'Plot 2'");
    gnuplot_send_xy(gnuplot, "$DATA2", xvals, yvals, num_points);
    gnuplot_send(gnuplot, "plot $DATA2 title 'data1' with lines lt 2");
  }

  // Clean up
  gnuplot_close(gnuplot);

  return 0;
}

/******************************************************************************
 * CONTROL
 ******************************************************************************/

int test_pid_ctrl(void) {
  const real_t kp = 0.1;
  const real_t ki = 0.2;
  const real_t kd = 0.3;
  pid_ctrl_t pid;
  pid_ctrl_setup(&pid, kp, ki, kd);

  MU_ASSERT(fltcmp(pid.error_prev, 0.0) == 0);
  MU_ASSERT(fltcmp(pid.error_sum, 0.0) == 0);

  MU_ASSERT(fltcmp(pid.error_p, 0.0) == 0);
  MU_ASSERT(fltcmp(pid.error_i, 0.0) == 0);
  MU_ASSERT(fltcmp(pid.error_d, 0.0) == 0);

  MU_ASSERT(fltcmp(pid.k_p, kp) == 0);
  MU_ASSERT(fltcmp(pid.k_i, ki) == 0);
  MU_ASSERT(fltcmp(pid.k_d, kd) == 0);

  return 0;
}

/******************************************************************************
 * MAV
 *****************************************************************************/

static void test_setup_mav(mav_model_t *mav) {
  // clang-format off
  const real_t x[12] = {
    // Attitude [rad]
    0.0, 0.0, 0.0,
    // Angular Velocity [rad / s]
    0.0, 0.0, 0.0,
    // Position [m]
    0.0, 0.0, 0.0,
    // Linear velocity [m / s]
    0.0, 0.0, 0.0
  };
  // clang-format on
  const real_t inertia[3] = {0.0963, 0.0963, 0.1927}; // Moment of inertia
  const real_t kr = 0.1;                              // Rotation drag constant
  const real_t kt = 0.2; // Translation drag constant
  const real_t l = 0.9;  // Arm Length
  const real_t d = 1.0;  // Drag constant
  const real_t m = 1.0;  // Mass
  const real_t g = 9.81; // Gravitational constant
  mav_model_setup(mav, x, inertia, kr, kt, l, d, m, g);
}

int test_mav_att_ctrl(void) {
  mav_model_t mav;
  test_setup_mav(&mav);

  mav_att_ctrl_t mav_att_ctrl;
  mav_pos_ctrl_t mav_pos_ctrl;
  mav_att_ctrl_setup(&mav_att_ctrl);
  mav_pos_ctrl_setup(&mav_pos_ctrl);

  const real_t att_sp[4] = {0.1, 0.2, -0.2, 0.0}; // roll, pitch, yaw, thrust
  const real_t dt = 0.001;
  const real_t t_end = 0.5;
  real_t t = 0.0;

  int idx = 0;
  const int N = t_end / dt;
  mav_model_telem_t *telem = mav_model_telem_malloc();

  while (idx < N) {
    const real_t att_pv[3] = {mav.x[0], mav.x[1], mav.x[2]};

    real_t u[4] = {0};
    mav_att_ctrl_update(&mav_att_ctrl, att_sp, att_pv, dt, u);
    mav_model_update(&mav, u, dt);
    mav_model_telem_update(telem, &mav, t);

    t += dt;
    idx += 1;
  }

  int debug = 0;
  if (debug) {
    mav_model_telem_plot(telem);
  }
  mav_model_telem_free(telem);

  return 0;
}

int test_mav_vel_ctrl(void) {
  mav_model_t mav;
  test_setup_mav(&mav);

  mav_att_ctrl_t mav_att_ctrl;
  mav_vel_ctrl_t mav_vel_ctrl;
  mav_att_ctrl_setup(&mav_att_ctrl);
  mav_vel_ctrl_setup(&mav_vel_ctrl);

  const real_t vel_sp[4] = {0.1, 0.2, 1.0, 0.0}; // vx, vy, vz, yaw
  const real_t dt = 0.001;
  const real_t t_end = 10.0;
  real_t t = 0.0;

  int idx = 0;
  const int N = t_end / dt;
  mav_model_telem_t *telem = mav_model_telem_malloc();

  while (idx < N) {
    const real_t vel_pv[4] = {mav.x[9], mav.x[10], mav.x[11], mav.x[2]};
    const real_t att_pv[3] = {mav.x[0], mav.x[1], mav.x[2]};

    real_t att_sp[4] = {0};
    real_t u[4] = {0};
    mav_vel_ctrl_update(&mav_vel_ctrl, vel_sp, vel_pv, dt, att_sp);
    mav_att_ctrl_update(&mav_att_ctrl, att_sp, att_pv, dt, u);
    mav_model_update(&mav, u, dt);
    mav_model_telem_update(telem, &mav, t);

    t += dt;
    idx += 1;
  }

  int debug = 0;
  if (debug) {
    mav_model_telem_plot(telem);
  }
  mav_model_telem_free(telem);

  return 0;
}

int test_mav_pos_ctrl(void) {
  mav_model_t mav;
  test_setup_mav(&mav);

  mav_att_ctrl_t mav_att_ctrl;
  mav_vel_ctrl_t mav_vel_ctrl;
  mav_pos_ctrl_t mav_pos_ctrl;
  mav_att_ctrl_setup(&mav_att_ctrl);
  mav_vel_ctrl_setup(&mav_vel_ctrl);
  mav_pos_ctrl_setup(&mav_pos_ctrl);

  const real_t pos_sp[4] = {10.0, 10.0, 5.0, 0.5}; // x, y, z, yaw
  const real_t dt = 0.001;
  const real_t t_end = 10.0;
  real_t t = 0.0;

  int idx = 0;
  const int N = t_end / dt;
  mav_model_telem_t *telem = mav_model_telem_malloc();

  while (idx < N) {
    const real_t pos_pv[4] = {mav.x[6], mav.x[7], mav.x[8], mav.x[2]};
    const real_t vel_pv[4] = {mav.x[9], mav.x[10], mav.x[11], mav.x[2]};
    const real_t att_pv[3] = {mav.x[0], mav.x[1], mav.x[2]};

    real_t vel_sp[4] = {0};
    real_t att_sp[4] = {0};
    real_t u[4] = {0};
    mav_pos_ctrl_update(&mav_pos_ctrl, pos_sp, pos_pv, dt, vel_sp);
    mav_vel_ctrl_update(&mav_vel_ctrl, vel_sp, vel_pv, dt, att_sp);
    mav_att_ctrl_update(&mav_att_ctrl, att_sp, att_pv, dt, u);
    mav_model_update(&mav, u, dt);
    mav_model_telem_update(telem, &mav, t);

    t += dt;
    idx += 1;
  }

  int debug = 0;
  if (debug) {
    mav_model_telem_plot(telem);
  }
  mav_model_telem_free(telem);

  return 0;
}

int test_mav_waypoints(void) {
  // Setup MAV model
  mav_model_t mav;
  test_setup_mav(&mav);

  // Setup MAV controllers
  mav_att_ctrl_t mav_att_ctrl;
  mav_vel_ctrl_t mav_vel_ctrl;
  mav_pos_ctrl_t mav_pos_ctrl;
  mav_att_ctrl_setup(&mav_att_ctrl);
  mav_vel_ctrl_setup(&mav_vel_ctrl);
  mav_pos_ctrl_setup(&mav_pos_ctrl);

  // Setup waypoints
  real_t waypoints[8][4] = {{0, 0, 1, 0},
                            {1, 1, 1, 0},
                            {1, -1, 1, 0},
                            {-1, -1, 1, 0},
                            {-1, 1, 1, 0},
                            {1, 1, 1, 0},
                            {0, 0, 1, 0},
                            {0, 0, 1, 1.0}};
  mav_waypoints_t *wps = mav_waypoints_malloc();
  for (int i = 0; i < 8; i++) {
    mav_waypoints_add(wps, waypoints[i]);
  }

  // Simulate
  const real_t dt = 0.001;
  const real_t t_end = 60.0;
  real_t t = 0.0;

  int idx = 0;
  const int N = t_end / dt;
  mav_model_telem_t *telem = mav_model_telem_malloc();

  while (idx < N) {
    const real_t pos_pv[4] = {mav.x[6], mav.x[7], mav.x[8], mav.x[2]};
    const real_t vel_pv[4] = {mav.x[9], mav.x[10], mav.x[11], mav.x[2]};
    const real_t att_pv[3] = {mav.x[0], mav.x[1], mav.x[2]};

    real_t pos_sp[4] = {0};
    mav_waypoints_update(wps, pos_pv, dt, pos_sp);

    real_t vel_sp[4] = {0};
    real_t att_sp[4] = {0};
    real_t u[4] = {0};
    mav_pos_ctrl_update(&mav_pos_ctrl, pos_sp, pos_pv, dt, vel_sp);
    mav_vel_ctrl_update(&mav_vel_ctrl, vel_sp, vel_pv, dt, att_sp);
    mav_att_ctrl_update(&mav_att_ctrl, att_sp, att_pv, dt, u);
    mav_model_update(&mav, u, dt);

    if (idx % 50 == 0) {
      mav_model_telem_update(telem, &mav, t);
    }

    t += dt;
    idx += 1;
  }

  // Plot and clean up
  int debug = 0;
  if (debug) {
    mav_model_telem_plot(telem);
    mav_model_telem_plot_xy(telem);
  }
  mav_model_telem_free(telem);
  mav_waypoints_free(wps);

  // gui_t gui;
  // gui.window_title = "Test";
  // gui.window_width = 640;
  // gui.window_height = 480;
  // gui_setup(&gui);
  // gui_loop(&gui);

  return 0;
}

/******************************************************************************
 * COMPUTER-VISION
 ******************************************************************************/

int test_image_setup(void) {
  return 0;
}

int test_image_load(void) {
  return 0;
}

int test_image_print_properties(void) {
  return 0;
}

int test_image_free(void) {
  return 0;
}

int test_radtan4_distort(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t p_d[2] = {0};
  radtan4_distort(params, p, p_d);

  // print_vector("p", p, 2);
  // print_vector("p_d", p_d, 2);

  return 0;
}

int test_radtan4_undistort(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};

  real_t p_d[2] = {0};
  real_t p_out[2] = {0};
  radtan4_distort(params, p, p_d);
  radtan4_undistort(params, p_d, p_out);

  // print_vector("p", p, 2);
  // print_vector("p_d", p_d, 2);
  // print_vector("p_out", p_out, 2);
  // printf("dp[0]: %f\n", p[0] - p_out[0]);
  // printf("dp[1]: %f\n", p[1] - p_out[1]);

  MU_ASSERT(fltcmp(p[0], p_out[0]) == 0);
  MU_ASSERT(fltcmp(p[1], p_out[1]) == 0);

  return 0;
}

int test_radtan4_point_jacobian(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t J_point[2 * 2] = {0};
  radtan4_point_jacobian(params, p, J_point);

  /* Calculate numerical diff */
  const real_t step = 1e-4;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 2] = {0};
  {
    real_t p_d[2] = {0};
    radtan4_distort(params, p, p_d);

    for (int i = 0; i < 2; i++) {
      real_t p_diff[2] = {p[0], p[1]};
      p_diff[i] = p[i] + step;

      real_t p_d_prime[2] = {0};
      radtan4_distort(params, p_diff, p_d_prime);

      J_numdiff[i] = (p_d_prime[0] - p_d[0]) / step;
      J_numdiff[i + 2] = (p_d_prime[1] - p_d[1]) / step;
    }
  }

  /* Check jacobian */
  // print_vector("p", p, 2);
  // print_matrix("J_point", J_point, 2, 2);
  // print_matrix("J_numdiff", J_numdiff, 2, 2);
  MU_ASSERT(check_jacobian("J", J_numdiff, J_point, 2, 2, tol, 0) == 0);

  return 0;
}

int test_radtan4_params_jacobian(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t J_param[2 * 4] = {0};
  radtan4_params_jacobian(params, p, J_param);

  /* Calculate numerical diff */
  const real_t step = 1e-4;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 4] = {0};
  {
    real_t p_d[2] = {0};
    radtan4_distort(params, p, p_d);

    for (int i = 0; i < 4; i++) {
      real_t params_diff[4] = {params[0], params[1], params[2], params[3]};
      params_diff[i] = params[i] + step;

      real_t p_d_prime[2] = {0};
      radtan4_distort(params_diff, p, p_d_prime);

      J_numdiff[i] = (p_d_prime[0] - p_d[0]) / step;
      J_numdiff[i + 4] = (p_d_prime[1] - p_d[1]) / step;
    }
  }

  /* Check jacobian */
  // print_vector("p", p, 2);
  // print_matrix("J_param", J_param, 2, 4);
  // print_matrix("J_numdiff", J_numdiff, 2, 4);
  MU_ASSERT(check_jacobian("J", J_numdiff, J_param, 2, 4, tol, 0) == 0);

  return 0;
}

int test_equi4_distort(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t p_d[2] = {0};
  equi4_distort(params, p, p_d);

  // print_vector("p", p, 2);
  // print_vector("p_d", p_d, 2);

  return 0;
}

int test_equi4_undistort(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t p_d[2] = {0};
  real_t p_out[2] = {0};
  equi4_distort(params, p, p_d);
  equi4_undistort(params, p_d, p_out);

  // print_vector("p", p, 2);
  // print_vector("p_d", p_d, 2);
  // print_vector("p_out", p_out, 2);
  // printf("dp[0]: %f\n", p[0] - p_out[0]);
  // printf("dp[1]: %f\n", p[1] - p_out[1]);

  MU_ASSERT(fltcmp(p[0], p_out[0]) == 0);
  MU_ASSERT(fltcmp(p[1], p_out[1]) == 0);
  return 0;
}

int test_equi4_point_jacobian(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t J_point[2 * 2] = {0};
  equi4_point_jacobian(params, p, J_point);

  /* Calculate numerical diff */
  const real_t step = 1e-4;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 2] = {0};
  {
    real_t p_d[2] = {0};
    equi4_distort(params, p, p_d);

    for (int i = 0; i < 2; i++) {
      real_t p_diff[2] = {p[0], p[1]};
      p_diff[i] = p[i] + step;

      real_t p_d_prime[2] = {0};
      equi4_distort(params, p_diff, p_d_prime);

      J_numdiff[i] = (p_d_prime[0] - p_d[0]) / step;
      J_numdiff[i + 2] = (p_d_prime[1] - p_d[1]) / step;
    }
  }

  /* Check jacobian */
  // print_vector("p", p, 2);
  // print_matrix("J_point", J_point, 2, 2);
  // print_matrix("J_numdiff", J_numdiff, 2, 2);
  MU_ASSERT(check_jacobian("J", J_numdiff, J_point, 2, 2, tol, 0) == 0);

  return 0;
}

int test_equi4_params_jacobian(void) {
  const real_t params[4] = {0.01, 0.01, 0.01, 0.01};
  const real_t p[2] = {0.1, 0.2};
  real_t J_param[2 * 4] = {0};
  equi4_params_jacobian(params, p, J_param);

  /* Calculate numerical diff */
  const real_t step = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 4] = {0};
  {
    real_t p_d[2] = {0};
    equi4_distort(params, p, p_d);

    for (int i = 0; i < 4; i++) {
      real_t params_diff[4] = {params[0], params[1], params[2], params[3]};
      params_diff[i] = params[i] + step;

      real_t p_d_prime[2] = {0};
      equi4_distort(params_diff, p, p_d_prime);

      J_numdiff[i] = (p_d_prime[0] - p_d[0]) / step;
      J_numdiff[i + 4] = (p_d_prime[1] - p_d[1]) / step;
    }
  }

  /* Check jacobian */
  // print_vector("p", p, 2);
  // print_matrix("J_param", J_param, 2, 4);
  // print_matrix("J_numdiff", J_numdiff, 2, 4);
  MU_ASSERT(check_jacobian("J", J_numdiff, J_param, 2, 4, tol, 0) == 0);

  return 0;
}

int test_pinhole_focal(void) {
  const real_t focal = pinhole_focal(640, 90.0);
  MU_ASSERT(fltcmp(focal, 320.0) == 0);
  return 0;
}

int test_pinhole_K(void) {
  const real_t params[4] = {1.0, 2.0, 3.0, 4.0};
  real_t K[3 * 3] = {0};
  pinhole_K(params, K);

  MU_ASSERT(fltcmp(K[0], 1.0) == 0);
  MU_ASSERT(fltcmp(K[1], 0.0) == 0);
  MU_ASSERT(fltcmp(K[2], 3.0) == 0);

  MU_ASSERT(fltcmp(K[3], 0.0) == 0);
  MU_ASSERT(fltcmp(K[4], 2.0) == 0);
  MU_ASSERT(fltcmp(K[5], 4.0) == 0);

  MU_ASSERT(fltcmp(K[6], 0.0) == 0);
  MU_ASSERT(fltcmp(K[7], 0.0) == 0);
  MU_ASSERT(fltcmp(K[8], 1.0) == 0);

  return 0;
}

int test_pinhole_projection_matrix(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t params[4] = {fx, fy, cx, cy};

  /* Camera pose */
  const real_t ypr_WC0[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC0[3] = {0.0, 0.0, 0.0};
  real_t T_WC0[4 * 4] = {0};
  tf_euler_set(T_WC0, ypr_WC0);
  tf_trans_set(T_WC0, r_WC0);

  /* Camera projection matrix */
  real_t P[3 * 4] = {0};
  pinhole_projection_matrix(params, T_WC0, P);

  /* Project point using projection matrix */
  const real_t p_W[3] = {1.0, 0.1, 0.2};
  const real_t hp_W[4] = {p_W[0], p_W[1], p_W[2], 1.0};
  real_t hp[3] = {0};
  dot(P, 3, 4, hp_W, 4, 1, hp);
  real_t z[2] = {hp[0], hp[1]};

  /* Project point by inverting T_WC0 and projecting the point */
  real_t p_C[3] = {0};
  real_t T_C0W[4 * 4] = {0};
  real_t z_gnd[2] = {0};
  tf_inv(T_WC0, T_C0W);
  tf_point(T_C0W, p_W, p_C);
  pinhole_project(params, p_C, z_gnd);

  /* Assert */
  MU_ASSERT(fltcmp(z_gnd[0], z[0]) == 0);
  MU_ASSERT(fltcmp(z_gnd[1], z[1]) == 0);

  return 0;
}

int test_pinhole_project(void) {
  const real_t img_w = 640;
  const real_t img_h = 480;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t params[4] = {fx, fy, cx, cy};
  const real_t p_C[3] = {0.0, 0.0, 1.0};
  real_t z[2] = {0.0, 0.0};
  pinhole_project(params, p_C, z);

  /* print_vector("p_C", p_C, 3); */
  /* print_vector("z", z, 2); */
  MU_ASSERT(fltcmp(z[0], 320.0) == 0);
  MU_ASSERT(fltcmp(z[1], 240.0) == 0);

  return 0;
}

int test_pinhole_point_jacobian(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t params[4] = {fx, fy, cx, cy};

  /* Calculate analytical jacobian */
  real_t J_point[2 * 2] = {0};
  pinhole_point_jacobian(params, J_point);

  /* Numerical differentiation */
  const real_t p_C[3] = {0.1, 0.2, 1.0};
  real_t z[2] = {0};
  pinhole_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 2] = {0};

  for (size_t i = 0; i < 2; i++) {
    real_t z_fd[2] = {0};
    real_t p_C_fd[3] = {p_C[0], p_C[1], p_C[2]};
    p_C_fd[i] += h;
    pinhole_project(params, p_C_fd, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 2] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  MU_ASSERT(check_jacobian("J_point", J_numdiff, J_point, 2, 2, tol, 0) == 0);

  return 0;
}

int test_pinhole_params_jacobian(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t params[4] = {fx, fy, cx, cy};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 1.0};
  const real_t x[2] = {p_C[0] / p_C[2], p_C[1] / p_C[2]};
  real_t J_params[2 * 4] = {0};
  pinhole_params_jacobian(params, x, J_params);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 4] = {0};

  for (size_t i = 0; i < 4; i++) {
    real_t z_fd[2] = {0};
    real_t params_fd[4] = {params[0], params[1], params[2], params[3]};
    params_fd[i] += h;
    pinhole_project(params_fd, p_C, z_fd);

    J_numdiff[i + 0] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 4] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  MU_ASSERT(check_jacobian("J_params", J_numdiff, J_params, 2, 4, tol, 0) == 0);

  return 0;
}

int test_pinhole_radtan4_project(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.3;
  const real_t k2 = 0.01;
  const real_t p1 = 0.01;
  const real_t p2 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, p1, p2};

  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t x[2] = {0};
  pinhole_radtan4_project(params, p_C, x);

  /* print_vector("x", x, 2); */
  MU_ASSERT(fltcmp(x[0], 323.204000) == 0);
  MU_ASSERT(fltcmp(x[1], 166.406400) == 0);

  return 0;
}

int test_pinhole_radtan4_project_jacobian(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.3;
  const real_t k2 = 0.01;
  const real_t p1 = 0.01;
  const real_t p2 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, p1, p2};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t J[2 * 3] = {0};
  pinhole_radtan4_project_jacobian(params, p_C, J);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_radtan4_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 3] = {0};

  for (size_t i = 0; i < 3; i++) {
    real_t z_fd[2] = {0};
    real_t p_C_fd[3] = {p_C[0], p_C[1], p_C[2]};
    p_C_fd[i] += h;

    pinhole_radtan4_project(params, p_C_fd, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 3] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  /* print_matrix("J_numdiff", J_numdiff, 2, 3); */
  /* print_matrix("J", J, 2, 3); */
  MU_ASSERT(check_jacobian("J", J_numdiff, J, 2, 3, tol, 0) == 0);

  return 0;
}

int test_pinhole_radtan4_params_jacobian(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.3;
  const real_t k2 = 0.01;
  const real_t p1 = 0.01;
  const real_t p2 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, p1, p2};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t J_params[2 * 8] = {0};
  pinhole_radtan4_params_jacobian(params, p_C, J_params);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_radtan4_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 8] = {0};

  for (size_t i = 0; i < 8; i++) {
    real_t z_fd[2] = {0};

    real_t params_fd[8] = {0};
    memcpy(params_fd, params, sizeof(real_t) * 8);
    params_fd[i] += h;

    pinhole_radtan4_project(params_fd, p_C, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 8] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  /* print_matrix("J_numdiff", J_numdiff, 2, 8); */
  /* print_matrix("J_params", J_params, 2, 8); */
  MU_ASSERT(check_jacobian("J_params", J_numdiff, J_params, 2, 8, tol, 0) == 0);

  return 0;
}

int test_pinhole_equi4_project(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.1;
  const real_t k2 = 0.01;
  const real_t k3 = 0.01;
  const real_t k4 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, k3, k4};

  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t x[2] = {0};
  pinhole_equi4_project(params, p_C, x);

  /* print_vector("x", x, 2); */
  MU_ASSERT(fltcmp(x[0], 323.199627) == 0);
  MU_ASSERT(fltcmp(x[1], 166.399254) == 0);

  return 0;
}

int test_pinhole_equi4_project_jacobian(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.1;
  const real_t k2 = 0.01;
  const real_t k3 = 0.01;
  const real_t k4 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, k3, k4};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t J[2 * 3] = {0};
  pinhole_equi4_project_jacobian(params, p_C, J);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_equi4_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 3] = {0};

  for (size_t i = 0; i < 3; i++) {
    real_t z_fd[2] = {0};
    real_t p_C_fd[3] = {p_C[0], p_C[1], p_C[2]};
    p_C_fd[i] += h;

    pinhole_equi4_project(params, p_C_fd, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 3] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  /* print_matrix("J_numdiff", J_numdiff, 2, 3); */
  /* print_matrix("J", J, 2, 3); */
  MU_ASSERT(check_jacobian("J", J_numdiff, J, 2, 3, tol, 0) == 0);

  return 0;
}

int test_pinhole_equi4_params_jacobian(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.1;
  const real_t k2 = 0.01;
  const real_t k3 = 0.01;
  const real_t k4 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, k3, k4};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t J_params[2 * 8] = {0};
  pinhole_equi4_params_jacobian(params, p_C, J_params);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_equi4_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 8] = {0};

  for (size_t i = 0; i < 8; i++) {
    real_t z_fd[2] = {0};

    real_t params_fd[8] = {0};
    memcpy(params_fd, params, sizeof(real_t) * 8);
    params_fd[i] += h;

    pinhole_equi4_project(params_fd, p_C, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 8] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  /* print_matrix("J_numdiff", J_numdiff, 2, 8); */
  /* print_matrix("J_params", J_params, 2, 8); */
  MU_ASSERT(check_jacobian("J_params", J_numdiff, J_params, 2, 8, tol, 0) == 0);

  return 0;
}

int test_linear_triangulation(void) {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2.0;
  const real_t cy = image_height / 2.0;
  const real_t proj_params[4] = {fx, fy, cx, cy};
  real_t K[3 * 3];
  pinhole_K(proj_params, K);

  // Setup camera pose T_WC0
  const real_t ypr_WC0[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC0[3] = {0.0, 0.0, 0.0};
  real_t T_WC0[4 * 4] = {0};
  tf_euler_set(T_WC0, ypr_WC0);
  tf_trans_set(T_WC0, r_WC0);

  // Setup camera pose T_WC1
  const real_t euler_WC1[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC1[3] = {0.1, 0.1, 0.0};
  real_t T_WC1[4 * 4] = {0};
  tf_euler_set(T_WC1, euler_WC1);
  tf_trans_set(T_WC1, r_WC1);

  // Setup projection matrices
  real_t P0[3 * 4] = {0};
  real_t P1[3 * 4] = {0};
  pinhole_projection_matrix(proj_params, T_WC0, P0);
  pinhole_projection_matrix(proj_params, T_WC1, P1);

  // Setup 3D and 2D correspondance points
  int num_tests = 100;
  for (int i = 0; i < num_tests; i++) {
    const real_t p_W[3] = {5.0, randf(-1.0, 1.0), randf(-1.0, 1.0)};

    real_t T_C0W[4 * 4] = {0};
    real_t T_C1W[4 * 4] = {0};
    tf_inv(T_WC0, T_C0W);
    tf_inv(T_WC1, T_C1W);

    real_t p_C0[3] = {0};
    real_t p_C1[3] = {0};
    tf_point(T_C0W, p_W, p_C0);
    tf_point(T_C1W, p_W, p_C1);

    real_t z0[2] = {0};
    real_t z1[2] = {0};
    pinhole_project(proj_params, p_C0, z0);
    pinhole_project(proj_params, p_C1, z1);

    // Test
    real_t p_W_est[3] = {0};
    linear_triangulation(P0, P1, z0, z1, p_W_est);

    // Assert
    real_t diff[3] = {0};
    vec_sub(p_W, p_W_est, diff, 3);
    const real_t norm = vec_norm(diff, 3);
    // print_vector("p_W [gnd]", p_W, 3);
    // print_vector("p_W [est]", p_W_est, 3);
    MU_ASSERT(norm < 1e-4);
    // break;
  }

  return 0;
}

int test_homography_find(void) {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2.0;
  const real_t cy = image_height / 2.0;
  const real_t proj_params[4] = {fx, fy, cx, cy};
  real_t K[3 * 3];
  pinhole_K(proj_params, K);

  // Setup camera pose T_WC0
  const real_t ypr_WC0[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC0[3] = {0.0, 0.0, 0.0};
  real_t T_WC0[4 * 4] = {0};
  tf_euler_set(T_WC0, ypr_WC0);
  tf_trans_set(T_WC0, r_WC0);

  // Setup camera pose T_WC1
  const real_t euler_WC1[3] = {-M_PI / 2.0, 0, -M_PI / 2.0 + 0.3};
  const real_t r_WC1[3] = {0.0, -0.3, 0.0};
  real_t T_WC1[4 * 4] = {0};
  tf_euler_set(T_WC1, euler_WC1);
  tf_trans_set(T_WC1, r_WC1);

  // Setup 3D and 2D correspondance points
  int num_points = 20;
  real_t *pts_i = malloc(sizeof(real_t) * num_points * 2);
  real_t *pts_j = malloc(sizeof(real_t) * num_points * 2);
  for (int i = 0; i < num_points; i++) {
    const real_t p_W[3] = {3.0, randf(-1.0, 1.0), randf(-1.0, 1.0)};

    real_t T_C0W[4 * 4] = {0};
    real_t T_C1W[4 * 4] = {0};
    tf_inv(T_WC0, T_C0W);
    tf_inv(T_WC1, T_C1W);

    real_t p_C0[3] = {0};
    real_t p_C1[3] = {0};
    tf_point(T_C0W, p_W, p_C0);
    tf_point(T_C1W, p_W, p_C1);

    real_t z0[2] = {0};
    real_t z1[2] = {0};
    pinhole_project(proj_params, p_C0, z0);
    pinhole_project(proj_params, p_C1, z1);
    real_t pt_i[2] = {(z0[0] - cx) / fx, (z0[1] - cy) / fy};
    real_t pt_j[2] = {(z1[0] - cx) / fx, (z1[1] - cy) / fy};

    pts_i[i * 2 + 0] = pt_i[0];
    pts_i[i * 2 + 1] = pt_i[1];
    pts_j[i * 2 + 0] = pt_j[0];
    pts_j[i * 2 + 1] = pt_j[1];
  }

  real_t H[3 * 3] = {0};
  int retval = homography_find(pts_i, pts_j, num_points, H);
  MU_ASSERT(retval == 0);

  for (int i = 0; i < num_points; i++) {
    const real_t p0[3] = {pts_i[i * 2 + 0], pts_i[i * 2 + 1], 1.0};
    const real_t p1[3] = {pts_j[i * 2 + 0], pts_j[i * 2 + 1], 1.0};

    real_t p1_est[3] = {0};
    dot(H, 3, 3, p0, 3, 1, p1_est);
    p1_est[0] /= p1_est[2];
    p1_est[1] /= p1_est[2];
    p1_est[2] /= p1_est[2];

    const real_t dx = p1[0] - p1_est[0];
    const real_t dy = p1[1] - p1_est[1];
    const real_t dz = p1[2] - p1_est[2];
    const real_t diff = sqrt(dx * dx + dy * dy + dz * dz);
    if (diff >= 1e-3) {
      print_vector("p1_gnd", p1, 3);
      print_vector("p1_est", p1_est, 3);
      printf("\n");
    }
    MU_ASSERT(diff < 1e-3);
  }

  // Clean up
  free(pts_i);
  free(pts_j);

  return 0;
}

int test_homography_pose(void) {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2.0;
  const real_t cy = image_height / 2.0;
  const real_t proj_params[4] = {fx, fy, cx, cy};

  // Setup camera pose T_WC
  const real_t ypr_WC[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC[3] = {0.0, 0.0, 0.0};
  real_t T_WC[4 * 4] = {0};
  tf_euler_set(T_WC, ypr_WC);
  tf_trans_set(T_WC, r_WC);

  // Calibration target pose T_WF
  const int num_rows = 4;
  const int num_cols = 4;
  const real_t tag_size = 0.1;
  const real_t target_x = ((num_cols - 1) * tag_size) / 2.0;
  const real_t target_y = -((num_rows - 1) * tag_size) / 2.0;
  const real_t ypr_WF[3] = {-M_PI / 2, 0.0, M_PI / 2};
  const real_t r_WF[3] = {0.5, target_x, target_y};
  TF_ER(ypr_WF, r_WF, T_WF);

  // Setup 3D and 2D correspondance points
  const int N = num_rows * num_cols;
  real_t *world_pts = malloc(sizeof(real_t) * N * 3);
  real_t *obj_pts = malloc(sizeof(real_t) * N * 3);
  real_t *img_pts = malloc(sizeof(real_t) * N * 2);

  int idx = 0;
  for (int i = 0; i < num_rows; i++) {
    for (int j = 0; j < num_cols; j++) {
      const real_t p_F[3] = {i * tag_size, j * tag_size, 0.0};
      TF_POINT(T_WF, p_F, p_W);
      TF_INV(T_WC, T_CW);
      TF_POINT(T_CW, p_W, p_C);

      real_t z[2] = {0};
      pinhole_project(proj_params, p_C, z);

      obj_pts[idx * 3 + 0] = p_F[0];
      obj_pts[idx * 3 + 1] = p_F[1];
      obj_pts[idx * 3 + 2] = p_F[2];

      world_pts[idx * 3 + 0] = p_W[0];
      world_pts[idx * 3 + 1] = p_W[1];
      world_pts[idx * 3 + 2] = p_W[2];

      img_pts[idx * 2 + 0] = z[0];
      img_pts[idx * 2 + 1] = z[1];

      idx++;
    }
  }

  // Find homography pose
  real_t T_CF_est[4 * 4] = {0};
  int retval = homography_pose(proj_params, img_pts, obj_pts, N, T_CF_est);
  MU_ASSERT(retval == 0);

  TF_INV(T_WC, T_CW);
  TF_CHAIN(T_CF_gnd, 2, T_CW, T_WF);
  // print_matrix("T_CF_gnd", T_CF_gnd, 4, 4);
  // print_matrix("T_CF_est", T_CF_est, 4, 4);

  // Cleanup
  free(obj_pts);
  free(world_pts);
  free(img_pts);

  return 0;
}

// int test_p3p_kneip(void) {
//   // Setup camera
//   const int image_width = 640;
//   const int image_height = 480;
//   const real_t fov = 120.0;
//   const real_t fx = pinhole_focal(image_width, fov);
//   const real_t fy = pinhole_focal(image_width, fov);
//   const real_t cx = image_width / 2;
//   const real_t cy = image_height / 2;
//   const real_t proj_params[4] = {fx, fy, cx, cy};
//   real_t K[3 * 3];
//   pinhole_K(proj_params, K);

//   // Setup camera pose T_WC
//   const real_t ypr_WC[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
//   const real_t r_WC[3] = {0.0, 0.0, 0.0};
//   real_t T_WC[4 * 4] = {0};
//   tf_euler_set(T_WC, ypr_WC);
//   tf_trans_set(T_WC, r_WC);
//   TF_INV(T_WC, T_CW);

//   // Setup points
//   real_t features[4][3] = {{1.0, -0.1, 0.1},
//                            {1.0, 0.1, 0.1},
//                            {1.0, 0.1, -0.1},
//                            {1.0, -0.1, -0.1}};
//   real_t points[4][3] = {0};
//   tf_point(T_CW, features[0], points[0]);
//   tf_point(T_CW, features[1], points[1]);
//   tf_point(T_CW, features[2], points[2]);

//   real_t solutions[4][4 * 4];
//   p3p_kneip(features, points, solutions);
//   // printf("retval: %d\n", retval);

//   return 0;
// }

int test_solvepnp(void) {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2.0;
  const real_t cy = image_height / 2.0;
  const real_t proj_params[4] = {fx, fy, cx, cy};

  // Setup camera pose T_WC
  const real_t ypr_WC[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC[3] = {0.0, 0.0, 0.0};
  real_t T_WC[4 * 4] = {0};
  tf_euler_set(T_WC, ypr_WC);
  tf_trans_set(T_WC, r_WC);

  // Calibration target pose T_WF
  const int num_rows = 4;
  const int num_cols = 4;
  const real_t tag_size = 0.1;
  const real_t target_x = ((num_cols - 1) * tag_size) / 2.0;
  const real_t target_y = -((num_rows - 1) * tag_size) / 2.0;
  const real_t ypr_WF[3] = {-M_PI / 2, 0.0, M_PI / 2};
  const real_t r_WF[3] = {0.5, target_x, target_y};
  TF_ER(ypr_WF, r_WF, T_WF);

  // Setup 3D and 2D correspondance points
  const int N = num_rows * num_cols;
  real_t *world_pts = malloc(sizeof(real_t) * N * 3);
  real_t *obj_pts = malloc(sizeof(real_t) * N * 3);
  real_t *img_pts = malloc(sizeof(real_t) * N * 2);

  int idx = 0;
  for (int i = 0; i < num_rows; i++) {
    for (int j = 0; j < num_cols; j++) {
      const real_t p_F[3] = {i * tag_size, j * tag_size, 0.0};
      TF_POINT(T_WF, p_F, p_W);
      TF_INV(T_WC, T_CW);
      TF_POINT(T_CW, p_W, p_C);

      real_t z[2] = {0};
      pinhole_project(proj_params, p_C, z);

      obj_pts[idx * 3 + 0] = p_F[0];
      obj_pts[idx * 3 + 1] = p_F[1];
      obj_pts[idx * 3 + 2] = p_F[2];

      world_pts[idx * 3 + 0] = p_W[0];
      world_pts[idx * 3 + 1] = p_W[1];
      world_pts[idx * 3 + 2] = p_W[2];

      img_pts[idx * 2 + 0] = z[0];
      img_pts[idx * 2 + 1] = z[1];

      idx++;
    }
  }

  // Find homography pose
  real_t T_CF_est[4 * 4] = {0};
  // struct timespec t_start = tic();
  int retval = solvepnp(proj_params, img_pts, obj_pts, N, T_CF_est);
  MU_ASSERT(retval == 0);
  // printf("time: %f\n", toc(&t_start));

  TF_INV(T_WC, T_CW);
  TF_CHAIN(T_CF_gnd, 2, T_CW, T_WF);

  real_t dr[3] = {0};
  real_t dr_norm = {0};
  real_t dtheta = 0;
  tf_diff2(T_CF_gnd, T_CF_est, dr, &dtheta);
  dr_norm = vec_norm(dr, 3);

  // printf("dr: %f, dtheta: %f\n", dr_norm, dtheta);
  MU_ASSERT(dr_norm < 1e-5);
  MU_ASSERT(dtheta < 1e-5);

  // print_matrix("T_CF_gnd", T_CF_gnd, 4, 4);
  // print_matrix("T_CF_est", T_CF_est, 4, 4);

  // Cleanup
  free(obj_pts);
  free(world_pts);
  free(img_pts);

  return 0;
}

/*******************************************************************************
 * APRILGRID
 ******************************************************************************/

int test_aprilgrid_malloc_and_free(void) {
  // Setup
  const int num_rows = 6;
  const int num_cols = 7;
  const real_t tsize = 0.1;
  const real_t tspacing = 0.1;
  aprilgrid_t *g = aprilgrid_malloc(num_rows, num_cols, tsize, tspacing);

  MU_ASSERT(g->timestamp == 0);
  MU_ASSERT(g->num_rows == num_rows);
  MU_ASSERT(g->num_cols == num_cols);
  MU_ASSERT_FLOAT(g->tag_size, tsize);
  MU_ASSERT_FLOAT(g->tag_spacing, tspacing);
  MU_ASSERT(g->corners_detected == 0);

  aprilgrid_free(g);

  return 0;
}

int test_aprilgrid_center(void) {
  // Setup
  int num_rows = 5;
  int num_cols = 2;
  real_t tag_size = 0.1;
  real_t tag_spacing = 0;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  // Aprilgrid center
  real_t cx = 0.0;
  real_t cy = 0.0;
  aprilgrid_center(grid, &cx, &cy);
  MU_ASSERT_FLOAT(cx, 0.1);
  MU_ASSERT_FLOAT(cy, 0.25);

  // Clean up
  aprilgrid_free(grid);

  return 0;
}

int test_aprilgrid_grid_index(void) {
  // Setup
  const int num_rows = 6;
  const int num_cols = 6;
  const real_t tag_size = 0.088;
  const real_t tag_spacing = 0.3;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  // Get grid index
  int i = 0;
  int j = 0;
  aprilgrid_grid_index(grid, 0, &i, &j);
  MU_ASSERT(i == 0);
  MU_ASSERT(j == 0);

  aprilgrid_grid_index(grid, 1, &i, &j);
  MU_ASSERT(i == 0);
  MU_ASSERT(j == 1);

  aprilgrid_grid_index(grid, 5, &i, &j);
  MU_ASSERT(i == 0);
  MU_ASSERT(j == 5);

  aprilgrid_grid_index(grid, 7, &i, &j);
  MU_ASSERT(i == 1);
  MU_ASSERT(j == 1);

  aprilgrid_grid_index(grid, 17, &i, &j);
  MU_ASSERT(i == 2);
  MU_ASSERT(j == 5);

  // Clean up
  aprilgrid_free(grid);

  return 0;
}

int test_aprilgrid_object_point(void) {
  // Setup
  const int num_rows = 6;
  const int num_cols = 6;
  const real_t tag_size = 0.1;
  const real_t tag_spacing = 0.0;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  // Get object point
  real_t p[3] = {0};
  aprilgrid_object_point(grid, 1, 0, p);
  MU_ASSERT_FLOAT(p[0], tag_size);
  MU_ASSERT_FLOAT(p[1], 0);
  MU_ASSERT_FLOAT(p[2], 0);

  aprilgrid_object_point(grid, 1, 1, p);
  MU_ASSERT_FLOAT(p[0], tag_size * 2);
  MU_ASSERT_FLOAT(p[1], 0);
  MU_ASSERT_FLOAT(p[2], 0);

  aprilgrid_object_point(grid, 1, 2, p);
  MU_ASSERT_FLOAT(p[0], tag_size * 2);
  MU_ASSERT_FLOAT(p[1], tag_size);
  MU_ASSERT_FLOAT(p[2], 0);

  aprilgrid_object_point(grid, 1, 3, p);
  MU_ASSERT_FLOAT(p[0], tag_size);
  MU_ASSERT_FLOAT(p[1], tag_size);
  MU_ASSERT_FLOAT(p[2], 0);

  // Clean up
  aprilgrid_free(grid);

  return 0;
}

int test_aprilgrid_add_and_remove_corner(void) {
  // Setup
  const int num_rows = 6;
  const int num_cols = 6;
  const real_t tag_size = 0.1;
  const real_t tag_spacing = 0.0;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  // Add corner
  const int tag_id = 5;
  const int corner_idx = 0;
  const real_t kp[2] = {1.0, 2.0};
  aprilgrid_add_corner(grid, tag_id, corner_idx, kp);

  const int data_row = (tag_id * 4) + corner_idx;
  MU_ASSERT(grid->corners_detected == 1);
  MU_ASSERT(grid->data[data_row * 6 + 0] == 1);
  MU_ASSERT_FLOAT(grid->data[data_row * 6 + 1], kp[0]);
  MU_ASSERT_FLOAT(grid->data[data_row * 6 + 2], kp[1]);

  // Remove corner
  aprilgrid_remove_corner(grid, tag_id, corner_idx);

  MU_ASSERT(grid->corners_detected == 0);
  MU_ASSERT(grid->data[data_row * 6 + 0] == 0);
  MU_ASSERT_FLOAT(grid->data[data_row * 6 + 1], 0.0);
  MU_ASSERT_FLOAT(grid->data[data_row * 6 + 2], 0.0);

  // Clean up
  aprilgrid_free(grid);

  return 0;
}

int test_aprilgrid_add_and_remove_tag(void) {
  // Setup
  const int num_rows = 6;
  const int num_cols = 6;
  const real_t tag_size = 0.1;
  const real_t tag_spacing = 0.2;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  // Add tag
  const int tag_id = 5;
  const real_t tag_kps[4][2] = {{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}, {7.0, 8.0}};
  aprilgrid_add_tag(grid, tag_id, tag_kps);

  MU_ASSERT(grid->corners_detected == 4);
  for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
    const int data_row = (tag_id * 4) + corner_idx;
    MU_ASSERT(grid->data[data_row * 6 + 0] == 1);
    MU_ASSERT_FLOAT(grid->data[data_row * 6 + 1], tag_kps[corner_idx][0]);
    MU_ASSERT_FLOAT(grid->data[data_row * 6 + 2], tag_kps[corner_idx][1]);
  }

  // Remove tag
  aprilgrid_remove_tag(grid, tag_id);

  MU_ASSERT(grid->corners_detected == 0);
  for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
    const int data_row = (tag_id * 4) + corner_idx;
    MU_ASSERT(grid->data[data_row * 6 + 0] == 0);
    MU_ASSERT_FLOAT(grid->data[data_row * 6 + 1], 0.0);
    MU_ASSERT_FLOAT(grid->data[data_row * 6 + 2], 0.0);
  }

  // Clean up
  aprilgrid_free(grid);

  return 0;
}

int test_aprilgrid_save_and_load(void) {
  // Setup
  const int num_rows = 6;
  const int num_cols = 6;
  const real_t tag_size = 0.088;
  const real_t tag_spacing = 0.3;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  // Add tag
  const int tag_id = 5;
  const real_t tag_kps[4][2] = {{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}, {7.0, 8.0}};
  aprilgrid_add_tag(grid, tag_id, tag_kps);

  MU_ASSERT(grid->corners_detected == 4);
  for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
    const int data_row = (tag_id * 4) + corner_idx;
    MU_ASSERT(grid->data[data_row * 6 + 0] == 1);
    MU_ASSERT_FLOAT(grid->data[data_row * 6 + 1], tag_kps[corner_idx][0]);
    MU_ASSERT_FLOAT(grid->data[data_row * 6 + 2], tag_kps[corner_idx][1]);
  }

  // Save
  const int retval = aprilgrid_save(grid, "/tmp/test_aprilgrid.dat");
  MU_ASSERT(retval == 0);

  // Load
  aprilgrid_t *grid_load = aprilgrid_load("/tmp/test_aprilgrid.dat");
  MU_ASSERT(grid_load->num_rows == grid->num_rows);
  MU_ASSERT(grid_load->num_cols == grid->num_cols);
  MU_ASSERT_FLOAT(grid_load->tag_size, grid->tag_size);
  MU_ASSERT_FLOAT(grid_load->tag_spacing, grid->tag_spacing);
  MU_ASSERT(grid_load->corners_detected == grid->corners_detected);
  const int max_corners = (grid->num_rows * grid->num_cols * 4);
  for (int i = 0; i < max_corners; i++) {
    for (int j = 0; j < 6; j++) {
      MU_ASSERT_FLOAT(grid_load->data[i * 6 + j], grid->data[i * 6 + j]);
    }
  }
  // aprilgrid_print(grid_load);
  aprilgrid_free(grid_load);
  aprilgrid_free(grid);

  return 0;
}

#if ENABLE_APRILGRID_DETECTOR == 1

int test_aprilgrid_detector_detect(void) {
  // Load test image
  // -- Load JPG
  const char *test_image = "./test_data/images/aprilgrid_tag36h11.jpg";
  int err = 0;
  pjpeg_t *pjpeg = pjpeg_create_from_file(test_image, 0, &err);
  if (pjpeg == NULL) {
    printf("Failed to load [%s]\n", test_image);
    return -1;
  }
  // -- Convert to single channel 8-bit image
  image_u8_t *im = pjpeg_to_u8_baseline(pjpeg);

  // Detect
  const timestamp_t ts = 0;
  const int num_rows = 10;
  const int num_cols = 10;
  const real_t tag_size = 1.0;
  const real_t tag_spacing = 0.0;

  aprilgrid_detector_t *det =
      aprilgrid_detector_malloc(num_rows, num_cols, tag_size, tag_spacing);
  aprilgrid_t *grid = aprilgrid_detector_detect(det,
                                                ts,
                                                im->width,
                                                im->height,
                                                im->stride,
                                                im->buf);
  MU_ASSERT(grid->corners_detected == 400);
  MU_ASSERT(grid->timestamp == 0);
  MU_ASSERT(grid->num_rows == num_rows);
  MU_ASSERT(grid->num_cols == num_cols);
  MU_ASSERT(fltcmp(grid->tag_size, tag_size) == 0);
  MU_ASSERT(fltcmp(grid->tag_spacing, tag_spacing) == 0);

  // Clean up
  pjpeg_destroy(pjpeg);
  image_u8_destroy(im);
  aprilgrid_detector_free(det);
  aprilgrid_free(grid);

  return 0;
}

#endif // ENABLE_APRILGRID_DETECTOR

/*******************************************************************************
 * STATE-ESTIMATION
 ******************************************************************************/

int test_pose(void) {
  timestamp_t ts = 1;
  pose_t pose;

  real_t data[7] = {0.1, 0.2, 0.3, 1.0, 1.1, 2.2, 3.3};
  pose_setup(&pose, ts, data);

  MU_ASSERT(pose.ts == 1);

  MU_ASSERT(fltcmp(pose.data[0], 0.1) == 0.0);
  MU_ASSERT(fltcmp(pose.data[1], 0.2) == 0.0);
  MU_ASSERT(fltcmp(pose.data[2], 0.3) == 0.0);
  MU_ASSERT(fltcmp(pose.data[3], 1.0) == 0.0);
  MU_ASSERT(fltcmp(pose.data[4], 1.1) == 0.0);
  MU_ASSERT(fltcmp(pose.data[5], 2.2) == 0.0);
  MU_ASSERT(fltcmp(pose.data[6], 3.3) == 0.0);

  return 0;
}

int test_extrinsics(void) {
  extrinsic_t extrinsic;

  real_t data[7] = {1.0, 2.0, 3.0, 1.0, 0.1, 0.2, 0.3};
  extrinsic_setup(&extrinsic, data);

  MU_ASSERT(fltcmp(extrinsic.data[0], 1.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsic.data[1], 2.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsic.data[2], 3.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsic.data[3], 1.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsic.data[4], 0.1) == 0.0);
  MU_ASSERT(fltcmp(extrinsic.data[5], 0.2) == 0.0);
  MU_ASSERT(fltcmp(extrinsic.data[6], 0.3) == 0.0);

  return 0;
}

int test_fiducial(void) {
  fiducial_t fiducial;

  real_t data[7] = {1.0, 2.0, 3.0, 1.0, 0.1, 0.2, 0.3};
  fiducial_setup(&fiducial, data);

  MU_ASSERT(fltcmp(fiducial.data[0], 1.0) == 0.0);
  MU_ASSERT(fltcmp(fiducial.data[1], 2.0) == 0.0);
  MU_ASSERT(fltcmp(fiducial.data[2], 3.0) == 0.0);
  MU_ASSERT(fltcmp(fiducial.data[3], 1.0) == 0.0);
  MU_ASSERT(fltcmp(fiducial.data[4], 0.1) == 0.0);
  MU_ASSERT(fltcmp(fiducial.data[5], 0.2) == 0.0);
  MU_ASSERT(fltcmp(fiducial.data[6], 0.3) == 0.0);

  return 0;
}

int test_imu_biases(void) {
  timestamp_t ts = 1;
  imu_biases_t biases;

  real_t ba[3] = {1.0, 2.0, 3.0};
  real_t bg[3] = {4.0, 5.0, 6.0};
  imu_biases_setup(&biases, ts, ba, bg);

  MU_ASSERT(biases.ts == 1);

  MU_ASSERT(fltcmp(biases.data[0], 1.0) == 0.0);
  MU_ASSERT(fltcmp(biases.data[1], 2.0) == 0.0);
  MU_ASSERT(fltcmp(biases.data[2], 3.0) == 0.0);

  MU_ASSERT(fltcmp(biases.data[3], 4.0) == 0.0);
  MU_ASSERT(fltcmp(biases.data[4], 5.0) == 0.0);
  MU_ASSERT(fltcmp(biases.data[5], 6.0) == 0.0);

  return 0;
}

int test_feature(void) {
  feature_t feature;

  size_t feature_id = 99;
  real_t data[3] = {0.1, 0.2, 0.3};
  feature_init(&feature, feature_id, data);

  MU_ASSERT(feature.feature_id == feature_id);
  MU_ASSERT(fltcmp(feature.data[0], 0.1) == 0.0);
  MU_ASSERT(fltcmp(feature.data[1], 0.2) == 0.0);
  MU_ASSERT(fltcmp(feature.data[2], 0.3) == 0.0);

  return 0;
}

int test_time_delay(void) {
  time_delay_t td;
  time_delay_setup(&td, 1.0);
  MU_ASSERT(fltcmp(td.data[0], 1.0) == 0);
  return 0;
}

int test_joint(void) {
  joint_t joint;
  joint_setup(&joint, 101, 2, 1.0);
  MU_ASSERT(joint.ts == 101);
  MU_ASSERT(joint.joint_idx == 2);
  MU_ASSERT(fltcmp(joint.data[0], 1.0) == 0);
  return 0;
}

int test_camera(void) {
  camera_t camera;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_setup(&camera, cam_idx, cam_res, proj_model, dist_model, data);
  // camera_print(&camera);

  return 0;
}

int test_triangulate_batch(void) {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const int cam_res[2] = {image_width, image_height};
  const char *pmodel = "pinhole";
  const char *dmodel = "radtan4";
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2.0;
  const real_t cy = image_height / 2.0;
  const real_t proj_params[4] = {fx, fy, cx, cy};
  const real_t data[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  camera_t cam_i;
  camera_t cam_j;
  camera_setup(&cam_i, 0, cam_res, pmodel, dmodel, data);
  camera_setup(&cam_j, 1, cam_res, pmodel, dmodel, data);

  // Setup camera pose T_WC0
  const real_t ypr_WC0[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC0[3] = {0.0, 0.0, 0.0};
  real_t T_WC0[4 * 4] = {0};
  tf_euler_set(T_WC0, ypr_WC0);
  tf_trans_set(T_WC0, r_WC0);

  // Setup camera pose T_WC1
  const real_t euler_WC1[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC1[3] = {0.1, 0.1, 0.0};
  real_t T_WC1[4 * 4] = {0};
  tf_euler_set(T_WC1, euler_WC1);
  tf_trans_set(T_WC1, r_WC1);

  // Setup camera extrinsics T_CiCj
  TF_INV(T_WC0, T_C0W);
  TF_CHAIN(T_CiCj, 2, T_C0W, T_WC1);

  // Setup 3D and 2D correspondance points
  int N = 10;
  real_t *kps_i = malloc(sizeof(real_t) * N * 2);
  real_t *kps_j = malloc(sizeof(real_t) * N * 2);
  real_t *points_gnd = malloc(sizeof(real_t) * N * 3);
  real_t *points_est = malloc(sizeof(real_t) * N * 3);
  int *status = malloc(sizeof(int) * N);

  for (int i = 0; i < N; i++) {
    const real_t p_W[3] = {5.0, randf(-1.0, 1.0), randf(-1.0, 1.0)};

    real_t T_C0W[4 * 4] = {0};
    real_t T_C1W[4 * 4] = {0};
    tf_inv(T_WC0, T_C0W);
    tf_inv(T_WC1, T_C1W);

    real_t p_C0[3] = {0};
    real_t p_C1[3] = {0};
    tf_point(T_C0W, p_W, p_C0);
    tf_point(T_C1W, p_W, p_C1);

    real_t z0[2] = {0};
    real_t z1[2] = {0};
    pinhole_project(proj_params, p_C0, z0);
    pinhole_project(proj_params, p_C1, z1);

    kps_i[i * 2 + 0] = z0[0];
    kps_i[i * 2 + 1] = z0[1];

    kps_j[i * 2 + 0] = z1[0];
    kps_j[i * 2 + 1] = z1[1];

    points_gnd[i * 3 + 0] = p_C0[0];
    points_gnd[i * 3 + 1] = p_C0[1];
    points_gnd[i * 3 + 2] = p_C0[2];
  }

  // Test triangulate batch
  triangulate_batch(&cam_i,
                    &cam_j,
                    T_CiCj,
                    kps_i,
                    kps_j,
                    N,
                    points_est,
                    status);
  for (int i = 0; i < N; i++) {
    const real_t *p_gnd = points_gnd + i * 3;
    const real_t *p_est = points_est + i * 3;
    const real_t dx = p_gnd[0] - p_est[0];
    const real_t dy = p_gnd[1] - p_est[1];
    const real_t dz = p_gnd[2] - p_est[2];
    const real_t diff = sqrt(dx * dx + dy * dy + dz * dz);

    MU_ASSERT(diff < 0.01);
    // printf("gnd: (%.2f, %.2f, %.2f), ", p_gnd[0], p_gnd[1], p_gnd[2]);
    // printf("est: (%.2f, %.2f, %.2f), ", p_est[0], p_est[1], p_est[2]);
    // printf("diff: %.2e\n", diff);
  }

  // Clean up
  free(kps_i);
  free(kps_j);
  free(points_gnd);
  free(points_est);
  free(status);

  return 0;
}

int test_pose_factor(void) {
  /* Pose */
  timestamp_t ts = 1;
  pose_t pose;
  real_t data[7] = {0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0};
  pose_setup(&pose, ts, data);

  /* Setup pose factor */
  pose_factor_t factor;
  real_t var[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  pose_factor_setup(&factor, &pose, var);

  /* Check Jacobians */
  const real_t step_size = 1e-8;
  const real_t tol = 1e-4;
  CHECK_FACTOR_J(0, factor, pose_factor_eval, step_size, tol, 0);

  return 0;
}

int test_ba_factor(void) {
  // Timestamp
  timestamp_t ts = 0;

  // Camera pose
  const real_t pose_data[7] = {0.01, 0.01, 0.0, 0.5, -0.5, 0.5, -0.5};
  pose_t pose;
  pose_setup(&pose, ts, pose_data);

  // Feature
  const real_t p_W[3] = {1.0, 0.1, 0.2};
  feature_t feature;
  feature_init(&feature, 0, p_W);

  // Camera parameters
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {320, 240, 320, 240, 0.03, 0.01, 0.001, 0.001};
  camera_t cam;
  camera_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  // Project point from world to image plane
  real_t T_WC[4 * 4] = {0};
  real_t T_CW[4 * 4] = {0};
  real_t p_C[3] = {0.0};
  real_t z[2] = {0.0};
  tf(pose_data, T_WC);
  tf_inv(T_WC, T_CW);
  tf_point(T_CW, p_W, p_C);
  pinhole_radtan4_project(cam_data, p_C, z);

  // Bundle adjustment factor
  ba_factor_t factor;
  real_t var[2] = {1.0, 1.0};
  ba_factor_setup(&factor, &pose, &feature, &cam, z, var);

  // Check Jacobians
  const real_t step_size = 1e-8;
  const real_t tol = 1e-4;
  CHECK_FACTOR_J(0, factor, ba_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(1, factor, ba_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(2, factor, ba_factor_eval, step_size, tol, 0);

  return 0;
}

int test_camera_factor(void) {
  // Timestamp
  timestamp_t ts = 0;

  // Body pose T_WB
  pose_t pose;
  const real_t pose_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  pose_setup(&pose, ts, pose_data);

  // Extrinsic T_BC
  extrinsic_t cam_ext;
  const real_t ext_data[7] = {0.01, 0.02, 0.03, 0.5, 0.5, -0.5, -0.5};
  extrinsic_setup(&cam_ext, ext_data);

  // Feature p_W
  feature_t feature;
  const real_t p_W[3] = {1.0, 0.0, 0.0};
  feature_init(&feature, 0, p_W);

  // Camera parameters
  camera_t cam;
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {320, 240, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  // Project point from world to image plane
  real_t z[2];
  TF(pose_data, T_WB);
  TF(ext_data, T_BCi);
  TF_INV(T_WB, T_BW);
  TF_INV(T_BCi, T_CiB);
  DOT(T_CiB, 4, 4, T_BW, 4, 4, T_CiW);
  TF_POINT(T_CiW, p_W, p_Ci);
  pinhole_radtan4_project(cam_data, p_Ci, z);

  // Setup camera factor
  camera_factor_t factor;
  real_t var[2] = {1.0, 1.0};
  camera_factor_setup(&factor, &pose, &cam_ext, &feature, &cam, z, var);

  // Check Jacobians
  const real_t step_size = 1e-8;
  const real_t tol = 1e-4;
  CHECK_FACTOR_J(0, factor, camera_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(1, factor, camera_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(2, factor, camera_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(3, factor, camera_factor_eval, step_size, tol, 0);

  return 0;
}

int test_imu_buffer_setup(void) {
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);

  return 0;
}

int test_imu_buffer_add(void) {
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buffer_add(&imu_buf, ts, acc, gyr);

  MU_ASSERT(imu_buf.size == 1);
  MU_ASSERT(imu_buf.ts[0] == ts);
  MU_ASSERT(fltcmp(imu_buf.acc[0][0], 1.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][1], 2.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][2], 3.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][0], 1.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][1], 2.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][2], 3.0) == 0);

  return 0;
}

int test_imu_buffer_clear(void) {
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buffer_add(&imu_buf, ts, acc, gyr);
  imu_buffer_clear(&imu_buf);

  MU_ASSERT(imu_buf.size == 0);
  MU_ASSERT(imu_buf.ts[0] == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][0], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][1], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.acc[0][2], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][0], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][1], 0.0) == 0);
  MU_ASSERT(fltcmp(imu_buf.gyr[0][2], 0.0) == 0);

  return 0;
}

int test_imu_buffer_copy(void) {
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buffer_add(&imu_buf, ts, acc, gyr);

  imu_buffer_t imu_buf2;
  imu_buffer_setup(&imu_buf2);
  imu_buffer_copy(&imu_buf, &imu_buf2);

  MU_ASSERT(imu_buf2.size == 1);
  MU_ASSERT(imu_buf2.ts[0] == ts);
  MU_ASSERT(fltcmp(imu_buf2.acc[0][0], 1.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.acc[0][1], 2.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.acc[0][2], 3.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.gyr[0][0], 1.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.gyr[0][1], 2.0) == 0);
  MU_ASSERT(fltcmp(imu_buf2.gyr[0][2], 3.0) == 0);

  return 0;
}

typedef struct imu_test_data_t {
  size_t num_measurements;
  real_t *timestamps;
  real_t **poses;
  real_t **velocities;
  real_t **imu_acc;
  real_t **imu_gyr;
} imu_test_data_t;

static int setup_imu_test_data(imu_test_data_t *test_data,
                               const real_t circle_r,
                               const real_t circle_v) {
  // Circle trajectory configurations
  const real_t imu_rate = 200.0;
  const real_t circle_dist = 2.0 * M_PI * circle_r;
  const real_t time_taken = circle_dist / circle_v;
  const real_t w = -2.0 * M_PI * (1.0 / time_taken);
  const real_t theta_init = M_PI;
  const real_t yaw_init = M_PI / 2.0;

  // Allocate memory for test data
  test_data->num_measurements = time_taken * imu_rate;
  test_data->timestamps = calloc(test_data->num_measurements, sizeof(real_t));
  test_data->poses = calloc(test_data->num_measurements, sizeof(real_t *));
  test_data->velocities = calloc(test_data->num_measurements, sizeof(real_t *));
  test_data->imu_acc = calloc(test_data->num_measurements, sizeof(real_t *));
  test_data->imu_gyr = calloc(test_data->num_measurements, sizeof(real_t *));

  // Simulate IMU poses
  const real_t dt = 1.0 / imu_rate;
  timestamp_t ts = 0.0;
  real_t theta = theta_init;
  real_t yaw = yaw_init;

  for (size_t k = 0; k < test_data->num_measurements; k++) {
    // IMU pose
    // -- Position
    const real_t rx = circle_r * cos(theta);
    const real_t ry = circle_r * sin(theta);
    const real_t rz = 0.0;
    // -- Orientation
    const real_t ypr[3] = {yaw, 0.0, 0.0};
    real_t q[4] = {0};
    euler2quat(ypr, q);
    // -- Pose vector
    const real_t pose[7] = {rx, ry, rz, q[0], q[1], q[2], q[3]};
    // print_vector("pose", pose, 7);

    // Velocity
    const real_t vx = -circle_r * w * sin(theta);
    const real_t vy = circle_r * w * cos(theta);
    const real_t vz = 0.0;
    const real_t v_WS[3] = {vx, vy, vz};

    // Acceleration
    const real_t ax = -circle_r * w * w * cos(theta);
    const real_t ay = -circle_r * w * w * sin(theta);
    const real_t az = 0.0;
    const real_t a_WS[3] = {ax, ay, az};

    // Angular velocity
    const real_t wx = 0.0;
    const real_t wy = 0.0;
    const real_t wz = w;
    const real_t w_WS[3] = {wx, wy, wz};

    // IMU measurements
    real_t C_WS[3 * 3] = {0};
    real_t C_SW[3 * 3] = {0};
    quat2rot(q, C_WS);
    mat_transpose(C_WS, 3, 3, C_SW);
    // -- Accelerometer measurement
    real_t acc[3] = {0};
    dot(C_SW, 3, 3, a_WS, 3, 1, acc);
    acc[2] += 9.81;
    // -- Gyroscope measurement
    real_t gyr[3] = {0};
    dot(C_SW, 3, 3, w_WS, 3, 1, gyr);

    // Update
    test_data->timestamps[k] = ts;
    test_data->poses[k] = vector_malloc(pose, 7);
    test_data->velocities[k] = vector_malloc(v_WS, 3);
    test_data->imu_acc[k] = vector_malloc(acc, 3);
    test_data->imu_gyr[k] = vector_malloc(gyr, 3);

    theta += w * dt;
    yaw += w * dt;
    ts += sec2ts(dt);
  }

  return 0;
}

static void free_imu_test_data(imu_test_data_t *test_data) {
  for (size_t k = 0; k < test_data->num_measurements; k++) {
    free(test_data->poses[k]);
    free(test_data->velocities[k]);
    free(test_data->imu_acc[k]);
    free(test_data->imu_gyr[k]);
  }

  free(test_data->timestamps);
  free(test_data->poses);
  free(test_data->velocities);
  free(test_data->imu_acc);
  free(test_data->imu_gyr);
}

int test_imu_propagate(void) {
  // Setup test data
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data, 1.0, 0.1);

  // Setup IMU buffer
  const int n = 100;
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);
  for (int k = 0; k < n; k++) {
    const timestamp_t ts = test_data.timestamps[k];
    const real_t *acc = test_data.imu_acc[k];
    const real_t *gyr = test_data.imu_gyr[k];
    imu_buffer_add(&imu_buf, ts, acc, gyr);
  }

  // Test imu propagate
  real_t pose_k[7] = {0};
  real_t vel_k[3] = {0};
  real_t pose_kp1[7] = {0};
  real_t vel_kp1[3] = {0};

  vec_copy(test_data.poses[0], 7, pose_k);
  vec_copy(test_data.velocities[0], 3, vel_k);
  imu_propagate(pose_k, vel_k, &imu_buf, pose_kp1, vel_kp1);

  real_t dr[3] = {0};
  real_t dtheta = 0;
  pose_diff2(test_data.poses[n], pose_kp1, dr, &dtheta);

  const real_t tol = 1e-3;
  MU_ASSERT(fabs(dr[0]) < tol);
  MU_ASSERT(fabs(dr[1]) < tol);
  MU_ASSERT(fabs(dr[2]) < tol);
  MU_ASSERT(fabs(dtheta) < tol);

  // Clean up
  free_imu_test_data(&test_data);

  return 0;
}

int test_imu_initial_attitude(void) {
  // Setup test data
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data, 5.0, 1.0);

  // Setup IMU buffer
  const int n = 1;
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);
  for (int k = 0; k < n; k++) {
    const timestamp_t ts = test_data.timestamps[k];
    const real_t *acc = test_data.imu_acc[k];
    const real_t *gyr = test_data.imu_gyr[k];
    imu_buffer_add(&imu_buf, ts, acc, gyr);
  }

  // Test imu initial attitude
  real_t q_WS[4] = {0};
  imu_initial_attitude(&imu_buf, q_WS);

  // Clean up
  free_imu_test_data(&test_data);

  return 0;
}

// static void imu_propagate_step(const real_t x_km1[16], real_t x_k[16]) {
//   // Setup
//   const real_t a_i[3] = {0.1, 0.1, 0.1};
//   const real_t a_j[3] = {0.2, 0.2, 0.2};
//   const real_t w_i[3] = {0.1, 0.1, 0.1};
//   const real_t w_j[3] = {0.2, 0.2, 0.2};
//   const real_t dt = 0.01;
//   const real_t dt_sq = dt * dt;

//   const real_t *r_i = x_km1 + 0;
//   const real_t *q_i = x_km1 + 3;
//   const real_t *v_i = x_km1 + 7;
//   const real_t *ba_i = x_km1 + 10;
//   const real_t *bg_i = x_km1 + 13;

//   // Gyroscope measurement
//   const real_t wx = 0.5 * (w_i[0] + w_j[0]) - bg_i[0];
//   const real_t wy = 0.5 * (w_i[1] + w_j[1]) - bg_i[1];
//   const real_t wz = 0.5 * (w_i[2] + w_j[2]) - bg_i[2];
//   const real_t dq[4] = {1.0, 0.5 * wx * dt, 0.5 * wy * dt, 0.5 * wz * dt};

//   // Update orientation
//   real_t q_j[4] = {0};
//   quat_mul(q_i, dq, q_j);
//   quat_normalize(q_j);

//   // Accelerometer measurement
//   const real_t a_ii[3] = {a_i[0] - ba_i[0], a_i[1] - ba_i[1], a_i[2] - ba_i[2]};
//   const real_t a_jj[3] = {a_j[0] - ba_i[0], a_j[1] - ba_i[1], a_j[2] - ba_i[2]};
//   real_t acc_i[3] = {0};
//   real_t acc_j[3] = {0};
//   quat_transform(q_i, a_ii, acc_i);
//   quat_transform(q_j, a_jj, acc_j);
//   real_t a[3] = {0};
//   a[0] = 0.5 * (acc_i[0] + acc_j[0]);
//   a[1] = 0.5 * (acc_i[1] + acc_j[1]);
//   a[2] = 0.5 * (acc_i[2] + acc_j[2]);

//   // Update position:
//   // r_j = r_i + (v_i * dt) + (0.5 * a * dt_sq)
//   real_t r_j[3] = {0};
//   r_j[0] = r_i[0] + (v_i[0] * dt) + (0.5 * a[0] * dt_sq);
//   r_j[1] = r_i[1] + (v_i[1] * dt) + (0.5 * a[1] * dt_sq);
//   r_j[2] = r_i[2] + (v_i[2] * dt) + (0.5 * a[2] * dt_sq);

//   // Update velocity:
//   // v_j = v_i + a * dt
//   real_t v_j[3] = {0};
//   v_j[0] = v_i[0] + a[0] * dt;
//   v_j[1] = v_i[1] + a[1] * dt;
//   v_j[2] = v_i[2] + a[2] * dt;

//   // Update biases
//   // ba_j = ba_i;
//   // bg_j = bg_i;
//   real_t ba_j[3] = {0};
//   real_t bg_j[3] = {0};
//   vec_copy(ba_i, 3, ba_j);
//   vec_copy(bg_i, 3, bg_j);

//   // Write outputs
//   imu_state_vector(r_j, q_j, v_j, ba_j, bg_j, x_k);
// }

int test_imu_factor_form_F_matrix(void) {
  // Setup test data
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data, 1.0, 0.1);

  // Setup IMU buffer
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);
  for (int k = 0; k < 10; k++) {
    const timestamp_t ts = test_data.timestamps[k];
    const real_t *acc = test_data.imu_acc[k];
    const real_t *gyr = test_data.imu_gyr[k];
    imu_buffer_add(&imu_buf, ts, acc, gyr);
  }

  // Setup IMU factor
  const int idx_i = 0;
  const int idx_j = 1;
  const timestamp_t ts_i = test_data.timestamps[idx_i];
  const timestamp_t ts_j = test_data.timestamps[idx_j];
  const real_t *v_i = test_data.velocities[idx_i];
  const real_t ba_i[3] = {0.0, 0.0, 0.0};
  const real_t bg_i[3] = {0.0, 0.0, 0.0};
  const real_t *v_j = test_data.velocities[idx_j];
  const real_t ba_j[3] = {0.0, 0.0, 0.0};
  const real_t bg_j[3] = {0.0, 0.0, 0.0};
  pose_t pose_i;
  pose_t pose_j;
  velocity_t vel_i;
  velocity_t vel_j;
  imu_biases_t biases_i;
  imu_biases_t biases_j;
  pose_setup(&pose_i, ts_i, test_data.poses[idx_i]);
  pose_setup(&pose_j, ts_j, test_data.poses[idx_j]);
  velocity_setup(&vel_i, ts_i, v_i);
  velocity_setup(&vel_j, ts_j, v_j);
  imu_biases_setup(&biases_i, ts_i, ba_i, bg_i);
  imu_biases_setup(&biases_j, ts_j, ba_j, bg_j);

  // imu_params_t imu_params;
  // imu_params.imu_idx = 0;
  // imu_params.rate = 200.0;
  // imu_params.sigma_a = 0.08;
  // imu_params.sigma_g = 0.004;
  // imu_params.sigma_aw = 0.00004;
  // imu_params.sigma_gw = 2.0e-6;
  // imu_params.g = 9.81;

  // Test form F Matrix
  const int k = idx_j;
  const real_t *q_i = pose_i.data + 3;
  const real_t *q_j = pose_j.data + 3;
  const real_t dt = ts2sec(ts_j) - ts2sec(ts_i);
  const real_t *a_i = imu_buf.acc[k - 1];
  const real_t *w_i = imu_buf.gyr[k - 1];
  const real_t *a_j = imu_buf.acc[k];
  const real_t *w_j = imu_buf.gyr[k];
  real_t F_dt[15 * 15] = {0};
  imu_factor_F_matrix(q_i, q_j, ba_i, bg_i, a_i, w_i, a_j, w_j, dt, F_dt);
  // mat_save("/tmp/F.csv", F_dt, 15, 15);

  // Clean up
  free_imu_test_data(&test_data);

  return 0;
}

int test_imu_factor(void) {
  // Setup test data
  const double circle_r = 1.0;
  const double circle_v = 0.1;
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data, circle_r, circle_v);

  // Setup IMU buffer
  int buf_size = 100;
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);
  for (int k = 0; k < buf_size; k++) {
    const timestamp_t ts = test_data.timestamps[k];
    const real_t *acc = test_data.imu_acc[k];
    const real_t *gyr = test_data.imu_gyr[k];
    imu_buffer_add(&imu_buf, ts, acc, gyr);
  }

  // Setup IMU factor
  const int idx_i = 0;
  const int idx_j = buf_size - 1;
  const timestamp_t ts_i = test_data.timestamps[idx_i];
  const timestamp_t ts_j = test_data.timestamps[idx_j];
  const real_t *v_i = test_data.velocities[idx_i];
  const real_t ba_i[3] = {0.0, 0.0, 0.0};
  const real_t bg_i[3] = {0.0, 0.0, 0.0};
  const real_t *v_j = test_data.velocities[idx_j];
  const real_t ba_j[3] = {0.0, 0.0, 0.0};
  const real_t bg_j[3] = {0.0, 0.0, 0.0};
  pose_t pose_i;
  pose_t pose_j;
  velocity_t vel_i;
  velocity_t vel_j;
  imu_biases_t biases_i;
  imu_biases_t biases_j;
  pose_setup(&pose_i, ts_i, test_data.poses[idx_i]);
  pose_setup(&pose_j, ts_j, test_data.poses[idx_j]);
  velocity_setup(&vel_i, ts_i, v_i);
  velocity_setup(&vel_j, ts_j, v_j);
  imu_biases_setup(&biases_i, ts_i, ba_i, bg_i);
  imu_biases_setup(&biases_j, ts_j, ba_j, bg_j);

  imu_params_t imu_params;
  imu_params.imu_idx = 0;
  imu_params.rate = 200.0;
  imu_params.sigma_a = 0.08;
  imu_params.sigma_g = 0.004;
  imu_params.sigma_aw = 0.00004;
  imu_params.sigma_gw = 2.0e-6;
  imu_params.g = 9.81;

  pose_j.data[0] += 0.01;
  pose_j.data[1] += 0.02;
  pose_j.data[2] += 0.03;

  imu_factor_t factor;
  imu_factor_setup(&factor,
                   &imu_params,
                   &imu_buf,
                   &pose_i,
                   &vel_i,
                   &biases_i,
                   &pose_j,
                   &vel_j,
                   &biases_j);
  imu_factor_eval(&factor);

  MU_ASSERT(factor.pose_i == &pose_i);
  MU_ASSERT(factor.vel_i == &vel_i);
  MU_ASSERT(factor.biases_i == &biases_i);
  MU_ASSERT(factor.pose_i == &pose_i);
  MU_ASSERT(factor.vel_j == &vel_j);
  MU_ASSERT(factor.biases_j == &biases_j);

  // Check Jacobians
  const double tol = 1e-4;
  const double step_size = 1e-8;
  eye(factor.sqrt_info, 15, 15);
  CHECK_FACTOR_J(0, factor, imu_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(1, factor, imu_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(2, factor, imu_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(3, factor, imu_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(4, factor, imu_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(5, factor, imu_factor_eval, step_size, tol, 0);

  // Clean up
  free_imu_test_data(&test_data);

  return 0;
}

int test_joint_factor(void) {
  // Joint angle
  const timestamp_t ts = 0;
  const int joint_idx = 0;
  const real_t z = 0.01;
  joint_t joint;
  joint_setup(&joint, ts, joint_idx, z);

  // Joint angle factor
  joint_factor_t factor;
  const real_t var = 0.1;
  joint_factor_setup(&factor, &joint, z, var);

  // Evaluate
  joint_factor_eval(&factor);

  // Check Jacobians
  const double tol = 1e-4;
  const double step_size = 1e-8;
  CHECK_FACTOR_J(0, factor, joint_factor_eval, step_size, tol, 0);

  return 0;
}

typedef struct test_calib_camera_data_t {
  real_t T_WF[4 * 4];
  real_t T_WB[4 * 4];
  real_t T_BF[4 * 4];
  real_t T_BCi[4 * 4];

  pose_t fiducial;     // T_WF
  pose_t pose;         // T_WB
  pose_t rel_pose;     // T_BF
  extrinsic_t cam_ext; // T_BCi
  camera_t camera;

  int cam_idx;
  int tag_id;
  int corner_idx;
  real_t p_FFi[3];
  real_t z[2];
} test_calib_camera_data_t;

void test_calib_camera_data_setup(test_calib_camera_data_t *data) {
  // Calibration target pose T_WF
  real_t fiducial_data[7] = {0};
  real_t ypr_WF[3] = {-M_PI / 2.0, 0.0, M_PI / 2.0};
  real_t r_WF[3] = {0.01, 0.01, 0.01};
  tf_er(ypr_WF, r_WF, data->T_WF);
  tf_vector(data->T_WF, fiducial_data);
  pose_setup(&data->fiducial, 0, fiducial_data);

  // Body pose T_WB
  real_t pose_data[7] = {0};
  real_t ypr_WB[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  real_t r_WB[3] = {-10.0, 0.001, 0.001};
  tf_er(ypr_WB, r_WB, data->T_WB);
  tf_vector(data->T_BF, pose_data);
  pose_setup(&data->pose, 0, pose_data);

  // Relative pose T_BF
  real_t rel_pose_data[7] = {0};
  TF_INV(data->T_WB, T_BW);
  tf_chain2(2, T_BW, data->T_WF, data->T_BF);
  tf_vector(data->T_BF, rel_pose_data);
  pose_setup(&data->rel_pose, 0, rel_pose_data);

  // Camera extrinsics T_BCi
  real_t cam_ext_data[7] = {0};
  real_t ypr_BCi[3] = {0.01, 0.01, 0.0};
  real_t r_BCi[3] = {0.001, 0.001, 0.001};
  tf_er(ypr_BCi, r_BCi, data->T_BCi);
  tf_vector(data->T_BCi, cam_ext_data);
  extrinsic_setup(&data->cam_ext, cam_ext_data);

  // Camera
  data->cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const real_t fov = 90.0;
  const real_t fx = pinhole_focal(cam_res[0], fov);
  const real_t fy = pinhole_focal(cam_res[0], fov);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  const real_t cam_data[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  camera_setup(&data->camera,
                      data->cam_idx,
                      cam_res,
                      proj_model,
                      dist_model,
                      cam_data);

  // Project to image plane
  int num_rows = 6;
  int num_cols = 6;
  double tag_size = 0.088;
  double tag_spacing = 0.3;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  data->tag_id = 1;
  data->corner_idx = 2;
  aprilgrid_object_point(grid, data->tag_id, data->corner_idx, data->p_FFi);

  TF_INV(data->T_BCi, T_CiB);
  TF_CHAIN(T_CiF, 2, T_CiB, data->T_BF);
  TF_POINT(T_CiF, data->p_FFi, p_CiFi);
  pinhole_radtan4_project(cam_data, p_CiFi, data->z);

  aprilgrid_free(grid);
}

int test_camchain(void) {
  // Form camera poses
  int num_cams = 5;
  real_t T_C0F[4 * 4] = {0};
  real_t T_C1F[4 * 4] = {0};
  real_t T_C2F[4 * 4] = {0};
  real_t T_C3F[4 * 4] = {0};
  real_t T_C4F[4 * 4] = {0};
  real_t *poses[5] = {T_C0F, T_C1F, T_C2F, T_C3F, T_C4F};

  for (int k = 0; k < num_cams; k++) {
    const real_t ypr[3] = {randf(-90, 90), randf(-90, 90), randf(-90, 90)};
    const real_t r[3] = {randf(-1, 1), randf(-1, 1), randf(-1, 1)};
    tf_er(ypr, r, poses[k]);
  }

  TF_INV(T_C0F, T_FC0);
  TF_INV(T_C1F, T_FC1);
  TF_INV(T_C2F, T_FC2);
  TF_INV(T_C3F, T_FC3);
  TF_INV(T_C4F, T_FC4);
  real_t *poses_inv[5] = {T_FC0, T_FC1, T_FC2, T_FC3, T_FC4};

  // Camchain
  camchain_t *camchain = camchain_malloc(num_cams);
  camchain_add_pose(camchain, 0, 0, T_C0F);
  camchain_add_pose(camchain, 1, 0, T_C1F);
  camchain_add_pose(camchain, 2, 0, T_C2F);
  camchain_add_pose(camchain, 3, 0, T_C3F);
  camchain_adjacency(camchain);
  // camchain_adjacency_print(camchain);

  for (int cam_i = 1; cam_i < num_cams; cam_i++) {
    for (int cam_j = 1; cam_j < num_cams; cam_j++) {
      // Get ground-truth
      TF_CHAIN(T_CiCj_gnd, 2, poses[cam_i], poses_inv[cam_j]);

      // Get camchain result
      real_t T_CiCj_est[4 * 4] = {0};
      int status = camchain_find(camchain, cam_i, cam_j, T_CiCj_est);

      if (cam_i != 4 && cam_j != 4) { // Camera 4 was not added
        MU_ASSERT(status == 0);
      } else {
        MU_ASSERT(status == -1);
      }
    }
  }

  // Clean up
  camchain_free(camchain);

  return 0;
}

int test_calib_camera_factor(void) {
  // Setup
  test_calib_camera_data_t calib_data;
  test_calib_camera_data_setup(&calib_data);

  struct calib_camera_factor_t factor;
  const real_t var[2] = {1.0, 1.0};
  calib_camera_factor_setup(&factor,
                            &calib_data.rel_pose,
                            &calib_data.cam_ext,
                            &calib_data.camera,
                            calib_data.cam_idx,
                            calib_data.tag_id,
                            calib_data.corner_idx,
                            calib_data.p_FFi,
                            calib_data.z,
                            var);

  // Evaluate
  calib_camera_factor_eval(&factor);

  // Check Jacobians
  const double tol = 1e-4;
  const double step_size = 1e-8;
  CHECK_FACTOR_J(0, factor, calib_camera_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(1, factor, calib_camera_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(2, factor, calib_camera_factor_eval, step_size, tol, 0);

  return 0;
}

typedef struct test_calib_imucam_data_t {
  real_t T_WF[4 * 4];
  real_t T_WS[4 * 4];
  real_t T_SC0[4 * 4];
  real_t T_C0Ci[4 * 4];

  fiducial_t fiducial; // T_WF
  pose_t imu_pose;     // T_WB
  extrinsic_t imu_ext; // T_SC0
  extrinsic_t cam_ext; // T_C0Ci
  camera_t camera;
  time_delay_t time_delay;

  int cam_idx;
  int tag_id;
  int corner_idx;
  real_t p_FFi[3];
  real_t z[2];
} test_calib_imucam_data_t;

void test_calib_imucam_data_setup(test_calib_imucam_data_t *data) {
  // Calibration target pose T_WF
  real_t fiducial_data[7] = {0};
  real_t ypr_WF[3] = {-M_PI / 2.0, 0.0, M_PI / 2.0};
  real_t r_WF[3] = {0.01, 0.01, 0.01};
  tf_er(ypr_WF, r_WF, data->T_WF);
  tf_vector(data->T_WF, fiducial_data);
  fiducial_setup(&data->fiducial, fiducial_data);

  // IMU pose T_WS
  real_t imu_pose_data[7] = {0};
  real_t ypr_WS[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  real_t r_WS[3] = {-10.0, 0.001, 0.001};
  tf_er(ypr_WS, r_WS, data->T_WS);
  tf_vector(data->T_WS, imu_pose_data);
  pose_setup(&data->imu_pose, 0, imu_pose_data);

  // IMU extrinsics T_SC0
  real_t imu_ext_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  tf(imu_ext_data, data->T_SC0);
  extrinsic_setup(&data->imu_ext, imu_ext_data);

  // Camera extrinsics T_C0Ci
  real_t cam_ext_data[7] = {0};
  real_t ypr_C0Ci[3] = {0.01, 0.01, 0.0};
  real_t r_C0Ci[3] = {0.001, 0.001, 0.001};
  tf_er(ypr_C0Ci, r_C0Ci, data->T_C0Ci);
  tf_vector(data->T_C0Ci, cam_ext_data);
  extrinsic_setup(&data->cam_ext, cam_ext_data);

  // Camera
  data->cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const real_t fov = 90.0;
  const real_t fx = pinhole_focal(cam_res[0], fov);
  const real_t fy = pinhole_focal(cam_res[0], fov);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  const real_t cam_data[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  camera_setup(&data->camera,
                      data->cam_idx,
                      cam_res,
                      proj_model,
                      dist_model,
                      cam_data);

  // Time delay
  time_delay_setup(&data->time_delay, 0.0);

  // Project to image plane
  int num_rows = 6;
  int num_cols = 6;
  double tag_size = 0.088;
  double tag_spacing = 0.3;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  data->tag_id = 1;
  data->corner_idx = 2;
  aprilgrid_object_point(grid, data->tag_id, data->corner_idx, data->p_FFi);

  TF_INV(data->T_WS, T_SW);
  TF_INV(data->T_SC0, T_C0S);
  TF_INV(data->T_C0Ci, T_CiC0);
  TF_CHAIN(T_CiF, 4, T_CiC0, T_C0S, T_SW, data->T_WF);
  TF_POINT(T_CiF, data->p_FFi, p_CiFi);
  pinhole_radtan4_project(cam_data, p_CiFi, data->z);

  aprilgrid_free(grid);
}

int test_calib_imucam_factor(void) {
  // Setup
  test_calib_imucam_data_t calib_data;
  test_calib_imucam_data_setup(&calib_data);

  calib_imucam_factor_t factor;
  const real_t var[2] = {1.0, 1.0};
  const real_t v[2] = {0.01, 0.02};
  calib_imucam_factor_setup(&factor,
                            &calib_data.fiducial,
                            &calib_data.imu_pose,
                            &calib_data.imu_ext,
                            &calib_data.cam_ext,
                            &calib_data.camera,
                            &calib_data.time_delay,
                            calib_data.cam_idx,
                            calib_data.tag_id,
                            calib_data.corner_idx,
                            calib_data.p_FFi,
                            calib_data.z,
                            v,
                            var);

  // Evaluate
  calib_imucam_factor_eval(&factor);

  // Check Jacobians
  const double tol = 1e-2;
  const double step_size = 1e-8;
  const int debug = 0;
  CHECK_FACTOR_J(0, factor, calib_imucam_factor_eval, step_size, tol, debug);
  CHECK_FACTOR_J(1, factor, calib_imucam_factor_eval, step_size, tol, debug);
  CHECK_FACTOR_J(2, factor, calib_imucam_factor_eval, step_size, tol, debug);
  CHECK_FACTOR_J(3, factor, calib_imucam_factor_eval, step_size, tol, debug);
  CHECK_FACTOR_J(4, factor, calib_imucam_factor_eval, step_size, tol, debug);
  CHECK_FACTOR_J(5, factor, calib_imucam_factor_eval, step_size, tol, debug);

  return 0;
}

int test_marg_factor(void) {
  // Timestamp
  timestamp_t ts = 0;

  // Extrinsic T_BC
  extrinsic_t cam_ext;
  const real_t ext_data[7] = {0.01, 0.02, 0.03, 0.5, 0.5, -0.5, -0.5};
  extrinsic_setup(&cam_ext, ext_data);
  cam_ext.fix = 1;

  // Camera parameters
  camera_t cam;
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {320, 240, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  // Setup features and poses
  int num_poses = 5;
  int num_features = 10;
  pose_t poses[5] = {0};
  feature_t features[10] = {0};
  real_t points[10 * 3] = {0};
  real_t keypoints[10 * 2 * 5] = {0};
  camera_factor_t factors[5 * 10] = {0};

  for (int i = 0; i < num_features; ++i) {
    const real_t dx = randf(-0.5, 0.5);
    const real_t dy = randf(-0.5, 0.5);
    const real_t dz = randf(-0.5, 0.5);
    const real_t p_W[3] = {3.0 + dx, 0.0 + dy, 0.0 + dz};
    feature_t *feature = &features[i];
    feature_init(feature, 0, p_W);
    points[i * 3 + 0] = p_W[0];
    points[i * 3 + 1] = p_W[1];
    points[i * 3 + 2] = p_W[2];
  }

  int factor_idx = 0;
  for (int k = 0; k < num_poses; ++k) {
    // Body pose T_WB
    const real_t dx = randf(-0.05, 0.05);
    const real_t dy = randf(-0.05, 0.05);
    const real_t dz = randf(-0.05, 0.05);

    const real_t droll = randf(-0.2, 0.2);
    const real_t dpitch = randf(-0.2, 0.2);
    const real_t dyaw = randf(-0.1, 0.1);
    const real_t ypr[3] = {dyaw, dpitch, droll};
    real_t q[4] = {0};
    euler2quat(ypr, q);

    pose_t *pose = &poses[k];
    real_t pose_data[7] = {dx, dy, dz, q[0], q[1], q[2], q[3]};
    pose_setup(pose, ts + k, pose_data);
    pose->marginalize = (k == 0) ? 1 : 0; // Marginalize 1st pose

    for (int i = 0; i < num_features; i++) {
      // Project point from world to image plane
      real_t *p_W = &points[i * 3];
      TF(pose_data, T_WB);
      TF(ext_data, T_BCi);
      TF_INV(T_BCi, T_CiB);
      TF_INV(T_WB, T_BW);
      DOT(T_CiB, 4, 4, T_BW, 4, 4, T_CiW);
      TF_POINT(T_CiW, p_W, p_Ci);

      real_t z[2];
      pinhole_radtan4_project(cam_data, p_Ci, z);
      keypoints[i * 2 + 0] = z[0] + 0.001;
      keypoints[i * 2 + 1] = z[1] - 0.001;

      // Setup camera factor
      camera_factor_t *cam_factor = &factors[factor_idx];
      feature_t *feature = &features[i];
      real_t var[2] = {1.0, 1.0};
      camera_factor_setup(cam_factor, pose, &cam_ext, feature, &cam, z, var);
      camera_factor_eval(cam_factor);
      factor_idx++;
    }
  }
  UNUSED(keypoints);

  // Determine parameter order
  rbt_t *param_index = rbt_malloc(default_cmp);
  int col_idx = 0;
  for (int i = 0; i < num_poses; i++) {
    param_index_add_pose(param_index, &poses[i], &col_idx);
  }
  for (int i = 0; i < num_features; i++) {
    param_index_add_feature(param_index, &features[i], &col_idx);
  }
  param_index_add_extrinsic(param_index, &cam_ext, &col_idx);
  param_index_add_camera(param_index, &cam, &col_idx);
  // -- Misc
  const int sv_size = col_idx;
  const int r_size = (factor_idx * 2);

  // param_index_print(param_index);
  // printf("col_idx: %d\n", col_idx);

  // Form Hessian **before** marginalization
  int r_idx = 0;
  real_t *H = calloc(sv_size * sv_size, sizeof(real_t));
  real_t *g = calloc(sv_size, sizeof(real_t));
  real_t *r = calloc(r_size, sizeof(real_t));
  for (int i = 0; i < (num_poses * num_features); i++) {
    camera_factor_t *factor = &factors[i];
    camera_factor_eval(factor);
    vec_copy(factor->r, factor->r_size, &r[r_idx]);

    solver_fill_hessian(param_index,
                        factor->num_params,
                        factor->params,
                        factor->jacs,
                        factor->r,
                        factor->r_size,
                        sv_size,
                        H,
                        g);
    r_idx += factor->r_size;
  }

  // Setup marginalization factor
  marg_factor_t *marg = marg_factor_malloc();
  for (int i = 0; i < (num_poses * num_features); i++) {
    marg_factor_add(marg, CAMERA_FACTOR, &factors[i]);
  }
  marg_factor_marginalize(marg);
  marg_factor_eval(marg);

  // Print timings
  // printf("marg->time_hessian_form:     %.4fs\n", marg->time_hessian_form);
  // printf("marg->time_schur_complement: %.4fs\n", marg->time_schur_complement);
  // printf("marg->time_hessian_decomp:   %.4fs\n", marg->time_hessian_decomp);
  // printf("marg->time_fejs:             %.4fs\n", marg->time_fejs);
  // printf("------------------------------------\n");
  // printf("marg->time_total:            %.4fs\n", marg->time_total);

  // // Determine parameter order for the marginalized Hessian
  // rbt_t *param_index_ = param_index_malloc();
  // int col_idx_ = 0;
  // // -- Add body poses
  // for (int i = 0; i < num_poses; i++) {
  //   void *data = poses[i].data;
  //   const int fix = poses[i].marginalize;
  //   param_order_add(&hash_, fix, data, &col_idx_);
  // }
  // // -- Add points
  // for (int i = 0; i < num_features; i++) {
  //   void *data = &features[i].data;
  //   const int fix = 0;
  //   param_order_add(&hash_, fix, data, &col_idx_);
  // }
  // // -- Add camera extrinsic
  // {
  //   void *data = cam_ext.data;
  //   const int fix = 1;
  //   param_order_add(&hash_, fix, data, &col_idx_);
  // }
  // // -- Add camera parameters
  // {
  //   void *data = cam.data;
  //   const int fix = 0;
  //   param_order_add(&hash_, fix, data, &col_idx_);
  // }
  // // -- Misc
  // const int sv_size_ = col_idx_;
  //
  // // Form marg hessian
  // real_t *H_ = calloc(sv_size_ * sv_size_, sizeof(real_t));
  // real_t *g_ = calloc(sv_size_ * 1, sizeof(real_t));
  // solver_fill_hessian(hash_,
  //                     marg->num_params,
  //                     marg->params,
  //                     marg->jacs,
  //                     marg->r,
  //                     marg->r_size,
  //                     sv_size_,
  //                     H_,
  //                     g_);

  // Clean up
  free(H);
  free(g);
  free(r);
  param_index_free(param_index);
  marg_factor_free(marg);

  // param_order_free(hash_);
  // free(H_);
  // free(g_);

  return 0;
}

/*******************************************************************************
 * TIMELINE
 ******************************************************************************/

// int test_timeline(void) {
//   const char *data_dir = TEST_IMU_APRIL;
//   const int num_cams = 2;
//   const int num_imus = 1;
//   timeline_t *timeline = timeline_load_data(data_dir, num_cams, num_imus);
//   // printf("timeline->num_cams: %d\n", timeline->num_cams);
//   // printf("timeline->num_imus: %d\n", timeline->num_imus);
//   // printf("timeline->num_event_types: %d\n", timeline->num_event_types);
//
//   FILE *imu_file = fopen("/tmp/imu.csv", "w");
//
//   for (int k = 0; k < timeline->timeline_length; k++) {
//     // Extract timeline events. Add either imu or fiducial event
//     for (int i = 0; i < timeline->timeline_events_lengths[k]; i++) {
//       timeline_event_t *event = timeline->timeline_events[k][i];
//       // const timestamp_t ts = event->ts;
//
//       if (event->type == IMU_EVENT) {
//         const imu_event_t *data = &event->data.imu;
//         // printf("imu_ts: %ld ", data->ts);
//         // printf("acc: (%f, %f, %f) ", data->acc[0], data->acc[1], data->acc[2]);
//         // printf("gyr: (%f, %f, %f) ", data->gyr[0], data->gyr[1], data->gyr[2]);
//         // printf("\n");
//
//         fprintf(imu_file, "%ld,", data->ts);
//         fprintf(imu_file,
//                 "%lf,%lf,%lf,",
//                 data->gyr[0],
//                 data->gyr[1],
//                 data->gyr[2]);
//         fprintf(imu_file,
//                 "%lf,%lf,%lf",
//                 data->acc[0],
//                 data->acc[1],
//                 data->acc[2]);
//         fprintf(imu_file, "\n");
//
//       } else if (event->type == FIDUCIAL_EVENT) {
//         // const fiducial_event_t *data = &event->data.fiducial;
//         // const int cam_idx = data->cam_idx;
//         // printf("cam_ts: %ld \n", data->ts);
//         // printf("  cam_idx: %d\n", data->cam_idx);
//         // printf("  num_corners: %d\n", data->num_corners);
//         // for (int i = 0; i < data->num_corners; i++) {
//         //   const real_t *p = data->object_points + i * 3;
//         //   const real_t *z = data->keypoints + i * 2;
//
//         //   printf("  ");
//         //   printf("%d, ", data->tag_ids[i]);
//         //   printf("%d, ", data->corner_indices[i]);
//         //   printf("%f, %f, %f, ", p[0], p[1], p[2]);
//         //   printf("%f, %f", z[0], z[1]);
//         //   printf("\n");
//         // }
//       }
//     }
//   }
//
//   // Clean up
//   timeline_free(timeline);
//   fclose(imu_file);
//
//   return 0;
// }

/*******************************************************************************
 * OCTREE
 ******************************************************************************/

int test_octree_node(void) {
  const float center[3] = {0.0, 0.0, 0.0};
  const float size = 1.0;
  const int depth = 0;
  const int max_depth = 10;
  const int max_points = 100;

  octree_node_t *n =
      octree_node_malloc(center, size, depth, max_depth, max_points);
  octree_node_free(n);

  return 0;
}

int test_octree(void) {
  // Setup
  const float octree_center[3] = {0.0, 0.0, 0.0};
  const float octree_size = 1.0;
  const int octree_max_depth = 3;
  const int voxel_max_points = 5;

  const int n = 2000;
  float *octree_points = malloc(sizeof(float) * 3 * n);
  for (int i = 0; i < n; ++i) {
    const float x = randf(-1.0, 1.0);
    const float y = randf(-1.0, 1.0);
    const float z = randf(-1.0, 1.0);
    octree_points[i * 3 + 0] = x;
    octree_points[i * 3 + 1] = y;
    octree_points[i * 3 + 2] = z;
  }

  // Build octree
  octree_t *octree = octree_malloc(octree_center,
                                   octree_size,
                                   octree_max_depth,
                                   voxel_max_points,
                                   octree_points,
                                   n);

  // Clean up
  free(octree_points);
  octree_free(octree);

  return 0;
}

int test_octree_points(void) {
  // Setup
  const float octree_center[3] = {0.0, 0.0, 0.0};
  const float octree_size = 1.0;
  const int voxel_max_points = 100;
  const int octree_max_depth = 8;

  const int n = 2000;
  float *octree_data = malloc(sizeof(float) * 3 * n);
  for (int i = 0; i < n; ++i) {
    const float x = randf(-1.0, 1.0);
    const float y = randf(-1.0, 1.0);
    const float z = randf(-1.0, 1.0);
    octree_data[i * 3 + 0] = x;
    octree_data[i * 3 + 1] = y;
    octree_data[i * 3 + 2] = z;
  }

  // Build octree
  octree_t *octree = octree_malloc(octree_center,
                                   octree_size,
                                   octree_max_depth,
                                   voxel_max_points,
                                   octree_data,
                                   n);

  // Get points
  octree_data_t data = {0};
  data.points = malloc(sizeof(float) * 3 * n);
  data.num_points = 0;
  data.capacity = n;
  octree_points(octree->root, &data);

  // Assert
  MU_ASSERT(data.num_points == n);

  // Clean up
  free(octree_data);
  octree_free(octree);
  free(data.points);

  return 0;
}

int test_octree_downsample(void) {
  // Setup
  const int n = 2000;
  float offset[3] = {10.0f, 20.0f, 30.0f};
  float *points = malloc(sizeof(float) * 3 * n);
  for (int i = 0; i < n; ++i) {
    const float x = randf(-1.0 + offset[0], 1.0 + offset[0]);
    const float y = randf(-1.0 + offset[1], 1.0 + offset[1]);
    const float z = randf(-1.0 + offset[2], 1.0 + offset[2]);
    points[i * 3 + 0] = x;
    points[i * 3 + 1] = y;
    points[i * 3 + 2] = z;
  }

  // Downsample
  float voxel_size = 0.5;
  size_t voxel_limit = 10;
  size_t n_out = 0;
  float *points_out =
      octree_downsample(points, n, voxel_size, voxel_limit, &n_out);

  // Assert
  MU_ASSERT(n_out < n);

  // Clean up
  free(points);
  free(points_out);

  return 0;
}

/*******************************************************************************
 * KD-TREE
 ******************************************************************************/

static int vec3_equals(const float src[3], const float target[3]) {
  const float eps = 1e-3;
  for (int i = 0; i < 3; ++i) {
    if (fabs(src[i] - target[i]) >= eps) {
      return 0;
    }
  }
  return 1;
}

static int point_cmp(const void *a, const void *b, void *arg) {
  const float *vecA = (const float *) a;
  const float *vecB = (const float *) b;
  const int k = *(int *) arg;
  const float valA = vecA[k];
  const float valB = vecB[k];
  if (valA < valB)
    return -1;
  if (valA > valB)
    return 1;
  return 0;
}

static int vec3_cmp(const void *a, const void *b, void *arg) {
  const float *v0 = (const float *) a;
  const float *v1 = (const float *) b;
  if (v0[0] < v1[0]) {
    return -1;
  } else if (v0[0] > v1[0]) {
    return 1;
  } else if (v0[1] < v1[1]) {
    return -1;
  } else if (v0[1] > v1[1]) {
    return 1;
  } else if (v0[2] < v1[2]) {
    return -1;
  } else if (v0[2] > v1[2]) {
    return 1;
  }
  return 0;
}

int test_sort(void) {
  // Setup
  const int N = 10;
  float *points = malloc(sizeof(float) * 3 * N);
  float *points_ = malloc(sizeof(float) * 3 * N);
  for (int i = 0; i < N; ++i) {
    const float x = randf(-1.0, 1.0);
    const float y = randf(-1.0, 1.0);
    const float z = randf(-1.0, 1.0);

    points[i * 3 + 0] = x;
    points[i * 3 + 1] = y;
    points[i * 3 + 2] = z;

    points_[i * 3 + 0] = points[i * 3 + 0];
    points_[i * 3 + 1] = points[i * 3 + 1];
    points_[i * 3 + 2] = points[i * 3 + 2];
  }

  printf("original:\n");
  for (int i = 0; i < N; ++i) {
    printf("(%.2f, %.2f, %.2f)\n",
           points_[i * 3 + 0],
           points_[i * 3 + 1],
           points_[i * 3 + 2]);
  }
  printf("\n");

  int start = 0;
  int end = N - 1;
  int k = 0;
  qsort_r(points + start, end - start + 1, sizeof(float) * 3, point_cmp, &k);

  printf("sorted:\n");
  for (int i = 0; i < N; ++i) {
    printf("(%.2f, %.2f, %.2f)\n",
           points[i * 3 + 0],
           points[i * 3 + 1],
           points[i * 3 + 2]);
  }
  printf("\n");

  return 0;
}

int test_kdtree_node(void) {
  const float p[3] = {0.0, 0.0, 0.0};
  const int k = 1;
  kdtree_node_t *node = kdtree_node_malloc(p, k);
  kdtree_node_free(node);

  return 0;
}

int test_kdtree(void) {
  // Generate random 3d points
  const int N = 20000;
  float *points = malloc(sizeof(float) * N * 3);
  float *points_gnd = malloc(sizeof(float) * N * 3);
  for (int i = 0; i < N; ++i) {
    const float x = randf(-10000.0, 10000.0);
    const float y = randf(-10000.0, 10000.0);
    const float z = randf(-10000.0, 10000.0);
    points[i * 3 + 0] = x;
    points[i * 3 + 1] = y;
    points[i * 3 + 2] = z;
    points_gnd[i * 3 + 0] = x;
    points_gnd[i * 3 + 1] = y;
    points_gnd[i * 3 + 2] = z;
  }

  // Build kdtree
  kdtree_t *kdtree = kdtree_malloc(points, N);
  kdtree_data_t data = {0};
  data.points = malloc(sizeof(float) * 3 * N);
  data.num_points = 0;
  data.capacity = N;
  kdtree_points(kdtree, &data);

  // Assert
  size_t checked = 0;
  int k = 0;
  qsort_r(data.points, N, sizeof(float) * 3, vec3_cmp, &k);
  qsort_r(points_gnd, N, sizeof(float) * 3, vec3_cmp, &k);
  for (int i = 0; i < N; ++i) {
    if (vec3_equals(&points_gnd[i * 3], &data.points[i * 3])) {
      checked++;
      continue;
    }
  }
  MU_ASSERT(checked == N);

  // Clean up
  free(points);
  free(points_gnd);
  free(data.points);
  kdtree_free(kdtree);

  return 0;
}

int test_kdtree_nn(void) {
  // Generate random 3d points
  const int N = 2000;
  float *points = malloc(sizeof(float) * N * 3);
  float *points_gnd = malloc(sizeof(float) * N * 3);
  for (int i = 0; i < N; ++i) {
    const float x = randf(-100.0, 100.0);
    const float y = randf(-100.0, 100.0);
    const float z = randf(-100.0, 100.0);
    points[i * 3 + 0] = x;
    points[i * 3 + 1] = y;
    points[i * 3 + 2] = z;
    points_gnd[i * 3 + 0] = x;
    points_gnd[i * 3 + 1] = y;
    points_gnd[i * 3 + 2] = z;
  }

  // Build kdtree
  kdtree_t *kdtree = kdtree_malloc(points, N);

  // Search closest point
  float p[3] = {5.0, 3.0, 0.0};
  float best_point[3] = {0};
  float best_dist = INFINITY;
  kdtree_nn(kdtree, p, best_point, &best_dist);

  // Check
  // -- Brute force get closest point
  float assert_point[3] = {0};
  float assert_dist = INFINITY;
  for (int i = 0; i < N; ++i) {
    float sq_dist = 0.0f;
    sq_dist += (points_gnd[i * 3 + 0] - p[0]) * (points_gnd[i * 3 + 0] - p[0]);
    sq_dist += (points_gnd[i * 3 + 1] - p[1]) * (points_gnd[i * 3 + 1] - p[1]);
    sq_dist += (points_gnd[i * 3 + 2] - p[2]) * (points_gnd[i * 3 + 2] - p[2]);
    if (sq_dist <= assert_dist) {
      assert_point[0] = points_gnd[i * 3 + 0];
      assert_point[1] = points_gnd[i * 3 + 1];
      assert_point[2] = points_gnd[i * 3 + 2];
      assert_dist = sq_dist;
    }
  }
  // -- Assert
  MU_ASSERT(fabs(best_point[0] - assert_point[0]) < 1e-3);
  MU_ASSERT(fabs(best_point[1] - assert_point[1]) < 1e-3);
  MU_ASSERT(fabs(best_point[2] - assert_point[2]) < 1e-3);
  MU_ASSERT(fabs(best_dist - assert_dist) < 1e-3);

  // Clean up
  kdtree_free(kdtree);
  free(points);
  free(points_gnd);

  return 0;
}

/*******************************************************************************
 * SIMULATION
 ******************************************************************************/

// SIM FEATURES //////////////////////////////////////////////////////////////

int test_sim_features_load(void) {
  const char *csv_file = TEST_SIM_DATA "/features.csv";
  sim_features_t *features_data = sim_features_load(csv_file);
  MU_ASSERT(features_data->num_features > 0);
  sim_features_free(features_data);
  return 0;
}

// SIM IMU DATA //////////////////////////////////////////////////////////////

int test_sim_imu_data_load(void) {
  // const char *csv_file = TEST_SIM_DATA "/imu0/data.csv";
  // sim_imu_data_t *imu_data = sim_imu_data_load(csv_file);
  // sim_imu_data_free(imu_data);
  return 0;
}

// SIM CAMERA DATA ///////////////////////////////////////////////////////////

int test_sim_camera_frame_load(void) {
  const char *frame_csv = TEST_SIM_DATA "/cam0/data/100000000.csv";
  sim_camera_frame_t *frame_data = sim_camera_frame_load(frame_csv);

  MU_ASSERT(frame_data != NULL);
  MU_ASSERT(frame_data->ts == 100000000);
  MU_ASSERT(frame_data->feature_ids[0] == 1);

  sim_camera_frame_free(frame_data);

  return 0;
}

int test_sim_camera_data_load(void) {
  const char *dir_path = TEST_SIM_DATA "/cam0";
  sim_camera_data_t *cam_data = sim_camera_data_load(dir_path);
  sim_camera_data_free(cam_data);
  return 0;
}

int test_sim_camera_circle_trajectory(void) {
  // Simulate features
  const real_t origin[3] = {0.0, 0.0, 0.0};
  const real_t dim[3] = {5.0, 5.0, 5.0};
  const int num_features = 1000;
  real_t features[3 * 1000] = {0};
  sim_create_features(origin, dim, num_features, features);

  // Camera
  const int cam_res[2] = {640, 480};
  const real_t fov = 90.0;
  const real_t fx = pinhole_focal(cam_res[0], fov);
  const real_t fy = pinhole_focal(cam_res[0], fov);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  const real_t cam_vec[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  const char *pmodel = "pinhole";
  const char *dmodel = "radtan4";
  camera_t camera;
  camera_setup(&camera, 0, cam_res, pmodel, dmodel, cam_vec);

  // Camera Extrinsic T_BC0
  const real_t cam_ext_ypr[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  const real_t cam_ext_r[3] = {0.05, 0.0, 0.0};
  TF_ER(cam_ext_ypr, cam_ext_r, T_BC0);
  TF_VECTOR(T_BC0, cam_ext);

  // Simulate camera trajectory
  sim_circle_t conf;
  sim_circle_defaults(&conf);
  sim_camera_data_t *cam_data = sim_camera_circle_trajectory(&conf,
                                                             T_BC0,
                                                             &camera,
                                                             features,
                                                             num_features);
  // ASSERT
  for (size_t k = 0; k < cam_data->num_frames; k++) {
    const sim_camera_frame_t *cam_frame = cam_data->frames[k];
    const real_t *cam_pose = &cam_data->poses[k * 7];

    for (int i = 0; i < cam_frame->n; i++) {
      const size_t feature_id = cam_frame->feature_ids[i];
      const real_t *p_W = &features[feature_id * 3];
      const real_t *z = &cam_frame->keypoints[i * 2];

      TF(cam_pose, T_WC0);
      TF_INV(T_WC0, T_C0W);
      TF_POINT(T_C0W, p_W, p_C0);

      real_t zhat[2] = {0};
      pinhole_radtan4_project(cam_vec, p_C0, zhat);

      const real_t r[2] = {zhat[0] - z[0], zhat[1] - z[1]};
      MU_ASSERT(fltcmp(r[0], 0.0) == 0);
      MU_ASSERT(fltcmp(r[1], 0.0) == 0);
    }
  }

  // Clean up
  sim_camera_data_free(cam_data);

  return 0;
}

/******************************************************************************
 * EUROC
 ******************************************************************************/

int test_euroc_imu_load(void) {
  const char *data_dir = "/data/euroc/imu_april/mav0/imu0";
  euroc_imu_t *data = euroc_imu_load(data_dir);
  // euroc_imu_print(data);
  euroc_imu_free(data);
  return 0;
}

int test_euroc_camera_load(void) {
  const char *data_dir = "/data/euroc/imu_april/mav0/cam0";
  euroc_camera_t *data = euroc_camera_load(data_dir, 1);
  // euroc_camera_print(data);
  euroc_camera_free(data);
  return 0;
}

int test_euroc_ground_truth_load(void) {
  const char *data_dir = "/data/euroc/V1_01/mav0/state_groundtruth_estimate0";
  euroc_ground_truth_t *data = euroc_ground_truth_load(data_dir);
  // euroc_ground_truth_print(data);
  euroc_ground_truth_free(data);
  return 0;
}

int test_euroc_data_load(void) {
  const char *data_dir = "/data/euroc/V1_01";
  euroc_data_t *data = euroc_data_load(data_dir);
  euroc_data_free(data);
  return 0;
}

int test_euroc_calib_target_load(void) {
  const char *config_path = "/data/euroc/imu_april/april_6x6.yaml";
  euroc_calib_target_t *data = euroc_calib_target_load(config_path);
  // euroc_calib_target_print(data);
  euroc_calib_target_free(data);
  return 0;
}

int test_euroc_calib_load(void) {
  const char *config_path = "/data/euroc/imu_april";
  euroc_calib_t *data = euroc_calib_load(config_path);
  euroc_calib_free(data);
  return 0;
}

/******************************************************************************
 * KITTI
 ******************************************************************************/


int test_kitti_camera_load(void) {
  const char *data_dir = KITTI_TEST_SEQ "/image_00";
  kitti_camera_t *data = kitti_camera_load(data_dir);
  kitti_camera_free(data);
  return 0;
}

int test_kitti_oxts_load(void) {
  const char *data_dir = KITTI_TEST_SEQ "/oxts";
  kitti_oxts_t *data = kitti_oxts_load(data_dir);
  kitti_oxts_free(data);
  return 0;
}

int test_kitti_velodyne_load(void) {
  const char *data_dir = KITTI_TEST_SEQ "/velodyne_points";
  kitti_velodyne_t *data = kitti_velodyne_load(data_dir);
  kitti_velodyne_free(data);
  return 0;
}

int test_kitti_calib_load(void) {
  const char *data_dir = KITTI_TEST_DATA;
  kitti_calib_t *data = kitti_calib_load(data_dir);
  // kitti_calib_print(data);
  kitti_calib_free(data);
  return 0;
}

int test_kitti_raw_load(void) {
  kitti_raw_t *data = kitti_raw_load(KITTI_TEST_DATA, KITTI_TEST_SEQ_NAME);
  kitti_raw_free(data);
  return 0;
}

/*******************************************************************************
 * TEST-SUITE
 ******************************************************************************/

void test_suite(void) {
  // MACROS
  MU_ADD_TEST(test_median_value);
  MU_ADD_TEST(test_mean_value);

  // TIME
  MU_ADD_TEST(test_tic_toc);
  MU_ADD_TEST(test_mtoc);
  MU_ADD_TEST(test_time_now);

  // DARRAY
  MU_ADD_TEST(test_darray_new_and_destroy);
  MU_ADD_TEST(test_darray_push_pop);
  MU_ADD_TEST(test_darray_contains);
  MU_ADD_TEST(test_darray_copy);
  MU_ADD_TEST(test_darray_new_element);
  MU_ADD_TEST(test_darray_set_and_get);
  MU_ADD_TEST(test_darray_update);
  MU_ADD_TEST(test_darray_remove);
  MU_ADD_TEST(test_darray_expand_and_contract);

  // LIST
  MU_ADD_TEST(test_list_malloc_and_free);
  MU_ADD_TEST(test_list_push_pop);
  MU_ADD_TEST(test_list_shift);
  MU_ADD_TEST(test_list_unshift);
  MU_ADD_TEST(test_list_remove);
  MU_ADD_TEST(test_list_remove_destroy);

  // RED-BLACK-TREE
  MU_ADD_TEST(test_rbt_node_malloc_and_free);
  MU_ADD_TEST(test_rbt_node_min_max);
  MU_ADD_TEST(test_rbt_node_height_size);
  MU_ADD_TEST(test_rbt_node_keys);
  MU_ADD_TEST(test_rbt_node_flip_colors);
  MU_ADD_TEST(test_rbt_node_rotate);
  MU_ADD_TEST(test_rbt_node_move_red_left);
  MU_ADD_TEST(test_rbt_node_move_red_right);
  MU_ADD_TEST(test_rbt_node_insert);
  MU_ADD_TEST(test_rbt_node_delete);
  MU_ADD_TEST(test_rbt_malloc_and_free);
  MU_ADD_TEST(test_rbt_insert);
  MU_ADD_TEST(test_rbt_delete);
  MU_ADD_TEST(test_rbt_search);
  MU_ADD_TEST(test_rbt_contains);
  MU_ADD_TEST(test_rbt_min_max);
  MU_ADD_TEST(test_rbt_keys);
  MU_ADD_TEST(test_rbt_rank);
  MU_ADD_TEST(test_rbt_select);
  MU_ADD_TEST(test_rbt_sandbox);

  // HASHMAP
  MU_ADD_TEST(test_hm_malloc_and_free);
  MU_ADD_TEST(test_hm_set_and_get);

  // NETWORK
  MU_ADD_TEST(test_tcp_server_setup);

  // MATH
  MU_ADD_TEST(test_min);
  MU_ADD_TEST(test_max);
  MU_ADD_TEST(test_randf);
  MU_ADD_TEST(test_deg2rad);
  MU_ADD_TEST(test_rad2deg);
  MU_ADD_TEST(test_wrap_180);
  MU_ADD_TEST(test_wrap_360);
  MU_ADD_TEST(test_wrap_pi);
  MU_ADD_TEST(test_wrap_2pi);
  MU_ADD_TEST(test_fltcmp);
  MU_ADD_TEST(test_fltcmp2);
  MU_ADD_TEST(test_cumsum);
  MU_ADD_TEST(test_logspace);
  MU_ADD_TEST(test_pythag);
  MU_ADD_TEST(test_lerp);
  MU_ADD_TEST(test_lerp3);
  MU_ADD_TEST(test_sinc);
  MU_ADD_TEST(test_mean);
  MU_ADD_TEST(test_median);
  MU_ADD_TEST(test_var);
  MU_ADD_TEST(test_stddev);

  // LINEAR ALGEBRA
  MU_ADD_TEST(test_eye);
  MU_ADD_TEST(test_ones);
  MU_ADD_TEST(test_zeros);
  MU_ADD_TEST(test_mat_set);
  MU_ADD_TEST(test_mat_val);
  MU_ADD_TEST(test_mat_copy);
  MU_ADD_TEST(test_mat_row_set);
  MU_ADD_TEST(test_mat_col_set);
  MU_ADD_TEST(test_mat_block_get);
  MU_ADD_TEST(test_mat_block_set);
  MU_ADD_TEST(test_mat_diag_get);
  MU_ADD_TEST(test_mat_diag_set);
  MU_ADD_TEST(test_mat_triu);
  MU_ADD_TEST(test_mat_tril);
  MU_ADD_TEST(test_mat_trace);
  MU_ADD_TEST(test_mat_transpose);
  MU_ADD_TEST(test_mat_add);
  MU_ADD_TEST(test_mat_sub);
  MU_ADD_TEST(test_mat_scale);
  MU_ADD_TEST(test_vec_add);
  MU_ADD_TEST(test_vec_sub);
  MU_ADD_TEST(test_dot);
  // MU_ADD_TEST(test_bdiag_inv);
  MU_ADD_TEST(test_hat);
  MU_ADD_TEST(test_check_jacobian);
  MU_ADD_TEST(test_svd);
  MU_ADD_TEST(test_pinv);
  MU_ADD_TEST(test_svd_det);
  MU_ADD_TEST(test_chol);
  MU_ADD_TEST(test_chol_solve);
  MU_ADD_TEST(test_qr);
  MU_ADD_TEST(test_eig_sym);
  MU_ADD_TEST(test_eig_inv);
  MU_ADD_TEST(test_schur_complement);

  // SUITE-SPARSE
  MU_ADD_TEST(test_suitesparse_chol_solve);

  // TRANSFORMS
  MU_ADD_TEST(test_tf_rot_set);
  MU_ADD_TEST(test_tf_trans_set);
  MU_ADD_TEST(test_tf_trans_get);
  MU_ADD_TEST(test_tf_rot_get);
  MU_ADD_TEST(test_tf_quat_get);
  MU_ADD_TEST(test_tf_inv);
  MU_ADD_TEST(test_tf_point);
  MU_ADD_TEST(test_tf_hpoint);
  MU_ADD_TEST(test_tf_perturb_rot);
  MU_ADD_TEST(test_tf_perturb_trans);
  MU_ADD_TEST(test_tf_chain);
  MU_ADD_TEST(test_euler321);
  MU_ADD_TEST(test_rot2quat);
  MU_ADD_TEST(test_quat2euler);
  MU_ADD_TEST(test_quat2rot);

  // LIE
  MU_ADD_TEST(test_lie_Exp_Log);

  // GNUPLOT
#if CI_MODE == 0
  // MU_ADD_TEST(test_gnuplot_xyplot);
  // MU_ADD_TEST(test_gnuplot_multiplot);
#endif

  // CONTROL
  MU_ADD_TEST(test_pid_ctrl);

  // MAV
  MU_ADD_TEST(test_mav_att_ctrl);
  MU_ADD_TEST(test_mav_vel_ctrl);
  MU_ADD_TEST(test_mav_pos_ctrl);
  MU_ADD_TEST(test_mav_waypoints);

  // COMPUTER-VISION
  MU_ADD_TEST(test_image_setup);
  MU_ADD_TEST(test_image_load);
  MU_ADD_TEST(test_image_print_properties);
  MU_ADD_TEST(test_image_free);
  MU_ADD_TEST(test_radtan4_distort);
  MU_ADD_TEST(test_radtan4_undistort);
  MU_ADD_TEST(test_radtan4_point_jacobian);
  MU_ADD_TEST(test_radtan4_params_jacobian);
  MU_ADD_TEST(test_equi4_distort);
  MU_ADD_TEST(test_equi4_undistort);
  MU_ADD_TEST(test_equi4_point_jacobian);
  MU_ADD_TEST(test_equi4_params_jacobian);
  MU_ADD_TEST(test_pinhole_focal);
  MU_ADD_TEST(test_pinhole_K);
  MU_ADD_TEST(test_pinhole_projection_matrix);
  MU_ADD_TEST(test_pinhole_project);
  MU_ADD_TEST(test_pinhole_point_jacobian);
  MU_ADD_TEST(test_pinhole_params_jacobian);
  MU_ADD_TEST(test_pinhole_radtan4_project);
  MU_ADD_TEST(test_pinhole_radtan4_project_jacobian);
  MU_ADD_TEST(test_pinhole_radtan4_params_jacobian);
  MU_ADD_TEST(test_pinhole_equi4_project);
  MU_ADD_TEST(test_pinhole_equi4_project_jacobian);
  MU_ADD_TEST(test_pinhole_equi4_params_jacobian);
  MU_ADD_TEST(test_linear_triangulation);
  MU_ADD_TEST(test_homography_find);
  MU_ADD_TEST(test_homography_pose);
  // MU_ADD_TEST(test_p3p_kneip);
  MU_ADD_TEST(test_solvepnp);

  // APRILGRID
  MU_ADD_TEST(test_aprilgrid_malloc_and_free);
  MU_ADD_TEST(test_aprilgrid_center);
  MU_ADD_TEST(test_aprilgrid_grid_index);
  MU_ADD_TEST(test_aprilgrid_object_point);
  MU_ADD_TEST(test_aprilgrid_add_and_remove_corner);
  MU_ADD_TEST(test_aprilgrid_add_and_remove_tag);
  MU_ADD_TEST(test_aprilgrid_save_and_load);
#if ENABLE_APRILGRID_DETECTOR == 1
  MU_ADD_TEST(test_aprilgrid_detector_detect);
#endif

  // STATE-ESTIMATION
  MU_ADD_TEST(test_pose);
  MU_ADD_TEST(test_extrinsics);
  MU_ADD_TEST(test_fiducial);
  MU_ADD_TEST(test_imu_biases);
  MU_ADD_TEST(test_feature);
  MU_ADD_TEST(test_time_delay);
  MU_ADD_TEST(test_joint);
  MU_ADD_TEST(test_camera);
  MU_ADD_TEST(test_triangulate_batch);
  MU_ADD_TEST(test_pose_factor);
  MU_ADD_TEST(test_ba_factor);
  MU_ADD_TEST(test_camera_factor);
  MU_ADD_TEST(test_imu_buffer_setup);
  MU_ADD_TEST(test_imu_buffer_add);
  MU_ADD_TEST(test_imu_buffer_clear);
  MU_ADD_TEST(test_imu_buffer_copy);
  MU_ADD_TEST(test_imu_propagate);
  MU_ADD_TEST(test_imu_initial_attitude);
  MU_ADD_TEST(test_imu_factor_form_F_matrix);
  MU_ADD_TEST(test_imu_factor);
  MU_ADD_TEST(test_joint_factor);
  MU_ADD_TEST(test_camchain);
  MU_ADD_TEST(test_calib_camera_factor);
  MU_ADD_TEST(test_calib_imucam_factor);
  MU_ADD_TEST(test_marg_factor);

  // TIMELINE
  // MU_ADD_TEST(test_timeline);

  // OCTREE
  MU_ADD_TEST(test_octree_node);
  MU_ADD_TEST(test_octree);
  MU_ADD_TEST(test_octree_points);
  MU_ADD_TEST(test_octree_downsample);

  // KD-TREE
  // MU_ADD_TEST(test_sort);
  MU_ADD_TEST(test_kdtree_node);
  MU_ADD_TEST(test_kdtree);
  MU_ADD_TEST(test_kdtree_nn);

  // SIMULATION
  MU_ADD_TEST(test_sim_features_load);
  MU_ADD_TEST(test_sim_imu_data_load);
  MU_ADD_TEST(test_sim_camera_frame_load);
  MU_ADD_TEST(test_sim_camera_data_load);
  MU_ADD_TEST(test_sim_camera_circle_trajectory);

  // EUROC
#if CI_MODE == 0
  MU_ADD_TEST(test_euroc_imu_load);
  MU_ADD_TEST(test_euroc_camera_load);
  MU_ADD_TEST(test_euroc_ground_truth_load);
  MU_ADD_TEST(test_euroc_data_load);
  MU_ADD_TEST(test_euroc_calib_target_load);
  // MU_ADD_TEST(test_euroc_calib_load);
#endif

  // KITTI
#if CI_MODE == 0
  MU_ADD_TEST(test_kitti_camera_load);
  MU_ADD_TEST(test_kitti_oxts_load);
  MU_ADD_TEST(test_kitti_velodyne_load);
  MU_ADD_TEST(test_kitti_calib_load);
  MU_ADD_TEST(test_kitti_raw_load);
#endif
}
MU_RUN_TESTS(test_suite)
