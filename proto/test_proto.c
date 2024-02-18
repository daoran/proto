#include "proto.h"
#include "munit.h"
#include "stb_ds.h"

/* TEST PARAMS */
#define TEST_DATA_PATH "./test_data/"
#define TEST_CSV TEST_DATA_PATH "test_csv.csv"
#define TEST_POSES_CSV TEST_DATA_PATH "poses.csv"
#define TEST_SIM_DATA TEST_DATA_PATH "sim_data"
#define TEST_SIM_GIMBAL TEST_DATA_PATH "sim_gimbal"
#define TEST_CAM_APRIL TEST_DATA_PATH "cam_april"
#define TEST_IMU_APRIL TEST_DATA_PATH "imu_april"

/******************************************************************************
 * TEST MACROS
 ******************************************************************************/

int test_median_value() {
  real_t median = 0.0f;
  real_t buf[5] = {4.0, 1.0, 0.0, 3.0, 2.0};
  MEDIAN_VALUE(real_t, fltcmp2, buf, 5, median);
  MU_ASSERT(fltcmp(median, 2.0) == 0);

  return 0;
}

int test_mean_value() {
  real_t mean = 0.0f;
  real_t buf[5] = {0.0, 1.0, 2.0, 3.0, 4.0};
  MEAN_VALUE(real_t, buf, 5, mean);
  MU_ASSERT(fltcmp(mean, 2.0) == 0);

  return 0;
}

/******************************************************************************
 * TEST FILESYSTEM
 ******************************************************************************/

int test_path_file_name() {
  const char *path = "/tmp/hello_world.csv";
  char fname[128] = {0};
  path_file_name(path, fname);
  MU_ASSERT(strcmp(fname, "hello_world.csv") == 0);

  return 0;
}

int test_path_file_ext() {
  const char *path = "/tmp/hello_world.csv";
  char fext[128] = {0};
  path_file_ext(path, fext);
  MU_ASSERT(strcmp(fext, "csv") == 0);

  return 0;
}

int test_path_dir_name() {
  const char *path = "/tmp/hello_world.csv";
  char dir_name[128] = {0};
  path_dir_name(path, dir_name);
  MU_ASSERT(strcmp(dir_name, "/tmp") == 0);

  return 0;
}

int test_path_join() {
  // Case A
  {
    const char *path_a = "/tmp/A";
    const char *path_b = "B.csv";
    char *c = path_join(path_a, path_b);
    MU_ASSERT(strcmp(c, "/tmp/A/B.csv") == 0);
    free(c);
  }

  // Case B
  {
    const char *path_a = "/tmp/A/";
    const char *path_b = "B.csv";
    char *c = path_join(path_a, path_b);
    MU_ASSERT(strcmp(c, "/tmp/A/B.csv") == 0);
    free(c);
  }

  return 0;
}

int test_list_files() {
  int num_files = 0;
  char **files = list_files("/tmp", &num_files);
  MU_ASSERT(files != NULL);
  MU_ASSERT(num_files != 0);

  /* printf("num_files: %d\n", num_files); */
  for (int i = 0; i < num_files; i++) {
    /* printf("file: %s\n", files[i]); */
    free(files[i]);
  }
  free(files);

  return 0;
}

int test_list_files_free() {
  int num_files = 0;
  char **files = list_files("/tmp", &num_files);
  list_files_free(files, num_files);

  return 0;
}

int test_file_read() {
  char *text = file_read("test_data/poses.csv");
  /* printf("%s\n", text); */
  MU_ASSERT(text != NULL);
  free(text);

  return 0;
}

int test_skip_line() {
  FILE *fp = fopen("test_data/poses.csv", "r");
  skip_line(fp);
  fclose(fp);

  return 0;
}

int test_file_rows() {
  int num_rows = file_rows("test_data/poses.csv");
  MU_ASSERT(num_rows > 0);
  return 0;
}

int test_file_copy() {
  int retval = file_copy("test_data/poses.csv", "/tmp/poses.csv");
  char *text0 = file_read("test_data/poses.csv");
  char *text1 = file_read("/tmp/poses.csv");
  MU_ASSERT(retval == 0);
  MU_ASSERT(strcmp(text0, text1) == 0);
  free(text0);
  free(text1);

  return 0;
}

/******************************************************************************
 * TEST DATA
 ******************************************************************************/

int test_string_malloc() {
  char *s = string_malloc("hello world!");
  MU_ASSERT(strcmp(s, "hello world!") == 0);
  free(s);
  return 0;
}

int test_dsv_rows() {
  int num_rows = dsv_rows(TEST_CSV);
  MU_ASSERT(num_rows == 10);
  return 0;
}

int test_dsv_cols() {
  int num_cols = dsv_cols(TEST_CSV, ',');
  MU_ASSERT(num_cols == 10);
  return 0;
}

int test_dsv_fields() {
  int num_fields = 0;
  char **fields = dsv_fields(TEST_CSV, ',', &num_fields);
  const char *expected[10] = {"a", "b", "c", "d", "e", "f", "g", "h", "i", "j"};
  if (fields == NULL) {
    printf("File not found [%s]\n", TEST_CSV);
    return -1;
  }

  MU_ASSERT(num_fields == 10);
  for (int i = 0; i < num_fields; i++) {
    MU_ASSERT(strcmp(fields[i], expected[i]) == 0);
    free(fields[i]);
  }
  free(fields);

  return 0;
}

int test_dsv_data() {
  int num_rows = 0;
  int num_cols = 0;
  real_t **data = dsv_data(TEST_CSV, ',', &num_rows, &num_cols);

  int index = 0;
  for (int i = 0; i < num_rows; i++) {
    for (int j = 0; j < num_rows; j++) {
      MU_ASSERT(fltcmp(data[i][j], index + 1) == 0);
      index++;
    }
  }
  dsv_free(data, num_rows);

  return 0;
}

int test_dsv_free() {
  int num_rows = 0;
  int num_cols = 0;
  real_t **data = dsv_data(TEST_CSV, ',', &num_rows, &num_cols);
  dsv_free(data, num_rows);

  return 0;
}

int test_csv_data() {
  int num_rows = 0;
  int num_cols = 0;
  real_t **data = csv_data(TEST_CSV, &num_rows, &num_cols);
  csv_free(data, num_rows);

  return 0;
}

/******************************************************************************
 * TEST DATA-STRUCTURE
 ******************************************************************************/

// DARRAY ////////////////////////////////////////////////////////////////////

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

// LIST //////////////////////////////////////////////////////////////////////

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

// STACK /////////////////////////////////////////////////////////////////////

int test_mstack_new_and_destroy(void) {
  mstack_t *s = stack_new();

  MU_ASSERT(s->size == 0);
  MU_ASSERT(s->root == NULL);
  MU_ASSERT(s->end == NULL);

  mstack_destroy(s);
  return 0;
}

int test_mstack_push(void) {
  mstack_t *s = stack_new();
  float f1 = 2.0;
  float f2 = 4.0;
  float f3 = 8.0;

  /* push float 1 */
  mstack_push(s, &f1);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) &f1) == 0);
  MU_ASSERT(s->size == 1);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(s->end->prev == NULL);

  /* push float 2 */
  mstack_push(s, &f2);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) &f2) == 0);
  MU_ASSERT(s->size == 2);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(s->end->prev->value == &f1);
  MU_ASSERT(fltcmp(*(float *) s->end->prev->value, *(float *) &f1) == 0);

  /* push float 3 */
  mstack_push(s, &f3);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) &f3) == 0);
  MU_ASSERT(s->size == 3);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(s->end->prev->value == &f2);
  MU_ASSERT(fltcmp(*(float *) s->end->prev->value, *(float *) &f2) == 0);

  mstack_destroy(s);
  return 0;
}

int test_mstack_pop(void) {
  mstack_t *s = stack_new();
  float f1 = 2.0;
  float f2 = 4.0;
  float f3 = 8.0;

  /* push float 1 */
  mstack_push(s, &f1);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) &f1) == 0);
  MU_ASSERT(s->size == 1);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(s->end->prev == NULL);

  /* push float 2 */
  mstack_push(s, &f2);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) &f2) == 0);
  MU_ASSERT(s->size == 2);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(s->end->prev->value == &f1);
  MU_ASSERT(fltcmp(*(float *) s->end->prev->value, *(float *) &f1) == 0);

  /* push float 3 */
  mstack_push(s, &f3);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) &f3) == 0);
  MU_ASSERT(s->size == 3);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(s->end->prev->value == &f2);
  MU_ASSERT(fltcmp(*(float *) s->end->prev->value, *(float *) &f2) == 0);

  /* pop float 3 */
  float *flt_ptr = mstack_pop(s);
  MU_ASSERT(fltcmp(*(float *) flt_ptr, *(float *) &f3) == 0);
  MU_ASSERT(s->size == 2);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(fltcmp(*(float *) s->root->value, *(float *) &f1) == 0);

  /* pop float 2 */
  flt_ptr = mstack_pop(s);
  MU_ASSERT(fltcmp(*(float *) flt_ptr, *(float *) &f2) == 0);
  MU_ASSERT(s->size == 1);
  MU_ASSERT(s->root->value == &f1);
  MU_ASSERT(fltcmp(*(float *) s->root->value, *(float *) &f1) == 0);

  /* pop float 1 */
  flt_ptr = mstack_pop(s);
  MU_ASSERT(fltcmp(*(float *) flt_ptr, *(float *) &f1) == 0);
  MU_ASSERT(s->size == 0);
  MU_ASSERT(s->root == NULL);
  MU_ASSERT(s->end == NULL);

  mstack_destroy(s);
  return 0;
}

// QUEUE /////////////////////////////////////////////////////////////////////

int test_queue_malloc_and_free(void) {
  queue_t *q = queue_malloc();
  MU_ASSERT(q != NULL);
  MU_ASSERT(q->count == 0);
  queue_free(q);

  return 0;
}

int test_queue_enqueue_dequeue(void) {
  queue_t *q = queue_malloc();
  char *t1 = "test1 data";
  char *t2 = "test2 data";
  char *t3 = "test3 data";

  /* Enqueue tests */
  queue_enqueue(q, t1);
  MU_ASSERT(queue_first(q) == t1);
  MU_ASSERT(queue_last(q) == t1);
  MU_ASSERT(q->count == 1);

  queue_enqueue(q, t2);
  MU_ASSERT(queue_first(q) == t1);
  MU_ASSERT(queue_last(q) == t2);
  MU_ASSERT(q->count == 2);

  queue_enqueue(q, t3);
  MU_ASSERT(queue_first(q) == t1);
  MU_ASSERT(queue_last(q) == t3);
  MU_ASSERT(q->count == 3);

  /* Dequeue tests */
  char *val = queue_dequeue(q);
  MU_ASSERT(val == t1);
  MU_ASSERT(queue_first(q) == t2);
  MU_ASSERT(queue_last(q) == t3);
  MU_ASSERT(q->count == 2);

  val = queue_dequeue(q);
  MU_ASSERT(val == t2);
  MU_ASSERT(queue_first(q) == t3);
  MU_ASSERT(queue_last(q) == t3);
  MU_ASSERT(q->count == 1);

  val = queue_dequeue(q);
  MU_ASSERT(val == t3);
  MU_ASSERT(queue_first(q) == NULL);
  MU_ASSERT(queue_last(q) == NULL);
  MU_ASSERT(q->count == 0);

  // Clean up
  queue_free(q);

  return 0;
}

// HASHMAP ///////////////////////////////////////////////////////////////////

static int traverse_called;

static int traverse_good_cb(hashmap_node_t *node) {
  UNUSED(node);
  traverse_called++;
  return 0;
}

static int traverse_fail_cb(hashmap_node_t *node) {
  UNUSED(node);
  traverse_called++;
  if (traverse_called == 2) {
    return 1;
  } else {
    return 0;
  }
}

hashmap_t *hashmap_test_setup(void) {
  hashmap_t *map;
  char *test1;
  char *test2;
  char *test3;
  char *expect1;
  char *expect2;
  char *expect3;

  /* setup */
  map = hashmap_new();

  /* key and values */
  test1 = "test data 1";
  test2 = "test data 2";
  test3 = "xest data 3";
  expect1 = "THE VALUE 1";
  expect2 = "THE VALUE 2";
  expect3 = "THE VALUE 3";

  /* set */
  hashmap_set(map, test1, expect1);
  hashmap_set(map, test2, expect2);
  hashmap_set(map, test3, expect3);

  return map;
}

void hashmap_test_teardown(hashmap_t *map) {
  hashmap_destroy(map);
}

int test_hashmap_new_destroy(void) {
  hashmap_t *map;

  map = hashmap_new();
  MU_ASSERT(map != NULL);
  hashmap_destroy(map);

  return 0;
}

int test_hashmap_clear_destroy(void) {
  hashmap_t *map;

  map = hashmap_new();
  hashmap_set(map, "test", "hello");
  hashmap_clear_destroy(map);

  return 0;
}

int test_hashmap_get_set(void) {
  /* Setup */
  int rc;
  char *result;

  hashmap_t *map = hashmap_test_setup();
  char *test1 = "test data 1";
  char *test2 = "test data 2";
  char *test3 = "xest data 3";
  char *expect1 = "THE VALUE 1";
  char *expect2 = "THE VALUE 2";
  char *expect3 = "THE VALUE 3";

  /* Set and get test1 */
  rc = hashmap_set(map, test1, expect1);
  MU_ASSERT(rc == 0);
  result = hashmap_get(map, test1);
  MU_ASSERT(strcmp(result, expect1) == 0);

  /* Set and get test2 */
  rc = hashmap_set(map, test2, expect2);
  MU_ASSERT(rc == 0);
  result = hashmap_get(map, test2);
  MU_ASSERT(strcmp(result, expect2) == 0);

  /* Set and get test3 */
  rc = hashmap_set(map, test3, expect3);
  MU_ASSERT(rc == 0);
  result = hashmap_get(map, test3);
  MU_ASSERT(strcmp(result, expect3) == 0);

  /* Clean up */
  hashmap_test_teardown(map);

  return 0;
}

int test_hashmap_delete(void) {
  /* Setup */
  char *deleted = NULL;
  char *result = NULL;

  hashmap_t *map = hashmap_test_setup();
  char *test1 = "test data 1";
  char *test2 = "test data 2";
  char *test3 = "xest data 3";
  char *expect1 = "THE VALUE 1";
  char *expect2 = "THE VALUE 2";
  char *expect3 = "THE VALUE 3";

  /* Delete test1 */
  deleted = hashmap_delete(map, test1);
  MU_ASSERT(deleted != NULL);
  MU_ASSERT(strcmp(deleted, expect1) == 0);
  free(deleted);

  result = hashmap_get(map, test1);
  MU_ASSERT(result == NULL);

  /* Delete test2 */
  deleted = hashmap_delete(map, test2);
  MU_ASSERT(deleted != NULL);
  MU_ASSERT(strcmp(deleted, expect2) == 0);
  free(deleted);

  result = hashmap_get(map, test2);
  MU_ASSERT(result == NULL);

  /* Delete test3 */
  deleted = hashmap_delete(map, test3);
  MU_ASSERT(deleted != NULL);
  MU_ASSERT(strcmp(deleted, expect3) == 0);
  free(deleted);

  result = hashmap_get(map, test3);
  MU_ASSERT(result == NULL);

  /* Clean up */
  hashmap_test_teardown(map);

  return 0;
}

int test_hashmap_traverse(void) {
  int retval;
  hashmap_t *map;

  /* setup */
  map = hashmap_test_setup();

  /* traverse good cb */
  traverse_called = 0;
  retval = hashmap_traverse(map, traverse_good_cb);
  MU_ASSERT(retval == 0);
  MU_ASSERT(traverse_called == 3);

  /* traverse good bad */
  traverse_called = 0;
  retval = hashmap_traverse(map, traverse_fail_cb);
  MU_ASSERT(retval == 1);
  MU_ASSERT(traverse_called == 2);

  /* clean up */
  hashmap_test_teardown(map);

  return 0;
}

/******************************************************************************
 * TEST TIME
 ******************************************************************************/

int test_tic_toc() {
  struct timespec t_start = tic();
  usleep(1);
  MU_ASSERT(fabs(toc(&t_start) - 1e-3) < 1e-2);
  return 0;
}

int test_mtoc() {
  struct timespec t_start = tic();
  usleep(1);
  MU_ASSERT(fabs(mtoc(&t_start) - 1e-3) < 1);
  return 0;
}

int test_time_now() {
  timestamp_t t_now = time_now();
  // printf("t_now: %ld\n", t_now);
  MU_ASSERT(t_now > 0);
  return 0;
}

/******************************************************************************
 * TEST NETWORK
 ******************************************************************************/

int test_tcp_server_setup() {
  tcp_server_t server;
  const int port = 8080;
  int retval = tcp_server_setup(&server, port);
  MU_ASSERT(retval == 0);
  return 0;
}

/******************************************************************************
 * TEST MATHS
 ******************************************************************************/

int test_min() {
  MU_ASSERT(MIN(1, 2) == 1);
  MU_ASSERT(MIN(2, 1) == 1);
  return 0;
}

int test_max() {
  MU_ASSERT(MAX(1, 2) == 2);
  MU_ASSERT(MAX(2, 1) == 2);
  return 0;
}

int test_randf() {
  const real_t val = randf(0.0, 10.0);
  MU_ASSERT(val < 10.0);
  MU_ASSERT(val > 0.0);
  return 0;
}

int test_deg2rad() {
  MU_ASSERT(fltcmp(deg2rad(180.0f), M_PI) == 0);
  return 0;
}

int test_rad2deg() {
  MU_ASSERT(fltcmp(rad2deg(M_PI), 180.0f) == 0);
  return 0;
}

int test_wrap_180() {
  MU_ASSERT(fltcmp(wrap_180(181), -179) == 0);
  MU_ASSERT(fltcmp(wrap_180(90), 90) == 0);
  MU_ASSERT(fltcmp(wrap_180(-181), 179) == 0);
  return 0;
}

int test_wrap_360() {
  MU_ASSERT(fltcmp(wrap_360(-1), 359) == 0);
  MU_ASSERT(fltcmp(wrap_360(180), 180) == 0);
  MU_ASSERT(fltcmp(wrap_360(361), 1) == 0);
  return 0;
}

int test_wrap_pi() {
  MU_ASSERT(fltcmp(wrap_pi(deg2rad(181)), deg2rad(-179)) == 0);
  MU_ASSERT(fltcmp(wrap_pi(deg2rad(90)), deg2rad(90)) == 0);
  MU_ASSERT(fltcmp(wrap_pi(deg2rad(-181)), deg2rad(179)) == 0);
  return 0;
}

int test_wrap_2pi() {
  MU_ASSERT(fltcmp(wrap_2pi(deg2rad(-1)), deg2rad(359)) == 0);
  MU_ASSERT(fltcmp(wrap_2pi(deg2rad(180)), deg2rad(180)) == 0);
  MU_ASSERT(fltcmp(wrap_2pi(deg2rad(361)), deg2rad(1)) == 0);
  return 0;
}

int test_fltcmp() {
  MU_ASSERT(fltcmp(1.0, 1.0) == 0);
  MU_ASSERT(fltcmp(1.0, 1.01) != 0);
  return 0;
}

int test_fltcmp2() {
  const real_t x = 1.0f;
  const real_t y = 1.0f;
  const real_t z = 1.01f;
  MU_ASSERT(fltcmp2(&x, &y) == 0);
  MU_ASSERT(fltcmp2(&x, &z) != 0);
  return 0;
}

int test_cumsum() {
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

int test_logspace() {
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

int test_pythag() {
  MU_ASSERT(fltcmp(pythag(3.0, 4.0), 5.0) == 0);
  return 0;
}

int test_lerp() {
  MU_ASSERT(fltcmp(lerp(0.0, 1.0, 0.5), 0.5) == 0);
  MU_ASSERT(fltcmp(lerp(0.0, 10.0, 0.8), 8.0) == 0);
  return 0;
}

int test_lerp3() {
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

int test_sinc() {
  return 0;
}

int test_mean() {
  real_t vals[4] = {1.0, 2.0, 3.0, 4.0};
  MU_ASSERT(fltcmp(mean(vals, 4), 2.5) == 0);

  return 0;
}

int test_median() {
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

int test_var() {
  real_t vals[4] = {1.0, 2.0, 3.0, 4.0};
  MU_ASSERT(fltcmp(var(vals, 4), 1.666666667) == 0);

  return 0;
}

int test_stddev() {
  real_t vals[4] = {1.0, 2.0, 3.0, 4.0};
  MU_ASSERT(fltcmp(stddev(vals, 4), sqrt(1.666666667)) == 0);

  return 0;
}

/******************************************************************************
 * TEST LINEAR ALGEBRA
 ******************************************************************************/

int test_eye() {
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

int test_ones() {
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

int test_zeros() {
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

int test_mat_set() {
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

int test_mat_val() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

  /* print_matrix("A", A, 3, 3); */
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 1), 2.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 2), 3.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 1, 0), 4.0) == 0);

  return 0;
}

int test_mat_copy() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {0};

  mat_copy(A, 3, 3, B);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(B[i], i + 1.0) == 0);
  }

  return 0;
}

int test_mat_row_set() {
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

int test_mat_col_set() {
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

int test_mat_block_get() {
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

int test_mat_block_set() {
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

int test_mat_diag_get() {
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

int test_mat_diag_set() {
  real_t A[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  real_t d[4] = {1.0, 2.0, 3.0};
  mat_diag_set(A, 3, 3, d);

  // print_matrix("A", A, 3, 3);
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 1, 1), 2.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 2, 2), 3.0) == 0);

  return 0;
}

int test_mat_triu() {
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

int test_mat_tril() {
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

int test_mat_trace() {
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

int test_mat_transpose() {
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

int test_mat_add() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
  real_t C[9] = {0.0};
  mat_add(A, B, C, 3, 3);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], 10.0) == 0);
  }

  return 0;
}

int test_mat_sub() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  mat_sub(A, B, C, 3, 3);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], 0.0) == 0);
  }

  return 0;
}

int test_mat_scale() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  mat_scale(A, 3, 3, 2.0);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(A[i], 2 * (i + 1)) == 0);
  }

  return 0;
}

int test_vec_add() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
  real_t C[9] = {0.0};
  vec_add(A, B, C, 9);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], 10.0) == 0);
  }

  return 0;
}

int test_vec_sub() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  vec_sub(A, B, C, 9);
  for (int i = 0; i < 9; i++) {
    MU_ASSERT(fltcmp(C[i], 0.0) == 0);
  }

  return 0;
}

int test_dot() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[3] = {1.0, 2.0, 3.0};
  real_t C[3] = {0.0};

  /* Multiply matrix A and B */
  dot(A, 3, 3, B, 3, 1, C);

  MU_ASSERT(fltcmp(C[0], 14.0) == 0);
  MU_ASSERT(fltcmp(C[1], 32.0) == 0);
  MU_ASSERT(fltcmp(C[2], 50.0) == 0);

  return 0;
}

int test_bdiag_inv() {
  int num_rows = 0;
  int num_cols = 0;
  real_t *H = mat_load("/tmp/H.csv", &num_rows, &num_cols);

  // Invert taking advantage of block diagonal structure
  {
    real_t *H_inv = CALLOC(real_t, num_rows * num_rows);

    // TIC(bdiag_time);
    bdiag_inv(H, num_rows, 6, H_inv);
    // printf("H: %dx%d\n", num_rows, num_cols);
    // printf("invert block diagonal -> time taken: %f\n", TOC(bdiag_time));
    MU_ASSERT(check_inv(H, H_inv, num_rows) == 0);

    free(H_inv);
  }

  // Invert the dumb way
  {

    real_t *H_inv = CALLOC(real_t, num_rows * num_rows);

    // TIC(pinv_time);
    pinv(H, num_rows, num_rows, H_inv);
    // eig_inv(H, num_rows, num_rows, 0, H_inv);
    // printf("invert dumb way -> time taken: %f\n", TOC(pinv_time));
    MU_ASSERT(check_inv(H, H_inv, num_rows) == 0);

    free(H_inv);
  }

  free(H);

  return 0;
}

int test_hat() {
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

int test_check_jacobian() {
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

int test_svd() {
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

int test_pinv() {
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

int test_svd_det() {
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

int test_chol() {
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

int test_chol_solve() {
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

int test_qr() {
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

int test_eig_sym() {
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

int test_eig_inv() {
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

int test_suitesparse_chol_solve() {
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
 * TEST TRANSFORMS
 ******************************************************************************/

int test_tf_rot_set() {
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

int test_tf_trans_set() {
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

int test_tf_trans_get() {
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

int test_tf_rot_get() {
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

int test_tf_quat_get() {
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

int test_tf_inv() {
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

int test_tf_point() {
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

int test_tf_hpoint() {
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

int test_tf_perturb_rot() {
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

int test_tf_perturb_trans() {
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

int test_tf_chain() {
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

int test_euler321() {
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

int test_rot2quat() {
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

int test_quat2euler() {
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

int test_quat2rot() {
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
 * TEST LIE
 ******************************************************************************/

int test_lie_Exp_Log() {
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

/******************************************************************************
 * TEST CV
 ******************************************************************************/

int test_image_setup() {
  return 0;
}

int test_image_load() {
  return 0;
}

int test_image_print_properties() {
  return 0;
}

int test_image_free() {
  return 0;
}

int test_radtan4_distort() {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t p_d[2] = {0};
  radtan4_distort(params, p, p_d);

  // print_vector("p", p, 2);
  // print_vector("p_d", p_d, 2);

  return 0;
}

int test_radtan4_undistort() {
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

int test_radtan4_point_jacobian() {
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

int test_radtan4_params_jacobian() {
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

int test_equi4_distort() {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t p_d[2] = {0};
  equi4_distort(params, p, p_d);

  // print_vector("p", p, 2);
  // print_vector("p_d", p_d, 2);

  return 0;
}

int test_equi4_undistort() {
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

int test_equi4_point_jacobian() {
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

int test_equi4_params_jacobian() {
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

int test_pinhole_focal() {
  const real_t focal = pinhole_focal(640, 90.0);
  MU_ASSERT(fltcmp(focal, 320.0) == 0);
  return 0;
}

int test_pinhole_K() {
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

int test_pinhole_projection_matrix() {
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

int test_pinhole_project() {
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

int test_pinhole_point_jacobian() {
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

int test_pinhole_params_jacobian() {
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

int test_pinhole_radtan4_project() {
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

int test_pinhole_radtan4_project_jacobian() {
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

int test_pinhole_radtan4_params_jacobian() {
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

int test_pinhole_equi4_project() {
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

int test_pinhole_equi4_project_jacobian() {
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

int test_pinhole_equi4_params_jacobian() {
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

int test_linear_triangulation() {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2;
  const real_t cy = image_height / 2;
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
  }

  return 0;
}

int test_homography_find() {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2;
  const real_t cy = image_height / 2;
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
  real_t *pts_i = MALLOC(real_t, num_points * 2);
  real_t *pts_j = MALLOC(real_t, num_points * 2);
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

int test_homography_pose() {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2;
  const real_t cy = image_height / 2;
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
  real_t *world_pts = MALLOC(real_t, N * 3);
  real_t *obj_pts = MALLOC(real_t, N * 3);
  real_t *img_pts = MALLOC(real_t, N * 2);

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

// int test_p3p_kneip() {
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

int test_solvepnp() {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2;
  const real_t cy = image_height / 2;
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
  real_t *world_pts = MALLOC(real_t, N * 3);
  real_t *obj_pts = MALLOC(real_t, N * 3);
  real_t *img_pts = MALLOC(real_t, N * 2);

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

/******************************************************************************
 * TEST CONTROL
 ******************************************************************************/

int test_pid_ctrl() {
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
 * TEST GIMBAL MODEL
 ******************************************************************************/

int test_gimbal() {
  // Gimbal model
  gimbal_model_t model;
  gimbal_model_setup(&model);

  // Gimbal controller
  gimbal_ctrl_t ctrl;
  gimbal_ctrl_setup(&ctrl);

  // Simulate gimbal
  const real_t sp[3] = {0.1, 0.2, 0.3};
  const real_t dt = 0.001;
  const real_t t_end = 3.0;
  real_t t = 0.0;

  int idx = 0;
  const int N = t_end / dt;
  real_t *time_vals = CALLOC(real_t, N);
  real_t *roll_vals = CALLOC(real_t, N);
  real_t *pitch_vals = CALLOC(real_t, N);
  real_t *yaw_vals = CALLOC(real_t, N);

  while (idx < N) {
    const real_t pv[3] = {model.x[0], model.x[2], model.x[4]};

    real_t u[3] = {0};
    gimbal_ctrl_update(&ctrl, sp, pv, dt, u);
    gimbal_model_update(&model, u, dt);

    time_vals[idx] = t;
    roll_vals[idx] = model.x[0];
    pitch_vals[idx] = model.x[2];
    yaw_vals[idx] = model.x[4];

    t += dt;
    idx += 1;
  }

  // Plot
  int debug = 0;
  if (debug) {
    FILE *g = gnuplot_init();
    gnuplot_send(g, "set title 'Gimbal Attitude'");
    gnuplot_send_xy(g, "$r", time_vals, roll_vals, N);
    gnuplot_send_xy(g, "$p", time_vals, pitch_vals, N);
    gnuplot_send_xy(g, "$y", time_vals, yaw_vals, N);
    gnuplot_send(g, "plot $r with lines, $p with lines, $y with lines");
    gnuplot_close(g);
  }

  // Clean up
  free(time_vals);
  free(roll_vals);
  free(pitch_vals);
  free(yaw_vals);

  return 0;
}

/******************************************************************************
 * TEST MAV MODEL
 ******************************************************************************/

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

int test_mav_att_ctrl() {
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

int test_mav_vel_ctrl() {
  mav_model_t mav;
  test_setup_mav(&mav);

  mav_att_ctrl_t mav_att_ctrl;
  mav_vel_ctrl_t mav_vel_ctrl;
  mav_att_ctrl_setup(&mav_att_ctrl);
  mav_vel_ctrl_setup(&mav_vel_ctrl);

  const real_t vel_sp[4] = {0.1, 0.2, 0.0, 0.0}; // vx, vy, vz, yaw
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

int test_mav_pos_ctrl() {
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

int test_mav_waypoints() {
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
 * TEST SENSOR FUSION
 ******************************************************************************/

int test_schur_complement() {
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

  // real_t *Hmm = MALLOC(real_t, m * m);
  // real_t *Hmr = MALLOC(real_t, m * r);
  // real_t *Hrm = MALLOC(real_t, m * r);
  // real_t *Hrr = MALLOC(real_t, r * r);
  // real_t *Hmm_inv = MALLOC(real_t, m * m);

  return 0;
}

int test_timeline() {
  const char *data_dir = TEST_IMU_APRIL;
  const int num_cams = 2;
  const int num_imus = 1;
  timeline_t *timeline = timeline_load_data(data_dir, num_cams, num_imus);
  // printf("timeline->num_cams: %d\n", timeline->num_cams);
  // printf("timeline->num_imus: %d\n", timeline->num_imus);
  // printf("timeline->num_event_types: %d\n", timeline->num_event_types);

  FILE *imu_file = fopen("/tmp/imu.csv", "w");

  for (int k = 0; k < timeline->timeline_length; k++) {
    // Extract timeline events. Add either imu or fiducial event
    for (int i = 0; i < timeline->timeline_events_lengths[k]; i++) {
      timeline_event_t *event = timeline->timeline_events[k][i];
      // const timestamp_t ts = event->ts;

      if (event->type == IMU_EVENT) {
        const imu_event_t *data = &event->data.imu;
        // printf("imu_ts: %ld ", data->ts);
        // printf("acc: (%f, %f, %f) ", data->acc[0], data->acc[1], data->acc[2]);
        // printf("gyr: (%f, %f, %f) ", data->gyr[0], data->gyr[1], data->gyr[2]);
        // printf("\n");

        fprintf(imu_file, "%ld,", data->ts);
        fprintf(imu_file,
                "%lf,%lf,%lf,",
                data->gyr[0],
                data->gyr[1],
                data->gyr[2]);
        fprintf(imu_file,
                "%lf,%lf,%lf",
                data->acc[0],
                data->acc[1],
                data->acc[2]);
        fprintf(imu_file, "\n");

      } else if (event->type == FIDUCIAL_EVENT) {
        // const fiducial_event_t *data = &event->data.fiducial;
        // const int cam_idx = data->cam_idx;
        // printf("cam_ts: %ld \n", data->ts);
        // printf("  cam_idx: %d\n", data->cam_idx);
        // printf("  num_corners: %d\n", data->num_corners);
        // for (int i = 0; i < data->num_corners; i++) {
        //   const real_t *p = data->object_points + i * 3;
        //   const real_t *z = data->keypoints + i * 2;

        //   printf("  ");
        //   printf("%d, ", data->tag_ids[i]);
        //   printf("%d, ", data->corner_indices[i]);
        //   printf("%f, %f, %f, ", p[0], p[1], p[2]);
        //   printf("%f, %f", z[0], z[1]);
        //   printf("\n");
        // }
      }
    }
  }

  // Clean up
  timeline_free(timeline);
  fclose(imu_file);

  return 0;
}

int test_pose() {
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

int test_extrinsics() {
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

int test_fiducial() {
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

int test_fiducial_buffer() {
  const timestamp_t ts = 0;
  const int cam_idx = 1;
  const int n = 2;
  const int tag_ids[2] = {1, 2};
  const int corner_idxs[2] = {1, 2};
  const real_t pts[2 * 3] = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
  const real_t kps[2 * 2] = {0.0, 0.0, 1.0, 1.0};

  fiducial_buffer_t *buf = fiducial_buffer_malloc();
  fiducial_buffer_add(buf, ts, cam_idx, n, tag_ids, corner_idxs, pts, kps);

  MU_ASSERT(buf->data[0]->ts == ts);
  MU_ASSERT(buf->data[0]->cam_idx == cam_idx);
  MU_ASSERT(buf->data[0]->num_corners == n);
  MU_ASSERT(vec_equals(buf->data[0]->object_points, pts, n * 3) == 1);
  MU_ASSERT(vec_equals(buf->data[0]->keypoints, kps, n * 2) == 1);
  MU_ASSERT(buf->size == 1);
  MU_ASSERT(buf->capacity > 1);

  fiducial_buffer_free(buf);

  return 0;
}

int test_imu_biases() {
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

int test_feature() {
  feature_t feature;

  size_t feature_id = 99;
  real_t data[3] = {0.1, 0.2, 0.3};
  feature_init(&feature, feature_id, data);

  printf("sizeof(feature_t): %ld bytes\n", sizeof(feature_t));

  MU_ASSERT(feature.feature_id == feature_id);
  MU_ASSERT(fltcmp(feature.data[0], 0.1) == 0.0);
  MU_ASSERT(fltcmp(feature.data[1], 0.2) == 0.0);
  MU_ASSERT(fltcmp(feature.data[2], 0.3) == 0.0);

  return 0;
}

// int test_idf() {
//   // Body pose
//   pose_t pose;
//   const real_t pose_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   pose_setup(&pose, 0, pose_data);

//   // Extrinsic
//   extrinsic_t cam_ext;
//   const real_t ext_data[7] = {0.01, 0.02, 0.03, 0.5, -0.5, 0.5, -0.5};
//   extrinsic_setup(&cam_ext, ext_data);

//   // Camera parameters
//   camera_params_t cam;
//   const int cam_idx = 0;
//   const int cam_res[2] = {640, 480};
//   const char *proj_model = "pinhole";
//   const char *dist_model = "radtan4";
//   const real_t cam_data[8] = {320, 240, 320, 240, 0.01, 0.01, 0.001, 0.001};
//   camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

//   // Setup feature and image point
//   TF(pose_data, T_WB);
//   TF(ext_data, T_BCi);
//   TF_INV(T_WB, T_BW);
//   TF_INV(T_BCi, T_CiB);
//   TF_CHAIN(T_CiW, 2, T_CiB, T_BW);
//   TF_INV(T_CiW, T_WCi);
//   TF_ROT(T_WCi, C_WCi);
//   TF_TRANS(T_WCi, r_WCi);

//   const size_t feature_id = 0;
//   const real_t p_W[3] = {10.0, randf(-0.5, 0.5), randf(-0.5, 0.5)};
//   real_t z[2] = {0};
//   TF_POINT(T_CiW, p_W, p_Ci);
//   pinhole_radtan4_project(cam_data, p_Ci, z);

//   // Setup IDF
//   pos_t idf_pos;
//   pos_setup(&idf_pos, r_WCi);

//   feature_t idf_param;
//   idf_setup(&idf_param, feature_id, 0, &cam, C_WCi, z);

//   // Reproject IDF to feature in world frame
//   real_t p_W_est[3] = {0};
//   idf_point(&idf_param, idf_pos.data, p_W_est);

//   const real_t dx = p_W_est[0] - p_W[0];
//   const real_t dy = p_W_est[1] - p_W[1];
//   const real_t dz = p_W_est[2] - p_W[2];
//   const real_t dist = sqrt(dx * dx + dy * dy + dz * dz);
//   MU_ASSERT(dist < 1e-1);

//   return 0;
// }

// int test_features() {
//   // XYZ Features
//   // clang-format off
//   size_t feature_ids[3] = {1, 2, 3};
//   real_t params[3 * 3] = {0.1, 0.2, 0.3,
//                           0.4, 0.5, 0.6,
//                           0.7, 0.8, 0.9};
//   // clang-format on

//   // Body pose
//   pose_t pose;
//   const real_t pose_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   pose_setup(&pose, 0, pose_data);

//   // Extrinsic
//   extrinsic_t cam_ext;
//   const real_t ext_data[7] = {0.01, 0.02, 0.03, 0.5, -0.5, 0.5, -0.5};
//   extrinsic_setup(&cam_ext, ext_data);

//   // Camera parameters
//   camera_params_t cam;
//   const int cam_idx = 0;
//   const int cam_res[2] = {640, 480};
//   const char *proj_model = "pinhole";
//   const char *dist_model = "radtan4";
//   const real_t cam_data[8] = {320, 240, 320, 240, 0.01, 0.01, 0.001, 0.001};
//   camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

//   // Setup feature and image point
//   TF(pose_data, T_WB);
//   TF(ext_data, T_BCi);
//   TF_INV(T_WB, T_BW);
//   TF_INV(T_BCi, T_CiB);
//   TF_CHAIN(T_CiW, 2, T_CiB, T_BW);
//   TF_INV(T_CiW, T_WCi);

//   // Setup inverse-depth features and keypoints
//   size_t num_idfs = 10;
//   size_t *idf_ids = MALLOC(size_t, num_idfs);
//   real_t *idf_features = MALLOC(real_t, num_idfs * 3);
//   real_t *idf_keypoints = MALLOC(real_t, num_idfs * 2);
//   for (size_t i = 0; i < num_idfs; i++) {
//     const size_t feature_id = i + (4);
//     const real_t p_W[3] = {10.0, randf(-0.5, 0.5), randf(-0.5, 0.5)};
//     real_t z[2] = {0};
//     TF_POINT(T_CiW, p_W, p_Ci);
//     pinhole_radtan4_project(cam_data, p_Ci, z);

//     idf_ids[i] = feature_id;
//     idf_features[i * 3 + 0] = p_W[0];
//     idf_features[i * 3 + 1] = p_W[1];
//     idf_features[i * 3 + 2] = p_W[2];
//     idf_keypoints[i * 2 + 0] = z[0];
//     idf_keypoints[i * 2 + 1] = z[1];
//   }

//   // Setup
//   features_t *features = features_malloc();

//   // -- Add XYZ features
//   features_add_xyzs(features, feature_ids, params, 3);
//   MU_ASSERT(features->num_features == 3);

//   // // -- Add IDF features
//   // features_add_idfs(features, idf_ids, &cam, T_WCi, idf_keypoints, num_idfs);
//   // MU_ASSERT(features->num_features == 3 + num_idfs);

//   // -- Check features exists
//   MU_ASSERT(features_exists(features, 1) == 1);
//   MU_ASSERT(features_exists(features, 2) == 1);
//   MU_ASSERT(features_exists(features, 3) == 1);
//   MU_ASSERT(features_exists(features, 99) == 0);

//   // -- Get features
//   feature_t *f0 = NULL;
//   feature_t *f1 = NULL;
//   feature_t *f2 = NULL;
//   feature_t *f3 = NULL;

//   features_get_xyz(features, 1, &f0);
//   features_get_xyz(features, 2, &f1);
//   features_get_xyz(features, 3, &f2);
//   features_get_xyz(features, 99, &f3);

//   MU_ASSERT(f0->feature_id == 1);
//   MU_ASSERT(f0->status == 1);
//   MU_ASSERT(vec_equals(f0->data, params + 0, 3) == 1);

//   MU_ASSERT(f1->feature_id == 2);
//   MU_ASSERT(f1->status == 1);
//   MU_ASSERT(vec_equals(f1->data, params + 3, 3) == 1);

//   MU_ASSERT(f2->feature_id == 3);
//   MU_ASSERT(f2->status == 1);
//   MU_ASSERT(vec_equals(f2->data, params + 6, 3) == 1);

//   MU_ASSERT(f3 == NULL);

//   // Clean up
//   features_free(features);
//   free(idf_ids);
//   free(idf_features);
//   free(idf_keypoints);

//   return 0;
// }

int test_time_delay() {
  time_delay_t td;
  time_delay_setup(&td, 1.0);
  MU_ASSERT(fltcmp(td.data[0], 1.0) == 0);
  return 0;
}

int test_joint() {
  joint_t joint;
  joint_setup(&joint, 101, 2, 1.0);
  MU_ASSERT(joint.ts == 101);
  MU_ASSERT(joint.joint_idx == 2);
  MU_ASSERT(fltcmp(joint.data[0], 1.0) == 0);
  return 0;
}

int test_camera_params() {
  camera_params_t camera;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&camera, cam_idx, cam_res, proj_model, dist_model, data);
  // camera_params_print(&camera);

  return 0;
}

int test_triangulation_batch() {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const int cam_res[2] = {image_width, image_height};
  const char *pmodel = "pinhole";
  const char *dmodel = "radtan4";
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2;
  const real_t cy = image_height / 2;
  const real_t proj_params[4] = {fx, fy, cx, cy};
  const real_t data[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  camera_params_t cam_i;
  camera_params_t cam_j;
  camera_params_setup(&cam_i, 0, cam_res, pmodel, dmodel, data);
  camera_params_setup(&cam_j, 1, cam_res, pmodel, dmodel, data);

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
  real_t *kps_i = MALLOC(real_t, N * 2);
  real_t *kps_j = MALLOC(real_t, N * 2);
  real_t *points_gnd = MALLOC(real_t, N * 3);
  real_t *points_est = MALLOC(real_t, N * 3);
  int *status = MALLOC(int, N);

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

int test_pose_factor() {
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

int test_ba_factor() {
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
  camera_params_t cam;
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

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

int test_camera_factor() {
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
  camera_params_t cam;
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {320, 240, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

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

// int test_idf_factor() {
//   // Timestamp
//   timestamp_t ts = 0;

//   // Body pose
//   pose_t pose;
//   const real_t pose_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//   pose_setup(&pose, ts, pose_data);

//   // Extrinsic
//   extrinsic_t cam_ext;
//   const real_t ext_data[7] = {0.01, 0.02, 0.03, 0.5, -0.5, 0.5, -0.5};
//   extrinsic_setup(&cam_ext, ext_data);

//   // Camera parameters
//   camera_params_t cam;
//   const int cam_idx = 0;
//   const int cam_res[2] = {640, 480};
//   const char *proj_model = "pinhole";
//   const char *dist_model = "radtan4";
//   const real_t cam_data[8] = {320, 240, 320, 240, 0.01, 0.01, 0.001, 0.001};
//   camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

//   // Setup feature and image point
//   TF(pose_data, T_WB);
//   TF(ext_data, T_BCi);
//   TF_INV(T_WB, T_BW);
//   TF_INV(T_BCi, T_CiB);
//   TF_CHAIN(T_CiW, 2, T_CiB, T_BW);
//   TF_INV(T_CiW, T_WCi);
//   TF_TRANS(T_WCi, r_WCi);

//   const size_t feature_id = 0;
//   const real_t p_W[3] = {10.0, randf(-0.5, 0.5), randf(-0.5, 0.5)};
//   real_t z[2] = {0};
//   TF_POINT(T_CiW, p_W, p_Ci);
//   pinhole_radtan4_project(cam_data, p_Ci, z);

//   // Setup IDF
//   pos_t idf_pos;
//   pos_setup(&idf_pos, r_WCi);

//   feature_t idf_param;
//   TF_ROT(T_WCi, C_WCi);
//   idf_setup(&idf_param, feature_id, 0, &cam, C_WCi, z);

//   // Setup IDF Factor
//   const real_t var[2] = {1.0, 1.0};
//   idf_factor_t factor;
//   idf_factor_setup(&factor,
//                    &pose,
//                    &cam_ext,
//                    &cam,
//                    &idf_pos,
//                    &idf_param,
//                    ts,
//                    cam_idx,
//                    feature_id,
//                    z,
//                    var);

//   // Check Jacobians
//   const real_t step_size = 1e-8;
//   const real_t tol = 1e-4;
//   const int debug = 0;
//   CHECK_FACTOR_J(0, factor, idf_factor_eval, step_size, tol, debug);
//   CHECK_FACTOR_J(1, factor, idf_factor_eval, step_size, tol, debug);
//   CHECK_FACTOR_J(2, factor, idf_factor_eval, step_size, tol, debug);
//   CHECK_FACTOR_J(3, factor, idf_factor_eval, step_size, tol, debug);
//   CHECK_FACTOR_J(4, factor, idf_factor_eval, step_size, tol, debug);

//   return 0;
// }

int test_imu_buffer_setup() {
  imu_buffer_t imu_buf;
  imu_buffer_setup(&imu_buf);

  return 0;
}

int test_imu_buffer_add() {
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

int test_imu_buffer_clear() {
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

int test_imu_buffer_copy() {
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
  test_data->timestamps = CALLOC(real_t, test_data->num_measurements);
  test_data->poses = CALLOC(real_t *, test_data->num_measurements);
  test_data->velocities = CALLOC(real_t *, test_data->num_measurements);
  test_data->imu_acc = CALLOC(real_t *, test_data->num_measurements);
  test_data->imu_gyr = CALLOC(real_t *, test_data->num_measurements);

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

int test_imu_propagate() {
  // Setup test data
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data, 5.0, 1.0);

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

  MU_PRINT_VECTOR("[gnd] pose_kp1", test_data.poses[n], 7);
  MU_PRINT_VECTOR("[est] pose_kp1", pose_kp1, 7);
  MU_PRINT_VECTOR("[gnd] vel_kp1", test_data.velocities[n], 3);
  MU_PRINT_VECTOR("[est] vel_kp1", vel_kp1, 3);

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

int test_imu_initial_attitude() {
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
  MU_PRINT_VECTOR("[gnd] q_WS", test_data.poses[0], 7);
  MU_PRINT_VECTOR("[est] q_WS", q_WS, 4);

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

int test_imu_factor_form_F_matrix() {
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
  mat_save("/tmp/F.csv", F_dt, 15, 15);

  // Clean up
  free_imu_test_data(&test_data);

  return 0;
}

int test_imu_factor() {
  // Setup test data
  const double circle_r = 1.0;
  const double circle_v = 0.1;
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data, circle_r, circle_v);

  // Setup IMU buffer
  int buf_size = 20;
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

  // print_vector("pose_i", pose_i.data, 7);
  // print_vector("pose_j", pose_j.data, 7);
  // print_vector("dr", factor.dr, 3);
  // print_vector("dv", factor.dv, 3);
  // print_vector("dq", factor.dq, 4);
  // printf("dt: %f\n", factor.Dt);
  // print_vector("r", factor.r, 15);
  mat_save("/tmp/F_.csv", factor.F, 15, 15);
  // mat_save("/tmp/P_.csv", factor.P, 15, 15);

  /*
  const char *cmd = "\
F = csvread('/tmp/F.csv'); \
state_F = csvread('/tmp/F_.csv'); \
subplot(311); \
imagesc(F); \
title('Python'); \
axis 'equal'; \
colorbar(); \
subplot(312); \
imagesc(state_F); \
title('C'); \
axis 'equal'; \
colorbar(); \
subplot(313); \
imagesc(F - state_F); \
title('F - stateF'); \
colorbar(); \
axis 'equal'; \
F(1:3, 1:3); \
state_F(1:3, 1:3); \
ginput();\
";
  char syscmd[9046] = {0};
  sprintf(syscmd, "octave-cli --eval \"%s\"", cmd);
  system(syscmd);
  */

  MU_ASSERT(factor.pose_i == &pose_i);
  MU_ASSERT(factor.vel_i == &vel_i);
  MU_ASSERT(factor.biases_i == &biases_i);
  MU_ASSERT(factor.pose_i == &pose_i);
  MU_ASSERT(factor.vel_j == &vel_j);
  MU_ASSERT(factor.biases_j == &biases_j);

  // Check Jacobians
  // const double tol = 1e-4;
  // const double step_size = 1e-8;
  // eye(factor.sqrt_info, 15, 15);
  // CHECK_FACTOR_J(0, factor, imu_factor_eval, step_size, tol, 0);
  // CHECK_FACTOR_J(1, factor, imu_factor_eval, step_size, tol, 0);
  // CHECK_FACTOR_J(2, factor, imu_factor_eval, step_size, tol, 0);
  // CHECK_FACTOR_J(3, factor, imu_factor_eval, step_size, tol, 0);
  // CHECK_FACTOR_J(4, factor, imu_factor_eval, step_size, tol, 0);
  // CHECK_FACTOR_J(5, factor, imu_factor_eval, step_size, tol, 0);

  // Clean up
  free_imu_test_data(&test_data);

  return 0;
}

int test_joint_factor() {
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
  camera_params_t cam_params;

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
  camera_params_setup(&data->cam_params,
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

int test_calib_camera_factor() {
  // Setup
  test_calib_camera_data_t calib_data;
  test_calib_camera_data_setup(&calib_data);

  calib_camera_factor_t factor;
  const real_t var[2] = {1.0, 1.0};
  calib_camera_factor_setup(&factor,
                            &calib_data.rel_pose,
                            &calib_data.cam_ext,
                            &calib_data.cam_params,
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
  camera_params_t cam_params;
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
  camera_params_setup(&data->cam_params,
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

int test_calib_imucam_factor() {
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
                            &calib_data.cam_params,
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

static void setup_calib_gimbal_factor(calib_gimbal_factor_t *factor,
                                      fiducial_t *fiducial_ext,
                                      extrinsic_t *gimbal_ext,
                                      pose_t *pose,
                                      extrinsic_t *link0,
                                      extrinsic_t *link1,
                                      joint_t *joint0,
                                      joint_t *joint1,
                                      joint_t *joint2,
                                      extrinsic_t *cam_exts,
                                      camera_params_t *cam) {
  // Body pose T_WB
  real_t ypr_WB[3] = {0.0, 0.0, 0.0};
  real_t r_WB[3] = {0.0, 0.0, 0.0};
  real_t T_WB[4 * 4] = {0};
  tf_er(ypr_WB, r_WB, T_WB);

  real_t x_WB[7] = {0};
  tf_vector(T_WB, x_WB);
  pose_setup(pose, 0, x_WB);

  // Fiducial pose T_WF
  real_t ypr_WF[3] = {-M_PI / 2.0, 0.0, M_PI / 2.0};
  real_t r_WF[3] = {0.5, 0.0, 0.0};
  real_t T_WF[4 * 4] = {0};
  tf_er(ypr_WF, r_WF, T_WF);

  real_t x_WF[7] = {0};
  tf_vector(T_WF, x_WF);
  fiducial_setup(fiducial_ext, x_WF);

  // Relative fiducial pose T_BF
  real_t T_BF[4 * 4] = {0};
  TF_INV(T_WB, T_BW);
  dot(T_BW, 4, 4, T_WF, 4, 4, T_BF);

  // Gimbal extrinsic
  real_t ypr_BM0[3] = {0.01, 0.01, 0.01};
  real_t r_BM0[3] = {0.0, 0.0, 0.0};
  real_t T_BM0[4 * 4] = {0};
  gimbal_setup_extrinsic(ypr_BM0, r_BM0, T_BM0, gimbal_ext);

  // Roll link
  real_t ypr_L0M1[3] = {0.0, M_PI / 2, 0.0};
  real_t r_L0M1[3] = {-0.1, 0.0, 0.15};
  real_t T_L0M1[4 * 4] = {0};
  gimbal_setup_extrinsic(ypr_L0M1, r_L0M1, T_L0M1, link0);

  // Pitch link
  real_t ypr_L1M2[3] = {0.0, 0.0, -M_PI / 2.0};
  real_t r_L1M2[3] = {0.0, -0.05, 0.1};
  real_t T_L1M2[4 * 4] = {0};
  gimbal_setup_extrinsic(ypr_L1M2, r_L1M2, T_L1M2, link1);

  // Joint0
  const real_t th0 = 0.01;
  real_t T_M0L0[4 * 4] = {0};
  gimbal_setup_joint(0, 0, th0, T_M0L0, joint0);

  // Joint1
  const real_t th1 = 0.02;
  real_t T_M1L1[4 * 4] = {0};
  gimbal_setup_joint(0, 1, th1, T_M1L1, joint1);

  // Joint2
  const real_t th2 = 0.03;
  real_t T_M2L2[4 * 4] = {0};
  gimbal_setup_joint(0, 2, th2, T_M2L2, joint2);

  // Camera extrinsic
  const real_t ypr_L2C0[3] = {-M_PI / 2, M_PI / 2, 0.0};
  const real_t r_L2C0[3] = {0.0, -0.05, 0.12};
  real_t T_L2C0[4 * 4] = {0};
  gimbal_setup_extrinsic(ypr_L2C0, r_L2C0, T_L2C0, cam_exts);

  // Camera parameters K
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(cam_res[0], fov);
  const real_t fy = pinhole_focal(cam_res[0], fov);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  const real_t cam_params[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(cam,
                      cam_idx,
                      cam_res,
                      proj_model,
                      dist_model,
                      cam_params);

  // Form T_C0F
  real_t T_BC0[4 * 4] = {0};
  real_t T_C0F[4 * 4] = {0};
  tf_chain2(7, T_BM0, T_M0L0, T_L0M1, T_M1L1, T_L1M2, T_M2L2, T_L2C0, T_BC0);
  TF_INV(T_BC0, T_C0B);
  tf_chain2(2, T_C0B, T_BF, T_C0F);

  // Project point to image plane
  const real_t p_FFi[3] = {0.0, 0.0, 0.0};
  real_t p_C0Fi[3] = {0};
  real_t z[2] = {0};
  tf_point(T_C0F, p_FFi, p_C0Fi);
  pinhole_radtan4_project(cam_params, p_C0Fi, z);

  // Setup factor
  const timestamp_t ts = 0;
  const int tag_id = 0;
  const int corner_idx = 0;
  const real_t var[2] = {1.0, 1.0};
  calib_gimbal_factor_setup(factor,
                            fiducial_ext,
                            gimbal_ext,
                            pose,
                            link0,
                            link1,
                            joint0,
                            joint1,
                            joint2,
                            cam_exts,
                            cam,
                            ts,
                            cam_idx,
                            tag_id,
                            corner_idx,
                            p_FFi,
                            z,
                            var);
}

int test_calib_gimbal_factor() {
  calib_gimbal_factor_t factor;
  fiducial_t fiducial_ext;
  extrinsic_t gimbal_ext;
  pose_t pose;
  extrinsic_t link0;
  extrinsic_t link1;
  joint_t joint0;
  joint_t joint1;
  joint_t joint2;
  extrinsic_t cam_exts;
  camera_params_t cam;
  setup_calib_gimbal_factor(&factor,
                            &fiducial_ext,
                            &gimbal_ext,
                            &pose,
                            &link0,
                            &link1,
                            &joint0,
                            &joint1,
                            &joint2,
                            &cam_exts,
                            &cam);

  // Evaluate
  calib_gimbal_factor_eval(&factor);

  // Check Jacobians
  const double tol = 1e-4;
  const double step_size = 1e-8;
  CHECK_FACTOR_J(0, factor, calib_gimbal_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(1, factor, calib_gimbal_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(2, factor, calib_gimbal_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(3, factor, calib_gimbal_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(4, factor, calib_gimbal_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(5, factor, calib_gimbal_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(6, factor, calib_gimbal_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(7, factor, calib_gimbal_factor_eval, step_size, tol, 0);
  CHECK_FACTOR_J(8, factor, calib_gimbal_factor_eval, step_size, tol, 0);

  return 0;
}

int test_marg() {
  // Timestamp
  timestamp_t ts = 0;

  // Extrinsic T_BC
  extrinsic_t cam_ext;
  const real_t ext_data[7] = {0.01, 0.02, 0.03, 0.5, 0.5, -0.5, -0.5};
  extrinsic_setup(&cam_ext, ext_data);
  cam_ext.fix = 1;

  // Camera parameters
  camera_params_t cam;
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {320, 240, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  // Setup features and poses
  int num_poses = 5;
  int num_features = 10;
  pose_t poses[20] = {0};
  feature_t features[100] = {0};
  real_t points[100 * 3] = {0};
  real_t keypoints[100 * 2] = {0};
  camera_factor_t factors[20 * 100];

  for (int i = 0; i < num_features; i++) {
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
  for (int k = 0; k < num_poses; k++) {
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
  param_order_t *hash = NULL;
  int col_idx = 0;
  // -- Add body poses
  for (int i = 0; i < num_poses; i++) {
    void *data = poses[i].data;
    const int fix = 0;
    param_order_add(&hash, POSE_PARAM, fix, data, &col_idx);
  }
  // -- Add points
  for (int i = 0; i < num_features; i++) {
    void *data = &features[i].data;
    const int fix = 0;
    param_order_add(&hash, FEATURE_PARAM, fix, data, &col_idx);
  }
  // -- Add camera extrinsic
  {
    void *data = cam_ext.data;
    const int fix = 1;
    param_order_add(&hash, EXTRINSIC_PARAM, fix, data, &col_idx);
  }
  // -- Add camera parameters
  {
    void *data = cam.data;
    const int fix = 0;
    param_order_add(&hash, CAMERA_PARAM, fix, data, &col_idx);
  }
  // -- Misc
  const int sv_size = col_idx;
  const int r_size = (factor_idx * 2);

  // Form Hessian **before** marginalization
  int r_idx = 0;
  real_t *H = CALLOC(real_t, sv_size * sv_size);
  real_t *g = CALLOC(real_t, sv_size * 1);
  real_t *r = CALLOC(real_t, r_size * 1);
  for (int i = 0; i < (num_poses * num_features); i++) {
    camera_factor_t *factor = &factors[i];
    camera_factor_eval(factor);
    vec_copy(factor->r, factor->r_size, &r[r_idx]);

    solver_fill_hessian(hash,
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
  // marg_factor_eval(marg);

  // Print timings
  // printf("marg->time_hessian_form:     %.4fs\n", marg->time_hessian_form);
  // printf("marg->time_schur_complement: %.4fs\n", marg->time_schur_complement);
  // printf("marg->time_hessian_decomp:   %.4fs\n", marg->time_hessian_decomp);
  // printf("marg->time_fejs:             %.4fs\n", marg->time_fejs);
  // printf("------------------------------------\n");
  // printf("marg->time_total:            %.4fs\n", marg->time_total);

  // Determine parameter order for the marginalized Hessian
  param_order_t *hash_ = NULL;
  int col_idx_ = 0;
  // -- Add body poses
  for (int i = 0; i < num_poses; i++) {
    void *data = poses[i].data;
    const int fix = poses[i].marginalize;
    param_order_add(&hash_, POSE_PARAM, fix, data, &col_idx_);
  }
  // -- Add points
  for (int i = 0; i < num_features; i++) {
    void *data = &features[i].data;
    const int fix = 0;
    param_order_add(&hash_, FEATURE_PARAM, fix, data, &col_idx_);
  }
  // -- Add camera extrinsic
  {
    void *data = cam_ext.data;
    const int fix = 1;
    param_order_add(&hash_, EXTRINSIC_PARAM, fix, data, &col_idx_);
  }
  // -- Add camera parameters
  {
    void *data = cam.data;
    const int fix = 0;
    param_order_add(&hash_, CAMERA_PARAM, fix, data, &col_idx_);
  }
  // -- Misc
  const int sv_size_ = col_idx_;

  // Form marg hessian
  real_t *H_ = CALLOC(real_t, sv_size_ * sv_size_);
  real_t *g_ = CALLOC(real_t, sv_size_ * 1);
  solver_fill_hessian(hash_,
                      marg->num_params,
                      marg->params,
                      marg->jacs,
                      marg->r,
                      marg->r_size,
                      sv_size_,
                      H_,
                      g_);

  // Clean up
  marg_factor_free(marg);
  param_order_free(hash);
  free(H);
  free(g);
  free(r);

  param_order_free(hash_);
  free(H_);
  free(g_);

  return 0;
}

int test_visual_odometry_batch() {
  // Simulate features
  const real_t origin[3] = {0.0, 0.0, 0.0};
  const real_t dim[3] = {5.0, 5.0, 5.0};
  const int num_features = 1000;
  real_t features[3 * 1000] = {0};
  sim_create_features(origin, dim, num_features, features);

  // Camera configuration
  const int res[2] = {640, 480};
  const real_t fov = 90.0;
  const real_t fx = pinhole_focal(res[0], fov);
  const real_t fy = pinhole_focal(res[0], fov);
  const real_t cx = res[0] / 2.0;
  const real_t cy = res[1] / 2.0;
  const real_t cam_vec[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  const char *pmodel = "pinhole";
  const char *dmodel = "radtan4";
  camera_params_t cam0_params;
  camera_params_setup(&cam0_params, 0, res, pmodel, dmodel, cam_vec);

  // IMU-Camera0 extrinsic
  const real_t cam0_ext_ypr[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  const real_t cam0_ext_r[3] = {0.05, 0.0, 0.0};
  TF_ER(cam0_ext_ypr, cam0_ext_r, T_BC0);
  TF_VECTOR(T_BC0, cam0_ext);
  TF_INV(T_BC0, T_C0B);

  // Simulate data
  sim_circle_t conf;
  sim_circle_defaults(&conf);
  sim_camera_data_t *cam0_data = sim_camera_circle_trajectory(&conf,
                                                              T_BC0,
                                                              &cam0_params,
                                                              features,
                                                              num_features);
  // Setup factor graph
  // feature_hash_t *feature_params = NULL;

  // fgraph_t *fg = fgraph_malloc();
  // // -- Add features
  // for (int feature_id = 0; feature_id < num_features; feature_id++) {
  //   fgraph_add_feature(fg, feature_id, features + feature_id * 3, 0);
  // }
  // // -- Add camera
  // const int cam0_id = fgraph_add_camera(fg, 0, res, pmodel, dmodel, cam_vec, 0);
  // const int cam0_ext_id = fgraph_add_cam_ext(fg, 0, cam0_ext, 0);

  // for (size_t k = 0; k < cam0_data->num_frames; k++) {
  //   const sim_camera_frame_t *cam0_frame = cam0_data->frames[k];

  //   // Add pose
  //   const real_t *cam0_pose = &cam0_data->poses[k * 7];
  //   TF(cam0_pose, T_WC0);
  //   TF_CHAIN(T_WB, 2, T_WC0, T_C0B);
  //   TF_VECTOR(T_WB, body_pose);
  //   pose_random_perturb(body_pose, 0.1, 0.1);
  //   const int pose_id = fgraph_add_pose(fg, k, body_pose, 0);

  //   // Add camera factors
  //   for (int i = 0; i < cam0_frame->num_measurements; i++) {
  //     const int feature_id = cam0_frame->feature_ids[i];
  //     const real_t *kp = cam0_frame->keypoints + i * 2;
  //     const int param_ids[4] = {pose_id, cam0_ext_id, feature_id, cam0_id};
  //     const real_t var[2] = {1.0, 1.0};
  //     fgraph_add_camera_factor(fg, param_ids, kp, var);
  //   }
  // }

  // // Solve
  // solver_t solver;
  // solver_setup(&solver);

  // solver.verbose = 1;
  // solver.max_iter = 5;
  // solver.cost_func = &fgraph_cost;
  // solver.param_order_func = &fgraph_param_order;
  // solver.linearize_func = &fgraph_linearize_compact;
  // solver_solve(&solver, fg);

  // Clean up
  sim_camera_data_free(cam0_data);
  // fgraph_free(fg);

  return 0;
}

int test_inertial_odometry_batch() {
  // Setup test data
  imu_test_data_t test_data;
  setup_imu_test_data(&test_data, 1.0, 1.0);

  // Inertial Odometry
  const int num_partitions = test_data.num_measurements / 20.0;
  const size_t N = test_data.num_measurements / (real_t) num_partitions;
  inertial_odometry_t *odom = MALLOC(inertial_odometry_t, 1);
  // -- IMU params
  odom->imu_params.imu_idx = 0;
  odom->imu_params.rate = 200.0;
  odom->imu_params.sigma_a = 0.08;
  odom->imu_params.sigma_g = 0.004;
  odom->imu_params.sigma_aw = 0.00004;
  odom->imu_params.sigma_gw = 2.0e-6;
  odom->imu_params.g = 9.81;
  // -- Variables
  odom->num_factors = 0;
  odom->factors = MALLOC(imu_factor_t, num_partitions);
  odom->poses = MALLOC(pose_t, num_partitions + 1);
  odom->vels = MALLOC(velocity_t, num_partitions + 1);
  odom->biases = MALLOC(imu_biases_t, num_partitions + 1);

  const timestamp_t ts_i = test_data.timestamps[0];
  const real_t *v_i = test_data.velocities[0];
  const real_t ba_i[3] = {0, 0, 0};
  const real_t bg_i[3] = {0, 0, 0};
  pose_setup(&odom->poses[0], ts_i, test_data.poses[0]);
  velocity_setup(&odom->vels[0], ts_i, v_i);
  imu_biases_setup(&odom->biases[0], ts_i, ba_i, bg_i);

  for (int i = 1; i < num_partitions; i++) {
    const int ks = i * N;
    const int ke = MIN((i + 1) * N - 1, test_data.num_measurements - 1);

    // Setup imu buffer
    imu_buffer_t imu_buf;
    imu_buffer_setup(&imu_buf);
    for (size_t k = 0; k < N; k++) {
      const timestamp_t ts = test_data.timestamps[ks + k];
      const real_t *acc = test_data.imu_acc[ks + k];
      const real_t *gyr = test_data.imu_gyr[ks + k];
      imu_buffer_add(&imu_buf, ts, acc, gyr);
    }

    // Setup parameters
    const timestamp_t ts_j = test_data.timestamps[ke];
    const real_t *v_j = test_data.velocities[ke];
    const real_t ba_j[3] = {0, 0, 0};
    const real_t bg_j[3] = {0, 0, 0};
    pose_setup(&odom->poses[i], ts_j, test_data.poses[ke]);
    velocity_setup(&odom->vels[i], ts_j, v_j);
    imu_biases_setup(&odom->biases[i], ts_j, ba_j, bg_j);

    // Setup IMU factor
    imu_factor_setup(&odom->factors[i - 1],
                     &odom->imu_params,
                     &imu_buf,
                     &odom->poses[i - 1],
                     &odom->vels[i - 1],
                     &odom->biases[i - 1],
                     &odom->poses[i],
                     &odom->vels[i],
                     &odom->biases[i]);
    odom->num_factors++;
  }

  // Save ground truth
  inertial_odometry_save(odom, "/tmp/imu_odom-gnd.csv");

  // Perturb ground truth
  for (int k = 0; k <= odom->num_factors; k++) {
    odom->poses[k].data[0] += randf(-1.0, 1.0);
    odom->poses[k].data[1] += randf(-1.0, 1.0);
    odom->poses[k].data[2] += randf(-1.0, 1.0);
    quat_perturb(odom->poses[k].data + 3, 0, randf(-1e-1, 1e-1));
    quat_perturb(odom->poses[k].data + 3, 1, randf(-1e-1, 1e-1));
    quat_perturb(odom->poses[k].data + 3, 2, randf(-1e-1, 1e-1));

    odom->vels[k].data[0] += randf(-1.0, 1.0);
    odom->vels[k].data[1] += randf(-1.0, 1.0);
    odom->vels[k].data[2] += randf(-1.0, 1.0);
  }
  inertial_odometry_save(odom, "/tmp/imu_odom-init.csv");

  // Solve
  solver_t solver;
  solver_setup(&solver);
  solver.verbose = 1;
  solver.param_order_func = &inertial_odometry_param_order;
  solver.cost_func = &inertial_odometry_cost;
  solver.linearize_func = &inertial_odometry_linearize_compact;
  solver_solve(&solver, odom);
  inertial_odometry_save(odom, "/tmp/imu_odom-est.csv");

  // Clean up
  inertial_odometry_free(odom);
  free_imu_test_data(&test_data);

  return 0;
}

// int test_visual_inertial_odometry_batch() {
//   // Simulate features
//   const real_t origin[3] = {0.0, 0.0, 0.0};
//   const real_t dim[3] = {5.0, 5.0, 5.0};
//   const int num_features = 1000;
//   real_t features[3 * 1000] = {0};
//   sim_create_features(origin, dim, num_features, features);

//   // Camera configuration
//   const int res[2] = {640, 480};
//   const real_t fov = 90.0;
//   const real_t fx = pinhole_focal(res[0], fov);
//   const real_t fy = pinhole_focal(res[0], fov);
//   const real_t cx = res[0] / 2.0;
//   const real_t cy = res[1] / 2.0;
//   const real_t cam_vec[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
//   const char *pmodel = "pinhole";
//   const char *dmodel = "radtan4";
//   camera_params_t cam0_params;
//   camera_params_setup(&cam0_params, 0, res, pmodel, dmodel, cam_vec);

//   // IMU configuration
//   imu_params_t imu_params;
//   imu_params.imu_idx = 0;
//   imu_params.rate = 200.0;
//   imu_params.sigma_a = 0.08;
//   imu_params.sigma_g = 0.004;
//   imu_params.sigma_aw = 0.00004;
//   imu_params.sigma_gw = 2.0e-6;
//   imu_params.g = 9.81;

//   // IMU-Camera0 extrinsic
//   const real_t cam0_ext_ypr[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
//   const real_t cam0_ext_r[3] = {0.05, 0.0, 0.0};
//   TF_ER(cam0_ext_ypr, cam0_ext_r, T_BC0);
//   TF_VECTOR(T_BC0, cam0_ext);
//   TF_INV(T_BC0, T_C0B);

//   // Simulate data
//   sim_circle_t conf;
//   sim_circle_defaults(&conf);
//   sim_imu_data_t *imu_data = sim_imu_circle_trajectory(&conf);
//   sim_camera_data_t *cam0_data = sim_camera_circle_trajectory(&conf,
//                                                               T_BC0,
//                                                               &cam0_params,
//                                                               features,
//                                                               num_features);
//   // Setup factor graph
//   fgraph_t *fg = fgraph_malloc();
//   // -- Add features
//   for (int feature_id = 0; feature_id < num_features; feature_id++) {
//     fgraph_add_feature(fg, feature_id, features + feature_id * 3, 0);
//   }
//   // -- Add camera
//   const int cam0_id = fgraph_add_camera(fg, 0, res, pmodel, dmodel, cam_vec, 0);
//   const int cam0_ext_id = fgraph_add_cam_ext(fg, 0, cam0_ext, 0);

//   int initialized = 0;
//   int pose_i_id = -1;
//   int vel_i_id = -1;
//   int biases_i_id = -1;
//   // for (size_t k = 1; k < cam0_data->num_frames; k++) {
//   for (size_t k = 1; k < 20; k++) {
//     const int64_t ts_i = cam0_data->timestamps[k - 1];
//     const int64_t ts_j = cam0_data->timestamps[k];
//     const sim_camera_frame_t *frame_i = cam0_data->frames[k - 1];
//     const sim_camera_frame_t *frame_j = cam0_data->frames[k];

//     // Add pose
//     const real_t *cam_pose_i = &cam0_data->poses[(k - 1) * 7];
//     const real_t *cam_pose_j = &cam0_data->poses[(k + 0) * 7];

//     // Add camera factors at i
//     if (initialized == 0) {
//       // Add pose i
//       TF(cam_pose_i, T_WC0_i);
//       TF_CHAIN(T_WB_i, 2, T_WC0_i, T_C0B);
//       TF_VECTOR(T_WB_i, pose_i);
//       // pose_random_perturb(pose_i, 0.1, 0.1);
//       pose_i_id = fgraph_add_pose(fg, ts_i, pose_i, 0);

//       // Add speed and biases at i
//       const real_t *vel_i = &imu_data->velocities[(k - 1) * 3];
//       const real_t ba_i[3] = {0.0, 0.0, 0.0};
//       const real_t bg_i[3] = {0.0, 0.0, 0.0};
//       vel_i_id = fgraph_add_velocity(fg, ts_i, vel_i, 0);
//       biases_i_id = fgraph_add_imu_biases(fg, ts_i, ba_i, bg_i, 0);

//       // // Add camera factors at i
//       // for (int i = 0; i < frame_i->num_measurements; i++) {
//       //   const int feature_id = frame_i->feature_ids[i];
//       //   const real_t *z = &frame_i->keypoints[i * 2];
//       //   const int param_ids[4] = {pose_i_id, cam0_ext_id, feature_id, cam0_id};
//       //   const real_t var[2] = {1.0, 1.0};
//       //   fgraph_add_camera_factor(fg, param_ids, z, var);
//       // }

//       initialized = 1;
//     }

//     // Add camera factors at j
//     {
//       // Add pose j
//       TF(cam_pose_j, T_WC0_j);
//       TF_CHAIN(T_WB_j, 2, T_WC0_j, T_C0B);
//       TF_VECTOR(T_WB_j, pose_j);
//       // pose_random_perturb(pose_j, 0.1, 0.1);
//       const int pose_j_id = fgraph_add_pose(fg, ts_j, pose_j, 0);

//       // Add speed and biases at j
//       const real_t *vel_j = &imu_data->velocities[k * 3];
//       const real_t ba_j[3] = {0.0, 0.0, 0.0};
//       const real_t bg_j[3] = {0.0, 0.0, 0.0};
//       const int vel_j_id = fgraph_add_velocity(fg, ts_j, vel_j, 0);
//       const int biases_j_id = fgraph_add_imu_biases(fg, ts_j, ba_j, bg_j, 0);

//       // // Add camera factors at j
//       // for (int i = 0; i < frame_j->num_measurements; i++) {
//       //   const int feature_id = frame_j->feature_ids[i];
//       //   const real_t *z = &frame_j->keypoints[i * 2];
//       //   const int param_ids[4] = {pose_j_id, cam0_ext_id, feature_id, cam0_id};
//       //   const real_t var[2] = {1.0, 1.0};
//       //   fgraph_add_camera_factor(fg, param_ids, z, var);
//       // }

//       // Add imu factor between i and j
//       int param_ids[6] = {0};
//       param_ids[0] = pose_i_id;
//       param_ids[1] = vel_i_id;
//       param_ids[2] = biases_i_id;
//       param_ids[3] = pose_j_id;
//       param_ids[4] = vel_j_id;
//       param_ids[5] = biases_j_id;
//       imu_buffer_t imu_buf;
//       sim_imu_measurements(imu_data, ts_i, ts_j, &imu_buf);
//       fgraph_add_imu_factor(fg, 0, param_ids, &imu_params, &imu_buf);

//       // Update
//       pose_i_id = pose_j_id;
//       vel_i_id = vel_j_id;
//       biases_i_id = biases_j_id;
//     }
//   }

//   // Solve
//   solver_t solver;
//   solver_setup(&solver);

//   solver.verbose = 1;
//   solver.max_iter = 5;
//   solver.cost_func = &fgraph_cost;
//   solver.param_order_func = &fgraph_param_order;
//   solver.linearize_func = &fgraph_linearize_compact;
//   solver_solve(&solver, fg);

//   // Clean up
//   sim_imu_data_free(imu_data);
//   sim_camera_data_free(cam0_data);
//   fgraph_free(fg);

//   return 0;
// }

int test_tsf() {
  // Camera config
  const int cam_res[2] = {640, 480};
  const real_t fov = 90.0;
  const real_t fx = pinhole_focal(cam_res[0], fov);
  const real_t fy = pinhole_focal(cam_res[0], fov);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  const real_t cam_vec[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  const char *pmodel = "pinhole";
  const char *dmodel = "radtan4";

  const real_t cam0_ext_ypr[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  const real_t cam0_ext_r[3] = {0.05, 0.0, 0.0};
  const real_t cam1_ext_ypr[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  const real_t cam1_ext_r[3] = {-0.05, 0.0, 0.0};
  TF_ER(cam0_ext_ypr, cam0_ext_r, T_SC0);
  TF_ER(cam1_ext_ypr, cam1_ext_r, T_SC1);
  TF_VECTOR(T_SC0, cam0_ext);
  TF_VECTOR(T_SC1, cam1_ext);

  // IMU config
  const real_t imu_rate = 200;
  const real_t sigma_a = 0.08;
  const real_t sigma_g = 0.004;
  const real_t sigma_aw = 0.00004;
  const real_t sigma_gw = 2.0e-6;
  const real_t g = 9.81;
  TF_IDENTITY(T_BS);
  TF_VECTOR(T_BS, imu0_ext);

  // Simulate VO
  tsf_t *tsf = tsf_malloc();
  tsf_add_camera(tsf, 0, cam_res, pmodel, dmodel, cam_vec, cam0_ext);
  tsf_add_camera(tsf, 1, cam_res, pmodel, dmodel, cam_vec, cam1_ext);
  tsf_add_imu(tsf, imu_rate, sigma_aw, sigma_gw, sigma_a, sigma_g, g, imu0_ext);

  // Loop through timeline
  sim_circle_camera_imu_t *sim_data = sim_circle_camera_imu();
  timeline_t *timeline = sim_data->timeline;
  // for (int k = 0; k < timeline->timeline_length; k++) {
  //   printf("HERE\n");
  //   // Extract timeline events
  //   for (int i = 0; i < timeline->timeline_events_lengths[k]; i++) {
  //     timeline_event_t *event = timeline->timeline_events[k][i];
  //     const timestamp_t ts = event->ts;

  //     if (event->type == IMU_EVENT) {
  //       const imu_event_t *data = &event->data.imu;
  //       tsf_imu_event(tsf, ts, data->acc, data->gyr);
  //     }
  //   }
  // }

  // Clean up
  sim_circle_camera_imu_free(sim_data);
  tsf_free(tsf);

  return 0;
}

#ifdef USE_CERES

/**
 * This is the equivalent of a use-defined CostFunction in the C++ Ceres API.
 * This is passed as a callback to the Ceres C API, which internally converts
 * the callback into a CostFunction.
 */
static int ceres_exp_residual(void *user_data,
                              double **parameters,
                              double *residuals,
                              double **jacobians) {
  double *measurement = (double *) user_data;
  double x = measurement[0];
  double y = measurement[1];
  double m = parameters[0][0];
  double c = parameters[1][0];
  residuals[0] = y - exp(m * x + c);
  if (jacobians == NULL) {
    return 1;
  }
  if (jacobians[0] != NULL) {
    jacobians[0][0] = -x * exp(m * x + c); /* dr/dm */
  }
  if (jacobians[1] != NULL) {
    jacobians[1][0] = -exp(m * x + c); /* dr/dc */
  }
  return 1;
}

int test_ceres_example() {
  int num_observations = 67;
  double data[] = {
      0.000000e+00, 1.133898e+00, 7.500000e-02, 1.334902e+00, 1.500000e-01,
      1.213546e+00, 2.250000e-01, 1.252016e+00, 3.000000e-01, 1.392265e+00,
      3.750000e-01, 1.314458e+00, 4.500000e-01, 1.472541e+00, 5.250000e-01,
      1.536218e+00, 6.000000e-01, 1.355679e+00, 6.750000e-01, 1.463566e+00,
      7.500000e-01, 1.490201e+00, 8.250000e-01, 1.658699e+00, 9.000000e-01,
      1.067574e+00, 9.750000e-01, 1.464629e+00, 1.050000e+00, 1.402653e+00,
      1.125000e+00, 1.713141e+00, 1.200000e+00, 1.527021e+00, 1.275000e+00,
      1.702632e+00, 1.350000e+00, 1.423899e+00, 1.425000e+00, 1.543078e+00,
      1.500000e+00, 1.664015e+00, 1.575000e+00, 1.732484e+00, 1.650000e+00,
      1.543296e+00, 1.725000e+00, 1.959523e+00, 1.800000e+00, 1.685132e+00,
      1.875000e+00, 1.951791e+00, 1.950000e+00, 2.095346e+00, 2.025000e+00,
      2.361460e+00, 2.100000e+00, 2.169119e+00, 2.175000e+00, 2.061745e+00,
      2.250000e+00, 2.178641e+00, 2.325000e+00, 2.104346e+00, 2.400000e+00,
      2.584470e+00, 2.475000e+00, 1.914158e+00, 2.550000e+00, 2.368375e+00,
      2.625000e+00, 2.686125e+00, 2.700000e+00, 2.712395e+00, 2.775000e+00,
      2.499511e+00, 2.850000e+00, 2.558897e+00, 2.925000e+00, 2.309154e+00,
      3.000000e+00, 2.869503e+00, 3.075000e+00, 3.116645e+00, 3.150000e+00,
      3.094907e+00, 3.225000e+00, 2.471759e+00, 3.300000e+00, 3.017131e+00,
      3.375000e+00, 3.232381e+00, 3.450000e+00, 2.944596e+00, 3.525000e+00,
      3.385343e+00, 3.600000e+00, 3.199826e+00, 3.675000e+00, 3.423039e+00,
      3.750000e+00, 3.621552e+00, 3.825000e+00, 3.559255e+00, 3.900000e+00,
      3.530713e+00, 3.975000e+00, 3.561766e+00, 4.050000e+00, 3.544574e+00,
      4.125000e+00, 3.867945e+00, 4.200000e+00, 4.049776e+00, 4.275000e+00,
      3.885601e+00, 4.350000e+00, 4.110505e+00, 4.425000e+00, 4.345320e+00,
      4.500000e+00, 4.161241e+00, 4.575000e+00, 4.363407e+00, 4.650000e+00,
      4.161576e+00, 4.725000e+00, 4.619728e+00, 4.800000e+00, 4.737410e+00,
      4.875000e+00, 4.727863e+00, 4.950000e+00, 4.669206e+00,
  };

  /* Note: Typically it is better to compact m and c into one block,
   * but in this case use separate blocks to illustrate the use of
   * multiple parameter blocks. */
  double m = 0.0;
  double c = 0.0;
  double *parameter_pointers[] = {&m, &c};
  int parameter_sizes[2] = {1, 1};
  ceres_problem_t *problem;
  ceres_init();
  problem = ceres_create_problem();

  /* Add all the residuals. */
  for (int i = 0; i < num_observations; ++i) {
    ceres_problem_add_residual_block(problem,
                                     ceres_exp_residual,
                                     &data[2 * i],
                                     NULL,
                                     NULL,
                                     1,
                                     2,
                                     parameter_sizes,
                                     parameter_pointers);
  }

  // ceres_solve(problem, 10, 0);
  ceres_solve(problem);
  ceres_free_problem(problem);
  // printf("Initial m: 0.0, c: 0.0\n");
  // printf("Final m: %g, c: %g\n", m, c);

  return 0;
}

#endif // USE_CERES

int test_solver_setup() {
  solver_t solver;
  solver_setup(&solver);
  return 0;
}

typedef struct cam_view_t {
  pose_t pose;
  ba_factor_t factors[1000];
  int num_factors;
  camera_params_t *cam_params;
} cam_view_t;

// int test_solver_eval() {
//   // Load test data
//   const char *dir_path = TEST_SIM_DATA "/cam0";
//   sim_camera_data_t *cam_data = sim_camera_data_load(dir_path);

//   // Camera parameters
//   camera_params_t cam;
//   const int cam_idx = 0;
//   const int cam_res[2] = {640, 480};
//   const char *proj_model = "pinhole";
//   const char *dist_model = "radtan4";
//   const real_t params[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
//   camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, params);

//   // Setup features
//   // -- Load features csv
//   int num_rows = 0;
//   int num_cols = 0;
//   char *features_csv = TEST_SIM_DATA "/features.csv";
//   real_t **features_data = csv_data(features_csv, &num_rows, &num_cols);
//   size_t *feature_ids = MALLOC(size_t, num_rows);
//   real_t *feature_xyzs = MALLOC(real_t, num_rows * 3);
//   for (int i = 0; i < num_rows; i++) {
//     feature_ids[i] = features_data[i][0];
//     feature_xyzs[i * 3 + 0] = features_data[i][1];
//     feature_xyzs[i * 3 + 1] = features_data[i][2];
//     feature_xyzs[i * 3 + 2] = features_data[i][3];
//     free(features_data[i]);
//   }
//   free(features_data);
//   // -- Add features to container
//   features_t *features = features_malloc();
//   features_add_xyzs(features, feature_ids, feature_xyzs, num_rows);
//   free(feature_ids);
//   free(feature_xyzs);

//   // Loop over simulated camera frames
//   const real_t var[2] = {1.0, 1.0};
//   cam_view_t *cam_views = MALLOC(cam_view_t, cam_data->num_frames);
//   for (int k = 0; k < cam_data->num_frames; k++) {
//     // Camera frame
//     const sim_camera_frame_t *frame = cam_data->frames[k];

//     // Pose
//     pose_t *pose = &cam_views[k].pose;
//     pose_setup(pose, frame->ts, &cam_data->poses[k]);

//     // Add factors
//     cam_views[k].num_factors = frame->num_measurements;
//     for (int i = 0; i < frame->num_measurements; i++) {
//       const int feature_id = frame->feature_ids[i];
//       const real_t *z = &frame->keypoints[i];
//       feature_t *f = NULL;
//       features_get_xyz(features, feature_id, &f);

//       // Factor
//       ba_factor_t *factor = &cam_views[k].factors[i];
//       ba_factor_setup(factor, pose, f, &cam, z, var);
//     }
//   }

//   solver_t solver;
//   solver_setup(&solver);

//   // Clean up
//   sim_camera_data_free(cam_data);
//   free(cam_views);
//   features_free(features);

//   return 0;
// }

int test_camchain() {
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

int test_calib_camera_mono_batch() {
  const char *data_path = TEST_CAM_APRIL "/cam0";

  // Initialize camera intrinsics
  const int cam_res[2] = {752, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  const real_t cam_params[8] =
      {495.864541, 495.864541, 375.500000, 239.500000, 0, 0, 0, 0};

  // Setup camera calibration problem
  calib_camera_t *calib = calib_camera_malloc();
  calib->verbose = 0;
  calib->max_iter = 30;
  calib_camera_add_camera(calib,
                          0,
                          cam_res,
                          proj_model,
                          dist_model,
                          cam_params,
                          cam_ext);

  // Batch solve
  calib_camera_add_data(calib, 0, data_path);
  calib_camera_solve(calib);

  // Asserts
  double reproj_rmse = 0.0;
  double reproj_mean = 0.0;
  double reproj_median = 0.0;
  calib_camera_errors(calib, &reproj_rmse, &reproj_mean, &reproj_median);
  MU_ASSERT(reproj_rmse < 0.5);
  MU_ASSERT(reproj_mean < 0.5);
  MU_ASSERT(reproj_median < 0.5);

  // Clean up
  calib_camera_free(calib);

  return 0;
}

int test_calib_camera_mono_ceres() {
  const char *data_path = TEST_CAM_APRIL "/cam0";

  // Initialize camera intrinsics
  const int cam_res[2] = {752, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  const real_t cam_params[8] =
      {495.864541, 495.864541, 375.500000, 239.500000, 0, 0, 0, 0};

  // Setup camera calibration problem
  calib_camera_t *calib = calib_camera_malloc();
  calib->verbose = 0;
  calib_camera_add_camera(calib,
                          0,
                          cam_res,
                          proj_model,
                          dist_model,
                          cam_params,
                          cam_ext);

  // Batch solve
  calib_camera_add_data(calib, 0, data_path);

  // Setup solver
  ceres_init();
  ceres_problem_t *problem = ceres_create_problem();
  ceres_local_parameterization_t *pose_pm =
      ceres_create_pose_local_parameterization();

  for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
    for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
      const timestamp_t ts = calib->timestamps[view_idx];
      calib_camera_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
      if (view == NULL) {
        continue;
      }

      for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
        calib_camera_factor_t *factor = &view->factors[factor_idx];
        real_t **param_ptrs = factor->params;
        int num_residuals = 2;
        int num_params = 3;
        int param_sizes[3] = {
            7, // Pose
            7, // Camera extrinsic
            8, // Camera parameters
        };
        ceres_problem_add_residual_block(problem,
                                         &calib_camera_factor_ceres_eval,
                                         factor,
                                         NULL,
                                         NULL,
                                         num_residuals,
                                         num_params,
                                         param_sizes,
                                         param_ptrs);

        ceres_set_parameterization(problem, param_ptrs[0], pose_pm);
        ceres_set_parameterization(problem, param_ptrs[1], pose_pm);
      } // For each calib factor
    }   // For each cameras
  }     // For each views

  // Solve
  // ceres_solve(problem, 20, 0);
  ceres_solve(problem);
  // calib_camera_print(calib);

  // Asserts
  double reproj_rmse = 0.0;
  double reproj_mean = 0.0;
  double reproj_median = 0.0;
  calib_camera_errors(calib, &reproj_rmse, &reproj_mean, &reproj_median);
  MU_ASSERT(reproj_rmse < 0.5);
  MU_ASSERT(reproj_mean < 0.5);
  MU_ASSERT(reproj_median < 0.5);

  // Clean up
  calib_camera_free(calib);
  ceres_free_problem(problem);

  return 0;
}

int test_calib_camera_mono_incremental() {
  const char *data_path = TEST_CAM_APRIL "/cam0";

  // Initialize camera intrinsics
  const int res[2] = {752, 480};
  const char *pm = "pinhole";
  const char *dm = "radtan4";
  const real_t cam_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  const real_t cam_vec[8] =
      {495.864541, 495.864541, 375.500000, 239.500000, 0, 0, 0, 0};

  // Setup camera calibration problem
  calib_camera_t *calib = calib_camera_malloc();
  calib->verbose = 0;
  calib_camera_add_camera(calib, 0, res, pm, dm, cam_vec, cam_ext);

  // Incremental solve
  int window_size = 2;
  int cam_idx = 0;
  int num_files = 0;
  char **files = list_files(data_path, &num_files);

  calib->verbose = 0;
  // TIC(calib_camera_loop);

  for (int view_idx = 0; view_idx < num_files; view_idx++) {
    // Load aprilgrid
    aprilgrid_t *grid = aprilgrid_load(files[view_idx]);

    // Get aprilgrid measurements
    const timestamp_t ts = grid->timestamp;
    const int num_corners = grid->corners_detected;
    int *tag_ids = MALLOC(int, num_corners);
    int *corner_indices = MALLOC(int, num_corners);
    real_t *kps = MALLOC(real_t, num_corners * 2);
    real_t *pts = MALLOC(real_t, num_corners * 3);
    aprilgrid_measurements(grid, tag_ids, corner_indices, kps, pts);

    // Add view
    calib_camera_add_view(calib,
                          ts,
                          view_idx,
                          cam_idx,
                          num_corners,
                          tag_ids,
                          corner_indices,
                          pts,
                          kps);

    // Incremental solve
    if (calib->num_views >= window_size) {
      calib_camera_marginalize(calib);
    }
    calib_camera_solve(calib);

    // Clean up
    free(tag_ids);
    free(corner_indices);
    free(kps);
    free(pts);
    aprilgrid_free(grid);
  }

  // calib_camera_print(calib);
  // const real_t time_taken = TOC(calib_camera_loop);
  // const real_t rate_hz = num_files / time_taken;
  // printf("%d frames in %.2f [s] or %.2f Hz\n", num_files, time_taken, rate_hz);

  // Clean up
  for (int view_idx = 0; view_idx < num_files; view_idx++) {
    free(files[view_idx]);
  }
  free(files);
  calib_camera_free(calib);

  return 0;
}

int test_calib_camera_stereo_batch() {
  // Initialize camera intrinsics
  int num_cams = 2;
  char *data_dir = TEST_CAM_APRIL "/cam%d";
  const int cam_res[2] = {752, 480};
  const char *pmodel = "pinhole";
  const char *dmodel = "radtan4";
  const real_t focal = pinhole_focal(cam_res[0], 90.0);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  const real_t cam_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  real_t cam[2][8] = {{focal, focal, cx, cy, 0.0, 0.0, 0.0, 0.0},
                      {focal, focal, cx, cy, 0.0, 0.0, 0.0, 0.0}};

  camera_params_t cam_params[2];
  camera_params_setup(&cam_params[0], 0, cam_res, pmodel, dmodel, cam[0]);
  camera_params_setup(&cam_params[1], 1, cam_res, pmodel, dmodel, cam[1]);

  for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
    char data_path[1024] = {0};
    sprintf(data_path, data_dir, cam_idx);

    calib_camera_t *cam_calib = calib_camera_malloc();
    cam_calib->verbose = 0;
    calib_camera_add_camera(cam_calib,
                            0,
                            cam_res,
                            pmodel,
                            dmodel,
                            cam[cam_idx],
                            cam_ext);
    calib_camera_add_data(cam_calib, 0, data_path);
    calib_camera_solve(cam_calib);
    vec_copy(cam_calib->cam_params[0].data, 8, cam_params[cam_idx].data);
    calib_camera_free(cam_calib);
  }

  // Initialize camera extrinsics
  camchain_t *camchain = camchain_malloc(num_cams);
  for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
    char data_path[1024] = {0};
    sprintf(data_path, data_dir, cam_idx);

    // Get camera data
    int num_files = 0;
    char **files = list_files(data_path, &num_files);

    // Exit if no calibration data
    if (num_files == 0) {
      for (int view_idx = 0; view_idx < num_files; view_idx++) {
        free(files[view_idx]);
      }
      free(files);
      return -1;
    }

    for (int view_idx = 0; view_idx < num_files; view_idx++) {
      // Load aprilgrid
      aprilgrid_t *grid = aprilgrid_load(files[view_idx]);
      if (grid->corners_detected == 0) {
        free(files[view_idx]);
        aprilgrid_free(grid);
        continue;
      }

      // Get aprilgrid measurements
      const timestamp_t ts = grid->timestamp;
      const int n = grid->corners_detected;
      int *tag_ids = MALLOC(int, n);
      int *corner_indices = MALLOC(int, n);
      real_t *kps = MALLOC(real_t, n * 2);
      real_t *pts = MALLOC(real_t, n * 3);
      aprilgrid_measurements(grid, tag_ids, corner_indices, kps, pts);

      // Estimate relative pose T_CiF and add to camchain
      real_t T_CiF[4 * 4] = {0};
      if (solvepnp_camera(&cam_params[cam_idx], kps, pts, n, T_CiF) == 0) {
        camchain_add_pose(camchain, cam_idx, ts, T_CiF);
      }

      // Clean up
      free(tag_ids);
      free(corner_indices);
      free(kps);
      free(pts);
      aprilgrid_free(grid);
      free(files[view_idx]);
    }
    free(files);
  }
  camchain_adjacency(camchain);
  real_t T_CiCj[4 * 4] = {0};
  camchain_find(camchain, 0, 1, T_CiCj);
  camchain_free(camchain);

  // Setup Camera calibrator
  const real_t cam0_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  TF_VECTOR(T_CiCj, cam1_ext);

  calib_camera_t *stereo_calib = calib_camera_malloc();
  stereo_calib->verbose = 0;
  stereo_calib->max_iter = 50;
  calib_camera_add_camera(stereo_calib,
                          0,
                          cam_res,
                          pmodel,
                          dmodel,
                          cam_params[0].data,
                          cam0_ext);
  calib_camera_add_camera(stereo_calib,
                          1,
                          cam_res,
                          pmodel,
                          dmodel,
                          cam_params[1].data,
                          cam1_ext);
  for (int cam_idx = 0; cam_idx < stereo_calib->num_cams; cam_idx++) {
    char data_path[1024] = {0};
    sprintf(data_path, data_dir, cam_idx);
    calib_camera_add_data(stereo_calib, cam_idx, data_path);
  }
  calib_camera_solve(stereo_calib);
  // calib_camera_print(stereo_calib);

  // Asserts
  double reproj_rmse = 0.0;
  double reproj_mean = 0.0;
  double reproj_median = 0.0;
  calib_camera_errors(stereo_calib, &reproj_rmse, &reproj_mean, &reproj_median);
  MU_ASSERT(reproj_rmse < 0.5);
  MU_ASSERT(reproj_mean < 0.5);
  MU_ASSERT(reproj_median < 0.5);

  // Clean up
  calib_camera_free(stereo_calib);

  return 0;
}

int test_calib_camera_stereo_ceres() {
  // Initialize camera intrinsics
  int num_cams = 2;
  char *data_dir = TEST_CAM_APRIL "/cam%d";
  const int cam_res[2] = {752, 480};
  const char *pmodel = "pinhole";
  const char *dmodel = "radtan4";
  const real_t focal = pinhole_focal(cam_res[0], 90.0);
  const real_t cx = cam_res[0] / 2.0;
  const real_t cy = cam_res[1] / 2.0;
  real_t cam[2][8] = {{focal, focal, cx, cy, 0.0, 0.0, 0.0, 0.0},
                      {focal, focal, cx, cy, 0.0, 0.0, 0.0, 0.0}};
  camera_params_t cam_params[2];
  camera_params_setup(&cam_params[0], 0, cam_res, pmodel, dmodel, cam[0]);
  camera_params_setup(&cam_params[1], 1, cam_res, pmodel, dmodel, cam[1]);

  // Initialize camera extrinsics
  camchain_t *camchain = camchain_malloc(num_cams);
  for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
    char data_path[1024] = {0};
    sprintf(data_path, data_dir, cam_idx);

    // Get camera data
    int num_files = 0;
    char **files = list_files(data_path, &num_files);

    // Exit if no calibration data
    if (num_files == 0) {
      for (int view_idx = 0; view_idx < num_files; view_idx++) {
        free(files[view_idx]);
      }
      free(files);
      return -1;
    }

    for (int view_idx = 0; view_idx < num_files; view_idx++) {
      // Load aprilgrid
      aprilgrid_t *grid = aprilgrid_load(files[view_idx]);
      if (grid->corners_detected == 0) {
        free(files[view_idx]);
        aprilgrid_free(grid);
        continue;
      }

      // Get aprilgrid measurements
      const timestamp_t ts = grid->timestamp;
      const int n = grid->corners_detected;
      int *tag_ids = MALLOC(int, n);
      int *corner_indices = MALLOC(int, n);
      real_t *kps = MALLOC(real_t, n * 2);
      real_t *pts = MALLOC(real_t, n * 3);
      aprilgrid_measurements(grid, tag_ids, corner_indices, kps, pts);

      // Estimate relative pose T_CiF and add to camchain
      real_t T_CiF[4 * 4] = {0};
      if (solvepnp_camera(&cam_params[cam_idx], kps, pts, n, T_CiF) == 0) {
        camchain_add_pose(camchain, cam_idx, ts, T_CiF);
      }

      // Clean up
      free(tag_ids);
      free(corner_indices);
      free(kps);
      free(pts);
      aprilgrid_free(grid);
      free(files[view_idx]);
    }
    free(files);
  }
  camchain_adjacency(camchain);
  real_t T_CiCj[4 * 4] = {0};
  camchain_find(camchain, 0, 1, T_CiCj);
  camchain_free(camchain);

  // Setup Camera calibrator
  const real_t cam0_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  TF_VECTOR(T_CiCj, cam1_ext);

  calib_camera_t *calib = calib_camera_malloc();
  calib->verbose = 0;
  calib_camera_add_camera(calib,
                          0,
                          cam_res,
                          pmodel,
                          dmodel,
                          cam_params[0].data,
                          cam0_ext);
  calib_camera_add_camera(calib,
                          1,
                          cam_res,
                          pmodel,
                          dmodel,
                          cam_params[1].data,
                          cam1_ext);
  for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
    char data_path[1024] = {0};
    sprintf(data_path, data_dir, cam_idx);
    calib_camera_add_data(calib, cam_idx, data_path);
  }
  // calib_camera_solve(calib);

  // Setup solver
  ceres_init();
  ceres_problem_t *problem = ceres_create_problem();
  ceres_local_parameterization_t *pose_pm =
      ceres_create_pose_local_parameterization();

  for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
    for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
      const timestamp_t ts = calib->timestamps[view_idx];
      calib_camera_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
      if (view == NULL) {
        continue;
      }

      for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
        calib_camera_factor_t *factor = &view->factors[factor_idx];
        real_t **param_ptrs = factor->params;
        int num_residuals = 2;
        int num_params = 3;
        int param_sizes[3] = {
            7, // Pose
            7, // Camera extrinsic
            8, // Camera parameters
        };
        ceres_problem_add_residual_block(problem,
                                         &calib_camera_factor_ceres_eval,
                                         factor,
                                         NULL,
                                         NULL,
                                         num_residuals,
                                         num_params,
                                         param_sizes,
                                         param_ptrs);

        ceres_set_parameterization(problem, param_ptrs[0], pose_pm);
        ceres_set_parameterization(problem, param_ptrs[1], pose_pm);
      } // For each calib factor
    }   // For each cameras
  }     // For each views

  // Solve
  // ceres_solve(problem, 20, 0);
  ceres_solve(problem);
  // calib_camera_print(calib);

  // Asserts
  double reproj_rmse = 0.0;
  double reproj_mean = 0.0;
  double reproj_median = 0.0;
  calib_camera_errors(calib, &reproj_rmse, &reproj_mean, &reproj_median);
  MU_ASSERT(reproj_rmse < 0.5);
  MU_ASSERT(reproj_mean < 0.5);
  MU_ASSERT(reproj_median < 0.5);

  // Clean up
  calib_camera_free(calib);
  ceres_free_problem(problem);

  return 0;
}

int test_calib_imucam_view() {
  // Setup Camera
  const int cam_res[2] = {752, 480};
  const char *pmodel = "pinhole";
  const char *dmodel = "radtan4";
  const real_t cam_vec[8] = {458.0, 457.0, 367.0, 248.0, 0.0, 0.0, 0.0, 0.0};
  const real_t cam_ext_vec[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

  camera_params_t camera_params;
  extrinsic_t camera_extrinsic;
  camera_params_setup(&camera_params, 0, cam_res, pmodel, dmodel, cam_vec);
  extrinsic_setup(&camera_extrinsic, cam_ext_vec);

  // Fiducial
  fiducial_t fiducial;
  const real_t fiducial_vec[7] = {1.0, 0.0, 0.0, 0.5, -0.5, 0.5, -0.5};
  fiducial_setup(&fiducial, fiducial_vec);

  // IMU pose
  const timestamp_t ts = 0;
  const real_t pose_vec[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  pose_t imu_pose;
  pose_setup(&imu_pose, ts, pose_vec);

  // IMU extrinsic
  const real_t imu_ext_vec[7] = {0.0, 0.0, 0.0, 0.5, -0.5, 0.5, -0.5};
  extrinsic_t imu_extrinsic;
  extrinsic_setup(&imu_extrinsic, imu_ext_vec);

  // Time delay
  time_delay_t time_delay;
  time_delay_setup(&time_delay, 0.0);

  // Create calib imucam view
  const int view_idx = 0;
  const int cam_idx = 0;
  const int num_corners = 1;
  const int tag_ids[1] = {0};
  const int corner_indices[1] = {0};
  const real_t object_points[3] = {
      0.0,
      0.0,
      0.0,
  };

  // -- Transform fiducial point from fiducial frame to camera frame
  const real_t p_F[3] = {object_points[0], object_points[1], object_points[2]};
  TF(fiducial_vec, T_WF);
  TF(pose_vec, T_WS);
  TF(imu_ext_vec, T_BS);
  TF(cam_ext_vec, T_BC0);
  TF_INV(T_WS, T_SW);
  TF_INV(T_BC0, T_C0B);
  TF_CHAIN(T_C0F, 2, T_C0B, T_BS, T_SW, T_WF);
  TF_POINT(T_C0F, p_F, p_C0);

  // -- Project keypoint
  real_t keypoints[2] = {0};
  pinhole_radtan4_project(cam_vec, p_C0, keypoints);

  // -- Create view
  calib_imucam_view_t *view = calib_imucam_view_malloc(ts,
                                                       view_idx,
                                                       cam_idx,
                                                       num_corners,
                                                       tag_ids,
                                                       corner_indices,
                                                       object_points,
                                                       keypoints,
                                                       &fiducial,
                                                       &imu_pose,
                                                       &imu_extrinsic,
                                                       &camera_extrinsic,
                                                       &camera_params,
                                                       &time_delay);

  // Clean up
  calib_imucam_view_free(view);

  return 0;
}

int test_calib_imucam_add_imu() {
  // Setup
  const int imu_rate = 200;
  const real_t n_a = 0.08;
  const real_t n_g = 0.004;
  const real_t n_aw = 0.00004;
  const real_t n_gw = 2.0e-6;
  const real_t g = 9.81;
  const real_t imu_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

  calib_imucam_t *calib = calib_imucam_malloc();
  calib_imucam_add_imu(calib, imu_rate, n_aw, n_gw, n_a, n_g, g, imu_ext);

  // Assert
  MU_ASSERT(calib->imu_params.rate == imu_rate);
  MU_ASSERT(fltcmp(calib->imu_params.sigma_aw, n_aw) == 0);
  MU_ASSERT(fltcmp(calib->imu_params.sigma_gw, n_gw) == 0);
  MU_ASSERT(fltcmp(calib->imu_params.sigma_a, n_a) == 0);
  MU_ASSERT(fltcmp(calib->imu_params.sigma_g, n_g) == 0);
  MU_ASSERT(fltcmp(calib->imu_params.g, g) == 0);
  MU_ASSERT(calib->num_imus == 1);

  // Clean up
  calib_imucam_free(calib);

  return 0;
}

int test_calib_imucam_add_camera() {
  // Setup
  const int res[2] = {752, 480};
  const char *pm = "pinhole";
  const char *dm = "radtan4";
  const real_t cam_vec[8] = {458.0, 457.0, 367.0, 248.0, 0.0, 0.0, 0.0, 0.0};
  const real_t cam_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

  calib_imucam_t *calib = calib_imucam_malloc();
  calib_imucam_add_camera(calib, 0, res, pm, dm, cam_vec, cam_ext);

  // Assert
  MU_ASSERT(calib->cam_params[0].resolution[0] == 752);
  MU_ASSERT(calib->cam_params[0].resolution[1] == 480);
  MU_ASSERT(strcmp(calib->cam_params[0].proj_model, pm) == 0);
  MU_ASSERT(strcmp(calib->cam_params[0].dist_model, dm) == 0);
  MU_ASSERT(vec_equals(calib->cam_params[0].data, cam_vec, 8) == 1);
  MU_ASSERT(vec_equals(calib->cam_exts[0].data, cam_ext, 7) == 1);
  MU_ASSERT(calib->num_cams == 1);

  // Clean up
  calib_imucam_free(calib);

  return 0;
}

int test_calib_imucam_add_imu_event() {
  // Setup
  calib_imucam_t *calib = calib_imucam_malloc();

  // -- Add imu
  const int imu_rate = 200;
  const real_t n_a = 0.08;
  const real_t n_g = 0.004;
  const real_t n_aw = 0.00004;
  const real_t n_gw = 2.0e-6;
  const real_t g = 9.81;
  const real_t imu_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  calib_imucam_add_imu(calib, imu_rate, n_aw, n_gw, n_a, n_g, g, imu_ext);
  MU_ASSERT(calib->imu_ok == 0);
  MU_ASSERT(calib->num_imus == 1);

  // -- Add imu event
  const timestamp_t ts = 1;
  const real_t acc[3] = {1.0, 2.0, 3.0};
  const real_t gyr[3] = {4.0, 5.0, 6.0};
  calib_imucam_add_imu_event(calib, ts, acc, gyr);

  // Assert
  MU_ASSERT(calib->imu_buf.size == 1);
  MU_ASSERT(calib->imu_buf.ts[0] == ts);
  MU_ASSERT(vec_equals(calib->imu_buf.acc[0], acc, 3) == 1);
  MU_ASSERT(vec_equals(calib->imu_buf.gyr[0], gyr, 3) == 1);
  MU_ASSERT(calib->imu_ok == 1);

  // Clean up
  calib_imucam_free(calib);

  return 0;
}

int test_calib_imucam_add_fiducial_event() {
  // Setup
  calib_imucam_t *calib = calib_imucam_malloc();
  // -- Add Imu
  const int imu_rate = 200;
  const real_t n_a = 0.08;
  const real_t n_g = 0.004;
  const real_t n_aw = 0.00004;
  const real_t n_gw = 2.0e-6;
  const real_t g = 9.81;
  const real_t imu_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  calib_imucam_add_imu(calib, imu_rate, n_aw, n_gw, n_a, n_g, g, imu_ext);
  calib->imu_ok = 1;
  // -- Add camera
  const int res[2] = {752, 480};
  const char *pm = "pinhole";
  const char *dm = "radtan4";
  const real_t cam_vec[8] = {458.0, 457.0, 367.0, 248.0, 0.0, 0.0, 0.0, 0.0};
  const real_t cam_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  calib_imucam_add_camera(calib, 0, res, pm, dm, cam_vec, cam_ext);

  // Add fiducial event
  const timestamp_t ts = 1;
  const int cam_idx = 1;
  const int n = 2;
  const int tag_ids[2] = {1, 2};
  const int corner_idxs[2] = {1, 2};
  const real_t pts[2 * 3] = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
  const real_t kps[2 * 2] = {0.0, 0.0, 1.0, 1.0};
  calib_imucam_add_fiducial_event(calib,
                                  ts,
                                  cam_idx,
                                  n,
                                  tag_ids,
                                  corner_idxs,
                                  pts,
                                  kps);

  // Assert
  const fiducial_buffer_t *buf = calib->fiducial_buffer;
  MU_ASSERT(buf->data[0]->ts == ts);
  MU_ASSERT(buf->data[0]->cam_idx == cam_idx);
  MU_ASSERT(buf->data[0]->num_corners == n);
  MU_ASSERT(vec_equals(buf->data[0]->object_points, pts, n * 3) == 1);
  MU_ASSERT(vec_equals(buf->data[0]->keypoints, kps, n * 2) == 1);
  MU_ASSERT(buf->size == 1);
  MU_ASSERT(buf->capacity > 1);

  // Clean up
  calib_imucam_free(calib);

  return 0;
}

int test_calib_imucam_update() {
  // Setup
  calib_imucam_t *calib = calib_imucam_malloc();
  // -- Add Imu
  const int imu_rate = 200;
  const real_t n_a = 0.08;
  const real_t n_g = 0.004;
  const real_t n_aw = 0.00004;
  const real_t n_gw = 2.0e-6;
  const real_t g = 9.81;
  const real_t imu_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  // const real_t imu_ext[7] = {0.0, 0.0, 0.0, 0.70710678, 0.0, 0.0, 0.70710678};
  calib_imucam_add_imu(calib, imu_rate, n_aw, n_gw, n_a, n_g, g, imu_ext);
  calib->imu_ok = 1;
  // -- Add cam0
  const int cam0_res[2] = {752, 480};
  const char *pm = "pinhole";
  const char *dm = "radtan4";
  const real_t cam0_vec[8] = {
      458.654,
      457.296,
      367.215,
      248.375,
      -0.28340811,
      0.07395907,
      0.00019359,
      1.76187114e-05,
  };
  const real_t cam0_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  calib_imucam_add_camera(calib, 0, cam0_res, pm, dm, cam0_vec, cam0_ext);
  // -- Add cam1
  const int cam1_res[2] = {752, 480};
  const real_t cam1_vec[8] = {
      457.587,
      456.134,
      379.999,
      255.238,
      -0.28368365,
      0.07451284,
      -0.00010473,
      -3.55590700e-05,
  };
  const real_t cam1_ext[7] = {
      0.11007414,
      -0.00015661,
      0.00088938,
      9.99974496e-01,
      7.04530576e-03,
      -1.79854893e-04,
      1.15733025e-03,
  };
  calib_imucam_add_camera(calib, 1, cam1_res, pm, dm, cam1_vec, cam1_ext);

  // Test update
  char *data_dir = TEST_IMU_APRIL;
  int num_cams = 1;
  int num_imus = 1;
  timeline_t *timeline = timeline_load_data(data_dir, num_cams, num_imus);

  for (int k = 0; k < timeline->timeline_length; k++) {
    // Extract timeline events. Add either imu or fiducial event
    for (int i = 0; i < timeline->timeline_events_lengths[k]; i++) {
      timeline_event_t *event = timeline->timeline_events[k][i];
      const timestamp_t ts = event->ts;

      if (event->type == IMU_EVENT) {
        const imu_event_t *data = &event->data.imu;
        calib_imucam_add_imu_event(calib, ts, data->acc, data->gyr);

      } else if (event->type == FIDUCIAL_EVENT) {
        const fiducial_event_t *data = &event->data.fiducial;
        const int cam_idx = data->cam_idx;
        calib_imucam_add_fiducial_event(calib,
                                        ts,
                                        cam_idx,
                                        data->num_corners,
                                        data->tag_ids,
                                        data->corner_indices,
                                        data->object_points,
                                        data->keypoints);
      }
    }

    // Trigger update
    calib_imucam_update(calib);
    if (calib->num_views == 10) {
      break;
    }
  }

  // Clean up
  timeline_free(timeline);
  calib_imucam_free(calib);

  return 0;
}

int test_calib_imucam_batch() {
  // clang-format off
  const int res[2] = {752, 480};
  const char *pm = "pinhole";
  const char *dm = "radtan4";
  const real_t cam_vec[2][8] = {
    {458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05},
    {457.587, 456.134, 379.999, 255.238, -0.28368365, 0.07451284, -0.00010473, -3.555e-05}
  };
  const real_t cam_exts[2][7] = {
    {0, 0, 0, 1, 0, 0, 0},
    {0.11007414, -0.00015661, 0.00088938,
     9.99974496e-01, 7.04530576e-03, -1.79854893e-04, 1.15733025e-03}
  };
  const real_t T_SC0[4 * 4] = {
    0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
    0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
    -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
    0.0, 0.0, 0.0, 1.0
  };
  TF_VECTOR(T_SC0, imu_ext);
  const int imu_rate = 200;
  const real_t n_a = 0.08;
  const real_t n_g = 0.004;
  const real_t n_aw = 0.00004;
  const real_t n_gw = 2.0e-6;
  const real_t g = 9.81;
  // clang-format on

  calib_imucam_t *calib = calib_imucam_malloc();
  calib_imucam_add_imu(calib, imu_rate, n_a, n_g, n_aw, n_gw, g, imu_ext);
  calib_imucam_add_camera(calib, 0, res, pm, dm, cam_vec[0], cam_exts[0]);
  calib_imucam_add_camera(calib, 1, res, pm, dm, cam_vec[1], cam_exts[1]);

  // Incremental solve
  char *data_dir = TEST_IMU_APRIL;
  int num_cams = 2;
  int num_imus = 1;
  // int window_size = 20;
  timeline_t *timeline = timeline_load_data(data_dir, num_cams, num_imus);

  for (int k = 0; k < timeline->timeline_length; k++) {
    // Extract timeline events
    for (int i = 0; i < timeline->timeline_events_lengths[k]; i++) {
      timeline_event_t *event = timeline->timeline_events[k][i];
      const timestamp_t ts = event->ts;

      if (event->type == IMU_EVENT) {
        const imu_event_t *data = &event->data.imu;
        calib_imucam_add_imu_event(calib, ts, data->acc, data->gyr);

      } else if (event->type == FIDUCIAL_EVENT) {
        const fiducial_event_t *data = &event->data.fiducial;
        const int cam_idx = data->cam_idx;
        calib_imucam_add_fiducial_event(calib,
                                        ts,
                                        cam_idx,
                                        data->num_corners,
                                        data->tag_ids,
                                        data->corner_indices,
                                        data->object_points,
                                        data->keypoints);
      }
    }

    // Trigger update
    // TIC(start);
    if (calib_imucam_update(calib) == 0) {
      // // Incremental solve
      // if (calib->num_views >= window_size) {
      //   calib->max_iter = 20;
      //   calib->verbose = 0;
      //   calib_imucam_solve(calib);
      //   // calib_imucam_marginalize(calib);
      //   // k = timeline->timeline_length;

      //   real_t reproj_rmse = 0.0;
      //   real_t reproj_mean = 0.0;
      //   real_t reproj_median = 0.0;
      //   if (calib->num_views) {
      //     calib_imucam_errors(calib,
      //                         &reproj_rmse,
      //                         &reproj_mean,
      //                         &reproj_median);
      //   }

      //   char cam0_str[100] = {0};
      //   char cam1_str[100] = {0};
      //   char cam_ext_str[100] = {0};
      //   char imu_ext_str[100] = {0};
      //   vec2str(calib->cam_params[0].data, 8, cam0_str);
      //   vec2str(calib->cam_params[1].data, 8, cam1_str);
      //   vec2str(calib->cam_exts[1].data, 7, cam_ext_str);
      //   vec2str(calib->imu_ext->data, 7, imu_ext_str);
      //   printf("cam0:    %s\n", cam0_str);
      //   printf("cam1:    %s\n", cam1_str);
      //   printf("cam ext: %s\n", cam_ext_str);
      //   printf("imu ext: %s\n", imu_ext_str);
      //   printf("rmse reproj error: %f\n", reproj_rmse);
      //   printf("\n");
      // }
      // PRINT_TOC("time", start);
    }
  }

  // Solve
  calib->max_iter = 10;
  calib->verbose = 0;
  calib_imucam_solve(calib);
  MU_ASSERT((calib->num_cam_factors + calib->num_imu_factors) > 0);

  // Clean up
  calib_imucam_free(calib);
  timeline_free(timeline);

  return 0;
}

int test_calib_imucam_batch_ceres() {
  // Setup
  calib_imucam_t *calib = calib_imucam_malloc();
  // -- Add Imu
  const int imu_rate = 200;
  const real_t n_a = 0.08;
  const real_t n_g = 0.004;
  const real_t n_aw = 0.00004;
  const real_t n_gw = 2.0e-6;
  const real_t g = 9.81;
  // const real_t imu_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  // const real_t imu_ext[7] = {0.0, 0.0, 0.0, 0.70710678, 0.0, 0.0, 0.70710678};
  const real_t T_SC0[4 * 4] = {0.0148655429818,
                               -0.999880929698,
                               0.00414029679422,
                               -0.0216401454975,
                               0.999557249008,
                               0.0149672133247,
                               0.025715529948,
                               -0.064676986768,
                               -0.0257744366974,
                               0.00375618835797,
                               0.999660727178,
                               0.00981073058949,
                               0.0,
                               0.0,
                               0.0,
                               1.0};
  TF_VECTOR(T_SC0, imu_ext);
  calib_imucam_add_imu(calib, imu_rate, n_aw, n_gw, n_a, n_g, g, imu_ext);
  calib->imu_ok = 1;
  // -- Add cam0
  const int cam0_res[2] = {752, 480};
  const char *pm = "pinhole";
  const char *dm = "radtan4";
  const real_t cam0_vec[8] = {
      458.654,
      457.296,
      367.215,
      248.375,
      -0.28340811,
      0.07395907,
      0.00019359,
      1.76187114e-05,
  };
  const real_t cam0_ext[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  calib_imucam_add_camera(calib, 0, cam0_res, pm, dm, cam0_vec, cam0_ext);
  // -- Add cam1
  const int cam1_res[2] = {752, 480};
  const real_t cam1_vec[8] = {
      457.587,
      456.134,
      379.999,
      255.238,
      -0.28368365,
      0.07451284,
      -0.00010473,
      -3.55590700e-05,
  };
  const real_t cam1_ext[7] = {
      0.11007414,
      -0.00015661,
      0.00088938,
      9.99974496e-01,
      7.04530576e-03,
      -1.79854893e-04,
      1.15733025e-03,
  };
  calib_imucam_add_camera(calib, 1, cam1_res, pm, dm, cam1_vec, cam1_ext);

  // Test update
  char *data_dir = TEST_IMU_APRIL;
  int num_cams = 2;
  int num_imus = 1;
  timeline_t *timeline = timeline_load_data(data_dir, num_cams, num_imus);

  for (int k = 0; k < timeline->timeline_length; k++) {
    // Extract timeline events. Add either imu or fiducial event
    for (int i = 0; i < timeline->timeline_events_lengths[k]; i++) {
      timeline_event_t *event = timeline->timeline_events[k][i];
      const timestamp_t ts = event->ts;

      if (event->type == IMU_EVENT) {
        const imu_event_t *data = &event->data.imu;
        calib_imucam_add_imu_event(calib, ts, data->acc, data->gyr);

      } else if (event->type == FIDUCIAL_EVENT) {
        const fiducial_event_t *data = &event->data.fiducial;
        const int cam_idx = data->cam_idx;
        calib_imucam_add_fiducial_event(calib,
                                        ts,
                                        cam_idx,
                                        data->num_corners,
                                        data->tag_ids,
                                        data->corner_indices,
                                        data->object_points,
                                        data->keypoints);
      }
    }

    // Trigger update
    calib_imucam_update(calib);
  }

  // Setup ceres-solver
  ceres_init();
  ceres_problem_t *problem = ceres_create_problem();
  ceres_local_parameterization_t *pose_pm =
      ceres_create_pose_local_parameterization();
  int num_factors = 0;

  for (int k = 0; k < arrlen(calib->timestamps); k++) {
    for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
      const timestamp_t ts = calib->timestamps[k];
      calib_imucam_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
      if (view == NULL) {
        continue;
      }

      for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
        calib_imucam_factor_t *factor = &view->cam_factors[factor_idx];
        real_t **param_ptrs = factor->params;
        int num_residuals = 2;
        int num_params = 6;
        int param_sizes[6] = {
            7, // Fiducial extrinsic
            7, // Imu pose
            7, // Imu extrinsic
            7, // Camera extrinsic
            8, // Camera parameters
            1, // Time delay
        };
        ceres_problem_add_residual_block(problem,
                                         &calib_imucam_factor_ceres_eval,
                                         factor,
                                         NULL,
                                         NULL,
                                         num_residuals,
                                         num_params,
                                         param_sizes,
                                         param_ptrs);
        num_factors++;

        ceres_set_parameterization(problem, param_ptrs[0], pose_pm);
        ceres_set_parameterization(problem, param_ptrs[1], pose_pm);
        ceres_set_parameterization(problem, param_ptrs[2], pose_pm);
        ceres_set_parameterization(problem, param_ptrs[3], pose_pm);
      }
    }
  }

  for (int k = 0; k < hmlen(calib->imu_factors); k++) {
    imu_factor_t *factor = calib->imu_factors[k].value;
    real_t **param_ptrs = factor->params;
    int num_residuals = 15;
    int num_params = 6;
    int param_sizes[6] = {
        7, // Pose i
        3, // Vel i
        6, // IMU biases i
        7, // Pose j
        3, // Vel j
        6, // IMU biases j
    };
    ceres_problem_add_residual_block(problem,
                                     &imu_factor_ceres_eval,
                                     factor,
                                     NULL,
                                     NULL,
                                     num_residuals,
                                     num_params,
                                     param_sizes,
                                     param_ptrs);
    num_factors++;
    ceres_set_parameterization(problem, param_ptrs[0], pose_pm);
    ceres_set_parameterization(problem, param_ptrs[3], pose_pm);
  }

  {
    camera_params_t *cam0_params = &calib->cam_params[0];
    camera_params_t *cam1_params = &calib->cam_params[1];
    ceres_set_parameter_constant(problem, cam0_params->data);
    ceres_set_parameter_constant(problem, cam1_params->data);
  }
  {
    extrinsic_t *cam0_ext = &calib->cam_exts[0];
    extrinsic_t *cam1_ext = &calib->cam_exts[1];
    ceres_set_parameter_constant(problem, cam0_ext->data);
    ceres_set_parameter_constant(problem, cam1_ext->data);
  }

  {
    time_delay_t *time_delay = calib->time_delay;
    ceres_set_parameter_constant(problem, time_delay->data);
  }

  // Solve
  // ceres_solve(problem, 100, 0);
  ceres_solve(problem);
  calib_imucam_print(calib);
  // printf("num_factors: %d\n", num_factors);

  // Clean up
  timeline_free(timeline);
  calib_imucam_free(calib);
  ceres_free_problem(problem);

  return 0;
}

int test_calib_gimbal_copy() {
  const char *data_path = TEST_SIM_GIMBAL;
  calib_gimbal_t *src = calib_gimbal_load(data_path);
  calib_gimbal_t *dst = calib_gimbal_copy(src);

  MU_ASSERT(src != NULL);
  MU_ASSERT(dst != NULL);
  MU_ASSERT(calib_gimbal_equals(src, dst));

  // calib_gimbal_print(src);
  calib_gimbal_free(src);
  calib_gimbal_free(dst);

  return 0;
}

int test_calib_gimbal_add_fiducial() {
  calib_gimbal_t *calib = calib_gimbal_malloc();

  real_t fiducial_pose[7] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  calib_gimbal_add_fiducial(calib, fiducial_pose);
  MU_ASSERT(vec_equals(calib->fiducial_ext.data, fiducial_pose, 7));

  calib_gimbal_free(calib);

  return 0;
}

int test_calib_gimbal_add_pose() {
  calib_gimbal_t *calib = calib_gimbal_malloc();

  real_t pose[7] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  calib_gimbal_add_pose(calib, 0, pose);
  MU_ASSERT(vec_equals(calib->poses[0].data, pose, 7));
  MU_ASSERT(calib->num_poses == 1);

  calib_gimbal_free(calib);

  return 0;
}

int test_calib_gimbal_add_gimbal_extrinsic() {
  calib_gimbal_t *calib = calib_gimbal_malloc();

  real_t gimbal_ext[7] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  calib_gimbal_add_gimbal_extrinsic(calib, gimbal_ext);
  MU_ASSERT(vec_equals(gimbal_ext, calib->gimbal_ext.data, 7));

  calib_gimbal_free(calib);
  return 0;
}

int test_calib_gimbal_add_gimbal_link() {
  calib_gimbal_t *calib = calib_gimbal_malloc();

  real_t link0[7] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  calib_gimbal_add_gimbal_link(calib, 0, link0);
  MU_ASSERT(vec_equals(link0, calib->links[0].data, 7));
  MU_ASSERT(calib->num_links == 1);

  real_t link1[7] = {8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0};
  calib_gimbal_add_gimbal_link(calib, 1, link1);
  MU_ASSERT(vec_equals(link1, calib->links[1].data, 7));
  MU_ASSERT(calib->num_links == 2);

  calib_gimbal_free(calib);
  return 0;
}

int test_calib_gimbal_add_camera() {
  calib_gimbal_t *calib = calib_gimbal_malloc();

  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  real_t cam0_params[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  real_t cam0_ext[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  real_t cam1_params[8] = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  real_t cam1_ext[7] = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

  calib_gimbal_add_camera(calib,
                          0,
                          cam_res,
                          proj_model,
                          dist_model,
                          cam0_params,
                          cam0_ext);
  MU_ASSERT(vec_equals(cam0_params, calib->cam_params[0].data, 8));
  MU_ASSERT(vec_equals(cam0_ext, calib->cam_exts[0].data, 7));
  MU_ASSERT(calib->num_cams == 1);

  calib_gimbal_add_camera(calib,
                          1,
                          cam_res,
                          proj_model,
                          dist_model,
                          cam1_params,
                          cam1_ext);
  MU_ASSERT(vec_equals(cam1_params, calib->cam_params[1].data, 8));
  MU_ASSERT(vec_equals(cam1_ext, calib->cam_exts[1].data, 7));
  MU_ASSERT(calib->num_cams == 2);

  calib_gimbal_free(calib);
  return 0;
}

int test_calib_gimbal_add_remove_view() {
  // Setup gimbal calibration
  calib_gimbal_t *calib = calib_gimbal_malloc();

  // -- Add fiducial
  real_t fiducial[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  calib_gimbal_add_fiducial(calib, fiducial);

  // -- Add pose
  const timestamp_t ts = 0;
  real_t pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  calib_gimbal_add_pose(calib, ts, pose);

  // -- Add gimbal links
  real_t link0[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  real_t link1[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  calib_gimbal_add_gimbal_link(calib, 0, link0);
  calib_gimbal_add_gimbal_link(calib, 1, link1);

  // -- Add camera
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  real_t cam0_params[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  real_t cam0_ext[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  calib_gimbal_add_camera(calib,
                          0,
                          cam_res,
                          proj_model,
                          dist_model,
                          cam0_params,
                          cam0_ext);

  // -- Add view
  const int pose_idx = 0;
  const int view_idx = 0;
  const int cam_idx = 0;
  const int num_corners = 4;
  const int tag_ids[4] = {0, 0, 0, 0};
  const int corner_indices[4] = {0, 1, 2, 3};
  const real_t object_points[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  const real_t keypoints[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  const real_t joints[3] = {0.0, 0.0, 0.0};
  const int num_joints = 3;
  calib_gimbal_add_view(calib,
                        pose_idx,
                        view_idx,
                        ts,
                        cam_idx,
                        num_corners,
                        tag_ids,
                        corner_indices,
                        object_points,
                        keypoints,
                        joints,
                        num_joints);
  MU_ASSERT(calib->num_cams == 1);
  MU_ASSERT(calib->num_views == 1);
  MU_ASSERT(calib->num_poses == 1);
  MU_ASSERT(calib->num_links == 2);
  MU_ASSERT(calib->num_joints == 3);
  MU_ASSERT(calib->num_calib_factors == 4);
  MU_ASSERT(calib->num_joint_factors == 3);
  MU_ASSERT(calib->num_joints == 3);

  // -- Remove view
  calib_gimbal_remove_view(calib, view_idx);
  MU_ASSERT(calib->num_cams == 1);
  MU_ASSERT(calib->num_views == 0);
  MU_ASSERT(calib->num_poses == 1);
  MU_ASSERT(calib->num_links == 2);
  MU_ASSERT(calib->num_joints == 3);
  MU_ASSERT(calib->num_calib_factors == 0);
  MU_ASSERT(calib->num_joint_factors == 0);
  MU_ASSERT(calib->num_joints == 3);

  // Clean up
  calib_gimbal_free(calib);

  return 0;
}

int test_calib_gimbal_load() {
  const char *data_path = TEST_SIM_GIMBAL;
  calib_gimbal_t *calib = calib_gimbal_load(data_path);
  MU_ASSERT(calib != NULL);
  calib_gimbal_print(calib);
  calib_gimbal_free(calib);

  return 0;
}

int test_calib_gimbal_save() {
  const char *data_path = TEST_SIM_GIMBAL;
  calib_gimbal_t *calib = calib_gimbal_load(data_path);
  calib_gimbal_save(calib, "/tmp/estimates.yaml");
  calib_gimbal_free(calib);

  return 0;
}

static void compare_gimbal_calib(const calib_gimbal_t *gnd,
                                 const calib_gimbal_t *est) {
  assert(gnd->num_views == est->num_views);
  assert(gnd->num_cams == est->num_cams);
  assert(gnd->num_calib_factors == est->num_calib_factors);
  assert(gnd->num_joint_factors == est->num_joint_factors);

  // Compare estimated vs ground-truth
  printf("\n");
  {
    printf("num_views: %d\n", gnd->num_views);
    printf("num_cams: %d\n", gnd->num_cams);
    printf("num_poses: %d\n", gnd->num_poses);
    printf("num_links: %d\n", gnd->num_links);
    printf("num_joints: %d\n", gnd->num_joints);
    printf("num_calib_factors: %d\n", gnd->num_calib_factors);
    printf("num_joint_factors: %d\n", gnd->num_joint_factors);

    // Fiducial
    {
      real_t dr[3] = {0};
      real_t dtheta = 0.0;
      pose_diff2(gnd->fiducial_ext.data, est->fiducial_ext.data, dr, &dtheta);
      printf("fiducial ");
      printf("dr: [%.4f, %.4f, %.4f], ", dr[0], dr[1], dr[2]);
      printf("dtheta: %f [deg]\n", rad2deg(dtheta));
    }

    // Links
    for (int link_idx = 0; link_idx < est->num_links; link_idx++) {
      real_t dr[3] = {0};
      real_t dtheta = 0.0;
      pose_diff2(gnd->links[link_idx].data,
                 est->links[link_idx].data,
                 dr,
                 &dtheta);
      printf("link_exts[%d] ", link_idx);
      printf("dr: [%.4f, %.4f, %.4f], ", dr[0], dr[1], dr[2]);
      printf("dtheta: %f [deg]\n", rad2deg(dtheta));
    }

    // Joints
    real_t joints[3] = {0};
    for (int view_idx = 0; view_idx < gnd->num_views; view_idx++) {
      for (int joint_idx = 0; joint_idx < gnd->num_joints; joint_idx++) {
        const real_t gnd_angle = gnd->joints[view_idx][joint_idx].data[0];
        const real_t est_angle = est->joints[view_idx][joint_idx].data[0];
        joints[joint_idx] += rad2deg(fabs(gnd_angle - est_angle));
      }
    }
    for (int joint_idx = 0; joint_idx < gnd->num_joints; joint_idx++) {
      printf("joint[%d] total diff: %f [deg]\n", joint_idx, joints[joint_idx]);
    }

    // Camera extrinsic
    for (int cam_idx = 0; cam_idx < est->num_cams; cam_idx++) {
      real_t dr[3] = {0};
      real_t dtheta = 0.0;
      pose_diff2(gnd->cam_exts[cam_idx].data,
                 est->cam_exts[cam_idx].data,
                 dr,
                 &dtheta);
      printf("cam_exts[%d] ", cam_idx);
      printf("dr: [%.4f, %.4f, %.4f], ", dr[0], dr[1], dr[2]);
      printf("dtheta: %f [deg]\n", rad2deg(dtheta));
    }

    // Camera parameters
    for (int cam_idx = 0; cam_idx < est->num_cams; cam_idx++) {
      real_t *cam_gnd = gnd->cam_params[cam_idx].data;
      real_t *cam_est = est->cam_params[cam_idx].data;
      real_t diff[8] = {0};
      vec_sub(cam_gnd, cam_est, diff, 8);

      printf("cam_params[%d] ", cam_idx);
      print_vector("diff", diff, 8);
    }
  }
  printf("\n");
}

int test_calib_gimbal_solve() {
  // Setup
  const int debug = 1;
  // const char *data_path = TEST_SIM_GIMBAL;
  // const char *data_path = "/tmp/calib_gimbal";
  // const char *data_path = "/tmp/sim_gimbal";
  const char *data_path = "/home/chutsu/calib_gimbal";
  calib_gimbal_t *calib_gnd = calib_gimbal_load(data_path);
  calib_gimbal_t *calib_est = calib_gimbal_load(data_path);
  MU_ASSERT(calib_gnd != NULL);
  MU_ASSERT(calib_est != NULL);

  // Perturb parameters
  // {
  //   // printf("Ground Truth:\n");
  //   // calib_gimbal_print(calib_gnd);
  //   // printf("\n");

  //   // Perturb
  //   real_t dx[6] = {0.01, 0.01, 0.01, 0.1, 0.1, 0.1};
  //   // pose_update(calib_est->fiducial_ext.data, dx);
  //   // pose_update(calib_est->cam_exts[0].data, dx);
  //   // pose_update(calib_est->cam_exts[1].data, dx);
  //   for (int link_idx = 0; link_idx < calib_est->num_links; link_idx++) {
  //     pose_update(calib_est->links[link_idx].data, dx);
  //   }
  //   // for (int view_idx = 0; view_idx < calib_est->num_views; view_idx++) {
  //   //   for (int joint_idx = 0; joint_idx < calib_est->num_joints; joint_idx++) {
  //   //     calib_est->joints[view_idx][joint_idx].data[0] += randf(-0.05, 0.05);
  //   //   }
  //   // }
  //   // printf("\n");

  //   //     printf("Initial:\n");
  //   //     calib_gimbal_print(calib_est);
  //   //     printf("\n");
  // }
  // if (debug) {
  //   compare_gimbal_calib(calib_gnd, calib_est);
  // }

  // Solve
  calib_gimbal_save(calib_est, "/tmp/estimates-before.yaml");
  calib_gimbal_print(calib_est);
  TIC(solve);
  solver_t solver;
  solver_setup(&solver);
  solver.verbose = debug;
  solver.max_iter = 10;
  solver.param_order_func = &calib_gimbal_param_order;
  solver.cost_func = &calib_gimbal_cost;
  solver.linearize_func = &calib_gimbal_linearize_compact;
  solver_solve(&solver, calib_est);
  if (debug) {
    calib_gimbal_print(calib_est);
    compare_gimbal_calib(calib_gnd, calib_est);
    PRINT_TOC("solve", solve);
  }

  // int sv_size = 0;
  // int r_size = 0;
  // param_order_t *hash = calib_gimbal_param_order(calib_est, &sv_size, &r_size);

  // const int J_rows = r_size;
  // const int J_cols = sv_size;
  // real_t *J = CALLOC(real_t, r_size * sv_size);
  // real_t *g = CALLOC(real_t, r_size * 1);
  // real_t *r = CALLOC(real_t, r_size * 1);
  // calib_gimbal_linearize(calib_est, J_rows, J_cols, hash, J, g, r);
  // mat_save("/tmp/J.csv", J, J_rows, J_cols);

  // free(J);
  // free(g);
  // free(r);
  // param_order_free(hash);

  // printf("Estimated:\n");
  // calib_gimbal_print(calib);
  // printf("\n");

  // Clean up
  // calib_gimbal_save(calib_gnd, "/tmp/estimates-gnd.yaml");
  // calib_gimbal_save(calib_est, "/tmp/estimates-after.yaml");
  calib_gimbal_free(calib_gnd);
  calib_gimbal_free(calib_est);

  return 0;
}

#ifdef USE_CERES
int test_calib_gimbal_ceres_solve() {
  // Setup simulation data
  const char *data_path = TEST_SIM_GIMBAL;
  calib_gimbal_t *calib_gnd = calib_gimbal_load(data_path);
  calib_gimbal_t *calib_est = calib_gimbal_load(data_path);
  MU_ASSERT(calib_gnd != NULL);
  MU_ASSERT(calib_est != NULL);

  // Perturb parameters
  {
    // printf("Ground Truth:\n");
    // calib_gimbal_print(calib_gnd);
    // printf("\n");

    // Perturb
    // real_t dx[6] = {0.01, 0.01, 0.01, 0.1, 0.1, 0.1};
    // pose_update(calib_est->fiducial_ext.data, dx);
    // pose_update(calib_est->cam_exts[0].data, dx);
    // pose_update(calib_est->cam_exts[1].data, dx);
    // for (int link_idx = 0; link_idx < 2; link_idx++) {
    //   pose_update(calib_est->links[link_idx].data, dx);
    // }
    for (int view_idx = 0; view_idx < calib_est->num_views; view_idx++) {
      for (int joint_idx = 0; joint_idx < 3; joint_idx++) {
        calib_est->joints[view_idx][joint_idx].data[0] += randf(-0.1, 0.1);
      }
    }
    // printf("\n");

    //     printf("Initial:\n");
    //     calib_gimbal_print(calib_est);
    //     printf("\n");
  }

  // Setup ceres problem
  ceres_init();
  ceres_problem_t *problem = ceres_create_problem();
  ceres_local_parameterization_t *pose_pm =
      ceres_create_pose_local_parameterization();

  const int num_residuals = 2;
  const int num_params = 10;
  for (int view_idx = 0; view_idx < calib_est->num_views; view_idx++) {
    for (int cam_idx = 0; cam_idx < calib_est->num_cams; cam_idx++) {
      calib_gimbal_view_t *view = calib_est->views[view_idx][cam_idx];
      for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
        real_t *param_ptrs[] = {calib_est->fiducial_ext.data,
                                calib_est->gimbal_ext.data,
                                calib_est->poses[0].data,
                                calib_est->links[0].data,
                                calib_est->links[1].data,
                                calib_est->joints[view_idx][0].data,
                                calib_est->joints[view_idx][1].data,
                                calib_est->joints[view_idx][2].data,
                                calib_est->cam_exts[cam_idx].data,
                                calib_est->cam_params[cam_idx].data};
        int param_sizes[10] = {
            7, // Fiducial extrinsic
            7, // Gimbal extrinscis
            7, // Pose
            7, // Link0
            7, // Link1
            1, // Joint0
            1, // Joint1
            1, // Joint2
            7, // Camera extrinsic
            8, // Camera Parameters
        };
        ceres_problem_add_residual_block(problem,
                                         &calib_gimbal_factor_ceres_eval,
                                         &view->calib_factors[factor_idx],
                                         NULL,
                                         NULL,
                                         num_residuals,
                                         num_params,
                                         param_sizes,
                                         param_ptrs);
      } // For each corners
    }   // For each cameras
  }     // For each views

  // for (int view_idx = 0; view_idx < calib_est->num_views; view_idx++) {
  //   ceres_set_parameter_constant(problem,
  //   calib_est->joints[view_idx][0].data);
  //   ceres_set_parameter_constant(problem,
  //   calib_est->joints[view_idx][1].data);
  //   ceres_set_parameter_constant(problem,
  //   calib_est->joints[view_idx][2].data);
  // }

  // ceres_set_parameter_constant(problem, calib_est->fiducial_ext.data);
  ceres_set_parameter_constant(problem, calib_est->gimbal_ext.data);
  // ceres_set_parameter_constant(problem, calib_est->links[0].data);
  // ceres_set_parameter_constant(problem, calib_est->links[1].data);
  ceres_set_parameter_constant(problem, calib_est->cam_exts[0].data);
  ceres_set_parameter_constant(problem, calib_est->cam_exts[1].data);
  ceres_set_parameter_constant(problem, calib_est->cam_params[0].data);
  ceres_set_parameter_constant(problem, calib_est->cam_params[1].data);

  for (int view_idx = 0; view_idx < calib_est->num_poses; view_idx++) {
    ceres_set_parameter_constant(problem, calib_est->poses[view_idx].data);
    ceres_set_parameterization(problem,
                               calib_est->poses[view_idx].data,
                               pose_pm);
  }
  ceres_set_parameterization(problem, calib_est->fiducial_ext.data, pose_pm);
  ceres_set_parameterization(problem, calib_est->gimbal_ext.data, pose_pm);
  ceres_set_parameterization(problem, calib_est->links[0].data, pose_pm);
  ceres_set_parameterization(problem, calib_est->links[1].data, pose_pm);
  ceres_set_parameterization(problem, calib_est->cam_exts[0].data, pose_pm);
  ceres_set_parameterization(problem, calib_est->cam_exts[1].data, pose_pm);
  TIC(solve);
  // ceres_solve(problem, 10, 0);
  ceres_solve(problem);
  PRINT_TOC("solve", solve);

  // Compare ground-truth vs estimates
  compare_gimbal_calib(calib_gnd, calib_est);

  // Clean up
  ceres_free_problem(problem);
  calib_gimbal_free(calib_gnd);
  calib_gimbal_free(calib_est);

  return 0;
}
#endif // USE_CERES

/******************************************************************************
 * TEST DATASET
 ******************************************************************************/

int test_assoc_pose_data() {
  const double threshold = 0.01;
  const char *matches_fpath = "./gnd_est_matches.csv";
  const char *gnd_data_path = "./test_data/euroc/MH01_groundtruth.csv";
  const char *est_data_path = "./test_data/euroc/MH01_estimate.csv";

  // Load ground-truth poses
  int num_gnd_poses = 0;
  pose_t *gnd_poses = load_poses(gnd_data_path, &num_gnd_poses);
  printf("num_gnd_poses: %d\n", num_gnd_poses);

  // Load estimate poses
  int num_est_poses = 0;
  pose_t *est_poses = load_poses(est_data_path, &num_est_poses);
  printf("num_est_poses: %d\n", num_est_poses);

  // Associate data
  size_t num_matches = 0;
  int **matches = assoc_pose_data(gnd_poses,
                                  num_gnd_poses,
                                  est_poses,
                                  num_est_poses,
                                  threshold,
                                  &num_matches);
  printf("Time Associated:\n");
  printf(" - [%s]\n", gnd_data_path);
  printf(" - [%s]\n", est_data_path);
  printf("threshold:  %.4f [s]\n", threshold);
  printf("num_matches: %ld\n", num_matches);

  // Save matches to file
  FILE *matches_csv = fopen(matches_fpath, "w");
  fprintf(matches_csv, "#gnd_idx,est_idx\n");
  for (size_t i = 0; i < num_matches; i++) {
    uint64_t gnd_ts = gnd_poses[matches[i][0]].ts;
    uint64_t est_ts = est_poses[matches[i][1]].ts;
    double t_diff = fabs(ts2sec(gnd_ts - est_ts));
    if (t_diff > threshold) {
      printf("ERROR! Time difference > threshold!\n");
      printf("ground_truth_index: %d\n", matches[i][0]);
      printf("estimate_index: %d\n", matches[i][1]);
      break;
    }
    fprintf(matches_csv, "%d,%d\n", matches[i][0], matches[i][1]);
  }
  fclose(matches_csv);

  // Clean up
  for (size_t i = 0; i < num_matches; i++) {
    free(matches[i]);
  }
  free(matches);
  free(gnd_poses);
  free(est_poses);

  return 0;
}

/******************************************************************************
 * TEST PLOTTING
 ******************************************************************************/

int test_gnuplot_xyplot() {
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

int test_gnuplot_multiplot() {
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
 * TEST SIM
 ******************************************************************************/

// SIM FEATURES //////////////////////////////////////////////////////////////

int test_sim_features_load() {
  const char *csv_file = TEST_SIM_DATA "/features.csv";
  sim_features_t *features_data = sim_features_load(csv_file);
  MU_ASSERT(features_data->num_features > 0);
  sim_features_free(features_data);
  return 0;
}

// SIM IMU DATA //////////////////////////////////////////////////////////////

int test_sim_imu_data_load() {
  // const char *csv_file = TEST_SIM_DATA "/imu0/data.csv";
  // sim_imu_data_t *imu_data = sim_imu_data_load(csv_file);
  // sim_imu_data_free(imu_data);
  return 0;
}

// SIM CAMERA DATA ///////////////////////////////////////////////////////////

int test_sim_camera_frame_load() {
  const char *frame_csv = TEST_SIM_DATA "/cam0/data/100000000.csv";
  sim_camera_frame_t *frame_data = sim_camera_frame_load(frame_csv);

  MU_ASSERT(frame_data != NULL);
  MU_ASSERT(frame_data->ts == 100000000);
  MU_ASSERT(frame_data->feature_ids[0] == 1);

  sim_camera_frame_free(frame_data);

  return 0;
}

int test_sim_camera_data_load() {
  const char *dir_path = TEST_SIM_DATA "/cam0";
  sim_camera_data_t *cam_data = sim_camera_data_load(dir_path);
  sim_camera_data_free(cam_data);
  return 0;
}

int test_sim_camera_circle_trajectory() {
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
  camera_params_t cam_params;
  camera_params_setup(&cam_params, 0, cam_res, pmodel, dmodel, cam_vec);

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
                                                             &cam_params,
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

int test_sim_gimbal_malloc_free() {
  sim_gimbal_t *sim = sim_gimbal_malloc();
  sim_gimbal_free(sim);
  return 0;
}

int test_sim_gimbal_view() {
  sim_gimbal_t *sim = sim_gimbal_malloc();

  const timestamp_t ts = 0;
  const int view_idx = 0;
  const int cam_idx = 0;
  real_t pose[7] = {0, 0, 0, 1, 0, 0, 0};

  sim_gimbal_view_t *view = sim_gimbal_view(sim, ts, view_idx, cam_idx, pose);
  sim_gimbal_view_free(view);

  sim_gimbal_free(sim);
  return 0;
}

int test_sim_gimbal_solve() {
  // Setup gimbal simulator
  sim_gimbal_t *sim = sim_gimbal_malloc();

  // Setup gimbal calibrator
  calib_gimbal_t *calib = calib_gimbal_malloc();
  const timestamp_t ts = 0;
  calib_gimbal_add_fiducial(calib, sim->fiducial_ext.data);
  calib_gimbal_add_pose(calib, ts, sim->gimbal_pose.data);
  calib_gimbal_add_gimbal_extrinsic(calib, sim->gimbal_ext.data);
  calib_gimbal_add_gimbal_link(calib, 0, sim->gimbal_links[0].data);
  calib_gimbal_add_gimbal_link(calib, 1, sim->gimbal_links[1].data);
  for (int cam_idx = 0; cam_idx < sim->num_cams; cam_idx++) {
    calib_gimbal_add_camera(calib,
                            cam_idx,
                            sim->cam_params[cam_idx].resolution,
                            sim->cam_params[cam_idx].proj_model,
                            sim->cam_params[cam_idx].dist_model,
                            sim->cam_params[cam_idx].data,
                            sim->cam_exts[cam_idx].data);
  }

  // Setup solver
  solver_t solver;
  solver_setup(&solver);
  solver.verbose = 1;
  solver.param_order_func = &calib_gimbal_param_order;
  solver.cost_func = &calib_gimbal_cost;
  solver.linearize_func = &calib_gimbal_linearize_compact;

  // Simulate gimbal views
  int num_views = 100;
  int num_cams = 2;
  const int pose_idx = 0;
  sim_gimbal_view_t *view = NULL;

  for (int view_idx = 0; view_idx < num_views; view_idx++) {
    // Add gimbal view
    for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
      // Simulate single gimbal view
      const timestamp_t ts = view_idx;
      view = sim_gimbal_view(sim, ts, view_idx, cam_idx, sim->gimbal_pose.data);

      // Add view to calibration problem
      real_t joints[3] = {0};
      sim_gimbal_get_joints(sim, 3, joints);
      calib_gimbal_add_view(calib,
                            pose_idx,
                            view_idx,
                            view_idx,
                            cam_idx,
                            view->num_measurements,
                            view->tag_ids,
                            view->corner_indices,
                            view->object_points,
                            view->keypoints,
                            joints,
                            sim->num_joints);
      sim_gimbal_view_free(view);
    }

    // Find gimbal NBV
    // real_t nbv_joints[3] = {0};
    // calib_gimbal_nbv(calib, nbv_joints);
    // sim_gimbal_set_joint(sim, 0, nbv_joints[0]);
    // sim_gimbal_set_joint(sim, 1, nbv_joints[1]);
    // sim_gimbal_set_joint(sim, 2, nbv_joints[2]);
  }

  // Solve
  solver_solve(&solver, calib);

  // Clean up
  calib_gimbal_free(calib);
  sim_gimbal_free(sim);

  return 0;
}

void test_suite() {
  // MACROS
  MU_ADD_TEST(test_median_value);
  MU_ADD_TEST(test_mean_value);

  // FILE SYSTEM
  MU_ADD_TEST(test_path_file_name);
  MU_ADD_TEST(test_path_file_ext);
  MU_ADD_TEST(test_path_dir_name);
  MU_ADD_TEST(test_path_join);
  MU_ADD_TEST(test_list_files);
  MU_ADD_TEST(test_list_files_free);
  MU_ADD_TEST(test_file_read);
  MU_ADD_TEST(test_skip_line);
  MU_ADD_TEST(test_file_rows);
  MU_ADD_TEST(test_file_copy);

  // DATA
  MU_ADD_TEST(test_string_malloc);
  MU_ADD_TEST(test_dsv_rows);
  MU_ADD_TEST(test_dsv_cols);
  MU_ADD_TEST(test_dsv_fields);
  MU_ADD_TEST(test_dsv_data);
  MU_ADD_TEST(test_dsv_free);

  // DATA-STRUCTURE
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
  // MU_ADD_TEST(test_list_push_pop);
  MU_ADD_TEST(test_list_shift);
  MU_ADD_TEST(test_list_unshift);
  MU_ADD_TEST(test_list_remove);
  MU_ADD_TEST(test_list_remove_destroy);
  MU_ADD_TEST(test_mstack_new_and_destroy);
  MU_ADD_TEST(test_mstack_push);
  MU_ADD_TEST(test_mstack_pop);
  MU_ADD_TEST(test_queue_malloc_and_free);
  MU_ADD_TEST(test_queue_enqueue_dequeue);
  MU_ADD_TEST(test_hashmap_new_destroy);
  MU_ADD_TEST(test_hashmap_clear_destroy);
  MU_ADD_TEST(test_hashmap_get_set);
  MU_ADD_TEST(test_hashmap_delete);
  MU_ADD_TEST(test_hashmap_traverse);

  // TIME
  MU_ADD_TEST(test_tic_toc);
  MU_ADD_TEST(test_mtoc);
  MU_ADD_TEST(test_time_now);

  // NETWORK
  MU_ADD_TEST(test_tcp_server_setup);

  // MATHS
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

  // CV
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

  // CONTROL
  MU_ADD_TEST(test_pid_ctrl);

  // GIMBAL MODEL
  MU_ADD_TEST(test_gimbal);

  // MAV MODEL
  MU_ADD_TEST(test_mav_att_ctrl);
  MU_ADD_TEST(test_mav_vel_ctrl);
  MU_ADD_TEST(test_mav_pos_ctrl);
  MU_ADD_TEST(test_mav_waypoints);

  // SENSOR FUSION
  MU_ADD_TEST(test_schur_complement);
  MU_ADD_TEST(test_timeline);
  MU_ADD_TEST(test_pose);
  MU_ADD_TEST(test_extrinsics);
  MU_ADD_TEST(test_fiducial);
  MU_ADD_TEST(test_fiducial_buffer);
  MU_ADD_TEST(test_imu_biases);
  MU_ADD_TEST(test_feature);
  // MU_ADD_TEST(test_features);
  // MU_ADD_TEST(test_idf);
  // MU_ADD_TEST(test_idfb);
  MU_ADD_TEST(test_time_delay);
  MU_ADD_TEST(test_joint);
  MU_ADD_TEST(test_camera_params);
  MU_ADD_TEST(test_triangulation_batch);
  MU_ADD_TEST(test_pose_factor);
  MU_ADD_TEST(test_ba_factor);
  MU_ADD_TEST(test_camera_factor);
  // MU_ADD_TEST(test_idf_factor);
  MU_ADD_TEST(test_imu_buffer_setup);
  MU_ADD_TEST(test_imu_buffer_add);
  MU_ADD_TEST(test_imu_buffer_clear);
  MU_ADD_TEST(test_imu_buffer_copy);
  MU_ADD_TEST(test_imu_propagate);
  MU_ADD_TEST(test_imu_initial_attitude);
  MU_ADD_TEST(test_imu_factor_form_F_matrix);
  MU_ADD_TEST(test_imu_factor);
  MU_ADD_TEST(test_joint_factor);
  MU_ADD_TEST(test_calib_camera_factor);
  MU_ADD_TEST(test_calib_imucam_factor);
  MU_ADD_TEST(test_calib_gimbal_factor);
  MU_ADD_TEST(test_marg);
  // MU_ADD_TEST(test_visual_odometry_batch);
  // MU_ADD_TEST(test_inertial_odometry_batch);
  // MU_ADD_TEST(test_visual_inertial_odometry_batch);
  MU_ADD_TEST(test_tsf);
#ifdef USE_CERES
  MU_ADD_TEST(test_ceres_example);
#endif // USE_CERES
  MU_ADD_TEST(test_solver_setup);
  // MU_ADD_TEST(test_solver_eval);
  MU_ADD_TEST(test_camchain);
  MU_ADD_TEST(test_calib_camera_mono_batch);
  MU_ADD_TEST(test_calib_camera_mono_ceres);
  MU_ADD_TEST(test_calib_camera_mono_incremental);
  MU_ADD_TEST(test_calib_camera_stereo_batch);
  MU_ADD_TEST(test_calib_camera_stereo_ceres);
  MU_ADD_TEST(test_calib_imucam_view);
  MU_ADD_TEST(test_calib_imucam_add_imu);
  MU_ADD_TEST(test_calib_imucam_add_camera);
  MU_ADD_TEST(test_calib_imucam_add_imu_event);
  MU_ADD_TEST(test_calib_imucam_add_fiducial_event);
  MU_ADD_TEST(test_calib_imucam_update);
  MU_ADD_TEST(test_calib_imucam_batch);
  // MU_ADD_TEST(test_calib_imucam_batch_ceres);
  // MU_ADD_TEST(test_calib_gimbal_copy);
  MU_ADD_TEST(test_calib_gimbal_add_fiducial);
  MU_ADD_TEST(test_calib_gimbal_add_pose);
  MU_ADD_TEST(test_calib_gimbal_add_gimbal_extrinsic);
  MU_ADD_TEST(test_calib_gimbal_add_gimbal_link);
  MU_ADD_TEST(test_calib_gimbal_add_camera);
  // MU_ADD_TEST(test_calib_gimbal_add_remove_view);
  // MU_ADD_TEST(test_calib_gimbal_load);
  // MU_ADD_TEST(test_calib_gimbal_save);
  // MU_ADD_TEST(test_calib_gimbal_solve);
#ifdef USE_CERES
  // MU_ADD_TEST(test_calib_gimbal_ceres_solve);
#endif // USE_CERES

  // DATASET
  // MU_ADD_TEST(test_assoc_pose_data);

  // PLOTTING
  // MU_ADD_TEST(test_gnuplot_xyplot);
  // MU_ADD_TEST(test_gnuplot_multiplot);

  // SIM
  MU_ADD_TEST(test_sim_features_load);
  MU_ADD_TEST(test_sim_imu_data_load);
  MU_ADD_TEST(test_sim_camera_frame_load);
  MU_ADD_TEST(test_sim_camera_data_load);
  MU_ADD_TEST(test_sim_camera_circle_trajectory);
  MU_ADD_TEST(test_sim_gimbal_malloc_free);
  MU_ADD_TEST(test_sim_gimbal_view);
  // MU_ADD_TEST(test_sim_gimbal_solve);
}

MU_RUN_TESTS(test_suite)
