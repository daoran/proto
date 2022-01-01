#include "munit.h"
#include "proto.h"

/* TEST PARAMS */
#define M 10
#define N 10
#define TEST_DATA_PATH "./test_data/"
#define TEST_CSV TEST_DATA_PATH "test_csv.csv"
#define TEST_POSES_CSV TEST_DATA_PATH "poses.csv"
#define TEST_SIM_DATA TEST_DATA_PATH "sim_data"

/******************************************************************************
 * LOGGING
 ******************************************************************************/

int test_debug() {
  DEBUG("Hello World!");
  return 0;
}

int test_log_error() {
  LOG_ERROR("Hello World!");
  return 0;
}

int test_log_warn() {
  LOG_WARN("Hello World!");
  return 0;
}

/******************************************************************************
 * FILESYSTEM
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
  int nb_files = 0;
  char **files = list_files("/tmp", &nb_files);
  MU_ASSERT(files != NULL);
  MU_ASSERT(nb_files != 0);

  /* printf("nb_files: %d\n", nb_files); */
  for (int i = 0; i < nb_files; i++) {
    /* printf("file: %s\n", files[i]); */
    free(files[i]);
  }
  free(files);

  return 0;
}

int test_list_files_free() {
  int nb_files = 0;
  char **files = list_files("/tmp", &nb_files);
  list_files_free(files, nb_files);

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
  int nb_rows = file_rows("test_data/poses.csv");
  MU_ASSERT(nb_rows > 0);
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
 * DATA
 ******************************************************************************/

int test_string_malloc() {
  char *s = string_malloc("hello world!");
  MU_ASSERT(strcmp(s, "hello world!") == 0);
  free(s);
  return 0;
}

int test_dsv_rows() {
  int nb_rows = dsv_rows(TEST_CSV);
  MU_ASSERT(nb_rows == 10);
  return 0;
}

int test_dsv_cols() {
  int nb_cols = dsv_cols(TEST_CSV, ',');
  MU_ASSERT(nb_cols == 10);
  return 0;
}

int test_dsv_fields() {
  int nb_fields = 0;
  char **fields = dsv_fields(TEST_CSV, ',', &nb_fields);
  const char *expected[10] = {"a", "b", "c", "d", "e", "f", "g", "h", "i", "j"};
  if (fields == NULL) {
    printf("File not found [%s]\n", TEST_CSV);
    return -1;
  }

  MU_ASSERT(nb_fields == 10);
  for (int i = 0; i < nb_fields; i++) {
    MU_ASSERT(strcmp(fields[i], expected[i]) == 0);
    free(fields[i]);
  }
  free(fields);

  return 0;
}

int test_dsv_data() {
  int nb_rows = 0;
  int nb_cols = 0;
  real_t **data = dsv_data(TEST_CSV, ',', &nb_rows, &nb_cols);

  int index = 0;
  for (int i = 0; i < nb_rows; i++) {
    for (int j = 0; j < nb_rows; j++) {
      MU_ASSERT(fltcmp(data[i][j], index + 1) == 0);
      index++;
    }
  }
  dsv_free(data, nb_rows);

  return 0;
}

int test_dsv_free() {
  int nb_rows = 0;
  int nb_cols = 0;
  real_t **data = dsv_data(TEST_CSV, ',', &nb_rows, &nb_cols);
  dsv_free(data, nb_rows);

  return 0;
}

int test_csv_data() {
  int nb_rows = 0;
  int nb_cols = 0;
  real_t **data = csv_data(TEST_CSV, &nb_rows, &nb_cols);
  csv_free(data, nb_rows);

  return 0;
}

/******************************************************************************
 * DATA-STRUCTURE
 ******************************************************************************/

// DARRAY //////////////////////////////////////////////////////////////////////

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

// LIST ////////////////////////////////////////////////////////////////////////

int test_list_new_and_destroy(void) {
  list_t *list = list_new();
  MU_ASSERT(list != NULL);
  list_clear_destroy(list);
  return 0;
}

int test_list_push_pop(void) {
  /* Setup */
  list_t *list = list_new();
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

  list_clear_destroy(list);
  return 0;
}

int test_list_shift(void) {
  /* Setup */
  list_t *list = list_new();
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

  list_clear_destroy(list);
  return 0;
}

int test_list_unshift(void) {
  /* Setup */
  list_t *list = list_new();
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
  list_clear_destroy(list);

  return 0;
}

int test_list_remove(void) {
  /* Push elements */
  list_t *list = list_new();
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
  list_clear_destroy(list);

  return 0;
}

int test_list_remove_destroy(void) {
  /* Setup */
  list_t *list = list_new();
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
  list_clear_destroy(list);

  return 0;
}

// STACK ///////////////////////////////////////////////////////////////////////

int test_stack_new_and_destroy(void) {
  stack_t *s = stack_new();

  MU_ASSERT(s->size == 0);
  MU_ASSERT(s->root == NULL);
  MU_ASSERT(s->end == NULL);

  stack_clear_destroy(s, free);
  return 0;
}

int test_stack_push(void) {
  stack_t *s = stack_new();
  float *f1 = malloc_float(2.0);
  float *f2 = malloc_float(4.0);
  float *f3 = malloc_float(8.0);

  /* push float 1 */
  stack_push(s, f1);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) f1) == 0);
  MU_ASSERT(s->size == 1);
  MU_ASSERT(s->root->value == f1);
  MU_ASSERT(s->end->prev == NULL);

  /* push float 2 */
  stack_push(s, f2);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) f2) == 0);
  MU_ASSERT(s->size == 2);
  MU_ASSERT(s->root->value == f1);
  MU_ASSERT(s->end->prev->value == f1);
  MU_ASSERT(fltcmp(*(float *) s->end->prev->value, *(float *) f1) == 0);

  /* push float 3 */
  stack_push(s, f3);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) f3) == 0);
  MU_ASSERT(s->size == 3);
  MU_ASSERT(s->root->value == f1);
  MU_ASSERT(s->end->prev->value == f2);
  MU_ASSERT(fltcmp(*(float *) s->end->prev->value, *(float *) f2) == 0);

  stack_clear_destroy(s, free);
  return 0;
}

int test_stack_pop(void) {
  stack_t *s = stack_new();
  float *f1 = malloc_float(2.0);
  float *f2 = malloc_float(4.0);
  float *f3 = malloc_float(8.0);
  float *flt_ptr;

  /* push float 1 */
  stack_push(s, f1);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) f1) == 0);
  MU_ASSERT(s->size == 1);
  MU_ASSERT(s->root->value == f1);
  MU_ASSERT(s->end->prev == NULL);

  /* push float 2 */
  stack_push(s, f2);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) f2) == 0);
  MU_ASSERT(s->size == 2);
  MU_ASSERT(s->root->value == f1);
  MU_ASSERT(s->end->prev->value == f1);
  MU_ASSERT(fltcmp(*(float *) s->end->prev->value, *(float *) f1) == 0);

  /* push float 3 */
  stack_push(s, f3);
  MU_ASSERT(fltcmp(*(float *) s->end->value, *(float *) f3) == 0);
  MU_ASSERT(s->size == 3);
  MU_ASSERT(s->root->value == f1);
  MU_ASSERT(s->end->prev->value == f2);
  MU_ASSERT(fltcmp(*(float *) s->end->prev->value, *(float *) f2) == 0);

  /* pop float 3 */
  flt_ptr = stack_pop(s);
  MU_ASSERT(fltcmp(*(float *) flt_ptr, *(float *) f3) == 0);
  MU_ASSERT(s->size == 2);
  MU_ASSERT(s->root->value == f1);
  MU_ASSERT(fltcmp(*(float *) s->root->value, *(float *) f1) == 0);
  free(flt_ptr);

  /* pop float 2 */
  flt_ptr = stack_pop(s);
  MU_ASSERT(fltcmp(*(float *) flt_ptr, *(float *) f2) == 0);
  MU_ASSERT(s->size == 1);
  MU_ASSERT(s->root->value == f1);
  MU_ASSERT(fltcmp(*(float *) s->root->value, *(float *) f1) == 0);
  free(flt_ptr);

  /* pop float 1 */
  flt_ptr = stack_pop(s);
  MU_ASSERT(fltcmp(*(float *) flt_ptr, *(float *) f1) == 0);
  MU_ASSERT(s->size == 0);
  MU_ASSERT(s->root == NULL);
  MU_ASSERT(s->end == NULL);
  free(flt_ptr);

  stack_clear_destroy(s, free);
  return 0;
}

// QUEUE ///////////////////////////////////////////////////////////////////////

int test_queue_new_and_destroy(void) {
  queue_t *q = queue_new();
  MU_ASSERT(q != NULL);
  MU_ASSERT(q->count == 0);
  queue_destroy(q);

  return 0;
}

int test_queue_enqueue_dequeue(void) {
  queue_t *q = queue_new();
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

  return 0;
}

/******************************************************************************
 * TIME
 ******************************************************************************/

int test_tic() {
  struct timespec t_start = tic();
  printf("t_start.sec: %ld\n", t_start.tv_sec);
  printf("t_start.nsec: %ld\n", t_start.tv_nsec);
  return 0;
}

int test_toc() {
  struct timespec t_start = tic();
  sleep(1.0);
  MU_ASSERT(fabs(toc(&t_start) - 1.0) < 1e-2);
  return 0;
}

int test_mtoc() {
  struct timespec t_start = tic();
  sleep(1.0);
  MU_ASSERT(fabs(mtoc(&t_start) - 1000) < 1);
  return 0;
}

int test_time_now() {
  timestamp_t t_now = time_now();
  printf("t_now: %ld\n", t_now);
  return 0;
}

/******************************************************************************
 * MATHS
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

int test_sinc() { return 0; }

int test_mean() {
  real_t vals[4] = {1.0, 2.0, 3.0, 4.0};
  MU_ASSERT(fltcmp(mean(vals, 4), 2.5) == 0);

  return 0;
}

int test_median() {
  real_t vals[5] = {1.0, 2.0, 0.0, 3.0, 4.0};
  MU_ASSERT(fltcmp(median(vals, 5), 2.0) == 0);

  real_t vals2[6] = {1.0, 2.0, 0.0, 3.0, 4.0, 5.0};
  MU_ASSERT(fltcmp(median(vals2, 6), 2.5f) == 0);

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
 * LINEAR ALGEBRA
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
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[4] = {0.0};
  mat_block_get(A, 3, 1, 1, 2, 2, B);

  print_matrix("A", A, 3, 3);
  print_matrix("B", B, 2, 2);
  MU_ASSERT(fltcmp(mat_val(B, 2, 0, 0), 5.0) == 0);
  MU_ASSERT(fltcmp(mat_val(B, 2, 0, 1), 6.0) == 0);
  MU_ASSERT(fltcmp(mat_val(B, 2, 1, 0), 8.0) == 0);
  MU_ASSERT(fltcmp(mat_val(B, 2, 1, 1), 9.0) == 0);

  return 0;
}

int test_mat_block_set() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[4] = {0.0, 0.0, 0.0, 0.0};

  print_matrix("A", A, 3, 3);
  print_matrix("B", B, 2, 2);
  mat_block_set(A, 3, 1, 1, 2, 2, B);
  print_matrix("A", A, 3, 3);
  print_matrix("B", B, 2, 2);

  MU_ASSERT(fltcmp(mat_val(A, 3, 1, 1), 0.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 1, 2), 0.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 2, 1), 0.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 2, 2), 0.0) == 0);

  return 0;
}

int test_mat_diag_get() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t d[3] = {0.0, 0.0, 0.0};
  mat_diag_get(A, 3, 3, d);

  print_matrix("A", A, 3, 3);
  print_vector("d", d, 3);
  MU_ASSERT(fltcmp(d[0], 1.0) == 0);
  MU_ASSERT(fltcmp(d[1], 5.0) == 0);
  MU_ASSERT(fltcmp(d[2], 9.0) == 0);

  return 0;
}

int test_mat_diag_set() {
  real_t A[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  real_t d[4] = {1.0, 2.0, 3.0};
  mat_diag_set(A, 3, 3, d);

  print_matrix("A", A, 3, 3);
  MU_ASSERT(fltcmp(mat_val(A, 3, 0, 0), 1.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 1, 1), 2.0) == 0);
  MU_ASSERT(fltcmp(mat_val(A, 3, 2, 2), 3.0) == 0);

  return 0;
}

int test_mat_triu() {
  /* clang-format off */
  real_t A[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  real_t U[16] = {0};
  /* clang-format on */
  mat_triu(A, 4, U);
  print_matrix("U", U, 4, 4);

  return 0;
}

int test_mat_tril() {
  /* clang-format off */
  real_t A[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  real_t L[16] = {0};
  /* clang-format on */
  mat_tril(A, 4, L);
  print_matrix("L", L, 4, 4);

  return 0;
}

int test_mat_trace() {
  /* clang-format off */
  real_t A[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  /* clang-format on */
  const real_t tr = mat_trace(A, 4, 4);
  MU_ASSERT(fltcmp(tr, 1.0 + 6.0 + 11.0 + 16.0) == 0.0);

  return 0;
}

int test_mat_transpose() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  mat_transpose(A, 3, 3, C);
  print_matrix("C", C, 3, 3);

  real_t B[3 * 4] =
      {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
  real_t D[9] = {0.0};
  print_matrix("B", B, 3, 4);
  mat_transpose(B, 3, 4, D);
  print_matrix("D", D, 4, 3);

  return 0;
}

int test_mat_add() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
  real_t C[9] = {0.0};
  mat_add(A, B, C, 3, 3);
  print_matrix("C", C, 3, 3);

  return 0;
}

int test_mat_sub() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  mat_sub(A, B, C, 3, 3);
  print_matrix("C", C, 3, 3);

  return 0;
}

int test_mat_scale() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  mat_scale(A, 3, 3, 2.0);
  print_matrix("A", A, 3, 3);

  return 0;
}

int test_vec_add() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0};
  real_t C[9] = {0.0};
  vec_add(A, B, C, 9);
  print_vector("C", C, 9);

  return 0;
}

int test_vec_sub() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t C[9] = {0.0};
  vec_sub(A, B, C, 9);
  print_vector("C", C, 9);

  return 0;
}

/* void dot(const real_t *A, const size_t A_m, const size_t A_n, */
/*          const real_t *B, const size_t B_m, const size_t B_n, */
/*          real_t *C) { */
/*   assert(A_n == B_m); */
/*  */
/*   cblas_dgemm( */
/*     CblasRowMajor, #<{(| Matrix data arrangement |)}># */
/*     CblasNoTrans,  #<{(| Transpose A |)}># */
/*     CblasNoTrans,  #<{(| Transpose B |)}># */
/*     A_m,           #<{(| Number of rows in A and C |)}># */
/*     B_n,           #<{(| Number of cols in B and C |)}># */
/*     A_n,           #<{(| Number of cols in A |)}># */
/*     1.0,           #<{(| Scaling factor for the product of A and B |)}># */
/*     A,             #<{(| Matrix A |)}># */
/*     A_n,           #<{(| First dimension of A |)}># */
/*     B,             #<{(| Matrix B |)}># */
/*     B_n,           #<{(| First dimension of B |)}># */
/*     1.0,           #<{(| Scale factor for C |)}># */
/*     C,             #<{(| Output |)}># */
/*     A_m            #<{(| First dimension of C |)}># */
/*   ); */
/* } */

int test_dot() {
  real_t A[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  real_t B[3] = {1.0, 2.0, 3.0};
  real_t C[3] = {0.0};

  /* Multiply matrix A and B */
  dot(A, 3, 3, B, 3, 1, C);
  print_vector("C", C, 3);

  MU_ASSERT(fltcmp(C[0], 14.0) == 0);
  MU_ASSERT(fltcmp(C[1], 32.0) == 0);
  MU_ASSERT(fltcmp(C[2], 50.0) == 0);

  return 0;
}

int test_skew() {
  real_t x[3] = {1.0, 2.0, 3.0};
  real_t S[3 * 3] = {0};

  skew(x, S);
  print_matrix("S", S, 3, 3);

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
  const int print = 1;

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

/******************************************************************************
 * SVD
 ******************************************************************************/

int test_lapack_svd() {
  /* clang-format off */
  real_t A[6 * 4] = {
     7.52, -1.10, -7.95,  1.08,
    -0.76,  0.62,  9.34, -7.10,
     5.13,  6.62, -5.66,  0.87,
    -4.75,  8.52,  5.75,  5.30,
     1.33,  4.91, -5.49, -3.52,
    -2.40, -6.77,  2.34,  3.95
  };
  /* clang-format on */

  const int m = 6;
  const int n = 4;
  real_t *S = NULL;
  real_t *U = NULL;
  real_t *V_t = NULL;

  lapack_svd(A, m, n, &S, &U, &V_t);

  print_matrix("A", A, m, n);
  print_vector("S", S, 4);
  print_matrix("U", U, m, n);
  print_matrix("V_t", V_t, m, n);

  free(S);
  free(U);
  free(V_t);

  return 0;
}

/******************************************************************************
 * CHOL
 ******************************************************************************/

int test_chol() {
  /* clang-format off */
  const int n = 3;
  real_t A[9] = {
    4.0, 12.0, -16.0,
    12.0, 37.0, -43.0,
    -16.0, -43.0, 98.0
  };
  /* clang-format on */

  struct timespec t = tic();
  real_t L[9] = {0};
  chol(A, n, L);
  printf("time taken: [%fs]\n", toc(&t));

  real_t Lt[9] = {0};
  real_t LLt[9] = {0};
  mat_transpose(L, n, n, Lt);
  dot(L, n, n, Lt, n, n, LLt);

  int debug = 1;
  /* int debug = 0; */
  if (debug) {
    print_matrix("L", L, n, n);
    printf("\n");
    print_matrix("Lt", Lt, n, n);
    printf("\n");
    print_matrix("LLt", LLt, n, n);
    printf("\n");
    print_matrix("A", A, n, n);
  }

  int retval = mat_equals(A, LLt, n, n, 1e-5);
  MU_ASSERT(retval == 0);

  return 0;
}

int test_chol_solve() {
  /* clang-format off */
  const int n = 3;
  real_t A[9] = {
    2.0, -1.0, 0.0,
    -1.0, 2.0, -1.0,
    0.0, -1.0, 1.0
  };
  real_t b[3] = {1.0, 0.0, 0.0};
  real_t x[3] = {0.0, 0.0, 0.0};
  /* clang-format on */

  struct timespec t = tic();
  chol_solve(A, b, x, n);
  printf("time taken: [%fs]\n", toc(&t));
  print_vector("x", x, n);

  MU_ASSERT(fltcmp(x[0], 1.0) == 0);
  MU_ASSERT(fltcmp(x[1], 1.0) == 0);
  MU_ASSERT(fltcmp(x[2], 1.0) == 0);

  return 0;
}

#ifdef USE_LAPACK
int test_chol_solve2() {
  /* #<{(| clang-format off |)}># */
  /* const int m = 3; */
  /* const real_t A[9] = { */
  /*   2.0, -1.0, 0.0, */
  /*   -1.0, 2.0, -1.0, */
  /*   0.0, -1.0, 1.0 */
  /* }; */
  /* const real_t b[3] = {1.0, 0.0, 0.0}; */
  /* real_t x[3] = {0.0, 0.0, 0.0}; */
  /* #<{(| clang-format on |)}># */

  /* real_t a[9] = { 1.0, .6, .3, .6, 1., .5, .3, .5, 1 }; */
  /* print_matrix("a", a, 3, 3); */
  /* int retval = LAPACKE_dpotrf(LAPACK_ROW_MAJOR, 'L', 3, a, 3); */
  /* if (retval != 0) { */
  /*   fprintf(stderr, "Failed to decompose A using Cholesky Decomposition!\n");
   */
  /* } */
  /* print_matrix("a", a, 3, 3); */
  /* mat_save("/tmp/A.csv", A, m, m); */

  /* clang-format off */
  int m = 4;
  real_t A[16] = {
    4.16, -3.12, 0.56, -0.10,
    -3.12, 5.03, -0.83, 1.18,
    0.56, -0.83, 0.76, 0.34,
    -0.10, 1.18,  0.34, 1.18
  };
  real_t b[4] = {1.0, 0.0, 0.0, 0.0};
  real_t x[4] = {0.0, 0.0, 0.0, 0.0};
  /* clang-format on */

  struct timespec t = tic();
  lapack_chol_solve(A, b, x, m);
  /* OCTAVE_SCRIPT("scripts/plot_matrix.m /tmp/A.csv"); */
  printf("time taken: [%fs]\n", toc(&t));
  print_vector("x", x, m);

  /* MU_ASSERT(fltcmp(x[0], 1.0) == 0); */
  /* MU_ASSERT(fltcmp(x[1], 1.0) == 0); */
  /* MU_ASSERT(fltcmp(x[2], 1.0) == 0); */

  return 0;
}
#endif

/******************************************************************************
 * TRANSFORMS
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
  /* clang-format off */
  real_t T[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  /* clang-format on */
  print_matrix("T", T, 4, 4);

  /* Get translation vector */
  real_t r[3];
  tf_trans_get(T, r);
  print_vector("r", r, 3);

  MU_ASSERT(fltcmp(r[0], 4.0) == 0);
  MU_ASSERT(fltcmp(r[1], 8.0) == 0);
  MU_ASSERT(fltcmp(r[2], 12.0) == 0);

  return 0;
}

int test_tf_rot_get() {
  /* Transform */
  /* clang-format off */
  real_t T[16] = {1.0, 2.0, 3.0, 4.0,
                  5.0, 6.0, 7.0, 8.0,
                  9.0, 10.0, 11.0, 12.0,
                  13.0, 14.0, 15.0, 16.0};
  /* clang-format on */
  print_matrix("T", T, 4, 4);

  /* Get rotation matrix */
  real_t C[9];
  tf_rot_get(T, C);
  print_matrix("C", C, 3, 3);

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
  /* clang-format off */
  real_t T[16] = {1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0};
  /* clang-format on */

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
  print_vector("ypr_out", ypr_out, 3);

  MU_ASSERT(fltcmp(rad2deg(ypr_out[0]), 10.0) == 0);
  MU_ASSERT(fltcmp(rad2deg(ypr_out[1]), 20.0) == 0);
  MU_ASSERT(fltcmp(rad2deg(ypr_out[2]), 30.0) == 0);

  return 0;
}

int test_tf_inv() {
  /* Create Transform */
  /* clang-format off */
  real_t T[16] = {1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0};
  /* clang-format on */
  /* -- Set rotation component */
  const real_t euler[3] = {deg2rad(10.0), deg2rad(20.0), deg2rad(30.0)};
  real_t C[9] = {0};
  euler321(euler, C);
  tf_rot_set(T, C);
  /* -- Set translation component */
  real_t r[3] = {1.0, 2.0, 3.0};
  tf_trans_set(T, r);
  print_matrix("T", T, 4, 4);
  printf("\n");

  /* Invert transform */
  real_t T_inv[16] = {0};
  tf_inv(T, T_inv);
  print_matrix("T_inv", T_inv, 4, 4);
  printf("\n");

  /* real_t Invert transform */
  real_t T_inv_inv[16] = {0};
  tf_inv(T_inv, T_inv_inv);
  print_matrix("T_inv_inv", T_inv_inv, 4, 4);

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
  /* clang-format off */
  real_t T[16] = {1.0, 0.0, 0.0, 1.0,
                  0.0, 1.0, 0.0, 2.0,
                  0.0, 0.0, 1.0, 3.0,
                  0.0, 0.0, 0.0, 1.0};
  /* clang-format on */
  print_matrix("T", T, 4, 4);

  /* Point */
  real_t p[3] = {1.0, 2.0, 3.0};
  print_vector("p", p, 3);

  /* Transform point */
  real_t result[3] = {0};
  tf_point(T, p, result);
  print_vector("result", result, 3);

  return 0;
}

int test_tf_hpoint() {
  /* Transform */
  /* clang-format off */
  real_t T[16] = {1.0, 0.0, 0.0, 1.0,
                  0.0, 1.0, 0.0, 2.0,
                  0.0, 0.0, 1.0, 3.0,
                  0.0, 0.0, 0.0, 1.0};
  /* clang-format on */
  print_matrix("T", T, 4, 4);

  /* Homogeneous point */
  real_t hp[4] = {1.0, 2.0, 3.0, 1.0};
  print_vector("hp", hp, 4);

  /* Transform homogeneous point */
  real_t result[4] = {0};
  tf_hpoint(T, hp, result);
  print_vector("result", result, 4);

  return 0;
}

int test_tf_perturb_rot() {
  /* Transform */
  /* clang-format off */
  real_t T[4 * 4] = {1.0, 0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0, 2.0,
                     0.0, 0.0, 1.0, 3.0,
                     0.0, 0.0, 0.0, 1.0};
  /* clang-format on */

  /* Perturb rotation */
  const real_t step_size = 1e-2;
  tf_perturb_rot(T, step_size, 0);
  print_matrix("T", T, 4, 4);

  /* Assert */
  MU_ASSERT(fltcmp(T[0], 1.0) == 0);
  MU_ASSERT(fltcmp(T[5], 1.0) != 0);
  MU_ASSERT(fltcmp(T[10], 1.0) != 0);

  return 0;
}

int test_tf_perturb_trans() {
  /* Transform */
  /* clang-format off */
  real_t T[4 * 4] = {1.0, 0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0, 2.0,
                     0.0, 0.0, 1.0, 3.0,
                     0.0, 0.0, 0.0, 1.0};
  /* clang-format on */

  /* Perturb translation */
  const real_t step_size = 1e-2;
  tf_perturb_trans(T, step_size, 0);
  print_matrix("T", T, 4, 4);

  /* Assert */
  MU_ASSERT(fltcmp(T[3], 1.01) == 0);
  MU_ASSERT(fltcmp(T[7], 2.0) == 0);
  MU_ASSERT(fltcmp(T[11], 3.0) == 0);

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

  print_vector("euler", euler, 3);
  print_vector("euler2", euler2, 3);

  return 0;
}

int test_rot2quat() {
  /* Rotation matrix to quaternion */
  const real_t C[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  real_t q[4] = {0.0};
  rot2quat(C, q);
  print_vector("q", q, 4);

  return 0;
}

int test_quat2euler() {
  const real_t C[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  /* Rotation matrix to quaternion */
  real_t q[4] = {0.0};
  rot2quat(C, q);
  print_vector("q", q, 4);

  /* Quaternion to Euler angles */
  real_t rpy[3] = {0.0};
  quat2euler(q, rpy);
  print_vector("euler", rpy, 3);

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
 * LIE
 ******************************************************************************/

int test_lie_Exp_Log() {
  const real_t phi[3] = {0.1, 0.2, 0.3};
  real_t C[3 * 3] = {0};
  lie_Exp(phi, C);

  real_t rvec[3] = {0};
  lie_Log(C, rvec);

  print_vector("phi", phi, 3);
  printf("\n");
  print_matrix("C", C, 3, 3);
  print_vector("rvec", rvec, 3);

  MU_ASSERT(fltcmp(phi[0], rvec[0]) == 0);
  MU_ASSERT(fltcmp(phi[1], rvec[1]) == 0);
  MU_ASSERT(fltcmp(phi[2], rvec[2]) == 0);

  return 0;
}

/******************************************************************************
 * CV
 ******************************************************************************/

/* IMAGE ---------------------------------------------------------------------*/

int test_image_setup() { return 0; }

int test_image_load() { return 0; }

int test_image_print_properties() { return 0; }

int test_image_free() { return 0; }

/* GEOMETRY ------------------------------------------------------------------*/

int test_linear_triangulation() {
  /* Setup camera */
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

  /* Setup camera pose T_WC0 */
  const real_t ypr_WC0[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC0[3] = {0.0, 0.0, 0.0};
  real_t T_WC0[4 * 4] = {0};
  tf_euler_set(T_WC0, ypr_WC0);
  tf_trans_set(T_WC0, r_WC0);

  /* Setup camera pose T_WC1 */
  const real_t euler_WC1[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC1[3] = {0.1, 0.1, 0.0};
  real_t T_WC1[4 * 4] = {0};
  tf_euler_set(T_WC1, euler_WC1);
  tf_trans_set(T_WC1, r_WC1);

  /* Setup projection matrices */
  real_t P0[3 * 4] = {0};
  real_t P1[3 * 4] = {0};
  pinhole_projection_matrix(proj_params, T_WC0, P0);
  pinhole_projection_matrix(proj_params, T_WC1, P1);

  /* Setup 3D and 2D correspondance points */
  int nb_tests = 100;
  for (int i = 0; i < nb_tests; i++) {
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

    /* Test */
    real_t p_W_est[3] = {0};
    linear_triangulation(P0, P1, z0, z1, p_W_est);

    /* Assert */
    real_t diff[3] = {0};
    vec_sub(p_W, p_W_est, diff, 3);
    const real_t norm = vec_norm(diff, 3);
    /* print_vector("p_W [gnd]", p_W, 3); */
    /* print_vector("p_W [est]", p_W_est, 3); */
    MU_ASSERT(norm < 1e-4);
  }

  return 0;
}

/* RADTAN --------------------------------------------------------------------*/

int test_radtan4_distort() {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t p_d[2] = {0};
  radtan4_distort(params, p, p_d);

  print_vector("p", p, 2);
  print_vector("p_d", p_d, 2);

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
  print_vector("p", p, 2);
  print_matrix("J_point", J_point, 2, 2);
  print_matrix("J_numdiff", J_numdiff, 2, 2);
  check_jacobian("J", J_numdiff, J_point, 2, 2, tol, 1);

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
  print_vector("p", p, 2);
  print_matrix("J_param", J_param, 2, 4);
  print_matrix("J_numdiff", J_numdiff, 2, 4);
  check_jacobian("J", J_numdiff, J_param, 2, 4, tol, 1);

  return 0;
}

/* EQUI ----------------------------------------------------------------------*/

int test_equi4_distort() {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t p_d[2] = {0};
  equi4_distort(params, p, p_d);

  print_vector("p", p, 2);
  print_vector("p_d", p_d, 2);

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
  print_vector("p", p, 2);
  print_matrix("J_point", J_point, 2, 2);
  print_matrix("J_numdiff", J_numdiff, 2, 2);
  check_jacobian("J", J_numdiff, J_point, 2, 2, tol, 1);

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
  print_vector("p", p, 2);
  print_matrix("J_param", J_param, 2, 4);
  print_matrix("J_numdiff", J_numdiff, 2, 4);
  check_jacobian("J", J_numdiff, J_param, 2, 4, tol, 1);

  return 0;
}

/* PINHOLE -------------------------------------------------------------------*/

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

/* PINHOLE-RADTAN4 -----------------------------------------------------------*/

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

/* PINHOLE-EQUI4 -------------------------------------------------------------*/

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

/******************************************************************************
 * SENSOR FUSION
 ******************************************************************************/

int test_pose_setup() {
  timestamp_t ts = 1;
  pose_t pose;

  real_t data[7] = {0.1, 0.2, 0.3, 1.1, 2.2, 3.3, 1.0};
  pose_setup(&pose, ts, data);

  MU_ASSERT(pose.ts == 1);

  MU_ASSERT(fltcmp(pose.pos[0], 0.1) == 0.0);
  MU_ASSERT(fltcmp(pose.pos[1], 0.2) == 0.0);
  MU_ASSERT(fltcmp(pose.pos[2], 0.3) == 0.0);

  MU_ASSERT(fltcmp(pose.quat[0], 1.0) == 0.0);
  MU_ASSERT(fltcmp(pose.quat[1], 1.1) == 0.0);
  MU_ASSERT(fltcmp(pose.quat[2], 2.2) == 0.0);
  MU_ASSERT(fltcmp(pose.quat[3], 3.3) == 0.0);

  return 0;
}

int test_speed_biases_setup() {
  timestamp_t ts = 1;
  speed_biases_t sb;

  real_t data[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  speed_biases_setup(&sb, ts, data);

  MU_ASSERT(sb.ts == 1);

  MU_ASSERT(fltcmp(sb.data[0], 1.0) == 0.0);
  MU_ASSERT(fltcmp(sb.data[1], 2.0) == 0.0);
  MU_ASSERT(fltcmp(sb.data[2], 3.0) == 0.0);

  MU_ASSERT(fltcmp(sb.data[3], 4.0) == 0.0);
  MU_ASSERT(fltcmp(sb.data[4], 5.0) == 0.0);
  MU_ASSERT(fltcmp(sb.data[5], 6.0) == 0.0);

  MU_ASSERT(fltcmp(sb.data[6], 7.0) == 0.0);
  MU_ASSERT(fltcmp(sb.data[7], 8.0) == 0.0);
  MU_ASSERT(fltcmp(sb.data[8], 9.0) == 0.0);

  return 0;
}

int test_feature_setup() {
  feature_t feature;

  real_t data[3] = {0.1, 0.2, 0.3};
  feature_setup(&feature, data);

  MU_ASSERT(fltcmp(feature.data[0], 0.1) == 0.0);
  MU_ASSERT(fltcmp(feature.data[1], 0.2) == 0.0);
  MU_ASSERT(fltcmp(feature.data[2], 0.3) == 0.0);

  return 0;
}

int test_extrinsics_setup() {
  extrinsics_t extrinsics;

  real_t data[7] = {1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 1.0};
  extrinsics_setup(&extrinsics, data);

  MU_ASSERT(fltcmp(extrinsics.pos[0], 1.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsics.pos[1], 2.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsics.pos[2], 3.0) == 0.0);

  MU_ASSERT(fltcmp(extrinsics.quat[0], 1.0) == 0.0);
  MU_ASSERT(fltcmp(extrinsics.quat[1], 0.1) == 0.0);
  MU_ASSERT(fltcmp(extrinsics.quat[2], 0.2) == 0.0);
  MU_ASSERT(fltcmp(extrinsics.quat[3], 0.3) == 0.0);

  return 0;
}

int test_camera_params_setup() {
  camera_params_t camera;
  const int cam_idx = 0;
  const int cam_res[2] = {752, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&camera, cam_idx, cam_res, proj_model, dist_model, data);
  camera_params_print(&camera);

  return 0;
}

int test_pose_factor_setup() {
  /* Pose */
  timestamp_t ts = 1;
  pose_t pose;
  real_t data[7] = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0};
  pose_setup(&pose, ts, data);

  /* Setup pose factor */
  pose_factor_t pose_factor;
  real_t var[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  pose_factor_setup(&pose_factor, &pose, var);

  print_matrix("pose_factor.pos_meas", pose_factor.pos_meas, 3, 1);
  print_matrix("pose_factor.quat_meas", pose_factor.quat_meas, 4, 1);
  print_matrix("pose_factor.covar", pose_factor.covar, 6, 6);

  return 0;
}

int test_pose_factor_eval() {
  /* Pose */
  timestamp_t ts = 1;
  pose_t pose;
  real_t data[7] = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0};
  pose_setup(&pose, ts, data);

  /* Setup pose factor */
  pose_factor_t pose_factor;
  real_t var[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  pose_factor_setup(&pose_factor, &pose, var);

  /* Evaluate pose factor */
  real_t *params[2] = {pose.pos, pose.quat};
  real_t r[6] = {0};
  real_t J0[6 * 3] = {0};
  real_t J1[6 * 3] = {0};
  real_t *jacs[2] = {J0, J1};
  const int retval = pose_factor_eval(&pose_factor, params, r, jacs);
  print_matrix("r", r, 6, 1);
  print_matrix("J0", J0, 6, 3);
  print_matrix("J1", J1, 6, 3);
  MU_ASSERT(retval == 0);

  /* Check jacobians */
  real_t step_size = 1e-8;
  real_t tol = 1e-4;

  /* -- Check pose position jacobian */
  real_t J0_numdiff[2 * 3] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[0][i] += step_size;
    pose_factor_eval(&pose_factor, params, r_fwd, NULL);
    params[0][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J0_numdiff, 3, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J0", J0_numdiff, J0, 2, 3, tol, 1) == 0);

  /* -- Check pose rotation jacobian */
  real_t J1_numdiff[2 * 3] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    quat_perturb(params[1], i, step_size);
    pose_factor_eval(&pose_factor, params, r_fwd, NULL);
    quat_perturb(params[1], i, -step_size);

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J1_numdiff, 3, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J1", J1_numdiff, J1, 2, 3, tol, 1) == 0);

  return 0;
}

int test_ba_factor_setup() {
  /* Timestamp */
  timestamp_t ts = 0;

  /* Camera pose */
  const real_t pose_data[7] = {0.01, 0.02, 0.0, -0.5, 0.5, -0.5, 0.5};
  pose_t pose;
  pose_setup(&pose, ts, pose_data);

  /* Feature */
  const real_t p_W[3] = {1.0, 0.0, 0.0};
  feature_t feature;
  feature_setup(&feature, p_W);

  /* Camera parameters */
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_t cam;
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  /* Project point from world to image plane */
  real_t T_WC[4 * 4] = {0};
  real_t T_CW[4 * 4] = {0};
  real_t p_C[3] = {0};
  real_t z[2];
  tf(pose_data, T_WC);
  tf_inv(T_WC, T_CW);
  tf_point(T_CW, p_W, p_C);
  pinhole_radtan4_project(cam_data, p_C, z);

  /* Bundle adjustment factor */
  ba_factor_t ba_factor;
  real_t var[2] = {1.0, 1.0};
  ba_factor_setup(&ba_factor, &pose, &feature, &cam, z, var);
  print_matrix("covar", ba_factor.covar, 2, 2);
  print_matrix("sqrt_info", ba_factor.sqrt_info, 2, 2);

  return 0;
}

int test_ba_factor_eval() {
  /* Timestamp */
  timestamp_t ts = 0;

  /* Camera pose */
  const real_t pose_data[7] = {0.01, 0.01, 0.0, -0.5, 0.5, -0.5, 0.5};
  pose_t pose;
  pose_setup(&pose, ts, pose_data);

  /* Feature */
  const real_t p_W[3] = {1.0, 0.0, 0.0};
  feature_t feature;
  feature_setup(&feature, p_W);

  /* Camera parameters */
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {640, 480, 320, 240, 0.03, 0.01, 0.001, 0.001};
  camera_params_t cam;
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  /* Project point from world to image plane */
  real_t T_WC[4 * 4] = {0};
  real_t T_CW[4 * 4] = {0};
  real_t p_C[3] = {0};
  real_t z[2];
  tf(pose_data, T_WC);
  tf_inv(T_WC, T_CW);
  tf_point(T_CW, p_W, p_C);
  pinhole_radtan4_project(cam_data, p_C, z);

  /* Bundle adjustment factor */
  ba_factor_t ba_factor;
  real_t var[2] = {1.0, 1.0};
  ba_factor_setup(&ba_factor, &pose, &feature, &cam, z, var);

  /* Evaluate bundle adjustment factor */
  real_t *params[4] = {pose.pos, pose.quat, feature.data, cam.data};
  real_t r[2] = {0};
  real_t J0[2 * 3] = {0};
  real_t J1[2 * 3] = {0};
  real_t J2[2 * 3] = {0};
  real_t J3[2 * 8] = {0};
  real_t *jacs[4] = {J0, J1, J2, J3};
  ba_factor_eval(&ba_factor, params, r, jacs);

  print_matrix("covar", ba_factor.covar, 2, 2);
  print_matrix("sqrt_info", ba_factor.sqrt_info, 2, 2);
  print_matrix("r", r, 2, 1);
  print_matrix("J0", J0, 2, 3);
  print_matrix("J1", J1, 2, 3);
  print_matrix("J2", J2, 2, 3);
  print_matrix("J3", J3, 2, 8);

  /* Check jacobians */
  real_t step_size = 1e-8;
  real_t tol = 1e-4;

  /* -- Check pose position jacobian */
  real_t J0_numdiff[2 * 3] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[0][i] += step_size;
    ba_factor_eval(&ba_factor, params, r_fwd, NULL);
    params[0][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J0_numdiff, 3, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J0", J0_numdiff, J0, 2, 3, tol, 1) == 0);

  /* -- Check pose rotation jacobian */
  real_t J1_numdiff[2 * 3] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    quat_perturb(params[1], i, step_size);
    ba_factor_eval(&ba_factor, params, r_fwd, NULL);
    quat_perturb(params[1], i, -step_size);

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J1_numdiff, 3, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J1", J1_numdiff, J1, 2, 3, tol, 1) == 0);

  /* -- Check feature jacobian */
  real_t J2_numdiff[2 * 3] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[2][i] += step_size;
    ba_factor_eval(&ba_factor, params, r_fwd, NULL);
    params[2][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J2_numdiff, 3, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J2", J2_numdiff, J2, 2, 3, tol, 1) == 0);

  /* -- Check camera parameters jacobian */
  real_t J3_numdiff[2 * 8] = {0};
  for (int i = 0; i < 8; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[3][i] += step_size;
    ba_factor_eval(&ba_factor, params, r_fwd, NULL);
    params[3][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J3_numdiff, 3, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J3", J3_numdiff, J3, 2, 3, tol, 1) == 0);

  return 0;
}

int test_ba_factor_ceres_eval() {
  /* Timestamp */
  timestamp_t ts = 0;

  /* Camera pose */
  const real_t pose_data[7] = {0.01, 0.01, 0.0, -0.5, 0.5, -0.5, 0.5};
  pose_t pose;
  pose_setup(&pose, ts, pose_data);

  /* Feature */
  const real_t p_W[3] = {1.0, 0.0, 0.0};
  feature_t feature;
  feature_setup(&feature, p_W);

  /* Camera parameters */
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {640, 480, 320, 240, 0.03, 0.01, 0.001, 0.001};
  camera_params_t cam;
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  /* Project point from world to image plane */
  real_t T_WC[4 * 4] = {0};
  real_t T_CW[4 * 4] = {0};
  real_t p_C[3] = {0};
  real_t z[2];
  tf(pose_data, T_WC);
  tf_inv(T_WC, T_CW);
  tf_point(T_CW, p_W, p_C);
  pinhole_radtan4_project(cam_data, p_C, z);

  /* Bundle adjustment factor */
  ba_factor_t ba_factor;
  real_t var[2] = {1.0, 1.0};
  ba_factor_setup(&ba_factor, &pose, &feature, &cam, z, var);

  /* Evaluate bundle adjustment factor */
  real_t *params[4] = {pose.pos, pose.quat, feature.data, cam.data};
  real_t r[2] = {0};
  real_t J0[2 * 3] = {0};
  real_t J1[2 * 3] = {0};
  real_t J2[2 * 3] = {0};
  real_t J3[2 * 8] = {0};
  real_t *jacs[4] = {J0, J1, J2, J3};
  ba_factor_ceres_eval(&ba_factor, params, r, jacs);

  print_matrix("covar", ba_factor.covar, 2, 2);
  print_matrix("sqrt_info", ba_factor.sqrt_info, 2, 2);
  print_matrix("r", r, 2, 1);
  print_matrix("J0", J0, 2, 7);
  print_matrix("J1", J1, 2, 3);
  print_matrix("J2", J2, 2, 8);

  return 0;
}

int test_cam_factor_setup() {
  /* Timestamp */
  timestamp_t ts = 0;

  /* Body pose */
  pose_t pose;
  const real_t pose_data[7] = {0.01, 0.02, 0.0, -0.5, 0.5, -0.5, 0.5};
  pose_setup(&pose, ts, pose_data);

  /* Extrinsics */
  extrinsics_t extrinsics;
  const real_t exts_data[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  extrinsics_setup(&extrinsics, exts_data);

  /* Feature */
  feature_t feature;
  const real_t p_W[3] = {1.0, 0.0, 0.0};
  feature_setup(&feature, p_W);

  /* Camera parameters */
  camera_params_t cam;
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  /* Project point from world to image plane */
  real_t T_WB[4 * 4] = {0};
  real_t T_BW[4 * 4] = {0};
  real_t T_BCi[4 * 4] = {0};
  real_t T_CiB[4 * 4] = {0};
  real_t T_CiW[4 * 4] = {0};
  real_t p_Ci[3] = {0};
  real_t z[2];
  tf(pose_data, T_WB);
  tf(exts_data, T_BCi);
  tf_inv(T_WB, T_BW);
  tf_inv(T_BCi, T_CiB);
  dot(T_CiB, 4, 4, T_BW, 4, 4, T_CiW);
  tf_point(T_CiW, p_W, p_Ci);
  pinhole_radtan4_project(cam_data, p_Ci, z);

  /* Camera factor */
  cam_factor_t cam_factor;
  real_t var[2] = {1.0, 1.0};
  cam_factor_setup(&cam_factor, &pose, &extrinsics, &feature, &cam, z, var);
  print_matrix("cam_factor.covar", cam_factor.covar, 2, 2);

  return 0;
}

int test_cam_factor_eval() {
  /* Timestamp */
  timestamp_t ts = 0;

  /* Body pose */
  pose_t pose;
  const real_t pose_data[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  pose_setup(&pose, ts, pose_data);

  /* Extrinsics */
  extrinsics_t cam_exts;
  /* const real_t exts_data[7] = {0.0, 0.0, 0.0, -0.5, 0.5, -0.5, 0.5}; */
  const real_t exts_data[7] = {0.01, 0.02, 0.03, -0.5, 0.5, -0.5, 0.5};
  extrinsics_setup(&cam_exts, exts_data);

  /* Feature */
  feature_t feature;
  const real_t p_W[3] = {1.0, 0.0, 0.0};
  feature_setup(&feature, p_W);

  /* Camera parameters */
  camera_params_t cam;
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {320, 240, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  /* Project point from world to image plane */
  real_t T_WB[4 * 4] = {0};
  real_t T_BW[4 * 4] = {0};
  real_t T_BCi[4 * 4] = {0};
  real_t T_CiB[4 * 4] = {0};
  real_t T_CiW[4 * 4] = {0};
  real_t p_Ci[3] = {0};
  real_t z[2];
  tf(pose_data, T_WB);
  tf(exts_data, T_BCi);
  tf_inv(T_WB, T_BW);
  tf_inv(T_BCi, T_CiB);
  dot(T_CiB, 4, 4, T_BW, 4, 4, T_CiW);
  tf_point(T_CiW, p_W, p_Ci);
  pinhole_radtan4_project(cam_data, p_Ci, z);

  /* Setup camera factor */
  cam_factor_t cam_factor;
  real_t var[2] = {1.0, 1.0};
  cam_factor_setup(&cam_factor, &pose, &cam_exts, &feature, &cam, z, var);

  /* Evaluate camera factor */
  real_t *params[6] = {pose.pos,
                       pose.quat,
                       cam_exts.pos,
                       cam_exts.quat,
                       cam.data,
                       feature.data};
  real_t r[2] = {0};
  real_t J0[2 * 3] = {0};
  real_t J1[2 * 3] = {0};
  real_t J2[2 * 3] = {0};
  real_t J3[2 * 3] = {0};
  real_t J4[2 * 8] = {0};
  real_t J5[2 * 3] = {0};
  real_t *jacs[6] = {J0, J1, J2, J3, J4, J5};
  cam_factor_eval(&cam_factor, params, r, jacs);

  /* print_matrix("covar", cam_factor.covar, 2, 2); */
  /* print_matrix("sqrt_info", cam_factor.sqrt_info, 2, 2); */
  /* print_matrix("r", r, 2, 1); */
  /* print_matrix("J0", J0, 2, 3); */
  /* print_matrix("J1", jacs[1], 2, 3); */
  /* print_matrix("J2", jacs[2], 2, 3); */
  /* print_matrix("J3", jacs[3], 2, 3); */
  /* print_matrix("J4", jacs[4], 2, 8); */
  /* print_matrix("J5", jacs[5], 2, 3); */

  /* Check jacobians */
  real_t step_size = 1e-8;
  real_t tol = 1e-4;

  /* -- Check pose position jacobian */
  real_t J0_numdiff[2 * 3] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[0][i] += step_size;
    cam_factor_eval(&cam_factor, params, r_fwd, NULL);
    params[0][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J0_numdiff, 3, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J0", J0_numdiff, J0, 2, 3, tol, 1) == 0);

  /* -- Check pose rotation jacobian */
  real_t J1_numdiff[2 * 3] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    quat_perturb(params[1], i, step_size);
    cam_factor_eval(&cam_factor, params, r_fwd, NULL);
    quat_perturb(params[1], i, -step_size);

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J1_numdiff, 3, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J1", J1_numdiff, J1, 2, 3, tol, 1) == 0);

  /* -- Check extrinsics position jacobian */
  real_t J2_numdiff[2 * 3] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[2][i] += step_size;
    cam_factor_eval(&cam_factor, params, r_fwd, NULL);
    params[2][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J2_numdiff, 3, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J2", J2_numdiff, J2, 2, 3, tol, 1) == 0);

  /* -- Check extrinsics rotation jacobian */
  real_t J3_numdiff[2 * 3] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    quat_perturb(params[3], i, step_size);
    cam_factor_eval(&cam_factor, params, r_fwd, NULL);
    quat_perturb(params[3], i, -step_size);

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J3_numdiff, 3, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J3", J3_numdiff, J3, 2, 3, tol, 1) == 0);

  /* -- Check camera parameters jacobian */
  real_t J4_numdiff[2 * 8] = {0};
  for (int i = 0; i < 8; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[4][i] += step_size;
    cam_factor_eval(&cam_factor, params, r_fwd, NULL);
    params[4][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J4_numdiff, 8, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J4", J4_numdiff, J4, 2, 8, tol, 1) == 0);

  /* -- Check feature jacobian */
  real_t J5_numdiff[2 * 3] = {0};
  for (int i = 0; i < 3; i++) {
    real_t r_fwd[2] = {0};
    real_t r_diff[2] = {0};

    params[5][i] += step_size;
    cam_factor_eval(&cam_factor, params, r_fwd, NULL);
    params[5][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, 2);
    vec_scale(r_diff, 2, 1.0 / step_size);
    mat_col_set(J5_numdiff, 3, 2, i, r_diff);
  }
  MU_ASSERT(check_jacobian("J5", J5_numdiff, J5, 2, 3, tol, 1) == 0);

  return 0;
}

int test_cam_factor_ceres_eval() {
  /* Timestamp */
  timestamp_t ts = 0;

  /* Body pose */
  pose_t pose;
  const real_t pose_data[7] = {0.01, 0.02, 0.0, -0.5, 0.5, -0.5, 0.5};
  pose_setup(&pose, ts, pose_data);

  /* Extrinsics */
  extrinsics_t extrinsics;
  const real_t exts_data[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  extrinsics_setup(&extrinsics, exts_data);

  /* Feature */
  feature_t feature;
  const real_t p_W[3] = {1.0, 0.0, 0.0};
  feature_setup(&feature, p_W);

  /* Camera parameters */
  camera_params_t cam;
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t cam_data[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, cam_data);

  /* Project point from world to image plane */
  real_t T_WB[4 * 4] = {0};
  real_t T_BW[4 * 4] = {0};
  real_t T_BCi[4 * 4] = {0};
  real_t T_CiB[4 * 4] = {0};
  real_t T_CiW[4 * 4] = {0};
  real_t p_Ci[3] = {0};
  real_t z[2];
  tf(pose_data, T_WB);
  tf(exts_data, T_BCi);
  tf_inv(T_WB, T_BW);
  tf_inv(T_BCi, T_CiB);
  dot(T_CiB, 4, 4, T_BW, 4, 4, T_CiW);
  tf_point(T_CiW, p_W, p_Ci);
  pinhole_radtan4_project(cam_data, p_Ci, z);

  /* Setup camera factor */
  cam_factor_t cam_factor;
  real_t var[2] = {1.0, 1.0};
  cam_factor_setup(&cam_factor, &pose, &extrinsics, &feature, &cam, z, var);

  /* Evaluate camera factor */
  real_t *params[6] = {pose.pos,
                       pose.quat,
                       extrinsics.pos,
                       extrinsics.quat,
                       cam.data,
                       feature.data};
  double residuals[2] = {0};
  double J0[2 * 7] = {0};
  double J1[2 * 7] = {0};
  double J2[2 * 3] = {0};
  double J3[2 * 8] = {0};
  double *jacobians[4] = {J0, J1, J2, J3};
  cam_factor_ceres_eval(&cam_factor, params, residuals, jacobians);
  print_vector("z", z, 2);

  print_matrix("cam_factor.covar", cam_factor.covar, 2, 2);
  print_matrix("cam_factor.sqrt_info", cam_factor.sqrt_info, 2, 2);
  print_matrix("cam_factor.r", residuals, 2, 1);
  print_matrix("cam_factor.J0", jacobians[0], 2, 6);
  print_matrix("cam_factor.J1", jacobians[1], 2, 6);
  print_matrix("cam_factor.J2", jacobians[2], 2, 8);
  print_matrix("cam_factor.J3", jacobians[3], 2, 3);

  return 0;
}

int test_imu_buf_setup() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  return 0;
}

int test_imu_buf_add() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);
  imu_buf_print(&imu_buf);

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

int test_imu_buf_clear() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);
  imu_buf_clear(&imu_buf);

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

int test_imu_buf_copy() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);

  imu_buf_t imu_buf2;
  imu_buf_setup(&imu_buf2);
  imu_buf_copy(&imu_buf, &imu_buf2);

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

int test_imu_buf_print() {
  imu_buf_t imu_buf;
  imu_buf_setup(&imu_buf);

  timestamp_t ts = 0;
  real_t acc[3] = {1.0, 2.0, 3.0};
  real_t gyr[3] = {1.0, 2.0, 3.0};
  imu_buf_add(&imu_buf, ts, acc, gyr);

  imu_buf_print(&imu_buf);
  return 0;
}

/* int test_imu_factor_setup() { */
/*   imu_factor_t imu_factor; */
/*   imu_params_t imu_params; */
/*   imu_buf_t imu_buf; */
/*  */
/*   pose_t pose_i; */
/*   speed_biases_t sb_i; */
/*   pose_t pose_j; */
/*   speed_biases_t sb_j; */
/*  */
/*   imu_buf_setup(&imu_buf); */
/*  */
/*   imu_factor_setup(&imu_factor, */
/*                    &imu_params, */
/*                    &imu_buf, */
/*                    &pose_i, */
/*                    &sb_i, */
/*                    &pose_j, */
/*                    &sb_j); */
/*  */
/*   return 0; */
/* } */

// int test_ceres_graph() {
//   int num_observations = 67;
//   double data[] = {
//       0.000000e+00, 1.133898e+00, 7.500000e-02, 1.334902e+00, 1.500000e-01,
//       1.213546e+00, 2.250000e-01, 1.252016e+00, 3.000000e-01, 1.392265e+00,
//       3.750000e-01, 1.314458e+00, 4.500000e-01, 1.472541e+00, 5.250000e-01,
//       1.536218e+00, 6.000000e-01, 1.355679e+00, 6.750000e-01, 1.463566e+00,
//       7.500000e-01, 1.490201e+00, 8.250000e-01, 1.658699e+00, 9.000000e-01,
//       1.067574e+00, 9.750000e-01, 1.464629e+00, 1.050000e+00, 1.402653e+00,
//       1.125000e+00, 1.713141e+00, 1.200000e+00, 1.527021e+00, 1.275000e+00,
//       1.702632e+00, 1.350000e+00, 1.423899e+00, 1.425000e+00, 1.543078e+00,
//       1.500000e+00, 1.664015e+00, 1.575000e+00, 1.732484e+00, 1.650000e+00,
//       1.543296e+00, 1.725000e+00, 1.959523e+00, 1.800000e+00, 1.685132e+00,
//       1.875000e+00, 1.951791e+00, 1.950000e+00, 2.095346e+00, 2.025000e+00,
//       2.361460e+00, 2.100000e+00, 2.169119e+00, 2.175000e+00, 2.061745e+00,
//       2.250000e+00, 2.178641e+00, 2.325000e+00, 2.104346e+00, 2.400000e+00,
//       2.584470e+00, 2.475000e+00, 1.914158e+00, 2.550000e+00, 2.368375e+00,
//       2.625000e+00, 2.686125e+00, 2.700000e+00, 2.712395e+00, 2.775000e+00,
//       2.499511e+00, 2.850000e+00, 2.558897e+00, 2.925000e+00, 2.309154e+00,
//       3.000000e+00, 2.869503e+00, 3.075000e+00, 3.116645e+00, 3.150000e+00,
//       3.094907e+00, 3.225000e+00, 2.471759e+00, 3.300000e+00, 3.017131e+00,
//       3.375000e+00, 3.232381e+00, 3.450000e+00, 2.944596e+00, 3.525000e+00,
//       3.385343e+00, 3.600000e+00, 3.199826e+00, 3.675000e+00, 3.423039e+00,
//       3.750000e+00, 3.621552e+00, 3.825000e+00, 3.559255e+00, 3.900000e+00,
//       3.530713e+00, 3.975000e+00, 3.561766e+00, 4.050000e+00, 3.544574e+00,
//       4.125000e+00, 3.867945e+00, 4.200000e+00, 4.049776e+00, 4.275000e+00,
//       3.885601e+00, 4.350000e+00, 4.110505e+00, 4.425000e+00, 4.345320e+00,
//       4.500000e+00, 4.161241e+00, 4.575000e+00, 4.363407e+00, 4.650000e+00,
//       4.161576e+00, 4.725000e+00, 4.619728e+00, 4.800000e+00, 4.737410e+00,
//       4.875000e+00, 4.727863e+00, 4.950000e+00, 4.669206e+00,
//   };
//
//   /* This is the equivalent of a use-defined CostFunction in the C++ Ceres
//   API.
//    * This is passed as a callback to the Ceres C API, which internally
//    converts
//    * the callback into a CostFunction. */
//   int exponential_residual(void *user_data,
//                            double **parameters,
//                            double *residuals,
//                            double **jacobians) {
//     double *measurement = (double *) user_data;
//     double x = measurement[0];
//     double y = measurement[1];
//     double m = parameters[0][0];
//     double c = parameters[1][0];
//     residuals[0] = y - exp(m * x + c);
//     if (jacobians == NULL) {
//       return 1;
//     }
//     if (jacobians[0] != NULL) {
//       jacobians[0][0] = -x * exp(m * x + c); /* dr/dm */
//     }
//     if (jacobians[1] != NULL) {
//       jacobians[1][0] = -exp(m * x + c); /* dr/dc */
//     }
//     return 1;
//   }
//
//   /* Note: Typically it is better to compact m and c into one block,
//    * but in this case use separate blocks to illustrate the use of
//    * multiple parameter blocks. */
//   double m = 0.0;
//   double c = 0.0;
//   double *parameter_pointers[] = {&m, &c};
//   int parameter_sizes[] = {1, 1};
//   int i;
//   ceres_problem_t *problem;
//   /* Ceres has some internal stuff that needs to get initialized. */
//   ceres_init();
//   problem = ceres_create_problem();
//   /* Add all the residuals. */
//   for (i = 0; i < num_observations; ++i) {
//     ceres_problem_add_residual_block(problem,
//                                      exponential_residual, /* Cost function
//                                      */ &data[2 * i],         /* Points to
//                                      (x,y)
//                                                               measurement
//                                                               */
//                                      NULL,                 /* Loss function
//                                      */ NULL, /* Loss function user data */
//                                      1,    /* Number of residuals */
//                                      2,    /* Number of parameter blocks */
//                                      parameter_sizes,
//                                      parameter_pointers);
//   }
//   ceres_solve(problem);
//   ceres_free_problem(problem);
//   printf("Initial m: 0.0, c: 0.0\n");
//   printf("Final m: %g, c: %g\n", m, c);
//
//   return 0;
// }

/******************************************************************************
 * DATASET
 ******************************************************************************/

int test_assoc_pose_data() {
  const double threshold = 0.01;
  const char *matches_fpath = "./gnd_est_matches.csv";
  const char *gnd_data_path = "test_data/euroc/MH01_groundtruth.csv";
  const char *est_data_path = "test_data/euroc/MH01_estimate.csv";

  /* Load ground-truth poses */
  int nb_gnd_poses = 0;
  pose_t *gnd_poses = load_poses(gnd_data_path, &nb_gnd_poses);

  /* Load estimate poses */
  int nb_est_poses = 0;
  pose_t *est_poses = load_poses(est_data_path, &nb_est_poses);

  /* Associate data */
  size_t nb_matches = 0;
  int **matches = assoc_pose_data(gnd_poses,
                                  nb_gnd_poses,
                                  est_poses,
                                  nb_est_poses,
                                  threshold,
                                  &nb_matches);
  printf("Time Associated:\n");
  printf(" - [%s]\n", gnd_data_path);
  printf(" - [%s]\n", est_data_path);
  printf("threshold:  %.4f [s]\n", threshold);
  printf("nb_matches: %ld\n", nb_matches);

  /* Save matches to file */
  FILE *matches_csv = fopen(matches_fpath, "w");
  fprintf(matches_csv, "#gnd_idx,est_idx\n");
  for (size_t i = 0; i < nb_matches; i++) {
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

  /* Clean up */
  for (size_t i = 0; i < nb_matches; i++) {
    free(matches[i]);
  }
  free(matches);
  free(gnd_poses);
  free(est_poses);

  return 0;
}

/******************************************************************************
 * SIM
 ******************************************************************************/

// SIM FEATURES ////////////////////////////////////////////////////////////////

int test_load_sim_features() {
  const char *csv_file = TEST_SIM_DATA "/features.csv";
  sim_features_t *features_data = load_sim_features(csv_file);
  MU_ASSERT(features_data->nb_features > 0);
  free_sim_features(features_data);
  return 0;
}

// SIM IMU DATA ////////////////////////////////////////////////////////////////

int test_load_sim_imu_data() {
  const char *csv_file = TEST_SIM_DATA "/imu0/data.csv";
  sim_imu_data_t *imu_data = load_sim_imu_data(csv_file);
  free_sim_imu_data(imu_data);
  return 0;
}

// SIM CAMERA DATA /////////////////////////////////////////////////////////////

int test_load_sim_cam_frame() {
  const char *frame_csv = TEST_SIM_DATA "/cam0/data/100000000.csv";
  sim_cam_frame_t *frame_data = load_sim_cam_frame(frame_csv);

  MU_ASSERT(frame_data != NULL);
  MU_ASSERT(frame_data->ts == 100000000);
  MU_ASSERT(frame_data->feature_ids[0] == 1);

  free_sim_cam_frame(frame_data);

  return 0;
}

int test_load_sim_cam_data() {
  const char *dir_path = TEST_SIM_DATA "/cam0";
  sim_cam_data_t *cam_data = load_sim_cam_data(dir_path);

  free_sim_cam_data(cam_data);
  return 0;
}

int test_graph_setup() {
  graph_t graph;
  graph_setup(&graph);
  return 0;
}

int test_graph_print() {
  graph_t graph;
  graph_setup(&graph);
  graph_print(&graph);
  return 0;
}

typedef struct cam_view_t {
  pose_t pose;
  ba_factor_t factors[1000];
  int nb_factors;
  camera_params_t *cam_params;
} cam_view_t;

int test_graph_eval() {
  /* Load test data */
  const char *dir_path = TEST_SIM_DATA "/cam0";
  sim_cam_data_t *cam_data = load_sim_cam_data(dir_path);

  /* Camera parameters */
  camera_params_t cam;
  const int cam_idx = 0;
  const int cam_res[2] = {640, 480};
  const char *proj_model = "pinhole";
  const char *dist_model = "radtan4";
  const real_t params[8] = {640, 480, 320, 240, 0.0, 0.0, 0.0, 0.0};
  camera_params_setup(&cam, cam_idx, cam_res, proj_model, dist_model, params);

  /* Features container */
  features_t features;
  features_setup(&features);

  /* Loop over simulated camera frames */
  const real_t var[2] = {1.0, 1.0};
  cam_view_t *cam_views = malloc(sizeof(cam_view_t) * cam_data->nb_frames);
  for (int k = 0; k < cam_data->nb_frames; k++) {
    /* Camera frame */
    const sim_cam_frame_t *frame = cam_data->frames[k];

    /* Pose */
    pose_t *pose = &cam_views[k].pose;
    pose_setup(pose, frame->ts, cam_data->poses[k]);

    /* Add factors */
    cam_views[k].nb_factors = frame->nb_measurements;
    for (int i = 0; i < frame->nb_measurements; i++) {
      const int feature_id = frame->feature_ids[i];
      const real_t *z = frame->keypoints[i];

      /* Feature */
      feature_t *feature = NULL;
      if (features_exists(&features, feature_id)) {
        feature = features_get(&features, feature_id);
      } else {
        const real_t param[3] = {0};
        feature = features_add(&features, feature_id, param);
      }

      /* Factor */
      ba_factor_t *factor = &cam_views[k].factors[i];
      ba_factor_setup(factor, pose, feature, &cam, z, var);
    }
  }

  /* graph_t graph; */
  /* graph_setup(&graph); */

  /* Clean up */
  free(cam_views);
  free_sim_cam_data(cam_data);

  return 0;
}

void test_suite() {
  /* LOGGING */
  MU_ADD_TEST(test_debug);
  MU_ADD_TEST(test_log_error);
  MU_ADD_TEST(test_log_warn);

  /* FILE SYSTEM */
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

  /* DATA */
  MU_ADD_TEST(test_string_malloc);
  MU_ADD_TEST(test_dsv_rows);
  MU_ADD_TEST(test_dsv_cols);
  MU_ADD_TEST(test_dsv_fields);
  MU_ADD_TEST(test_dsv_data);
  MU_ADD_TEST(test_dsv_free);

  /* DATA-STRUCTURE */
  MU_ADD_TEST(test_darray_new_and_destroy);
  MU_ADD_TEST(test_darray_push_pop);
  MU_ADD_TEST(test_darray_contains);
  MU_ADD_TEST(test_darray_copy);
  MU_ADD_TEST(test_darray_new_element);
  MU_ADD_TEST(test_darray_set_and_get);
  MU_ADD_TEST(test_darray_update);
  MU_ADD_TEST(test_darray_remove);
  MU_ADD_TEST(test_darray_expand_and_contract);
  MU_ADD_TEST(test_list_new_and_destroy);
  MU_ADD_TEST(test_list_push_pop);
  MU_ADD_TEST(test_list_shift);
  MU_ADD_TEST(test_list_unshift);
  MU_ADD_TEST(test_list_remove);
  MU_ADD_TEST(test_list_remove_destroy);
  MU_ADD_TEST(test_stack_new_and_destroy);
  MU_ADD_TEST(test_stack_push);
  MU_ADD_TEST(test_stack_pop);
  MU_ADD_TEST(test_queue_new_and_destroy);
  MU_ADD_TEST(test_queue_enqueue_dequeue);

  /* TIME */
  MU_ADD_TEST(test_tic);
  MU_ADD_TEST(test_toc);
  MU_ADD_TEST(test_mtoc);
  MU_ADD_TEST(test_time_now);

  /* MATHS */
  MU_ADD_TEST(test_min);
  MU_ADD_TEST(test_max);
  MU_ADD_TEST(test_randf);
  MU_ADD_TEST(test_deg2rad);
  MU_ADD_TEST(test_rad2deg);
  MU_ADD_TEST(test_fltcmp);
  MU_ADD_TEST(test_fltcmp2);
  MU_ADD_TEST(test_pythag);
  MU_ADD_TEST(test_lerp);
  MU_ADD_TEST(test_lerp3);
  MU_ADD_TEST(test_sinc);
  MU_ADD_TEST(test_mean);
  MU_ADD_TEST(test_median);
  MU_ADD_TEST(test_var);
  MU_ADD_TEST(test_stddev);

  /* LINEAR ALGEBRA */
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
  MU_ADD_TEST(test_skew);
  MU_ADD_TEST(test_check_jacobian);

  /* SVD */
  /* MU_ADD_TEST(test_svd); */
  MU_ADD_TEST(test_lapack_svd);

  /* CHOL */
  MU_ADD_TEST(test_chol);
  MU_ADD_TEST(test_chol_solve);
#ifdef USE_LAPACK
  MU_ADD_TEST(test_chol_solve2);
#endif

  /* TRANSFORMS */
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
  MU_ADD_TEST(test_euler321);
  MU_ADD_TEST(test_rot2quat);
  MU_ADD_TEST(test_quat2euler);
  MU_ADD_TEST(test_quat2rot);

  /* LIE */
  MU_ADD_TEST(test_lie_Exp_Log);

  /* CV */
  /* -- IMAGE */
  MU_ADD_TEST(test_image_setup);
  MU_ADD_TEST(test_image_load);
  MU_ADD_TEST(test_image_print_properties);
  MU_ADD_TEST(test_image_free);
  /* -- GEOMETRY */
  MU_ADD_TEST(test_linear_triangulation);
  /* -- RADTAN */
  MU_ADD_TEST(test_radtan4_distort);
  MU_ADD_TEST(test_radtan4_point_jacobian);
  MU_ADD_TEST(test_radtan4_params_jacobian);
  /* -- EQUI */
  MU_ADD_TEST(test_equi4_distort);
  MU_ADD_TEST(test_equi4_point_jacobian);
  MU_ADD_TEST(test_equi4_params_jacobian);
  /* -- PINHOLE */
  MU_ADD_TEST(test_pinhole_focal);
  MU_ADD_TEST(test_pinhole_K);
  MU_ADD_TEST(test_pinhole_projection_matrix);
  MU_ADD_TEST(test_pinhole_project);
  MU_ADD_TEST(test_pinhole_point_jacobian);
  MU_ADD_TEST(test_pinhole_params_jacobian);
  /* -- PINHOLE-RADTAN4  */
  MU_ADD_TEST(test_pinhole_radtan4_project);
  MU_ADD_TEST(test_pinhole_radtan4_project_jacobian);
  MU_ADD_TEST(test_pinhole_radtan4_params_jacobian);
  /* -- PINHOLE-EQUI4  */
  MU_ADD_TEST(test_pinhole_equi4_project);
  MU_ADD_TEST(test_pinhole_equi4_project_jacobian);
  MU_ADD_TEST(test_pinhole_equi4_params_jacobian);

  /* SENSOR FUSION */
  /* -- Parameters */
  MU_ADD_TEST(test_pose_setup);
  MU_ADD_TEST(test_speed_biases_setup);
  MU_ADD_TEST(test_feature_setup);
  MU_ADD_TEST(test_extrinsics_setup);
  MU_ADD_TEST(test_camera_params_setup);
  /* -- Pose factor */
  MU_ADD_TEST(test_pose_factor_setup);
  MU_ADD_TEST(test_pose_factor_eval);
  /* -- BA factor */
  MU_ADD_TEST(test_ba_factor_setup);
  MU_ADD_TEST(test_ba_factor_eval);
  /* MU_ADD_TEST(test_ba_factor_ceres_eval); */
  /* -- Camera factor */
  MU_ADD_TEST(test_cam_factor_setup);
  MU_ADD_TEST(test_cam_factor_eval);
  MU_ADD_TEST(test_cam_factor_ceres_eval);
  /* -- IMU factor */
  MU_ADD_TEST(test_imu_buf_setup);
  MU_ADD_TEST(test_imu_buf_add);
  MU_ADD_TEST(test_imu_buf_clear);
  MU_ADD_TEST(test_imu_buf_copy);
  MU_ADD_TEST(test_imu_buf_print);
  /* MU_ADD_TEST(test_imu_factor_setup); */
  /* MU_ADD_TEST(test_imu_factor_eval); */
  /* -- Graph */
  /* MU_ADD_TEST(test_ceres_graph); */
  MU_ADD_TEST(test_graph_setup);
  MU_ADD_TEST(test_graph_print);
  MU_ADD_TEST(test_graph_eval);
  /* MU_ADD_TEST(test_graph_solve); */

  /* DATASET */
  /* MU_ADD_TEST(test_assoc_pose_data); */

  /* SIM */
  MU_ADD_TEST(test_load_sim_features);
  MU_ADD_TEST(test_load_sim_imu_data);
  MU_ADD_TEST(test_load_sim_cam_frame);
  MU_ADD_TEST(test_load_sim_cam_data);
}

MU_RUN_TESTS(test_suite)
