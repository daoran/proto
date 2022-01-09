#include "proto.h"

#ifdef USE_STB_IMAGE
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#endif

/******************************************************************************
 * FILE SYSTEM
 ******************************************************************************/

/**
 * Extract filename from `path` to `fname`.
 */
void path_file_name(const char *path, char *fname) {
  assert(path != NULL);
  assert(fname != NULL);

  char path_copy[9046] = {0};
  strncpy(path_copy, path, strlen(path));

  char *base = strrchr(path_copy, '/');
  base = base ? base + 1 : path_copy;

  strncpy(fname, base, strlen(base));
}

/**
 * Extract file extension from `path` to `fext`.
 */
void path_file_ext(const char *path, char *fext) {
  assert(path != NULL);
  assert(fext != NULL);

  char path_copy[9046] = {0};
  strncpy(path_copy, path, strlen(path));

  char *base = strrchr(path, '.');
  if (base) {
    base = base ? base + 1 : path_copy;
    strncpy(fext, base, strlen(base));
  } else {
    fext[0] = '\0';
  }
}

/**
 * Extract dir name from `path` to `dirname`.
 */
void path_dir_name(const char *path, char *dir_name) {
  assert(path != NULL);
  assert(dir_name != NULL);

  char path_copy[9046] = {0};
  strncpy(path_copy, path, strlen(path));

  char *base = strrchr(path_copy, '/');
  strncpy(dir_name, path_copy, base - path_copy);
}

/**
 * Join two paths `x` and `y`
 */
char *path_join(const char *x, const char *y) {
  assert(x != NULL && y != NULL);

  char *retval = NULL;
  if (x[strlen(x) - 1] == '/') {
    retval = malloc(sizeof(char) * (strlen(x) + strlen(y)) + 1);
    string_copy(retval, x);
    string_copy(retval + strlen(retval), (y[0] == '/') ? y + 1 : y);
  } else {
    retval = malloc(sizeof(char) * (strlen(x) + strlen(y)) + 2);
    string_copy(retval, x);
    string_cat(retval + strlen(retval), "/");
    string_copy(retval + strlen(retval), (y[0] == '/') ? y + 1 : y);
  }

  return retval;
}

/**
 * List files in `path`.
 * @returns List of files in directory and number of files `n`.
 */
char **list_files(const char *path, int *n) {
  assert(path != NULL);
  assert(n != NULL);

  struct dirent **namelist;
  int N = scandir(path, &namelist, 0, alphasort);
  if (N < 0) {
    return NULL;
  }

  /* The first two are '.' and '..' */
  free(namelist[0]);
  free(namelist[1]);

  /* Allocate memory for list of files */
  char **files = malloc(sizeof(char *) * (N - 2));
  *n = 0;

  /* Create list of files */
  for (int i = 2; i < N; i++) {
    char fp[9046] = {0};
    const char *c = (fp[strlen(fp) - 1] == '/') ? "" : "/";
    string_cat(fp, path);
    string_cat(fp, c);
    string_cat(fp, namelist[i]->d_name);

    files[*n] = malloc(sizeof(char) * (strlen(fp) + 1));
    strncpy(files[*n], fp, strlen(fp));
    (*n)++;

    free(namelist[i]);
  }
  free(namelist);

  return files;
}

/**
 * Free list of `files` of length `n`.
 */
void list_files_free(char **data, const int n) {
  assert(data != NULL);
  for (int i = 0; i < n; i++) {
    free(data[i]);
  }
  free(data);
}

/**
 * Read file contents in file path `fp`.
 * @returns
 * - Success: File contents
 * - Failure: NULL
 */
char *file_read(const char *fp) {
  assert(fp != NULL);
  FILE *f = fopen(fp, "rb");
  if (f == NULL) {
    return NULL;
  }

  fseek(f, 0, SEEK_END);
  long int len = ftell(f);
  fseek(f, 0, SEEK_SET);

  char *buf = malloc(sizeof(char) * (len + 1));
  if (buf) {
    fread(buf, 1, len, f);
    buf[len] = '\0';
  }
  fclose(f);

  return buf;
}

/**
 * Skip line in file.
 */
void skip_line(FILE *fp) {
  assert(fp != NULL);

  char header[BUFSIZ];
  char *retval = fgets(header, BUFSIZ, fp);
  if (retval == NULL) {
    FATAL("Failed to skip line!");
  }
}

/**
 * Check if file exists.
 * @returns
 * - 1 File exists
 * - 0 File does not exist
 */
int file_exists(const char *fp) {
  return (access(fp, F_OK) == 0) ? 1 : 0;
}

/**
 * Get number of rows in file `fp`.
 * @returns
 * - Number of rows in file
 * - -1 for failure.
 */
int file_rows(const char *fp) {
  assert(fp != NULL);

  FILE *file = fopen(fp, "rb");
  if (file == NULL) {
    fclose(file);
    return -1;
  }

  /* Obtain number of lines */
  int nb_rows = 0;
  char *line = NULL;
  size_t len = 0;
  while (getline(&line, &len, file) != -1) {
    nb_rows++;
  }
  free(line);

  /* Clean up */
  fclose(file);

  return nb_rows;
}

/**
 * Copy file from path `src` to path `dst`.
 * @returns
 * - 0 for success
 * - -1 if src file could not be opend
 * - -2 if dst file could not be opened
 */
int file_copy(const char *src, const char *dst) {
  assert(src != NULL);
  assert(dst != NULL);

  FILE *src_file = fopen(src, "rb");
  if (src_file == NULL) {
    fclose(src_file);
    return -1;
  }

  FILE *dst_file = fopen(dst, "wb");
  if (dst_file == NULL) {
    fclose(src_file);
    fclose(dst_file);
    return -2;
  }

  char *line = NULL;
  size_t len = 0;
  ssize_t read = 0;
  while ((read = getline(&line, &len, src_file)) != -1) {
    printf("[%ld,%ld]-->%s", read, len, line);
    fwrite(line, sizeof(char), read, dst_file);
  }
  if (line) {
    free(line);
  }

  /* Clean up */
  fclose(src_file);
  fclose(dst_file);

  return 0;
}

/******************************************************************************
 * DATA
 ******************************************************************************/

/**
 * String copy from `src` to `dst`.
 */
size_t string_copy(char *dst, const char *src) {
  memset(dst, '\0', sizeof(char) * strlen(dst));
  strncpy(dst, src, strlen(src));
  dst[strlen(src)] = '\0'; /* strncpy does not null terminate */
  return strlen(dst);
}

/**
 * Concatenate string from `src` to `dst`.
 */
void string_cat(char *dst, const char *src) {
  size_t dst_len = strlen(dst);
  strncat(dst + dst_len, src, strlen(src));
  dst[dst_len + strlen(src)] = '\0'; /* strncat does not null terminate */
}

/**
 * Allocate heap memory for string `s`.
 */
char *string_malloc(const char *s) {
  assert(s != NULL);
  char *retval = malloc(sizeof(char) * strlen(s) + 1);
  strncpy(retval, s, strlen(s));
  retval[strlen(s)] = '\0'; /* strncpy does not null terminate */
  return retval;
}

/**
 * Strip whitespace from string `s`.
 */
char *string_strip(char *s) {
  char *end;

  /* Trim leading space */
  while (*s == ' ') {
    s++;
  }

  if (*s == 0) { /* All spaces? */
    return s;
  }

  /* Trim trailing space */
  end = s + strlen(s) - 1;
  while (end > s && *end == ' ') {
    end--;
  }

  /* Write new null terminator character */
  end[1] = '\0';

  return s;
}

/**
 * Parse integer array line.
 * @returns
 * - 1D vector of integers
 * - NULL for failure
 */
static int *parse_iarray_line(char *line) {
  assert(line != NULL);
  char entry[MAX_LINE_LENGTH] = {0};
  int index = 0;
  int *data = NULL;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (data == NULL) {
        size_t array_size = strtod(entry, NULL);
        data = calloc(array_size + 1, sizeof(int));
      }
      data[index] = strtod(entry, NULL);
      index++;
      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return data;
}

/**
 * Parse 2D integer arrays from csv file.
 * @returns
 * - List of 1D vector of integers
 * - NULL for failure
 */
int **load_iarrays(const char *csv_path, int *nb_arrays) {
  assert(csv_path != NULL);
  FILE *csv_file = fopen(csv_path, "r");
  *nb_arrays = dsv_rows(csv_path);
  int **array = calloc(*nb_arrays, sizeof(int *));

  char line[MAX_LINE_LENGTH] = {0};
  int frame_idx = 0;
  while (fgets(line, MAX_LINE_LENGTH, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    array[frame_idx] = parse_iarray_line(line);
    frame_idx++;
  }
  fclose(csv_file);

  return array;
}

/**
 * Parse real array line.
 * @returns
 * - 1D vector of real
 * - NULL for failure
 */
static real_t *parse_darray_line(char *line) {
  assert(line != NULL);
  char entry[MAX_LINE_LENGTH] = {0};
  int index = 0;
  real_t *data = NULL;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (data == NULL) {
        size_t array_size = strtod(entry, NULL);
        data = calloc(array_size, sizeof(real_t));
      }
      data[index] = strtod(entry, NULL);
      index++;
      memset(entry, '\0', sizeof(char) * 100);
    } else {
      entry[strlen(entry)] = c;
    }
  }

  return data;
}

/**
 * Parse 2D real arrays from csv file at `csv_path`, on success `nb_arrays`
 * will return number of arrays.
 * @returns
 * - List of 1D vector of reals
 * - NULL for failure
 */
real_t **load_darrays(const char *csv_path, int *nb_arrays) {
  assert(csv_path != NULL);
  assert(nb_arrays != NULL);
  FILE *csv_file = fopen(csv_path, "r");
  *nb_arrays = dsv_rows(csv_path);
  real_t **array = calloc(*nb_arrays, sizeof(real_t *));

  char line[MAX_LINE_LENGTH] = {0};
  int frame_idx = 0;
  while (fgets(line, MAX_LINE_LENGTH, csv_file) != NULL) {
    if (line[0] == '#') {
      continue;
    }

    array[frame_idx] = parse_darray_line(line);
    frame_idx++;
  }
  fclose(csv_file);

  return array;
}

/**
 * Allocate heap memory for integer `val`.
 */
int *int_malloc(const int val) {
  int *i = malloc(sizeof(int));
  *i = val;
  return i;
}

/**
 * Allocate heap memory for float `val`.
 */
float *float_malloc(const float val) {
  float *f = malloc(sizeof(float));
  *f = val;
  return f;
}

/**
 * Allocate heap memory for double `val`.
 */
double *double_malloc(const double val) {
  double *d = malloc(sizeof(double));
  *d = val;
  return d;
}

/**
 * Get number of rows in a delimited file at `fp`.
 * @returns
 * - Number of rows
 * - -1 for failure
 */
int dsv_rows(const char *fp) {
  assert(fp != NULL);

  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    return -1;
  }

  /* Loop through lines */
  int nb_rows = 0;
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      nb_rows++;
    }
  }

  /* Cleanup */
  fclose(infile);

  return nb_rows;
}

/**
 * Get number of columns in a delimited file at `fp`.
 * @returns
 * - Number of columns
 * - -1 for failure
 */
int dsv_cols(const char *fp, const char delim) {
  assert(fp != NULL);

  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    return -1;
  }

  /* Get line that isn't the header */
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      break;
    }
  }

  /* Parse line to obtain number of elements */
  int nb_elements = 1;
  int found_separator = 0;
  for (size_t i = 0; i < MAX_LINE_LENGTH; i++) {
    if (line[i] == delim) {
      found_separator = 1;
      nb_elements++;
    }
  }

  /* Cleanup */
  fclose(infile);

  return (found_separator) ? nb_elements : -1;
}

/**
 * Get the fields of the delimited file at `fp`, where `delim` is the value
 * separated symbol and `nb_fields` returns the length of the fields returned.
 * @returns
 * - List of field strings
 * - NULL for failure
 */
char **dsv_fields(const char *fp, const char delim, int *nb_fields) {
  assert(fp != NULL);

  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Get last header line */
  char field_line[MAX_LINE_LENGTH] = {0};
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      break;
    } else {
      strncpy(field_line, line, strlen(line));
    }
  }

  /* Parse fields */
  *nb_fields = dsv_cols(fp, delim);
  char **fields = malloc(sizeof(char *) * *nb_fields);
  int field_idx = 0;
  char field_name[100] = {0};

  for (size_t i = 0; i < strlen(field_line); i++) {
    char c = field_line[i];

    /* Ignore # and ' ' */
    if (c == '#' || c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      /* Add field name to fields */
      fields[field_idx] = string_malloc(field_name);
      memset(field_name, '\0', 100);
      field_idx++;
    } else {
      /* Append field name */
      field_name[strlen(field_name)] = c;
    }
  }

  /* Cleanup */
  fclose(infile);

  return fields;
}

/**
 * Load delimited separated value data as a matrix.
 * @returns
 * - Matrix of DSV data
 * - NULL for failure
 */
real_t **
dsv_data(const char *fp, const char delim, int *nb_rows, int *nb_cols) {
  assert(fp != NULL);

  /* Obtain number of rows and columns in dsv data */
  *nb_rows = dsv_rows(fp);
  *nb_cols = dsv_cols(fp, delim);
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    return NULL;
  }

  /* Loop through data */
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;

  /* Loop through data line by line */
  real_t **data = malloc(sizeof(real_t *) * *nb_rows);
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    /* Ignore if comment line */
    if (line[0] == '#') {
      continue;
    }

    /* Iterate through values in line separated by commas */
    data[row_idx] = malloc(sizeof(real_t) * *nb_cols);
    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        data[row_idx][col_idx] = strtod(entry, NULL);
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  /* Clean up */
  fclose(infile);

  return data;
}

/**
 * Free DSV data.
 */
void dsv_free(real_t **data, const int nb_rows) {
  assert(data != NULL);
  for (int i = 0; i < nb_rows; i++) {
    free(data[i]);
  }
  free(data);
}

/**
 * Load comma separated data as a matrix, where `fp` is the csv file path, on
 * success `nb_rows` and `nb_cols` will be filled.
 * @returns
 * - Matrix of CSV data
 * - NULL for failure
 */
real_t **csv_data(const char *fp, int *nb_rows, int *nb_cols) {
  assert(fp != NULL);
  return dsv_data(fp, ',', nb_rows, nb_cols);
}

/**
 * Free CSV data.
 */
void csv_free(real_t **data, const int nb_rows) {
  for (int i = 0; i < nb_rows; i++) {
    free(data[i]);
  }
  free(data);
}

/******************************************************************************
 * DATA-STRUCTURES
 ******************************************************************************/

// DARRAY //////////////////////////////////////////////////////////////////////

darray_t *darray_new(size_t element_size, size_t initial_max) {
  assert(element_size > 0);
  assert(initial_max > 0);

  darray_t *array = malloc(sizeof(darray_t));
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

  /* Push */
  array->contents[array->end] = el;
  array->end++;

  /* Expand darray if necessary */
  if (array->end >= array->max) {
    return darray_expand(array);
  }

  return 0;
}

void *darray_pop(darray_t *array) {
  assert(array != NULL);

  /* pop */
  void *el = darray_remove(array, array->end - 1);
  array->end--;

  /* contract */
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

  /* Check first element */
  void *element = darray_get(array, 0);
  if (element != NULL && cmp(element, el) == 0) {
    return 1;
  }

  /* Rest of element */
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

  /* Copy first element */
  darray_t *array_copy = darray_new(array->element_size, (size_t) array->max);
  void *el = darray_get(array, 0);
  void *el_copy = NULL;

  if (el != NULL) {
    el_copy = darray_new_element(array_copy);
    memcpy(el_copy, el, array->element_size);
    darray_set(array_copy, 0, el_copy);
  }

  /* Copy the rest of the elements */
  for (int i = 1; i < array->end; i++) {
    el = darray_get(array, i);
    /* el_copy = NULL; */

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

  /* Set */
  array->contents[i] = el;

  /* Update end */
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

  /* Update */
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

  /* Calculate new max and size */
  int old_max = (int) array->max;
  array->max = (int) new_max;

  /* Reallocate new memory */
  void *contents = realloc(array->contents, new_max * sizeof(void *));
  if (contents == NULL) {
    return -1;
  }
  array->contents = contents;

  /* Initialize new memory to NULL */
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

  /* Contract */
  int new_size = 0;
  if (array->end < (int) array->expand_rate) {
    new_size = (int) array->expand_rate;
  } else {
    new_size = array->end;
  }

  return darray_resize(array, (size_t) new_size + 1);
}

// LIST ////////////////////////////////////////////////////////////////////////

list_t *list_new() {
  list_t *list = calloc(1, sizeof(list_t));
  list->length = 0;
  list->first = NULL;
  list->last = NULL;
  return list;
}

void list_destroy(list_t *list) {
  assert(list != NULL);

  list_node_t *node;
  list_node_t *next_node;

  /* Destroy */
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

void list_clear_destroy(list_t *list) {
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

  /* Initialize node */
  list_node_t *node = calloc(1, sizeof(list_node_t));
  if (node == NULL) {
    return;
  }
  node->value = value;

  /* Push node */
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

  /* Get last */
  list_node_t *last = list->last;
  if (last == NULL) {
    return NULL;
  }
  void *value = last->value;
  list_node_t *before_last = last->prev;
  free(last);

  /* Pop */
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

  /* pop front */
  list_node_t *first_node = list->first;
  void *data = first_node->value;
  list_node_t *next_node = first_node->next;

  if (next_node != NULL) {
    list->first = next_node;
  } else {
    list->first = NULL;
  }
  list->length--;

  /* clean up */
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

  /* Iterate list */
  list_node_t *node = list->first;
  while (node != NULL) {

    /* Compare target with node value */
    if (cmp(node->value, value) == 0) {
      value = node->value;

      if (list->length == 1) {
        /* Last node in list */
        list->first = NULL;
        list->last = NULL;

      } else if (node == list->first) {
        /* First node in list */
        list->first = node->next;
        node->next->prev = NULL;

      } else if (node == list->last) {
        /* In the case of removing last node in list */
        list->last = node->prev;
        node->prev->next = NULL;

      } else {
        /* Remove others */
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

// STACK ///////////////////////////////////////////////////////////////////////

stack_t *stack_new() {
  stack_t *s = malloc(sizeof(stack_t));
  s->size = 0;
  s->root = NULL;
  s->end = NULL;
  return s;
}

void stack_destroy_traverse(stack_node_t *n, void (*free_func)(void *)) {
  if (n->next) {
    stack_destroy_traverse(n->next, free_func);
  }
  if (free_func) {
    free_func(n->value);
  }
  free(n);
  n = NULL;
}

void stack_clear_destroy(stack_t *s, void (*free_func)(void *)) {
  if (s->root) {
    stack_destroy_traverse(s->root, free_func);
  }
  free(s);
  s = NULL;
}

void stack_destroy(stack_t *s) {
  if (s->root) {
    stack_destroy_traverse(s->root, NULL);
  }
  free(s);
  s = NULL;
}

int stack_push(stack_t *s, void *value) {
  stack_node_t *n = malloc(sizeof(stack_node_t));
  if (n == NULL) {
    return -1;
  }

  stack_node_t *prev_end = s->end;
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

void *stack_pop(stack_t *s) {
  void *value = s->end->value;
  stack_node_t *previous = s->end->prev;

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

// QUEUE ///////////////////////////////////////////////////////////////////////

queue_t *queue_new() {
  queue_t *q = calloc(1, sizeof(queue_t));
  q->queue = list_new();
  q->count = 0;
  return q;
}

void queue_destroy(queue_t *q) {
  assert(q != NULL);
  list_destroy(q->queue);
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

// HASHMAP /////////////////////////////////////////////////////////////////////

static inline int default_cmp(void *a, void *b) {
  return strcmp(a, b);
}

static uint32_t default_hash(void *a) {
  /* Simple bob jenkins's hash algorithm */
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

hashmap_t *hashmap_new() {
  hashmap_t *map = malloc(sizeof(hashmap_t));
  if (map == NULL) {
    return NULL;
  }

  /* Create bucket */
  map->buckets = darray_new(sizeof(darray_t *), DEFAULT_NUMBER_OF_BUCKETS);
  map->buckets->end = map->buckets->max; // fake out expanding it
  if (map->buckets == NULL) {
    free(map);
    return NULL;
  }

  /* Set comparator and hash functions */
  map->cmp = default_cmp;
  map->hash = default_hash;

  /* Set key and value copy functions */
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

  /* Clear free bucket */
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

  /* Free buckets */
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

  /* Setup */
  hashmap_node_t *node = calloc(1, sizeof(hashmap_node_t));
  if (node == NULL) {
    return NULL;
  }

  /* Create hashmap node */
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

  /* Pre-check */
  uint32_t hash = map->hash(k);
  int bucket_n = hash % DEFAULT_NUMBER_OF_BUCKETS;
  if ((bucket_n >= 0) == 0) {
    return NULL;
  }
  *hash_out = hash; /* Store it for return so caller can use it */

  /* Find bucket */
  darray_t *bucket = darray_get(map->buckets, bucket_n);

  /* Coundn't find bucket, create one instead */
  if (!bucket && create) {
    /* New bucket, set it up */
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

  /* Pre-check */
  uint32_t hash = 0;
  darray_t *bucket = hashmap_find_bucket(map, k, 1, &hash);
  if (bucket == NULL) {
    return -1;
  }

  /* Set hashmap  */
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

  /* Find bucket */
  uint32_t hash = 0;
  darray_t *bucket = hashmap_find_bucket(map, k, 0, &hash);
  if (bucket == NULL) {
    return NULL;
  }

  /* Find hashmap node */
  int i = hashmap_get_node(map, hash, bucket, k);
  if (i == -1) {
    return NULL;
  }

  /* Get value */
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

  /* Traverse */
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

  /* Find bucket containing hashmap node */
  uint32_t hash = 0;
  darray_t *bucket = hashmap_find_bucket(map, k, 0, &hash);
  if (bucket == NULL) {
    return NULL;
  }

  /* From bucket get hashmap node and free it */
  int i = hashmap_get_node(map, hash, bucket, k);
  if (i == -1) {
    return NULL;
  }

  /* Get node */
  hashmap_node_t *node = darray_get(bucket, i);
  void *v = node->value;
  if (map->copy_kv) {
    map->k_free(node->key);
  }
  free(node);

  /* Check to see if last element in bucket is a node */
  hashmap_node_t *ending = darray_pop(bucket);
  if (ending != node) {
    /* Alright looks like it's not the last one, swap it */
    darray_set(bucket, i, ending);
  }

  return v;
}

/******************************************************************************
 * TIME
 ******************************************************************************/

/**
 * Tic, start timer.
 * @returns A timespec encapsulating the time instance when tic() is called
 */
struct timespec tic() {
  struct timespec time_start;
  clock_gettime(CLOCK_MONOTONIC, &time_start);
  return time_start;
}

/**
 * Toc, stop timer.
 * @returns Time elapsed in seconds
 */
float toc(struct timespec *tic) {
  assert(tic != NULL);
  struct timespec toc;
  float time_elasped;

  clock_gettime(CLOCK_MONOTONIC, &toc);
  time_elasped = (toc.tv_sec - tic->tv_sec);
  time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

  return time_elasped;
}

/**
 * Toc, stop timer.
 * @returns Time elapsed in milli-seconds
 */
float mtoc(struct timespec *tic) {
  assert(tic != NULL);
  return toc(tic) * 1000.0;
}

/**
 * Get time now since epoch.
 * @return Time now in nano-seconds since epoch
 */
timestamp_t time_now() {
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);

  const time_t sec = spec.tv_sec;
  const long int ns = spec.tv_nsec;
  const uint64_t BILLION = 1000000000L;

  return (uint64_t) sec * BILLION + (uint64_t) ns;
}

/**
 * Convert timestamp to seconds
 */
real_t ts2sec(const timestamp_t ts) {
  return ts * 1e-9;
}

/**
 * Convert seconds to timestamp
 */
timestamp_t sec2ts(const real_t time_s) {
  return time_s * 1e9;
}

/******************************************************************************
 * NETWORK
 ******************************************************************************/

/**
 * Return IP and Port info from socket file descriptor `sockfd` to `ip` and
 * `port`. Returns `0` for success and `-1` for failure.
 * @returns
 * - 0 for success
 * - -1 for failure
 */
int ip_port_info(const int sockfd, char *ip, int *port) {
  assert(ip != NULL);
  assert(port != NULL);

  struct sockaddr_storage addr;
  socklen_t len = sizeof addr;
  if (getpeername(sockfd, (struct sockaddr *) &addr, &len) != 0) {
    return -1;
  }

  // Deal with both IPv4 and IPv6:
  char ipstr[INET6_ADDRSTRLEN];

  if (addr.ss_family == AF_INET) {
    // IPV4
    struct sockaddr_in *s = (struct sockaddr_in *) &addr;
    *port = ntohs(s->sin_port);
    inet_ntop(AF_INET, &s->sin_addr, ipstr, sizeof(ipstr));
  } else {
    // IPV6
    struct sockaddr_in6 *s = (struct sockaddr_in6 *) &addr;
    *port = ntohs(s->sin6_port);
    inet_ntop(AF_INET6, &s->sin6_addr, ipstr, sizeof(ipstr));
  }
  strncpy(ip, ipstr, strlen(ipstr));

  return 0;
}

/**
 * Configure TCP server
 */
int tcp_server_setup(tcp_server_t *server, const int port) {
  assert(server != NULL);

  /* Setup server struct */
  server->port = port;
  server->sockfd = -1;
  server->conn = -1;

  /* Create socket */
  server->sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (server->sockfd == -1) {
    LOG_ERROR("Socket creation failed...");
    return -1;
  }

  /* Socket options */
  const int en = 1;
  const size_t int_sz = sizeof(int);
  if (setsockopt(server->sockfd, SOL_SOCKET, SO_REUSEADDR, &en, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEADDR) failed");
  }
  if (setsockopt(server->sockfd, SOL_SOCKET, SO_REUSEPORT, &en, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEPORT) failed");
  }

  /* Assign IP, PORT */
  struct sockaddr_in addr;
  bzero(&addr, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(server->port);

  /* Bind newly created socket to given IP */
  int retval = bind(server->sockfd, (struct sockaddr *) &addr, sizeof(addr));
  if (retval != 0) {
    LOG_ERROR("Socket bind failed: %s", strerror(errno));
    return -1;
  }

  return 0;
}

/**
 * Loop TCP server
 * @returns 0 for success, -1 for failure
 */
int tcp_server_loop(tcp_server_t *server) {
  assert(server != NULL);

  /* Server is ready to listen */
  if ((listen(server->sockfd, 5)) != 0) {
    LOG_ERROR("Listen failed...");
    return -1;
  }

  /* Accept the data packet from client and verification */
  DEBUG("Server ready!");
  while (1) {
    /* Accept incomming connections */
    struct sockaddr_in sockaddr;
    socklen_t len = sizeof(sockaddr);
    int connfd = accept(server->sockfd, (struct sockaddr *) &sockaddr, &len);
    if (connfd < 0) {
      LOG_ERROR("Server acccept failed!");
      return -1;
    } else {
      server->conn = connfd;
      server->conn_handler(&server);
    }
  }
  DEBUG("Server shutting down ...");

  return 0;
}

/**
 * Configure TCP client
 */
int tcp_client_setup(tcp_client_t *client,
                     const char *server_ip,
                     const int server_port) {
  assert(client != NULL);
  assert(server_ip != NULL);

  /* Setup client struct */
  strncpy(client->server_ip, server_ip, strlen(server_ip));
  client->server_port = server_port;
  client->sockfd = -1;

  /* Create socket */
  client->sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (client->sockfd == -1) {
    LOG_ERROR("Socket creation failed!");
    return -1;
  }

  /* Assign IP, PORT */
  struct sockaddr_in server;
  size_t server_size = sizeof(server);
  bzero(&server, server_size);
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = inet_addr(client->server_ip);
  server.sin_port = htons(client->server_port);

  /* Connect to server */
  if (connect(client->sockfd, (struct sockaddr *) &server, server_size) != 0) {
    LOG_ERROR("Failed to connect to server!");
    return -1;
  }
  DEBUG("Connected to the server!");

  return 0;
}

/**
 * Loop TCP client
 */
int tcp_client_loop(tcp_client_t *client) {
  while (1) {
    if (client->loop_cb) {
      int retval = client->loop_cb(client);
      switch (retval) {
        case -1:
          return -1;
        case 1:
          break;
      }
    }
  }

  return 0;
}

// HTTP ////////////////////////////////////////////////////////////////////////

static char b64_encode_table[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
                                  'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
                                  'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
                                  'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
                                  'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
                                  'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
                                  'w', 'x', 'y', 'z', '0', '1', '2', '3',
                                  '4', '5', '6', '7', '8', '9', '+', '/'};

char *base64_encode(const uint8_t *data, size_t in_len, size_t *out_len) {
  int mod_table[3] = {0, 2, 1};
  unsigned long i;
  unsigned long j;
  uint32_t octet_a;
  uint32_t octet_b;
  uint32_t octet_c;
  uint32_t triple;
  char *encoded_data;

  *out_len = (4 * ((in_len + 2) / 3));    /* length of the encoding string */
  encoded_data = calloc(1, *out_len + 1); /* +1 for the null char */

  if (encoded_data == NULL) {
    return NULL;
  }

  for (i = 0, j = 0; i < in_len;) {
    octet_a = i < in_len ? (uint8_t) data[i++] : 0;
    octet_b = i < in_len ? (uint8_t) data[i++] : 0;
    octet_c = i < in_len ? (uint8_t) data[i++] : 0;
    triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;

    encoded_data[j++] = b64_encode_table[(triple >> 3 * 6) & 0x3F];
    encoded_data[j++] = b64_encode_table[(triple >> 2 * 6) & 0x3F];
    encoded_data[j++] = b64_encode_table[(triple >> 1 * 6) & 0x3F];
    encoded_data[j++] = b64_encode_table[(triple >> 0 * 6) & 0x3F];
  }

  for (i = 0; i < (unsigned long) mod_table[in_len % 3]; i++) {
    encoded_data[*out_len - 1 - i] = '=';
  }

  return encoded_data;
}

uint8_t *base64_decode(const char *data, size_t in_len, size_t *out_len) {
  unsigned long i;
  unsigned long j;
  uint32_t sextet_a;
  uint32_t sextet_b;
  uint32_t sextet_c;
  uint32_t sextet_d;
  uint32_t triple;
  uint8_t *decoded_data;

  char *decode_table = malloc(256);
  for (int i = 0; i < 64; i++) {
    decode_table[(uint8_t) b64_encode_table[i]] = (char) i;
  }

  if (in_len % 4 != 0) {
    return NULL;
  }

  *out_len = in_len / 4 * 3;
  if (data[in_len - 1] == '=') {
    (*out_len)--;
  }

  if (data[in_len - 2] == '=') {
    (*out_len)--;
  }

  decoded_data = calloc(1, *out_len + 1);
  if (decoded_data == NULL) {
    return NULL;
  }

  for (i = 0, j = 0; i < in_len;) {
    sextet_a =
        (data[i] == '=') ? 0 & i++ : (uint32_t) decode_table[(int) data[i++]];
    sextet_b =
        (data[i] == '=') ? 0 & i++ : (uint32_t) decode_table[(int) data[i++]];
    sextet_c =
        (data[i] == '=') ? 0 & i++ : (uint32_t) decode_table[(int) data[i++]];
    sextet_d =
        (data[i] == '=') ? 0 & i++ : (uint32_t) decode_table[(int) data[i++]];

    triple = ((sextet_a << 3 * 6) + (sextet_b << 2 * 6) + (sextet_c << 1 * 6) +
              (sextet_d << 0 * 6));

    if (j < *out_len) {
      decoded_data[j++] = (triple >> 2 * 8) & 0xFF;
    }
    if (j < *out_len) {
      decoded_data[j++] = (triple >> 1 * 8) & 0xFF;
    }
    if (j < *out_len) {
      decoded_data[j++] = (triple >> 0 * 8) & 0xFF;
    }
  }

  free(decode_table);
  return decoded_data;
}

void http_msg_setup(http_msg_t *msg) {
  /* Protocol */
  msg->protocol = NULL;

  /* Request */
  msg->method = NULL;
  msg->path = NULL;

  /* Response */
  msg->status = NULL;

  /* Headers */
  msg->user_agent = NULL;
  msg->host = NULL;
  msg->upgrade = NULL;
  msg->connection = NULL;
  msg->sec_websocket_key = NULL;
  msg->sec_websocket_version = NULL;
}

void http_msg_free(http_msg_t *msg) {
  /* Protocol */
  FREE_MEM(msg->protocol, free);

  /* Request */
  FREE_MEM(msg->method, free);
  FREE_MEM(msg->path, free);

  /* Response */
  FREE_MEM(msg->status, free);

  /* Headers */
  FREE_MEM(msg->user_agent, free);
  FREE_MEM(msg->host, free);
  FREE_MEM(msg->upgrade, free);
  FREE_MEM(msg->connection, free);
  FREE_MEM(msg->sec_websocket_key, free);
  FREE_MEM(msg->sec_websocket_version, free);
}

void http_msg_print(http_msg_t *msg) {
  if (msg->method) {
    printf("%s %s %s\r\n", msg->method, msg->path, msg->protocol);
  }

  if (msg->status) {
    printf("%s %s\r\n", msg->status, msg->protocol);
  }

  if (msg->user_agent) {
    printf("User-Agent: %s\r\n", msg->user_agent);
  }

  if (msg->host) {
    printf("Host: %s\r\n", msg->host);
  }

  if (msg->upgrade) {
    printf("Upgrade: %s\r\n", msg->upgrade);
  }

  if (msg->connection) {
    printf("Connection: %s\r\n", msg->connection);
  }

  if (msg->sec_websocket_key) {
    printf("Sec-WebSocket-Key: %s\r\n", msg->sec_websocket_key);
  }

  if (msg->sec_websocket_version) {
    printf("Sec-WebSocket-Version: %s\r\n", msg->sec_websocket_version);
  }
}

int http_parse_request(char *msg_str, http_msg_t *msg) {
  int line_idx = 0;
  char line[1024] = {0};
  char *line_end = NULL;
  char *line_tok = __strtok_r(msg_str, "\r\n", &line_end);

  while (line_tok != NULL) {
    string_copy(line, line_tok);

    if (line_idx == 0) {
      /* Parse request line */
      /* Method */
      char *tok_end = NULL;
      char *tok = __strtok_r(line, " ", &tok_end);
      msg->method = string_malloc(tok);

      /* Path */
      tok = __strtok_r(NULL, " ", &tok_end);
      msg->path = string_malloc(tok);

      /* Protocol */
      tok = __strtok_r(NULL, " ", &tok_end);
      msg->protocol = string_malloc(tok);

    } else {
      /* Parse headers */
      char *tok_end = NULL;
      char *tok = __strtok_r(line, " ", &tok_end);

      if (strcmp(tok, "User-Agent:") == 0) {
        msg->user_agent = string_malloc(tok_end);
      } else if (strcmp(tok, "Host:") == 0) {
        msg->host = string_malloc(tok_end);
      } else if (strcmp(tok, "Upgrade:") == 0) {
        msg->upgrade = string_malloc(tok_end);
      } else if (strcmp(tok, "Connection:") == 0) {
        msg->connection = string_malloc(tok_end);
      } else if (strcmp(tok, "Sec-WebSocket-Key:") == 0) {
        msg->sec_websocket_key = string_malloc(tok_end);
      } else if (strcmp(tok, "Sec-WebSocket-Version:") == 0) {
        msg->sec_websocket_version = string_malloc(tok_end);
      }
    }

    line_tok = __strtok_r(NULL, "\r\n", &line_end);
    line_idx++;
  }

  return 0;
}

int http_request_websocket_handshake(const http_msg_t *msg) {
  const int upgrade_ok = strcmp(msg->upgrade, "websocket") == 0;
  const int connection_ok = strcmp(msg->connection, "Upgrade") == 0;
  if (upgrade_ok && connection_ok) {
    return 1;
  }
  return 0;
}

ws_frame_t *ws_frame_malloc() {
  ws_frame_t *frame;

  frame = malloc(sizeof(ws_frame_t));
  frame->header = 0x0;
  frame->mask[0] = 0x0;
  frame->mask[1] = 0x0;
  frame->mask[2] = 0x0;
  frame->mask[3] = 0x0;
  frame->payload_size = 0x0;
  frame->payload_data = NULL;

  return frame;
}

void ws_frame_free(ws_frame_t *frame) {
  /* FREE_MEM(frame->payload_data, free); */
  FREE_MEM(frame, free);
}

void ws_frame_print(ws_frame_t *frame) {
  printf("ws frame [size: %ld, type: 0x%x]: ",
         frame->payload_size,
         frame->header);
  for (int i = 0; i < (int) frame->payload_size; i++) {
    printf("%c", ((char *) frame->payload_data)[i]);
  }
  printf("\n");
}

uint8_t *ws_frame_serialize(ws_frame_t *frame) {
  /* Setup */
  uint8_t header[10];
  bzero(header, 10);
  header[0] = frame->header;

  /* Payload details */
  size_t header_size = 0;
  size_t payload_size = frame->payload_size;

  if (payload_size <= 126) {
    header[1] = (uint8_t)(payload_size & 0x00000000000000FFU);
    header_size = 2;

  } else if (payload_size >= 126 && payload_size <= 65535) {
    header[1] = WS_MASK_OFF | 0x7E;
    header[2] = (payload_size >> 8) & 0xFF;
    header[3] = payload_size & 0xFF;
    header_size = 4;

  } else {
    header[1] = WS_MASK_OFF | 0x7F;
    header[2] = (payload_size >> 56) & 0xFF;
    header[3] = (payload_size >> 48) & 0xFF;
    header[4] = (payload_size >> 40) & 0xFF;
    header[5] = (payload_size >> 32) & 0xFF;
    header[6] = (payload_size >> 24) & 0xFF;
    header[7] = (payload_size >> 16) & 0xFF;
    header[8] = (payload_size >> 8) & 0xFF;
    header[9] = payload_size & 0xFF;
    header_size = 10;
  }

  /* Serialize ws frame */
  size_t frame_size = header_size + payload_size;
  uint8_t *frame_bytes = calloc(1, frame_size);
  memcpy(frame_bytes, header, header_size);
  memcpy(frame_bytes + header_size, frame->payload_data, payload_size);
  return frame_bytes;
}

int ws_frame_fin_bit(uint8_t *data_frame) {
  return data_frame[0] >> 7;
}

int ws_frame_rsv_bit(uint8_t *data_frame) {
  return (data_frame[0] ^ 0x80) >> 4;
}

int ws_frame_op_code(uint8_t *data_frame) {
  return data_frame[0] & 0x0F;
}

int ws_frame_mask_enabled(uint8_t *data_frame) {
  return data_frame[1] >> 7;
}

ws_frame_t *ws_frame_parse(int connfd) {
  /* Parse header */
  uint8_t header[2] = {0};
  int retval = (int) recv(connfd, header, 2, 0);
  if (retval != 0) {
    goto error;
  }
  ws_frame_t *ws_frame = ws_frame_malloc();
  ws_frame->header = header[0];
  ws_frame->payload_size = header[1] & 0x7F;

  /* Additional payload size */
  if (ws_frame->payload_size == 126) {
    /* Obtain extended data size - 2 bytes */
    uint8_t buf_2bytes[2] = {0};
    retval = (int) recv(connfd, buf_2bytes, 2, 0);
    if (retval != 0) {
      goto error;
    }

    /* Parse payload size */
    ws_frame->payload_size = (((unsigned long long) buf_2bytes[0] << 8) |
                              ((unsigned long long) buf_2bytes[1]));

  } else if (ws_frame->payload_size == 127) {
    /* Obtain extended data size - 8 bytes */
    uint8_t buf_8bytes[8] = {0};
    retval = (int) recv(connfd, buf_8bytes, 8, 0);
    if (retval != 0) {
      goto error;
    }

    /* Parse payload size */
    ws_frame->payload_size =
        ((((unsigned long long) buf_8bytes[0] << 56) & 0xFF00000000000000U) |
         (((unsigned long long) buf_8bytes[1] << 48) & 0x00FF000000000000U) |
         (((unsigned long long) buf_8bytes[2] << 40) & 0x0000FF0000000000U) |
         (((unsigned long long) buf_8bytes[3] << 32) & 0x000000FF00000000U) |
         (((unsigned long long) buf_8bytes[4] << 24) & 0x00000000FF000000U) |
         (((unsigned long long) buf_8bytes[5] << 16) & 0x0000000000FF0000U) |
         (((unsigned long long) buf_8bytes[6] << 8) & 0x000000000000FF00U) |
         (((unsigned long long) buf_8bytes[7]) & 0x00000000000000FFU));
  }

  /* Recv mask */
  uint8_t mask[4] = {0};
  if (ws_frame_mask_enabled(header)) {
    retval = (int) recv(connfd, mask, 4, 0);
    if (retval != 0) {
      goto error;
    }
  }

  /* Recv payload */
  if (ws_frame->payload_size) {
    char *payload_data = calloc(1, sizeof(char) * ws_frame->payload_size);
    retval = (int) recv(connfd, payload_data, ws_frame->payload_size, 0);
    if (retval != 0) {
      goto error;
    }

    /* Decode payload data with mask */
    if (ws_frame_mask_enabled(header)) {
      for (size_t i = 0; i < ws_frame->payload_size; i++) {
        payload_data[i] = payload_data[i] ^ mask[i % 4];
      }
    }
    ws_frame->payload_data = payload_data;
  }

  return ws_frame;
error:
  return NULL;
}

char *ws_recv(int connfd) {
  ws_frame_t *frame = ws_frame_parse(connfd);
  if (frame->header != WS_TEXT || frame->header != (WS_FIN | WS_TEXT)) {
    ws_frame_free(frame);
    return NULL;
  } else {
    return (char *) frame->payload_data;
  }
}

void ws_send(int connfd, char *msg) {
  /* Setup */
  ws_frame_t *frame = ws_frame_malloc();
  frame->header = WS_FIN | WS_TEXT;
  frame->payload_size = strlen(msg);
  frame->payload_data = msg;

  /* Write */
  uint8_t *frame_bytes = ws_frame_serialize(frame);
  write(connfd, frame_bytes, frame->payload_size + 2);

  /* Clean up */
  free(frame_bytes);
  ws_frame_free(frame);
}

char *ws_read(ws_frame_t *ws_frame) {
  int i;
  char *message;

  message = calloc(1, sizeof(char) * ws_frame->payload_size + 1);
  for (i = 0; i < (int) ws_frame->payload_size; i++) {
    message[i] = ((char *) ws_frame->payload_data)[i];
  }

  return message;
}

char *ws_hash(const char *ws_key) {
  /* Concatenate websocket key and guid */
  const char *WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  char key[1024] = {0};
  string_copy(key, ws_key);
  string_cat(key, WS_GUID);

  /* Perform SHA1 hash on key */
  uint8_t hash[SHA_DIGEST_LENGTH];
  memset(hash, '\0', SHA_DIGEST_LENGTH);
  SHA1((const uint8_t *) key, strlen(key), hash);

  /* Encode SHA1 hash with base64 encoding */
  size_t length = 0;
  return base64_encode(hash, SHA_DIGEST_LENGTH, &length);
}

int ws_handshake(const int connfd) {
  /* Get incoming websocket handshake request */
  char buf[9046] = {0};
  recv(connfd, buf, 9046, 0);

  /* Parse HTTP Request */
  http_msg_t req;
  http_msg_setup(&req);
  http_parse_request(buf, &req);
  if (req.sec_websocket_key == NULL) {
    http_msg_free(&req);
    return -1;
  }

  /* Get WebSocket Key */
  char ws_key[128] = {0};
  string_copy(ws_key, req.sec_websocket_key);
  http_msg_free(&req);

  /* Respond websocket handshake and establish connection */
  char *hash = ws_hash(ws_key);
  char resp[1024] = {0};
  snprintf(resp, sizeof(resp), WEBSOCKET_HANDSHAKE_RESPONSE, hash);
  write(connfd, resp, strlen(resp));
  free(hash);

  return 0;
}

int ws_server() {
  /* Setup server */
  tcp_server_t server;
  const int port = 5000;
  if (tcp_server_setup(&server, port) != 0) {
    return -1;
  }

  /* Server is ready to listen */
  if ((listen(server.sockfd, 5)) != 0) {
    LOG_ERROR("Listen failed...");
    return -1;
  }

  /* Accept incomming connections */
  struct sockaddr_in sockaddr;
  socklen_t len = sizeof(sockaddr);
  int connfd = accept(server.sockfd, (struct sockaddr *) &sockaddr, &len);
  if (connfd < 0) {
    LOG_ERROR("Server acccept failed!");
    return -1;
  }

  /* Perform websocket handshake */
  ws_handshake(connfd);

  while (1) {
    char *msg = "Hello World!";
    ws_send(connfd, msg);
  }

  return 0;
}

/******************************************************************************
 *                                 MATHS
 ******************************************************************************/

/**
 * Generate random number between a and b from a uniform distribution.
 * @returns Random number
 */
float randf(const float a, const float b) {
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

/**
 * Degrees to radians.
 * @returns Radians
 */
real_t deg2rad(const real_t d) {
  return d * (M_PI / 180.0);
}

/**
 * Radians to degrees.
 * @returns Degrees
 */
real_t rad2deg(const real_t r) {
  return r * (180.0 / M_PI);
}

/**
 * Compare ints.
 * @returns
 * - 0 if v1 == v2
 * - 1 if v1 > v2
 * - -1 if v1 < v2
 */
int intcmp(const int x, int y) {
  if (x > y) {
    return 1;
  } else if (x < y) {
    return -1;
  }
  return 0;
}

/**
 Compare ints.
 * @returns
 * - 0 if v1 == v2
 * - 1 if v1 > v2
 * - -1 if v1 < v2
 */
int intcmp2(const void *x, const void *y) {
  return intcmp(*(int *) x, *(int *) y);
}

/**
 * Compare reals.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int fltcmp(const real_t x, const real_t y) {
  if (fabs(x - y) < CMP_TOL) {
    return 0;
  } else if (x > y) {
    return 1;
  }

  return -1;
}

/**
 * Compare reals.
 * @returns
 * - 0 if x == y
 * - 1 if x > y
 * - -1 if x < y
 */
int fltcmp2(const void *x, const void *y) {
  assert(x != NULL);
  assert(y != NULL);
  return fltcmp(*(real_t *) x, *(real_t *) y);
}

/**
 * Compare strings.
 */
int strcmp2(const void *x, const void *y) {
  return strcmp((char *) x, (char *) y);
}

/**
 * Pythagoras
 *
 *   c = sqrt(a^2 + b^2)
 *
 * @returns Hypotenuse of a and b
 */
real_t pythag(const real_t a, const real_t b) {
  real_t at = fabs(a);
  real_t bt = fabs(b);
  real_t ct = 0.0;
  real_t result = 0.0;

  if (at > bt) {
    ct = bt / at;
    result = at * sqrt(1.0 + ct * ct);
  } else if (bt > 0.0) {
    ct = at / bt;
    result = bt * sqrt(1.0 + ct * ct);
  } else {
    result = 0.0;
  }

  return result;
}

/**
 * Perform 1D Linear interpolation between `a` and `b` with `t` as the
 * interpolation hyper-parameter.
 * @returns Linear interpolated value between a and b
 */
real_t lerp(const real_t a, const real_t b, const real_t t) {
  return a * (1.0 - t) + b * t;
}

/**
 * Perform 3D Linear interpolation between `a` and `b` with `t` as the
 * interpolation hyper-parameter.
 */
void lerp3(const real_t a[3], const real_t b[3], const real_t t, real_t x[3]) {
  assert(a != NULL);
  assert(b != NULL);
  assert(x != NULL);

  x[0] = lerp(a[0], b[0], t);
  x[1] = lerp(a[1], b[1], t);
  x[2] = lerp(a[2], b[2], t);
}

/**
 * Sinc.
 * @return Result of sinc
 */
real_t sinc(const real_t x) {
  if (fabs(x) > 1e-6) {
    return sin(x) / x;
  } else {
    const real_t c2 = 1.0 / 6.0;
    const real_t c4 = 1.0 / 120.0;
    const real_t c6 = 1.0 / 5040.0;
    const real_t x2 = x * x;
    const real_t x4 = x2 * x2;
    const real_t x6 = x2 * x2 * x2;
    return 1.0 - c2 * x2 + c4 * x4 - c6 * x6;
  }
}

/**
 * Calculate mean from vector `x` of length `n`.
 * @returns Mean of x
 */
real_t mean(const real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  real_t sum = 0.0;
  for (size_t i = 0; i < n; i++) {
    sum += x[i];
  }
  return sum / n;
}

/**
 * Calculate median from vector `x` of length `n`.
 * @returns Median of x
 */
real_t median(const real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  /* Make a copy of the original input vector x */
  real_t *vals = malloc(sizeof(real_t) * n);
  for (size_t i = 0; i < n; i++) {
    vals[i] = x[i];
  }

  /* Sort the values */
  qsort(vals, n, sizeof(real_t), fltcmp2);

  /* Get median value */
  const size_t midpoint = n / 2.0;
  real_t median_value = 0.0;
  if ((n % 2) == 0) {
    median_value = (vals[n] + vals[n - 1]) / 2.0;
  } else {
    median_value = vals[midpoint];
  }

  /* Clean up */
  free(vals);

  return median_value;
}

/**
 * Calculate variance from vector `x` of length `n`.
 * @returns Variance of x
 */
real_t var(const real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  const real_t mu = mean(x, n);
  real_t sse = 0.0;
  for (size_t i = 0; i < n; i++) {
    sse += (x[i] - mu) * (x[i] - mu);
  }

  return sse / (n - 1);
}

/**
 * Calculate standard deviation from vector `x` of length `n`.
 * @returns Standard deviation of x
 */
real_t stddev(const real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);
  return sqrt(var(x, n));
}

/******************************************************************************
 * LINEAR ALGEBRA
 ******************************************************************************/

/**
 * Print matrix `A` of size `m x n`.
 */
void print_matrix(const char *prefix,
                  const real_t *A,
                  const size_t m,
                  const size_t n) {
  assert(prefix != NULL);
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0;
  printf("%s:\n", prefix);
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      printf("%f  ", A[idx]);
      idx++;
    }
    printf("\n");
  }
  printf("\n");
}

/**
 * Print vector `v` of length `n`.
 */
void print_vector(const char *prefix, const real_t *v, const size_t n) {
  assert(prefix != NULL);
  assert(v != NULL);
  assert(n != 0);

  size_t idx = 0;
  printf("%s: ", prefix);
  for (size_t i = 0; i < n; i++) {
    printf("%.4f ", v[idx]);
    idx++;
  }
  printf("\n");
}

/**
 * Form identity matrix `A` of size `m x n`.
 */
void eye(real_t *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A[idx] = (i == j) ? 1.0 : 0.0;
      idx++;
    }
  }
}

/**
 * Form ones matrix `A` of size `m x n`.
 */
void ones(real_t *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A[idx] = 1.0;
      idx++;
    }
  }
}

/**
 * Form zeros matrix `A` of size `m x n`.
 */
void zeros(real_t *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m != 0);
  assert(n != 0);

  size_t idx = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A[idx] = 0.0;
      idx++;
    }
  }
}

/**
 * Malloc matrix of size `m x n`.
 */
real_t *mat_malloc(const size_t m, const size_t n) {
  assert(m > 0);
  assert(n > 0);
  return calloc(m * n, sizeof(real_t));
}

/**
 * Compare two matrices `A` and `B` of size `m x n`.
 *
 * @returns
 * - 0 if A == B
 * - 1 if A > B
 * - -1 if A < B
 */
int mat_cmp(const real_t *A, const real_t *B, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(B != NULL);
  assert(m > 0);
  assert(n > 0);

  size_t index = 0;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      int retval = fltcmp(A[index], B[index]);
      if (retval != 0) {
        printf("Failed at index[%zu]\n", index);
        return retval;
      }
      index++;
    }
  }

  return 0;
}

/**
 * Check to see if two matrices `A` and `B` of size `m x n` are equal to a
 * tolerance.
 * @returns 0 if A == B or -1 if A != B
 */
int mat_equals(const real_t *A,
               const real_t *B,
               const size_t m,
               const size_t n,
               const real_t tol) {
  assert(A != NULL);
  assert(B != NULL);
  assert(m > 0);
  assert(n > 0);
  assert(tol > 0);

  size_t index = 0;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      if (fabs(A[index] - B[index]) > tol) {
        printf("Failed at index[%zu]\n", index);
        return -1;
      }
      index++;
    }
  }

  return 0;
}

/**
 * Save matrix `A` of size `m x n` to `save_path`.
 * @returns 0 for success, -1 for failure
 */
int mat_save(const char *save_path, const real_t *A, const int m, const int n) {
  assert(save_path != NULL);
  assert(A != NULL);
  assert(m > 0);
  assert(n > 0);

  FILE *csv_file = fopen(save_path, "w");
  if (csv_file == NULL) {
    return -1;
  }

  int idx = 0;
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      fprintf(csv_file, "%e", A[idx]);
      idx++;
      if ((j + 1) != n) {
        fprintf(csv_file, ",");
      }
    }
    fprintf(csv_file, "\n");
  }
  fclose(csv_file);

  return 0;
}

/**
 * Load matrix from file in `mat_path`, on success `nb_rows` and `nb_cols` will
 * be set respectively.
 */
real_t *mat_load(const char *mat_path, int *nb_rows, int *nb_cols) {
  assert(mat_path != NULL);
  assert(nb_rows != NULL);
  assert(nb_cols != NULL);

  /* Obtain number of rows and columns in csv data */
  *nb_rows = dsv_rows(mat_path);
  *nb_cols = dsv_cols(mat_path, ',');
  if (*nb_rows == -1 || *nb_cols == -1) {
    return NULL;
  }

  /* Initialize memory for csv data */
  real_t *A = malloc(sizeof(real_t) * *nb_rows * *nb_cols);

  /* Load file */
  FILE *infile = fopen(mat_path, "r");
  if (infile == NULL) {
    fclose(infile);
    free(A);
    return NULL;
  }

  /* Loop through data */
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;
  int idx = 0;

  /* Loop through data line by line */
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    /* Ignore if comment line */
    if (line[0] == '#') {
      continue;
    }

    /* Iterate through values in line separated by commas */
    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        A[idx] = strtod(entry, NULL);
        idx++;

        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;
      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  /* Clean up */
  fclose(infile);

  return A;
}

/**
 * Set matrix `A` with value `val` at `(i, j)`.
 */
void mat_set(real_t *A,
             const size_t stride,
             const size_t i,
             const size_t j,
             const real_t val) {
  assert(A != NULL);
  assert(stride != 0);
  A[(i * stride) + j] = val;
}

/**
 * Get value from matrix `A` with `stride` at `(i, j)`.
 */
real_t
mat_val(const real_t *A, const size_t stride, const size_t i, const size_t j) {
  assert(A != NULL);
  assert(stride != 0);
  return A[(i * stride) + j];
}

/**
 * Copy matrix `src` of size `m x n` to `dest`.
 */
void mat_copy(const real_t *src, const int m, const int n, real_t *dest) {
  assert(src != NULL);
  assert(m > 0);
  assert(n > 0);
  assert(dest != NULL);

  for (int i = 0; i < (m * n); i++) {
    dest[i] = src[i];
  }
}

/**
 * Set matrix row.
 */
void mat_row_set(real_t *A,
                 const size_t stride,
                 const int row_idx,
                 const real_t *x) {
  int vec_idx = 0;
  for (int i = 0; i < stride; i++) {
    A[(stride * row_idx) + i] = x[vec_idx++];
  }
}

/**
 * Set matrix column.
 */
void mat_col_set(real_t *A,
                 const size_t stride,
                 const int nb_rows,
                 const int col_idx,
                 const real_t *x) {
  int vec_idx = 0;
  for (int i = 0; i < nb_rows; i++) {
    A[i * stride + col_idx] = x[vec_idx++];
  }
}

/**
 * Get matrix sub-block from `A` with `stride` from row and column start `rs`
 * and `cs`, to row and column end `re` and `ce`. The sub-block is written to
 * `block`.
 */
void mat_block_get(const real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   real_t *block) {
  assert(A != NULL);
  assert(block != NULL);
  assert(A != block);
  assert(stride != 0);

  size_t idx = 0;
  for (size_t i = rs; i <= re; i++) {
    for (size_t j = cs; j <= ce; j++) {
      block[idx] = mat_val(A, stride, i, j);
      idx++;
    }
  }
}

/**
 * Set matrix sub-block `block` to `A` with `stride` from row and column start
 * `rs` and `cs`, to row and column end `re` and `ce`.
 */
void mat_block_set(real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t cs,
                   const size_t re,
                   const size_t ce,
                   const real_t *block) {
  assert(A != NULL);
  assert(block != NULL);
  assert(A != block);
  assert(stride != 0);

  size_t idx = 0;
  for (size_t i = rs; i <= re; i++) {
    for (size_t j = cs; j <= ce; j++) {
      mat_set(A, stride, i, j, block[idx]);
      idx++;
    }
  }
}

/**
 * Get diagonal vector `d` from matrix `A` of size `m x n`.
 */
void mat_diag_get(const real_t *A, const int m, const int n, real_t *d) {
  int mat_index = 0;
  int vec_index = 0;

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j) {
        d[vec_index] = A[mat_index];
        vec_index++;
      }
      mat_index++;
    }
  }
}

/**
 * Set the diagonal of matrix `A` of size `m x n` with vector `d`.
 */
void mat_diag_set(real_t *A, const int m, const int n, const real_t *d) {
  assert(A != NULL);
  assert(m > 0);
  assert(n > 0);
  assert(d != NULL);

  int mat_index = 0;
  int vec_index = 0;

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (i == j) {
        A[mat_index] = d[vec_index];
        vec_index++;
      } else {
        A[mat_index] = 0.0;
      }
      mat_index++;
    }
  }
}

/**
 * Get upper triangular square matrix of `A` of size `m x m`, results are
 * outputted to `U`.
 */
void mat_triu(const real_t *A, const size_t m, real_t *U) {
  assert(A != NULL);
  assert(m > 0);
  assert(U != NULL);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < m; j++) {
      U[i * m + j] = (j >= i) ? A[i * m + j] : 0.0;
    }
  }
}

/**
 * Get lower triangular square matrix of `A` of size `m x m`, results are
 * outputted to `L`.
 */
void mat_tril(const real_t *A, const size_t m, real_t *L) {
  assert(A != NULL);
  assert(m > 0);
  assert(L != NULL);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < m; j++) {
      L[i * m + j] = (j <= i) ? A[i * m + j] : 0.0;
    }
  }
}

/**
 * Get the trace matrix of `A` of size `m x n`.
 */
real_t mat_trace(const real_t *A, const size_t m, const size_t n) {
  assert(A != NULL);
  assert(m > 0);
  assert(n > 0);

  real_t tr = 0.0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      tr += (i == j) ? A[i * n + j] : 0.0;
    }
  }
  return tr;
}

/**
 * Transpose of matrix `A` of size `m x n`, results are outputted to `A_t`.
 */
void mat_transpose(const real_t *A, size_t m, size_t n, real_t *A_t) {
  assert(A != NULL && A != A_t);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      mat_set(A_t, m, j, i, mat_val(A, n, i, j));
    }
  }
}

/**
 * Add two matrices `A` and `B` of size `m x n`, results are outputted to `C`.
 */
void mat_add(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n) {
  assert(A != NULL && B != NULL && C != NULL);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < (m * n); i++) {
    C[i] = A[i] + B[i];
  }
}

/**
 * Subtract two matrices `A` and `B` of size `m x n`, results are outputted to
 * matrix `C`.
 */
void mat_sub(const real_t *A, const real_t *B, real_t *C, size_t m, size_t n) {
  assert(A != NULL && B != NULL && C != NULL && B != C && A != C);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < (m * n); i++) {
    C[i] = A[i] - B[i];
  }
}

/**
 * Scale matrix `A` of size `m x n` inplace with `scale`.
 */
void mat_scale(real_t *A, const size_t m, const size_t n, const real_t scale) {
  assert(A != NULL);
  assert(m > 0 && n > 0);

  for (size_t i = 0; i < (m * n); i++) {
    A[i] = A[i] * scale;
  }
}

/**
 * Create new vector of length `n` in heap memory.
 * @returns Heap allocated vector
 */
real_t *vec_malloc(const size_t n) {
  assert(n > 0);
  return calloc(n, sizeof(real_t));
}

/**
 * Copy vector `src` of length `n` to `dest`.
 */
void vec_copy(const real_t *src, const size_t n, real_t *dest) {
  assert(src != NULL);
  assert(n > 0);
  assert(dest != NULL);

  for (size_t i = 0; i < n; i++) {
    dest[i] = src[i];
  }
}

/**
 * Check if vectors `x` and `y` of length `n` are equal.
 * @returns
 * - 1 for x == y
 * - 0 for x != y
 */
int vec_equals(const real_t *x, const real_t *y, const size_t n) {
  assert(x != NULL);
  assert(y != NULL);
  assert(n > 0);

  for (size_t i = 0; i < n; i++) {
    if (fltcmp(x[i], y[i]) != 0) {
      return 0;
    }
  }

  return 1;
}

/**
 * Sum two vectors `x` and `y` of length `n` to `z`.
 */
void vec_add(const real_t *x, const real_t *y, real_t *z, size_t n) {
  assert(x != NULL && y != NULL && z != NULL && x != y && x != z);
  assert(n > 0);

  for (size_t i = 0; i < n; i++) {
    z[i] = x[i] + y[i];
  }
}

/**
 * Subtract two vectors `x` and `y` of length `n` to `z`.
 */
void vec_sub(const real_t *x, const real_t *y, real_t *z, size_t n) {
  assert(x != NULL && y != NULL && z != NULL && x != y && x != z);
  assert(n > 0);

  for (size_t i = 0; i < n; i++) {
    z[i] = x[i] - y[i];
  }
}

/**
 * Scale a vector `x` of length `n` with `scale` in-place.
 */
void vec_scale(real_t *x, const size_t n, const real_t scale) {
  assert(x != NULL);
  assert(n > 0);

  for (size_t i = 0; i < n; i++) {
    x[i] = x[i] * scale;
  }
}

/**
 * Calculate vector norm of `x` of length `n`.
 * @returns Norm of vector x
 */
real_t vec_norm(const real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  real_t sum = 0.0;
  for (size_t i = 0; i < n; i++) {
    sum += x[i] * x[i];
  }
  return sqrt(sum);
}

/**
 * Normalize vector `x` of length `n` in place.
 */
void vec_normalize(real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  const real_t norm = vec_norm(x, n);
  for (size_t i = 0; i < n; i++) {
    x[i] = x[i] / norm;
  }
}

/**
 * Dot product of two matrices or vectors `A` and `B` of size `A_m x A_n` and
 * `B_m x B_n`. Results are written to `C`.
 */
void dot(const real_t *A,
         const size_t A_m,
         const size_t A_n,
         const real_t *B,
         const size_t B_m,
         const size_t B_n,
         real_t *C) {
  assert(A != NULL && B != NULL && A != C && B != C);
  assert(A_m > 0 && A_n > 0 && B_m > 0 && B_n > 0);
  assert(A_n == B_m);

  size_t m = A_m;
  size_t n = B_n;

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      for (size_t k = 0; k < A_n; k++) {
        C[(i * n) + j] += A[(i * A_n) + k] * B[(k * B_n) + j];
      }
    }
  }
}

/**
 * Create skew-symmetric matrix `A` from a 3x1 vector `x`.
 */
void skew(const real_t x[3], real_t A[3 * 3]) {
  assert(x != NULL);
  assert(A != NULL);

  /* First row */
  A[0] = 0.0;
  A[1] = -x[2];
  A[2] = x[1];

  /* Second row */
  A[3] = x[2];
  A[4] = 0.0;
  A[5] = -x[0];

  /* Third row */
  A[6] = -x[1];
  A[7] = x[0];
  A[8] = 0.0;
}

/**
 * Opposite of the skew-symmetric matrix
 */
void skew_inv(const real_t A[3 * 3], real_t x[3]) {
  assert(A != NULL);
  assert(x != NULL);

  const real_t A02 = A[2];
  const real_t A10 = A[3];
  const real_t A21 = A[7];

  x[0] = A21;
  x[1] = A02;
  x[2] = A10;
}

/**
 * Perform forward substitution with a lower triangular matrix `L`, column
 * vector `b` and solve for vector `y` of size `n`.
 */
void fwdsubs(const real_t *L, const real_t *b, real_t *y, const size_t n) {
  assert(L != NULL);
  assert(b != NULL);
  assert(y != NULL);
  assert(n > 0);

  for (size_t i = 0; i < n; i++) {
    real_t alpha = b[i];
    for (size_t j = 0; j < i; j++) {
      alpha -= L[i * n + j] * y[j];
    }
    y[i] = alpha / L[i * n + i];
  }
}

/**
 * Perform backward substitution with a upper triangular matrix `U`, column
 * vector `y` and solve for vector `x` of size `n`.
 */
void bwdsubs(const real_t *U, const real_t *y, real_t *x, const size_t n) {
  assert(U != NULL);
  assert(y != NULL);
  assert(x != NULL);
  assert(n > 0);

  for (int i = n - 1; i >= 0; i--) {
    real_t alpha = y[i];
    for (int j = i; j < (int) n; j++) {
      alpha -= U[i * n + j] * x[j];
    }
    x[i] = alpha / U[i * n + i];
  }
}

/**
 * Check analytical jacobian `jac` with name `jac_name` and compare it with a
 * numerically differentiated jacobian `fdiff` (finite diff). Where both
 * matrices are of size `m x n`, `tol` denotes the tolerance to consider both
 * `jac` and `fdiff` to be close enough. For debugging purposes use `verbose`
 * to show the matrices and differences.
 *
 * @returns
 * - 0 for success
 * - -1 for failure
 */
int check_jacobian(const char *jac_name,
                   const real_t *fdiff,
                   const real_t *jac,
                   const size_t m,
                   const size_t n,
                   const real_t tol,
                   const int verbose) {
  assert(jac_name != NULL);
  assert(fdiff != NULL);
  assert(jac != NULL);
  assert(m > 0);
  assert(n > 0);
  assert(tol > 0);

  int retval = 0;
  int ok = 1;
  real_t *delta = mat_malloc(m, n);
  mat_sub(fdiff, jac, delta, m, n);

  /* Check if any of the values are beyond the tol */
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      if (fabs(mat_val(delta, n, i, j)) >= tol) {
        ok = 0;
      }
    }
  }

  /* Print result */
  if (ok == 0) {
    if (verbose) {
      LOG_ERROR("Bad jacobian [%s]!\n", jac_name);
      print_matrix("analytical jac", jac, m, n);
      print_matrix("num diff jac", fdiff, m, n);
      print_matrix("difference matrix", delta, m, n);
    }
    retval = -1;
  } else {
    if (verbose) {
      printf("Check [%s] ok!\n", jac_name);
    }
    retval = 0;
  }

  /* Clean up */
  free(delta);

  return retval;
}

#ifdef USE_CBLAS
/**
 * Dot product of two matrices or vectors `A` and `B` of size `A_m x A_n` and
 * `B_m x B_n` using CBLAS. Results are written to `C`.
 */
void cblas_dot(const real_t *A,
               const size_t A_m,
               const size_t A_n,
               const real_t *B,
               const size_t B_m,
               const size_t B_n,
               real_t *C) {
  UNUSED(B_m);
  assert(A != NULL && B != NULL && C != NULL);
  assert(A_m > 0 && A_n > 0 && B_m > 0 && B_n > 0);
  assert(A_n == B_m);

#if PRECISION == 1
  cblas_sgemm(CblasRowMajor, /* Matrix data arrangement */
              CblasNoTrans,  /* Transpose A */
              CblasNoTrans,  /* Transpose B */
              A_m,           /* Number of rows in A and C */
              B_n,           /* Number of cols in B and C */
              A_n,           /* Number of cols in A */
              1.0,           /* Scaling factor for the product of A and B */
              A,             /* Matrix A */
              A_n,           /* First dimension of A */
              B,             /* Matrix B */
              B_n,           /* First dimension of B */
              0.0,           /* Scale factor for C */
              C,             /* Output */
              B_n);          /* First dimension of C */
#elif PRECISION == 2
  cblas_dgemm(CblasRowMajor, /* Matrix data arrangement */
              CblasNoTrans,  /* Transpose A */
              CblasNoTrans,  /* Transpose B */
              A_m,           /* Number of rows in A and C */
              B_n,           /* Number of cols in B and C */
              A_n,           /* Number of cols in A */
              1.0,           /* Scaling factor for the product of A and B */
              A,             /* Matrix A */
              A_n,           /* First dimension of A */
              B,             /* Matrix B */
              B_n,           /* First dimension of B */
              0.0,           /* Scale factor for C */
              C,             /* Output */
              B_n);          /* First dimension of C */
#endif
}
#endif

/******************************************************************************
 * SVD
 ******************************************************************************/

#ifdef USE_LAPACK
void lapack_svd(real_t *A, int m, int n, real_t **S, real_t **U, real_t **V_t) {
  const int lda = n;
  const int diag_size = (m < n) ? m : n;
  *S = malloc(sizeof(real_t) * diag_size);
  *U = malloc(sizeof(real_t) * m * m);
  *V_t = malloc(sizeof(real_t) * n * n);
#if PRECISION == 1
  LAPACKE_sgesdd(LAPACK_ROW_MAJOR, 'S', m, n, A, lda, *S, *U, m, *V_t, n);
#elif PRECISION == 2
  LAPACKE_dgesdd(LAPACK_ROW_MAJOR, 'S', m, n, A, lda, *S, *U, m, *V_t, n);
#endif
}
#endif

/******************************************************************************
 * CHOL
 ******************************************************************************/

/**
 * Cholesky decomposition. Takes a `m x m` matrix `A` and decomposes it into a
 * lower and upper triangular matrix `L` and `U` with Cholesky decomposition.
 * This function only returns the `L` triangular matrix.
 */
void chol(const real_t *A, const size_t m, real_t *L) {
  assert(A != NULL);
  assert(m > 0);
  assert(L != NULL);

  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < (i + 1); j++) {

      if (i == j) {
        real_t s = 0.0;
        for (size_t k = 0; k < j; k++) {
          s += L[j * m + k] * L[j * m + k];
        }
        L[i * m + j] = sqrt(A[i * m + i] - s);

      } else {
        real_t s = 0.0;
        for (size_t k = 0; k < j; k++) {
          s += L[i * m + k] * L[j * m + k];
        }
        L[i * m + j] = (1.0 / L[j * m + j] * (A[i * m + j] - s));
      }
    }
  }
}

/**
 * Solve `Ax = b` using Cholesky decomposition, where `A` is a square matrix,
 * `b` is a vector and `x` is the solution vector of size `n`.
 */
void chol_solve(const real_t *A, const real_t *b, real_t *x, const size_t n) {
  assert(A != NULL);
  assert(b != NULL);
  assert(x != NULL);
  assert(n > 0);

  /* Allocate memory */
  real_t *L = calloc(n * n, sizeof(real_t));
  real_t *Lt = calloc(n * n, sizeof(real_t));
  real_t *y = calloc(n, sizeof(real_t));

  /* Cholesky decomposition */
  chol(A, n, L);
  mat_transpose(L, n, n, Lt);

  /* Forward substitution */
  /* Ax = b -> LLt x = b. */
  /* Let y = Lt x, L y = b (Solve for y) */
  for (int i = 0; i < (int) n; i++) {
    real_t alpha = b[i];

    if (fltcmp(L[i * n + i], 0.0) == 0) {
      y[i] = 0.0;

    } else {
      for (int j = 0; j < i; j++) {
        alpha -= L[i * n + j] * y[j];
      }
      y[i] = alpha / L[i * n + i];
    }
  }

  /* Backward substitution */
  /* Now we have y, we can go back to (Lt x = y) and solve for x */
  for (int i = n - 1; i >= 0; i--) {
    real_t alpha = y[i];

    if (fltcmp(Lt[i * n + i], 0.0) == 0) {
      x[i] = 0.0;

    } else {
      for (int j = i; j < (int) n; j++) {
        alpha -= Lt[i * n + j] * x[j];
      }
      x[i] = alpha / Lt[i * n + i];
    }
  }

  /* Clean up */
  free(y);
  free(L);
  free(Lt);
}

#ifdef USE_LAPACK
/**
 * Solve Ax = b using LAPACK's implementation of Cholesky decomposition, where
 * `A` is a square matrix, `b` is a vector and `x` is the solution vector of
 * size `n`.
 */
void lapack_chol_solve(const real_t *A,
                       const real_t *b,
                       real_t *x,
                       const size_t m) {
  assert(A != NULL);
  assert(b != NULL);
  assert(x != NULL);
  assert(m > 0);

  /* Cholesky Decomposition */
  int info = 0;
  int lda = m;
  int n = m;
  char uplo = 'L';
  real_t *a = mat_malloc(m, m);
  mat_copy(A, m, m, a);
#if PRECISION == 1
  spotrf_(&uplo, &n, a, &lda, &info);
#elif PRECISION == 2
  dpotrf_(&uplo, &n, a, &lda, &info);
#endif
  if (info != 0) {
    fprintf(stderr, "Failed to decompose A using Cholesky Decomposition!\n");
  }

  /* Solve Ax = b using Cholesky decomposed A from above */
  vec_copy(b, m, x);
  int nhrs = 1;
  int ldb = m;
#if PRECISION == 1
  spotrs_(&uplo, &n, &nhrs, a, &lda, x, &ldb, &info);
#elif PRECISION == 2
  dpotrs_(&uplo, &n, &nhrs, a, &lda, x, &ldb, &info);
#endif
  if (info != 0) {
    fprintf(stderr, "Failed to solve Ax = b!\n");
  }

  free(a);
}
#endif

/******************************************************************************
 * Lie
 ******************************************************************************/

/**
 * Exponential Map
 */
void lie_Exp(const real_t phi[3], real_t C[3 * 3]) {
  assert(phi != NULL);
  assert(C != NULL);

  real_t phi_norm = vec_norm(phi, 3);
  real_t phi_skew[3 * 3] = {0};
  real_t phi_skew_sq[3 * 3] = {0};

  skew(phi, phi_skew);
  dot(phi_skew, 3, 3, phi_skew, 3, 3, phi_skew_sq);

  if (phi_norm < 1e-3) {
    /* C = eye(3) + skew(phi); */
    eye(C, 3, 3);
    mat_add(C, phi_skew, C, 3, 3);
  } else {
    /* C = eye(3); */
    /* C += (sin(phi_norm) / phi_norm) * phi_skew; */
    /* C += ((1 - cos(phi_norm)) / phi_norm ^ 2) * phi_skew_sq; */
    real_t A[3 * 3] = {0};
    mat_copy(phi_skew, 3, 3, A);
    mat_scale(A, 3, 3, (sin(phi_norm) / phi_norm));

    real_t B[3 * 3] = {0};
    mat_copy(phi_skew_sq, 3, 3, B);
    mat_scale(B, 3, 3, (1.0 - cos(phi_norm)) / (phi_norm * phi_norm));

    eye(C, 3, 3);
    mat_add(C, A, C, 3, 3);
    mat_add(C, B, C, 3, 3);
  }
}

/**
 * Logarithmic Map
 */
void lie_Log(const real_t C[3 * 3], real_t rvec[3]) {
  assert(C != NULL);
  assert(rvec != NULL);

  /**
   * phi = acos((trace(C) - 1) / 2);
   * vec = skew_inv(C - C') / (2 * sin(phi));
   * rvec = phi * vec;
   */
  const real_t tr = C[0] + C[4] + C[8];
  const real_t phi = acos((tr - 1.0) / 2.0);

  real_t C_t[3 * 3] = {0};
  real_t dC[3 * 3] = {0};
  mat_transpose(C, 3, 3, C_t);
  mat_sub(C, C_t, dC, 3, 3);
  real_t u[3] = {0};
  skew_inv(dC, u);
  const real_t s = 1.0 / (2 * sin(phi));
  const real_t vec[3] = {s * u[0], s * u[1], s * u[2]};

  rvec[0] = phi * vec[0];
  rvec[1] = phi * vec[1];
  rvec[2] = phi * vec[2];
}

/******************************************************************************
 * TRANSFORMS
 ******************************************************************************/

/**
 * Form 4x4 homogeneous transformation matrix `T` from a 7x1 pose vector
 * `params`.
 *
 *    pose = (translation, rotation)
 *    pose = (rx, ry, rz, qx, qy, qz, qw)
 */
void tf(const real_t params[7], real_t T[4 * 4]) {
  assert(params != NULL);
  assert(T != NULL);

  const real_t r[3] = {params[0], params[1], params[2]};
  const real_t q[4] = {params[6], params[3], params[4], params[5]};

  real_t C[3 * 3] = {0};
  quat2rot(q, C);

  T[0] = C[0];
  T[1] = C[1];
  T[2] = C[2];
  T[3] = r[0];

  T[4] = C[3];
  T[5] = C[4];
  T[6] = C[5];
  T[7] = r[1];

  T[8] = C[6];
  T[9] = C[7];
  T[10] = C[8];
  T[11] = r[2];

  T[12] = 0.0;
  T[13] = 0.0;
  T[14] = 0.0;
  T[15] = 1.0;
}

/**
 * Form 7x1 pose parameter vector `params` from 4x4 homogeneous transformation
 * matrix `T`.
 */
void tf_vector(const real_t T[4 * 4], real_t params[7]) {
  assert(T != NULL);
  assert(params != NULL);

  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);

  real_t r[3] = {0};
  tf_trans_get(T, r);

  real_t q[4] = {0};
  rot2quat(C, q);

  params[0] = r[0];
  params[1] = r[1];
  params[2] = r[2];

  params[3] = q[1];
  params[4] = q[2];
  params[5] = q[3];
  params[6] = q[0];
}

/**
 * Decompose transform `T` into the rotation `C` and translation `r`
 * components.
 */
void tf_decompose(const real_t T[4 * 4], real_t C[3 * 3], real_t r[3]) {
  assert(T != NULL);
  assert(C != NULL);
  assert(r != NULL);

  /* clang-format off */
  C[0] = T[0]; C[1] = T[1]; C[2] = T[2];  r[0] = T[3];
  C[3] = T[4]; C[4] = T[5]; C[5] = T[6];  r[1] = T[7];
  C[6] = T[8]; C[7] = T[9]; C[8] = T[10]; r[1] = T[11];
  /* clang-format on */
}

/**
 * Set the rotational component in the 4x4 transformation matrix `T` using a
 * 3x3 rotation matrix `C`.
 */
void tf_rot_set(real_t T[4 * 4], const real_t C[3 * 3]) {
  assert(T != NULL);
  assert(C != NULL);
  assert(T != C);

  T[0] = C[0];
  T[1] = C[1];
  T[2] = C[2];

  T[4] = C[3];
  T[5] = C[4];
  T[6] = C[5];

  T[8] = C[6];
  T[9] = C[7];
  T[10] = C[8];
}

/**
 * Get the rotation matrix `C` from the 4x4 transformation matrix `T`.
 */
void tf_rot_get(const real_t T[4 * 4], real_t C[3 * 3]) {
  assert(T != NULL);
  assert(C != NULL);
  assert(T != C);

  C[0] = T[0];
  C[1] = T[1];
  C[2] = T[2];

  C[3] = T[4];
  C[4] = T[5];
  C[5] = T[6];

  C[6] = T[8];
  C[7] = T[9];
  C[8] = T[10];
}

/**
 * Set the rotation component in the 4x4 transformation matrix `T` using a 4x1
 * quaternion `q`.
 */
void tf_quat_set(real_t T[4 * 4], const real_t q[4]) {
  assert(T != NULL);
  assert(q != NULL);
  assert(T != q);

  real_t C[3 * 3] = {0};
  quat2rot(q, C);
  tf_rot_set(T, C);
}

/**
 * Get the quaternion `q` from the 4x4 transformation matrix `T`.
 */
void tf_quat_get(const real_t T[4 * 4], real_t q[4]) {
  assert(T != NULL);
  assert(q != NULL);
  assert(T != q);

  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);
  rot2quat(C, q);
}

/**
 * Set the rotational component in the 4x4 transformation matrix `T` using a
 * 3x1 euler angle vector `euler`.
 */
void tf_euler_set(real_t T[4 * 4], const real_t ypr[3]) {
  assert(T != NULL);
  assert(ypr != NULL);
  assert(T != ypr);

  real_t C[3 * 3] = {0};
  euler321(ypr, C);
  tf_rot_set(T, C);
}

/**
 * Get the rotational component in the 4x4 transformation matrix `T` in the
 * form of a 3x1 euler angle vector `ypr`.
 */
void tf_euler_get(const real_t T[4 * 4], real_t ypr[3]) {
  assert(T != NULL);
  assert(ypr != NULL);
  assert(T != ypr);

  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);
  rot2euler(C, ypr);
}

/**
 * Set the translational component in the 4x4 transformation matrix `T` using
 * a 3x1 translation vector `r`.
 */
void tf_trans_set(real_t T[4 * 4], const real_t r[3]) {
  assert(T != NULL);
  assert(r != NULL);
  assert(T != r);

  T[3] = r[0];
  T[7] = r[1];
  T[11] = r[2];
}

/**
 * Get the translational vector `r` from the 4x4 transformation matrix `T`.
 */
void tf_trans_get(const real_t T[4 * 4], real_t r[3]) {
  assert(T != NULL);
  assert(r != NULL);
  assert(T != r);

  r[0] = T[3];
  r[1] = T[7];
  r[2] = T[11];
}

/**
 * Invert the 4x4 homogeneous transformation matrix `T` where results are
 * written to `T_inv`.
 */
void tf_inv(const real_t T[4 * 4], real_t T_inv[4 * 4]) {
  assert(T != NULL);
  assert(T_inv != NULL);
  assert(T != T_inv);

  /* Get original rotation and translation component */
  real_t C[3 * 3] = {0};
  real_t r[3] = {0};
  tf_rot_get(T, C);
  tf_trans_get(T, r);

  /* Invert rotation component */
  real_t C_inv[3 * 3] = {0};
  mat_transpose(C, 3, 3, C_inv);

  /* Set rotation component */
  tf_rot_set(T_inv, C_inv);

  /* Set translation component */
  real_t r_inv[3] = {0};
  mat_scale(C_inv, 3, 3, -1.0);
  dot(C_inv, 3, 3, r, 3, 1, r_inv);
  tf_trans_set(T_inv, r_inv);

  /* Make sure the last element is 1 */
  T_inv[15] = 1.0;
}

/**
 * Transform 3x1 point `p` using 4x4 homogeneous transformation matrix `T` and
 * output to 3x1 `retval`.
 */
void tf_point(const real_t T[4 * 4], const real_t p[3], real_t retval[3]) {
  assert(T != NULL);
  assert(p != NULL);
  assert(retval != NULL);
  assert(p != retval);

  const real_t hp_a[4] = {p[0], p[1], p[2], 1.0};
  real_t hp_b[4] = {0.0, 0.0, 0.0, 0.0};
  dot(T, 4, 4, hp_a, 4, 1, hp_b);

  retval[0] = hp_b[0];
  retval[1] = hp_b[1];
  retval[2] = hp_b[2];
}

/**
 * Transform 4x1 homogeneous point `hp` using 4x4 homogeneous transformation
 * matrix `T` and output to 4x1 `retval`.
 */
void tf_hpoint(const real_t T[4 * 4], const real_t hp[4], real_t retval[4]) {
  assert(T != NULL);
  assert(hp != retval);
  assert(retval != NULL);
  dot(T, 4, 4, hp, 4, 1, retval);
}

/**
 * Perturb the `i`-th rotational component of a 4x4 homogeneous transformation
 * matrix `T` with `step_size`.
 */
void tf_perturb_rot(real_t T[4 * 4], const real_t step_size, const int i) {
  assert(T != NULL);
  assert(i >= 0 && i <= 2);

  /* Build perturb drvec */
  real_t drvec[3] = {0};
  drvec[i] = step_size;

  /* Decompose transform to rotation and translation */
  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);

  /* Perturb rotation */
  real_t C_rvec[3 * 3] = {0};
  real_t C_diff[3 * 3] = {0};
  rvec2rot(drvec, 1e-8, C_rvec);
  dot(C, 3, 3, C_rvec, 3, 3, C_diff);
  tf_rot_set(T, C_diff);
}

/**
 * Perturb the `i`-th translation component of a 4x4 homogeneous
 * transformation matrix with `step_size`.
 */
void tf_perturb_trans(real_t T[4 * 4], const real_t step_size, const int i) {
  assert(T != NULL);
  assert(i >= 0 && i <= 2);

  /* Build perturb dr */
  real_t dr[3] = {0};
  dr[i] = step_size;

  /* Decompose transform get translation */
  real_t r[3] = {0};
  tf_trans_get(T, r);

  /* Perturb translation */
  const real_t r_diff[3] = {r[0] + dr[0], r[1] + dr[1], r[2] + dr[2]};
  tf_trans_set(T, r_diff);
}

/**
 * Print pose vector
 */
void print_pose_vector(const char *prefix, const real_t pose[7]) {
  if (prefix) {
    printf("%s: ", prefix);
  }

  for (int i = 0; i < 7; i++) {
    printf("%.2f", pose[i]);
    if ((i + 1) < 7) {
      printf(", ");
    }
  }
  printf("\n");
}

/**
 * Convert rotation vector `rvec` to 3x3 rotation matrix `R`, where `eps` is
 * the tolerance to determine if the rotation is too small.
 */
void rvec2rot(const real_t *rvec, const real_t eps, real_t *R) {
  assert(rvec != NULL);
  assert(eps > 0);
  assert(R != NULL);

  /* Magnitude of rvec */
  const real_t theta = sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1]);
  // ^ basically norm(rvec), but faster

  /* Check if rotation is too small */
  if (theta < eps) {
    R[0] = 1.0;
    R[1] = -rvec[2];
    R[2] = rvec[1];

    R[3] = rvec[2];
    R[4] = 1.0;
    R[5] = -rvec[0];

    R[6] = -rvec[1];
    R[7] = rvec[0], R[8] = 1.0;
    return;
  }

  /* Convert rvec to rotation matrix */
  real_t rvec_normed[3] = {rvec[0], rvec[1], rvec[2]};
  vec_scale(rvec_normed, 3, 1 / theta);
  const real_t x = rvec_normed[0];
  const real_t y = rvec_normed[1];
  const real_t z = rvec_normed[2];

  const real_t c = cos(theta);
  const real_t s = sin(theta);
  const real_t C = 1 - c;

  const real_t xs = x * s;
  const real_t ys = y * s;
  const real_t zs = z * s;

  const real_t xC = x * C;
  const real_t yC = y * C;
  const real_t zC = z * C;

  const real_t xyC = x * yC;
  const real_t yzC = y * zC;
  const real_t zxC = z * xC;

  R[0] = x * xC + c;
  R[1] = xyC - zs;
  R[2] = zxC + ys;

  R[3] = xyC + zs;
  R[4] = y * yC + c;
  R[5] = yzC - xs;

  R[6] = zxC - ys;
  R[7] = yzC + xs;
  R[8] = z * zC + c;
}

/**
 * Convert Euler angles `ypr` (yaw, pitch, roll) in degrees to a 3x3 rotation
 * matrix `C`.
 */
void euler321(const real_t ypr[3], real_t C[3 * 3]) {
  assert(ypr != NULL);
  assert(C != NULL);

  const real_t psi = ypr[0];
  const real_t theta = ypr[1];
  const real_t phi = ypr[2];

  /* 1st row */
  C[0] = cos(psi) * cos(theta);
  C[1] = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
  C[2] = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
  /* 2nd row */
  C[3] = sin(psi) * cos(theta);
  C[4] = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
  C[5] = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
  /* 3rd row */
  C[6] = -sin(theta);
  C[7] = cos(theta) * sin(phi);
  C[8] = cos(theta) * cos(phi);
}

/**
 * Convert Euler angles `ypr` in radians to a Hamiltonian Quaternion.
 */
void euler2quat(const real_t ypr[3], real_t q[4]) {
  const real_t psi = ypr[0];
  const real_t theta = ypr[1];
  const real_t phi = ypr[2];

  const real_t c_phi = cos(phi / 2.0);
  const real_t c_theta = cos(theta / 2.0);
  const real_t c_psi = cos(psi / 2.0);
  const real_t s_phi = sin(phi / 2.0);
  const real_t s_theta = sin(theta / 2.0);
  const real_t s_psi = sin(psi / 2.0);

  const real_t qx = s_phi * c_theta * c_psi - c_phi * s_theta * s_psi;
  const real_t qy = c_phi * s_theta * c_psi + s_phi * c_theta * s_psi;
  const real_t qz = c_phi * c_theta * s_psi - s_phi * s_theta * c_psi;
  const real_t qw = c_phi * c_theta * c_psi + s_phi * s_theta * s_psi;

  const real_t mag = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  q[0] = qw / mag;
  q[1] = qx / mag;
  q[2] = qy / mag;
  q[3] = qz / mag;
}

/**
 * Convert 3x3 rotation matrix `C` to Quaternion `q`.
 */
void rot2quat(const real_t C[3 * 3], real_t q[4]) {
  assert(C != NULL);
  assert(q != NULL);

  const real_t C00 = C[0];
  const real_t C01 = C[1];
  const real_t C02 = C[2];
  const real_t C10 = C[3];
  const real_t C11 = C[4];
  const real_t C12 = C[5];
  const real_t C20 = C[6];
  const real_t C21 = C[7];
  const real_t C22 = C[8];

  const real_t tr = C00 + C11 + C22;
  real_t S = 0.0f;
  real_t qw = 0.0f;
  real_t qx = 0.0f;
  real_t qy = 0.0f;
  real_t qz = 0.0f;

  if (tr > 0) {
    S = sqrt(tr + 1.0) * 2; // S=4*qw
    qw = 0.25 * S;
    qx = (C21 - C12) / S;
    qy = (C02 - C20) / S;
    qz = (C10 - C01) / S;
  } else if ((C00 > C11) && (C[0] > C22)) {
    S = sqrt(1.0 + C[0] - C11 - C22) * 2; // S=4*qx
    qw = (C21 - C12) / S;
    qx = 0.25 * S;
    qy = (C01 + C10) / S;
    qz = (C02 + C20) / S;
  } else if (C11 > C22) {
    S = sqrt(1.0 + C11 - C[0] - C22) * 2; // S=4*qy
    qw = (C02 - C20) / S;
    qx = (C01 + C10) / S;
    qy = 0.25 * S;
    qz = (C12 + C21) / S;
  } else {
    S = sqrt(1.0 + C22 - C[0] - C11) * 2; // S=4*qz
    qw = (C10 - C01) / S;
    qx = (C02 + C20) / S;
    qy = (C12 + C21) / S;
    qz = 0.25 * S;
  }

  q[0] = qw;
  q[1] = qx;
  q[2] = qy;
  q[3] = qz;
}

/**
 * Convert 3 x 3 rotation matrix `C` to euler angles `euler`.
 */
void rot2euler(const real_t C[3 * 3], real_t ypr[3]) {
  assert(C != NULL);
  assert(ypr != NULL);

  real_t q[4] = {0};
  rot2quat(C, q);
  quat2euler(q, ypr);
}

/**
 * Convert Quaternion `q` to Euler angles 3x1 vector `euler`.
 */
void quat2euler(const real_t q[4], real_t ypr[3]) {
  assert(q != NULL);
  assert(ypr != NULL);

  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  const real_t qw2 = qw * qw;
  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;

  const real_t t1 = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2));
  const real_t t2 = asin(2 * (qy * qw - qx * qz));
  const real_t t3 = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));

  ypr[0] = t3;
  ypr[1] = t2;
  ypr[2] = t1;
}

real_t quat_norm(const real_t q[4]) {
  return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

void quat_normalize(real_t q[4]) {
  const real_t n = quat_norm(q);
  q[0] = q[0] / n;
  q[1] = q[1] / n;
  q[2] = q[2] / n;
  q[3] = q[3] / n;
}

/**
 * Convert Quaternion `q` to 3x3 rotation matrix `C`.
 */
void quat2rot(const real_t q[4], real_t C[3 * 3]) {
  assert(q != NULL);
  assert(C != NULL);

  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;
  const real_t qw2 = qw * qw;

  /* Homogeneous form */
  /* -- 1st row */
  C[0] = qw2 + qx2 - qy2 - qz2;
  C[1] = 2 * (qx * qy - qw * qz);
  C[2] = 2 * (qx * qz + qw * qy);
  /* -- 2nd row */
  C[3] = 2 * (qx * qy + qw * qz);
  C[4] = qw2 - qx2 + qy2 - qz2;
  C[5] = 2 * (qy * qz - qw * qx);
  /* -- 3rd row */
  C[6] = 2 * (qx * qz - qw * qy);
  C[7] = 2 * (qy * qz + qw * qx);
  C[8] = qw2 - qx2 - qy2 + qz2;
}

/**
 * Inverse Quaternion `q`.
 */
void quat_inv(const real_t q[4], real_t q_inv[4]) {
  q_inv[0] = q[0];
  q_inv[1] = -q[1];
  q_inv[2] = -q[2];
  q_inv[3] = -q[3];
}

/**
 * Form Quaternion left multiplication matrix.
 */
void quat_left(const real_t q[4], real_t left[4 * 4]) {
  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  /* clang-format off */
  left[0]  = qw; left[1]  = -qx; left[2]  = -qy; left[3]  = -qz;
  left[4]  = qx; left[5]  =  qw; left[6]  = -qz; left[7]  =  qy;
  left[8]  = qy; left[9]  =  qz; left[10] =  qw; left[11] = -qx;
  left[12] = qz; left[13] = -qy; left[14] =  qx; left[15] =  qw;
  /* clang-format on */
}

/**
 * Form Quaternion right multiplication matrix.
 */
void quat_right(const real_t q[4], real_t right[4 * 4]) {
  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  /* clang-format off */
  right[0]  = qw; right[1]  = -qx; right[2]  = -qy; right[3]  = -qz;
  right[4]  = qx; right[5]  =  qw; right[6]  =  qz; right[7]  = -qy;
  right[8]  = qy; right[9]  = -qz; right[10] =  qw; right[11] =  qx;
  right[12] = qz; right[13] =  qy; right[14] = -qx; right[15] =  qw;
  /* clang-format on */
}

/**
 * Quaternion left-multiply `p` with `q`, results are outputted to `r`.
 */
void quat_lmul(const real_t p[4], const real_t q[4], real_t r[4]) {
  assert(p != NULL);
  assert(q != NULL);
  assert(r != NULL);

  const real_t pw = p[0];
  const real_t px = p[1];
  const real_t py = p[2];
  const real_t pz = p[3];

  /* clang-format off */
  const real_t lprod[4*4] = {
    pw, -px, -py, -pz,
    px, pw, -pz, py,
    py, pz, pw, -px,
    pz, -py, px, pw
  };
  /* clang-format on */

  cblas_dot(lprod, 4, 4, q, 4, 1, r);
}

/**
 * Quaternion right-multiply `p` with `q`, results are outputted to `r`.
 */
void quat_rmul(const real_t p[4], const real_t q[4], real_t r[4]) {
  assert(p != NULL);
  assert(q != NULL);
  assert(r != NULL);

  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  /* clang-format off */
  const real_t rprod[4*4] = {
    qw, -qx, -qy, -qz,
    qx, qw, qz, -qy,
    qy, -qz, qw, qx,
    qz, qy, -qx, qw
  };
  /* clang-format on */

  dot(rprod, 4, 4, p, 4, 1, r);
}

/**
 * Quaternion multiply `p` with `q`, results are outputted to `r`.
 */
void quat_mul(const real_t p[4], const real_t q[4], real_t r[4]) {
  assert(p != NULL);
  assert(q != NULL);
  assert(r != NULL);
  quat_lmul(p, q, r);
}

/**
 * Form delta quaternion `dq` from a small rotation vector `dalpha`.
 */
void quat_delta(const real_t dalpha[3], real_t dq[4]) {
  assert(dalpha != NULL);
  assert(dq != NULL);

  const real_t half_norm = 0.5 * vec_norm(dalpha, 3);
  const real_t k = sinc(half_norm) * 0.5;
  const real_t vector[3] = {k * dalpha[0], k * dalpha[1], k * dalpha[2]};
  real_t scalar = cos(half_norm);

  dq[0] = scalar;
  dq[1] = vector[0];
  dq[2] = vector[1];
  dq[3] = vector[2];
}

/**
 * Perturb quaternion
 */
void quat_perturb(real_t q[4], const int i, const real_t h) {
  assert(i >= 0 && i <= 2);

  /* Form small pertubation quaternion dq */
  real_t dalpha[3] = {0};
  real_t dq[4] = {0};
  dalpha[i] = h;
  quat_delta(dalpha, dq);

  /* Perturb quaternion */
  real_t q_[4] = {q[0], q[1], q[2], q[3]};
  quat_mul(q_, dq, q);
  quat_normalize(q);
}

/*****************************************************************************
 * CV
 *****************************************************************************/

// IMAGE ///////////////////////////////////////////////////////////////////////

/**
 * Setup image `img` with `width`, `height` and `data`.
 */
void image_setup(image_t *img,
                 const int width,
                 const int height,
                 uint8_t *data) {
  assert(img != NULL);
  img->width = width;
  img->height = height;
  img->data = data;
}

/**
 * Load image at `file_path`.
 * @returns Heap allocated image
 */
image_t *image_load(const char *file_path) {
  assert(file_path != NULL);

  int img_w = 0;
  int img_h = 0;
  int img_c = 0;
  stbi_set_flip_vertically_on_load(1);
  uint8_t *data = stbi_load(file_path, &img_w, &img_h, &img_c, 0);
  if (!data) {
    FATAL("Failed to load image file: [%s]", file_path);
  }

  image_t *img = malloc(sizeof(image_t));
  img->width = img_w;
  img->height = img_h;
  img->channels = img_c;
  img->data = data;
  return img;
}

/**
 * Print image properties.
 */
void image_print_properties(const image_t *img) {
  assert(img != NULL);
  printf("img.width: %d\n", img->width);
  printf("img.height: %d\n", img->height);
  printf("img.channels: %d\n", img->channels);
}

/**
 * Free image.
 */
void image_free(image_t *img) {
  assert(img != NULL);
  free(img->data);
  free(img);
}

// GEOMETRY ////////////////////////////////////////////////////////////////////

/**
 * Triangulate a single 3D point `p` observed by two different camera frames
 * represented by two 3x4 camera projection matrices `P_i` and `P_j`, and the
 * 2D image point correspondance `z_i` and `z_j`.
 */
void linear_triangulation(const real_t P_i[3 * 4],
                          const real_t P_j[3 * 4],
                          const real_t z_i[2],
                          const real_t z_j[2],
                          real_t p[3]) {
  /* Form A matrix */
  real_t A[4 * 4] = {0};
  /* -- ROW 1 */
  A[0] = -P_i[4] + P_i[8] * z_i[1];
  A[1] = -P_i[5] + P_i[9] * z_i[1];
  A[2] = P_i[10] * z_i[1] - P_i[6];
  A[3] = P_i[11] * z_i[1] - P_i[7];
  /* -- ROW 2 */
  A[4] = -P_i[0] + P_i[8] * z_i[0];
  A[5] = -P_i[1] + P_i[9] * z_i[0];
  A[6] = P_i[10] * z_i[0] - P_i[2];
  A[7] = P_i[11] * z_i[0] - P_i[3];
  /* -- ROW 3 */
  A[8] = -P_j[4] + P_j[8] * z_j[1];
  A[9] = -P_j[5] + P_j[9] * z_j[1];
  A[10] = P_j[10] * z_j[1] - P_j[6];
  A[11] = P_j[11] * z_j[1] - P_j[7];
  /* -- ROW 4 */
  A[12] = -P_j[0] + P_j[8] * z_j[0];
  A[13] = -P_j[1] + P_j[9] * z_j[0];
  A[14] = P_j[10] * z_j[0] - P_j[2];
  A[15] = P_j[11] * z_j[0] - P_j[3];

  /* Form A_t */
  real_t A_t[4 * 4] = {0};
  mat_transpose(A, 4, 4, A_t);

  /* SVD */
  real_t A2[4 * 4] = {0};
  dot(A_t, 4, 4, A, 4, 4, A2);

  real_t *S = NULL;
  real_t *U = NULL;
  real_t *V_t = NULL;
  lapack_svd(A2, 4, 4, &S, &U, &V_t);

  /* Get last row of V_t and normalize the scale to obtain the 3D point */
  const real_t x = V_t[12];
  const real_t y = V_t[13];
  const real_t z = V_t[14];
  const real_t w = V_t[15];
  p[0] = x / w;
  p[1] = y / w;
  p[2] = z / w;

  /* Clean up */
  free(S);
  free(U);
  free(V_t);
}

// RADTAN //////////////////////////////////////////////////////////////////////

/**
 * Distort 2x1 point `p` using Radial-Tangential distortion, where the
 * distortion params are stored in `params` (k1, k2, p1, p2), results are
 * written to 2x1 vector `p_d`.
 */
void radtan4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(p_d != NULL);

  /* Distortion parameters */
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t p1 = params[2];
  const real_t p2 = params[3];

  /* Point */
  const real_t x = p[0];
  const real_t y = p[1];

  /* Apply radial distortion */
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;
  const real_t radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  const real_t x_dash = x * radial_factor;
  const real_t y_dash = y * radial_factor;

  /* Apply tangential distortion */
  const real_t xy = x * y;
  const real_t x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  const real_t y_ddash = y_dash + (p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy);

  /* Distorted point */
  p_d[0] = x_ddash;
  p_d[1] = y_ddash;
}

/**
 * Form Radial-Tangential point jacobian, using distortion `params` (k1, k2,
 * p1, p2), 2x1 image point `p`, the jacobian is written to 2x2 `J_point`.
 */
void radtan4_point_jacobian(const real_t params[4],
                            const real_t p[2],
                            real_t J_point[2 * 2]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(J_point != NULL);

  /* Distortion parameters */
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t p1 = params[2];
  const real_t p2 = params[3];

  /* Point */
  const real_t x = p[0];
  const real_t y = p[1];

  /* Apply radial distortion */
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  /* Point Jacobian is 2x2 */
  J_point[0] = k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x;
  J_point[0] += x * (2 * k1 * x + 4 * k2 * x * r2) + 1;
  J_point[1] = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
  J_point[2] = J_point[1];
  J_point[3] = k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x;
  J_point[3] += y * (2 * k1 * y + 4 * k2 * y * r2) + 1;
}

/**
 * Form Radial-Tangential parameter jacobian, using distortion `params` (k1,
 * k2, p1, p2), 2x1 image point `p`, the jacobian is written to 2x4 `J_param`.
 */
void radtan4_params_jacobian(const real_t params[4],
                             const real_t p[2],
                             real_t J_param[2 * 4]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(J_param != NULL);
  UNUSED(params);

  /* Point */
  const real_t x = p[0];
  const real_t y = p[1];

  /* Setup */
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t xy = x * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  /* Param Jacobian is 2x4 */
  J_param[0] = x * r2;
  J_param[1] = x * r4;
  J_param[2] = 2 * xy;
  J_param[3] = 3 * x2 + y2;

  J_param[4] = y * r2;
  J_param[5] = y * r4;
  J_param[6] = x2 + 3 * y2;
  J_param[7] = 2 * xy;
}

// EQUI ////////////////////////////////////////////////////////////////////////

/**
 * Distort 2x1 point `p` using Equi-Distant distortion, where the
 * distortion params are stored in `params` (k1, k2, k3, k4), results are
 * written to 2x1 vector `p_d`.
 */
void equi4_distort(const real_t params[4], const real_t p[2], real_t p_d[2]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(p_d != NULL);

  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t k3 = params[2];
  const real_t k4 = params[3];

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const real_t s = thd / r;

  p_d[0] = s * x;
  p_d[1] = s * y;
}

/**
 * Form Equi-Distant point jacobian, using distortion `params` (k1, k2, k3,
 * k4), 2x1 image point `p`, the jacobian is written to 2x2 `J_point`.
 */
void equi4_point_jacobian(const real_t params[4],
                          const real_t p[2],
                          real_t J_point[2 * 2]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(J_point != NULL);

  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t k3 = params[2];
  const real_t k4 = params[3];

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);

  const real_t th_r = 1.0 / (r * r + 1.0);
  const real_t thd_th =
      1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
  const real_t s = thd / r;
  const real_t s_r = thd_th * th_r / r - thd / (r * r);
  const real_t r_x = 1.0 / r * x;
  const real_t r_y = 1.0 / r * y;

  /* Point Jacobian is 2x2 */
  J_point[0] = s + x * s_r * r_x;
  J_point[1] = x * s_r * r_y;
  J_point[2] = y * s_r * r_x;
  J_point[3] = s + y * s_r * r_y;
}

/**
 * Form Equi-Distant parameter jacobian, using distortion `params` (k1, k2,
 * k3, k4), 2x1 image point `p`, the jacobian is written to 2x4 `J_param`.
 */
void equi4_params_jacobian(const real_t params[4],
                           const real_t p[2],
                           real_t J_param[2 * 4]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(J_param != NULL);
  UNUSED(params);

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th3 = th2 * th;
  const real_t th5 = th3 * th2;
  const real_t th7 = th5 * th2;
  const real_t th9 = th7 * th2;

  /* Param Jacobian is 2x4 */
  J_param[0] = x * th3 / r;
  J_param[1] = x * th5 / r;
  J_param[2] = x * th7 / r;
  J_param[3] = x * th9 / r;

  J_param[4] = y * th3 / r;
  J_param[5] = y * th5 / r;
  J_param[6] = y * th7 / r;
  J_param[7] = y * th9 / r;
}

// PINHOLE /////////////////////////////////////////////////////////////////////

/**
 * Estimate pinhole focal length. The focal length is estimated using
 * `image_width` [pixels], and `fov` (Field of view of the camera) [rad].
 */
real_t pinhole_focal(const int image_width, const real_t fov) {
  return ((image_width / 2.0) / tan(deg2rad(fov) / 2.0));
}

/**
 * From 3x3 camera matrix K using pinhole camera parameters.
 *
 *   K = [fx,  0,  cx,
 *         0  fy,  cy,
 *         0   0,   1];
 *
 * where `params` is assumed to contain the fx, fy, cx, cy in that order.
 */
void pinhole_K(const real_t params[4], real_t K[3 * 3]) {
  K[0] = params[0];
  K[1] = 0.0;
  K[2] = params[2];

  K[3] = 0.0;
  K[4] = params[1];
  K[5] = params[3];

  K[6] = 0.0;
  K[7] = 0.0;
  K[8] = 1.0;
}

/**
 * Form 3x4 pinhole projection matrix `P`:
 *
 *   P = K * [-C | -C * r];
 *
 * Where K is the pinhole camera matrix formed using the camera parameters
 * `params` (fx, fy, cx, cy), C and r is the rotation and translation component
 * of the camera pose represented as a 4x4 homogenous transform `T`.
 */
void pinhole_projection_matrix(const real_t params[4],
                               const real_t T[4 * 4],
                               real_t P[3 * 4]) {
  assert(params != NULL);
  assert(T != NULL);
  assert(P != NULL);

  /* Form K matrix */
  real_t K[3 * 3] = {0};
  pinhole_K(params, K);

  /* Invert camera pose */
  real_t T_inv[4 * 4] = {0};
  tf_inv(T, T_inv);

  /* Extract rotation and translation component */
  real_t C[3 * 3] = {0};
  real_t r[3] = {0};
  tf_rot_get(T_inv, C);
  tf_trans_get(T_inv, r);

  /* Form [C | r] matrix */
  real_t Cr[3 * 4] = {0};
  Cr[0] = C[0];
  Cr[1] = C[1];
  Cr[2] = C[2];
  Cr[3] = r[0];

  Cr[4] = C[3];
  Cr[5] = C[4];
  Cr[6] = C[5];
  Cr[7] = r[1];

  Cr[8] = C[6];
  Cr[9] = C[7];
  Cr[10] = C[8];
  Cr[11] = r[2];

  /* Form projection matrix P = K * [C | r] */
  dot(K, 3, 3, Cr, 3, 4, P);
}

/**
 * Project 3D point `p_C` observed from the camera to the image plane `z`
 * using pinhole parameters `params` (fx, fy, cx, cy).
 */
void pinhole_project(const real_t params[4], const real_t p_C[3], real_t z[2]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(z != NULL);

  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  const real_t px = p_C[0] / p_C[2];
  const real_t py = p_C[1] / p_C[2];

  z[0] = px * fx + cx;
  z[1] = py * fy + cy;
}

/**
 * Form Pinhole point jacobian `J` using pinhole parameters `params`.
 */
void pinhole_point_jacobian(const real_t params[4], real_t J[2 * 2]) {
  assert(params != NULL);
  assert(J != NULL);

  J[0] = params[0];
  J[1] = 0.0;
  J[2] = 0.0;
  J[3] = params[1];
}

/**
 * Form Pinhole parameter jacobian `J` using pinhole parameters `params` and
 * 2x1 image point `x`.
 */
void pinhole_params_jacobian(const real_t params[4],
                             const real_t x[2],
                             real_t J[2 * 4]) {
  assert(params != NULL);
  assert(x != NULL);
  assert(J != NULL);
  UNUSED(params);

  J[0] = x[0];
  J[1] = 0.0;
  J[2] = 1.0;
  J[3] = 0.0;

  J[4] = 0.0;
  J[5] = x[1];
  J[6] = 0.0;
  J[7] = 1.0;
}

// PINHOLE-RADTAN4 /////////////////////////////////////////////////////////////

/**
 * Projection of 3D point to image plane using Pinhole + Radial-Tangential.
 */
void pinhole_radtan4_project(const real_t params[8],
                             const real_t p_C[3],
                             real_t x[2]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(x != NULL);

  /* Project */
  const real_t p[2] = {p_C[0] / p_C[2], p_C[1] / p_C[2]};

  /* Distort */
  const real_t d[4] = {params[4], params[5], params[6], params[7]};
  real_t p_d[2] = {0};
  radtan4_distort(d, p, p_d);

  /* Scale and center */
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  x[0] = p_d[0] * fx + cx;
  x[1] = p_d[1] * fy + cy;
}

/**
 * Projection Jacobian of Pinhole + Radial-Tangential.
 */
void pinhole_radtan4_project_jacobian(const real_t params[8],
                                      const real_t p_C[3],
                                      real_t J[2 * 3]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  /* Project */
  const real_t x = p_C[0];
  const real_t y = p_C[1];
  const real_t z = p_C[2];
  const real_t p[2] = {x / z, y / z};

  /* Projection Jacobian */
  real_t J_p[2 * 3] = {0};
  J_p[0] = 1.0 / z;
  J_p[1] = 0.0;
  J_p[2] = -x / (z * z);
  J_p[3] = 0.0;
  J_p[4] = 1.0 / z;
  J_p[5] = -y / (z * z);

  /* Distortion Point Jacobian */
  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};
  real_t J_d[2 * 2] = {0};
  radtan4_point_jacobian(d, p, J_d);

  /* Project Point Jacobian */
  real_t J_k[2 * 3] = {0};
  pinhole_point_jacobian(params, J_k);

  /* J = J_k * J_d * J_p; */
  real_t J_dp[2 * 3] = {0};
  dot(J_d, 2, 2, J_p, 2, 3, J_dp);
  dot(J_k, 2, 2, J_dp, 2, 3, J);
}

/**
 * Parameter Jacobian of Pinhole + Radial-Tangential.
 */
void pinhole_radtan4_params_jacobian(const real_t params[8],
                                     const real_t p_C[3],
                                     real_t J[2 * 8]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];
  const real_t k[4] = {fx, fy, cx, cy};

  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};

  /* Project */
  const real_t x = p_C[0];
  const real_t y = p_C[1];
  const real_t z = p_C[2];
  const real_t p[2] = {x / z, y / z};

  /* Distort */
  real_t p_d[2] = {0};
  radtan4_distort(d, p, p_d);

  /* Project params Jacobian: J_proj_params */
  real_t J_proj_params[2 * 4] = {0};
  pinhole_params_jacobian(k, p_d, J_proj_params);

  /* Project point Jacobian: J_proj_point */
  real_t J_proj_point[2 * 2] = {0};
  pinhole_point_jacobian(k, J_proj_point);

  /* Distortion point Jacobian: J_dist_params */
  real_t J_dist_params[2 * 4] = {0};
  radtan4_params_jacobian(d, p, J_dist_params);

  /* Radtan4 params Jacobian: J_radtan4 */
  real_t J_radtan4[2 * 4] = {0};
  dot(J_proj_point, 2, 2, J_dist_params, 2, 4, J_radtan4);

  /* J = [J_proj_params, J_proj_point * J_dist_params] */
  J[0] = J_proj_params[0];
  J[1] = J_proj_params[1];
  J[2] = J_proj_params[2];
  J[3] = J_proj_params[3];

  J[8] = J_proj_params[4];
  J[9] = J_proj_params[5];
  J[10] = J_proj_params[6];
  J[11] = J_proj_params[7];

  J[4] = J_radtan4[0];
  J[5] = J_radtan4[1];
  J[6] = J_radtan4[2];
  J[7] = J_radtan4[3];

  J[12] = J_radtan4[4];
  J[13] = J_radtan4[5];
  J[14] = J_radtan4[6];
  J[15] = J_radtan4[7];
}

// PINHOLE-EQUI4 ///////////////////////////////////////////////////////////////

/**
 * Projection of 3D point to image plane using Pinhole + Equi-Distant.
 */
void pinhole_equi4_project(const real_t params[8],
                           const real_t p_C[3],
                           real_t x[2]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(x != NULL);

  /* Project */
  const real_t p[2] = {p_C[0] / p_C[2], p_C[1] / p_C[2]};

  /* Distort */
  const real_t d[4] = {params[4], params[5], params[6], params[7]};
  real_t p_d[2] = {0};
  equi4_distort(d, p, p_d);

  /* Scale and center */
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  x[0] = p_d[0] * fx + cx;
  x[1] = p_d[1] * fy + cy;
}

/**
 * Projection Jacobian of Pinhole + Equi-Distant.
 */
void pinhole_equi4_project_jacobian(const real_t params[8],
                                    const real_t p_C[3],
                                    real_t J[2 * 3]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  /* Project */
  const real_t x = p_C[0];
  const real_t y = p_C[1];
  const real_t z = p_C[2];
  const real_t p[2] = {x / z, y / z};

  /* Projection Jacobian */
  real_t J_p[2 * 3] = {0};
  J_p[0] = 1.0 / z;
  J_p[1] = 0.0;
  J_p[2] = -x / (z * z);
  J_p[3] = 0.0;
  J_p[4] = 1.0 / z;
  J_p[5] = -y / (z * z);

  /* Distortion Point Jacobian */
  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t k3 = params[6];
  const real_t k4 = params[7];
  const real_t d[4] = {k1, k2, k3, k4};
  real_t J_d[2 * 2] = {0};
  equi4_point_jacobian(d, p, J_d);

  /* Project Point Jacobian */
  real_t J_k[2 * 3] = {0};
  pinhole_point_jacobian(params, J_k);

  /* J = J_k * J_d * J_p; */
  real_t J_dp[2 * 3] = {0};
  dot(J_d, 2, 2, J_p, 2, 3, J_dp);
  dot(J_k, 2, 2, J_dp, 2, 3, J);
}

void pinhole_equi4_params_jacobian(const real_t params[8],
                                   const real_t p_C[3],
                                   real_t J[2 * 8]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];
  const real_t k[4] = {fx, fy, cx, cy};

  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};

  /* Project */
  const real_t x = p_C[0];
  const real_t y = p_C[1];
  const real_t z = p_C[2];
  const real_t p[2] = {x / z, y / z};

  /* Distort */
  real_t p_d[2] = {0};
  equi4_distort(d, p, p_d);

  /* Project params Jacobian: J_proj_params */
  real_t J_proj_params[2 * 4] = {0};
  pinhole_params_jacobian(k, p_d, J_proj_params);

  /* Project point Jacobian: J_proj_point */
  real_t J_proj_point[2 * 2] = {0};
  pinhole_point_jacobian(k, J_proj_point);

  /* Distortion point Jacobian: J_dist_params */
  real_t J_dist_params[2 * 4] = {0};
  equi4_params_jacobian(d, p, J_dist_params);

  /* Radtan4 params Jacobian: J_equi4 */
  real_t J_equi4[2 * 4] = {0};
  dot(J_proj_point, 2, 2, J_dist_params, 2, 4, J_equi4);

  /* J = [J_proj_params, J_proj_point * J_dist_params] */
  J[0] = J_proj_params[0];
  J[1] = J_proj_params[1];
  J[2] = J_proj_params[2];
  J[3] = J_proj_params[3];

  J[8] = J_proj_params[4];
  J[9] = J_proj_params[5];
  J[10] = J_proj_params[6];
  J[11] = J_proj_params[7];

  J[4] = J_equi4[0];
  J[5] = J_equi4[1];
  J[6] = J_equi4[2];
  J[7] = J_equi4[3];

  J[12] = J_equi4[4];
  J[13] = J_equi4[5];
  J[14] = J_equi4[6];
  J[15] = J_equi4[7];
}

/******************************************************************************
 * SENSOR FUSION
 ******************************************************************************/

// POSE ////////////////////////////////////////////////////////////////////////

void pose_setup(pose_t *pose, const timestamp_t ts, const real_t *data) {
  assert(pose != NULL);
  assert(data != NULL);

  /* Timestamp */
  pose->ts = ts;

  /* Translation */
  pose->pos[0] = data[0]; /* rx */
  pose->pos[1] = data[1]; /* ry */
  pose->pos[2] = data[2]; /* rz */

  /* Rotation (Quaternion) */
  pose->quat[0] = data[6]; /* qw */
  pose->quat[1] = data[3]; /* qx */
  pose->quat[2] = data[4]; /* qy */
  pose->quat[3] = data[5]; /* qz */
}

/**
 * Print pose
 */
void pose_print(const char *prefix, const pose_t *pose) {
  const timestamp_t ts = pose->ts;

  const real_t x = pose->pos[0];
  const real_t y = pose->pos[1];
  const real_t z = pose->pos[2];

  const real_t qw = pose->quat[0];
  const real_t qx = pose->quat[1];
  const real_t qy = pose->quat[2];
  const real_t qz = pose->quat[3];

  printf("[%s] ", prefix);
  printf("ts: %ld, ", ts);
  printf("pos: (%.2f, %.2f, %.2f), ", x, y, z);
  printf("quat: (%.2f, %.2f, %.2f, %.2f)\n", qw, qx, qy, qz);
}

// SPEED AND BIASES ////////////////////////////////////////////////////////////

/**
 * Setup speed and biases
 */
void speed_biases_setup(speed_biases_t *sb,
                        const timestamp_t ts,
                        const real_t *data) {
  assert(sb != NULL);
  assert(data != NULL);

  /* Timestamp */
  sb->ts = ts;

  /* Velocity */
  sb->data[0] = data[0];
  sb->data[1] = data[1];
  sb->data[2] = data[2];

  /* Accel biases */
  sb->data[3] = data[3];
  sb->data[4] = data[4];
  sb->data[5] = data[5];

  /* Gyro biases */
  sb->data[6] = data[6];
  sb->data[7] = data[7];
  sb->data[8] = data[8];
}

// FEATURE /////////////////////////////////////////////////////////////////////

/**
 * Setup feature
 */
void feature_setup(feature_t *feature, const real_t *data) {
  assert(feature != NULL);
  assert(data != NULL);

  feature->data[0] = data[0];
  feature->data[1] = data[1];
  feature->data[2] = data[2];
}

/**
 * Setup features
 */
void features_setup(features_t *features) {
  assert(features);
  features->nb_features = 0;
  for (int i = 0; i < MAX_FEATURES; i++) {
    features->status[i] = 0;
  }
}

/**
 * Check whether feature with `feature_id` exists
 * @returns 1 for yes, 0 for no
 */
int features_exists(const features_t *features, const int feature_id) {
  return features->status[feature_id];
}

/**
 * Returns pointer to feature with `feature_id`
 */
feature_t *features_get(features_t *features, const int feature_id) {
  return &features->data[feature_id];
}

/**
 * Add feature
 * @returns Pointer to feature
 */
feature_t *features_add(features_t *features,
                        const int feature_id,
                        const real_t *param) {
  feature_setup(&features->data[feature_id], param);
  features->status[feature_id] = 1;
  return &features->data[feature_id];
}

/**
 * Remove feature with `feature_id`
 */
void features_remove(features_t *features, const int feature_id) {
  const real_t param[3] = {0};
  feature_setup(&features->data[feature_id], param);
  features->status[feature_id] = 0;
}

// EXTRINSICS //////////////////////////////////////////////////////////////////

/**
 * Setup extrinsics
 */
void extrinsics_setup(extrinsics_t *exts, const real_t *data) {
  assert(exts != NULL);
  assert(data != NULL);

  /* Translation */
  exts->pos[0] = data[0]; /* rx */
  exts->pos[1] = data[1]; /* ry */
  exts->pos[2] = data[2]; /* rz */

  /* Rotation (Quaternion) */
  exts->quat[0] = data[6]; /* qw */
  exts->quat[1] = data[3]; /* qx */
  exts->quat[2] = data[4]; /* qy */
  exts->quat[3] = data[5]; /* qz */
}

/**
 * Print extrinsics
 */
void extrinsics_print(const char *prefix, const extrinsics_t *exts) {
  const real_t x = exts->pos[0];
  const real_t y = exts->pos[1];
  const real_t z = exts->pos[2];

  const real_t qw = exts->quat[0];
  const real_t qx = exts->quat[1];
  const real_t qy = exts->quat[2];
  const real_t qz = exts->quat[3];

  printf("[%s] ", prefix);
  printf("pos: (%.2f, %.2f, %.2f), ", x, y, z);
  printf("quat: (%.2f, %.2f, %.2f, %.2f)\n", qw, qx, qy, qz);
}

// CAMERA PARAMS ///////////////////////////////////////////////////////////////

/**
 * Setup camera parameters
 */
void camera_params_setup(camera_params_t *camera,
                         const int cam_idx,
                         const int cam_res[2],
                         const char *proj_model,
                         const char *dist_model,
                         const real_t data[8]) {
  assert(camera != NULL);
  assert(cam_res != NULL);
  assert(proj_model != NULL);
  assert(dist_model != NULL);
  assert(data != NULL);

  camera->cam_idx = cam_idx;
  camera->resolution[0] = cam_res[0];
  camera->resolution[1] = cam_res[1];

  strncpy(camera->proj_model, proj_model, strlen(proj_model));
  strncpy(camera->dist_model, dist_model, strlen(dist_model));

  camera->data[0] = data[0];
  camera->data[1] = data[1];
  camera->data[2] = data[2];
  camera->data[3] = data[3];
  camera->data[4] = data[4];
  camera->data[5] = data[5];
  camera->data[6] = data[6];
  camera->data[7] = data[7];
}

/**
 * Print camera parameters
 */
void camera_params_print(const camera_params_t *camera) {
  assert(camera != NULL);

  printf("cam_idx: %d\n", camera->cam_idx);
  printf("cam_res: [%d, %d]\n", camera->resolution[0], camera->resolution[1]);
  printf("proj_model: %s\n", camera->proj_model);
  printf("dist_model: %s\n", camera->dist_model);
  printf("data: [");
  for (int i = 0; i < 8; i++) {
    if ((i + 1) < 8) {
      printf("%f, ", camera->data[i]);
    } else {
      printf("%f", camera->data[i]);
    }
  }
  printf("]\n");
}

// POSE FACTOR /////////////////////////////////////////////////////////////////

/**
 * Setup pose factor
 */
void pose_factor_setup(pose_factor_t *factor,
                       pose_t *pose,
                       const real_t var[6]) {
  assert(factor != NULL);
  assert(pose != NULL);
  assert(var != NULL);

  /* Parameters */
  factor->pose_est = pose;
  factor->nb_params = 1;

  /* Measurement */
  factor->pos_meas[0] = pose->pos[0];
  factor->pos_meas[1] = pose->pos[1];
  factor->pos_meas[2] = pose->pos[2];
  factor->quat_meas[0] = pose->quat[0];
  factor->quat_meas[1] = pose->quat[1];
  factor->quat_meas[2] = pose->quat[2];
  factor->quat_meas[3] = pose->quat[3];

  /* Measurement covariance matrix */
  zeros(factor->covar, 6, 6);
  factor->covar[0] = 1.0 / (var[0] * var[0]);
  factor->covar[7] = 1.0 / (var[1] * var[1]);
  factor->covar[14] = 1.0 / (var[2] * var[2]);
  factor->covar[21] = 1.0 / (var[3] * var[3]);
  factor->covar[28] = 1.0 / (var[4] * var[4]);
  factor->covar[35] = 1.0 / (var[5] * var[5]);

  /* Square root information matrix */
  zeros(factor->sqrt_info, 6, 6);
  factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);
  factor->sqrt_info[7] = sqrt(1.0 / factor->covar[7]);
  factor->sqrt_info[14] = sqrt(1.0 / factor->covar[14]);
  factor->sqrt_info[21] = sqrt(1.0 / factor->covar[21]);
  factor->sqrt_info[28] = sqrt(1.0 / factor->covar[28]);
  factor->sqrt_info[35] = sqrt(1.0 / factor->covar[35]);
}

/**
 * Evaluate pose factor
 */
int pose_factor_eval(pose_factor_t *factor,
                     real_t **params,
                     real_t *r_out,
                     real_t **J_out) {
  assert(factor != NULL);

  /* Map params */
  const real_t *r_est = params[0];
  const real_t *q_est = params[1];
  const real_t *r_meas = factor->pos_meas;
  const real_t *q_meas = factor->quat_meas;

  /* Calculate pose error */
  /* -- Translation error */
  /* dr = r_meas - r_est; */
  real_t dr[3] = {0};
  dr[0] = r_meas[0] - r_est[0];
  dr[1] = r_meas[1] - r_est[1];
  dr[2] = r_meas[2] - r_est[2];

  /* -- Rotation error */
  /* dq = quat_mul(quat_inv(q_meas), q_est); */
  real_t dq[4] = {0};
  real_t q_meas_inv[4] = {0};
  quat_inv(q_meas, q_meas_inv);
  quat_mul(q_meas_inv, q_est, dq);

  /* dtheta = 2 * dq; */
  real_t dtheta[3] = {0};
  dtheta[0] = 2 * dq[1];
  dtheta[1] = 2 * dq[2];
  dtheta[2] = 2 * dq[3];

  /* -- Set residuals */
  /* r = factor.sqrt_info * [dr; dtheta]; */
  real_t r[6] = {0};
  r[0] = dr[0];
  r[1] = dr[1];
  r[2] = dr[2];
  r[3] = dtheta[0];
  r[4] = dtheta[1];
  r[5] = dtheta[2];
  dot(factor->sqrt_info, 6, 6, r, 6, 1, r_out);

  /* Calculate Jacobians */
  if (J_out == NULL) {
    return 0;
  }

  if (J_out[0]) {
    /* J0 = zeros(6, 3); */
    /* J0(1:3, 1:3) = -eye(3); */
    /* J0 = factor.sqrt_info * J0; */

    /* clang-format off */
    real_t J0[6 * 3] = {0};
    J0[0] = -1.0; J0[1] =  0.0; J0[2] =  0.0;
    J0[3] =  0.0; J0[4] = -1.0; J0[5] =  0.0;
    J0[6] =  0.0; J0[7] =  0.0; J0[8] = -1.0;
    dot(factor->sqrt_info, 6, 6, J0, 6, 3, J_out[0]);
    /* clang-format on */
  }

  if (J_out[1]) {
    /* J1 = zeros(6, 3); */
    /* J1(4:6, 4:6) = quat_left(dq)(2:4, 2:4); */
    /* J1 = factor.sqrt_info * J1; */

    /* clang-format off */
    const real_t dqw = dq[0];
    const real_t dqx = dq[1];
    const real_t dqy = dq[2];
    const real_t dqz = dq[3];

    real_t J1[6 * 3] = {0};
    J1[9] =  dqw;  J1[10] = -dqz; J1[11] =  dqy;
    J1[12] =  dqz; J1[13] =  dqw; J1[14] = -dqx;
    J1[15] = -dqy; J1[16] =  dqx; J1[17] =  dqw;
    dot(factor->sqrt_info, 6, 6, J1, 6, 3, J_out[1]);
    /* clang-format on */
  }

  return 0;
}

/**
 * Evaluate pose factor (wrapper for ceres-solver)
 */
int pose_factor_ceres_eval(void *factor,
                           double **params,
                           double *r_out,
                           double **J_out) {
  assert(factor != NULL);
  assert(factor != NULL);
  assert(params != NULL);
  assert(r_out != NULL);

  real_t J0[6 * 3] = {0};
  real_t J1[6 * 3] = {0};
  real_t *factor_jacs[2] = {J0, J1};
  pose_factor_t *pose_factor = (pose_factor_t *) factor;
  int retval = pose_factor_eval(pose_factor, params, r_out, factor_jacs);

  if (J_out == NULL) {
    return retval;
  }

  if (J_out[0]) {
    mat_copy(factor_jacs[0], 6, 3, J_out[0]);
  }

  if (J_out[1]) {
    mat_copy(factor_jacs[1], 6, 3, J_out[1]);
  }

  return retval;
}

// BA FACTOR ///////////////////////////////////////////////////////////////////

/**
 * Setup bundle adjustment factor
 */
void ba_factor_setup(ba_factor_t *factor,
                     const pose_t *pose,
                     const feature_t *feature,
                     const camera_params_t *camera,
                     const real_t z[2],
                     const real_t var[2]) {
  assert(factor != NULL);
  assert(pose != NULL);
  assert(feature != NULL);
  assert(camera != NULL);
  assert(var != NULL);

  /* Parameters */
  factor->pose = pose;
  factor->feature = feature;
  factor->camera = camera;
  factor->nb_params = 3;

  /* Measurement covariance */
  factor->covar[0] = 1.0 / (var[0] * var[0]);
  factor->covar[1] = 0.0;
  factor->covar[2] = 0.0;
  factor->covar[3] = 1.0 / (var[1] * var[1]);

  /* Square-root information matrix */
  factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);
  factor->sqrt_info[1] = 0.0;
  factor->sqrt_info[2] = 0.0;
  factor->sqrt_info[3] = sqrt(1.0 / factor->covar[3]);

  /* Measurement */
  factor->z[0] = z[0];
  factor->z[1] = z[1];
}

/**
 * Camera pose jacobian
 */
static void ba_factor_pose_jacobian(const real_t Jh_weighted[2 * 3],
                                    const real_t T_WC[4 * 4],
                                    const real_t p_W[3],
                                    real_t *J_pos,
                                    real_t *J_rot) {
  /* Pre-check */
  if (J_pos == NULL || J_rot == NULL) {
    return;
  }

  /* Jh_weighted = -1 * sqrt_info * Jh; */
  /* J_pos = Jh_weighted * -C_CW; */
  /* J_rot = Jh_weighted * -C_CW * skew(p_W - r_WC) * -C_WC; */
  /* J = [J_rot, J_pos] */

  /* Setup */
  real_t C_WC[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};
  real_t r_WC[3] = {0};
  tf_rot_get(T_WC, C_WC);
  mat_transpose(C_WC, 3, 3, C_CW);
  tf_trans_get(T_WC, r_WC);

  /* J_pos = -1 * sqrt_info * Jh * -C_CW; */
  real_t neg_C_CW[3 * 3] = {0};
  mat_copy(C_CW, 3, 3, neg_C_CW);
  mat_scale(neg_C_CW, 3, 3, -1.0);
  dot(Jh_weighted, 2, 3, neg_C_CW, 3, 3, J_pos);

  /**
   * Jh_weighted = -1 * sqrt_info * Jh;
   * J_rot = Jh_weighted * -C_CW * skew(p_W - r_WC) * -C_WC;
   * where:
   *
   *   A = -C_CW;
   *   B = skew(p_W - r_WC);
   *   C = -C_WC;
   */
  real_t A[3 * 3] = {0};
  mat_copy(neg_C_CW, 3, 3, A);

  real_t B[3 * 3] = {0};
  real_t dp[3] = {0};
  dp[0] = p_W[0] - r_WC[0];
  dp[1] = p_W[1] - r_WC[1];
  dp[2] = p_W[2] - r_WC[2];
  skew(dp, B);

  real_t C[3 * 3] = {0};
  mat_copy(C_WC, 3, 3, C);
  mat_scale(C, 3, 3, -1.0);

  real_t AB[3 * 3] = {0};
  real_t ABC[3 * 3] = {0};
  dot(A, 3, 3, B, 3, 3, AB);
  dot(AB, 3, 3, C, 3, 3, ABC);
  dot(Jh_weighted, 2, 3, ABC, 3, 3, J_rot);
}

/**
 * Feature jacobian
 */
static void ba_factor_feature_jacobian(const real_t Jh_weighted[2 * 3],
                                       const real_t T_WC[4 * 4],
                                       real_t *J) {
  /* Pre-check */
  if (J == NULL) {
    return;
  }

  /* Jh_weighted = -1 * sqrt_info * Jh; */
  /* J = Jh_weighted * C_CW; */
  real_t C_WC[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};
  tf_rot_get(T_WC, C_WC);
  mat_transpose(C_WC, 3, 3, C_CW);
  dot(Jh_weighted, 2, 3, C_CW, 3, 3, J);
}

/**
 * Camera parameters jacobian
 */
static void ba_factor_camera_jacobian(const real_t neg_sqrt_info[2 * 2],
                                      const real_t J_cam_params[2 * 8],
                                      real_t *J) {
  /* Pre-check */
  if (J == NULL) {
    return;
  }

  /* J = -1 * sqrt_info * J_cam_params; */
  dot(neg_sqrt_info, 2, 2, J_cam_params, 2, 8, J);
}

/**
 * Evaluate bundle adjustment factor
 * @returns 0 for success, -1 for failure
 */
int ba_factor_eval(ba_factor_t *factor,
                   real_t **params,
                   real_t *r_out,
                   real_t **J_out) {
  assert(factor != NULL);
  assert(params != NULL);
  assert(r_out != NULL);

  /* Map params */
  const real_t *r_WCi = params[0];
  const real_t *q_WCi = params[1];
  const real_t *p_W = params[2];
  const real_t *cam_params = params[3];

  /* Camera pose */
  real_t T_WCi[4 * 4] = {0};
  eye(T_WCi, 4, 4);
  tf_trans_set(T_WCi, r_WCi);
  tf_quat_set(T_WCi, q_WCi);

  /* Calculate residuals */
  /* -- Project point from world to image plane */
  real_t T_CiW[4 * 4] = {0};
  real_t p_Ci[3] = {0};
  real_t z_hat[2];
  tf_inv(T_WCi, T_CiW);
  tf_point(T_CiW, p_W, p_Ci);
  pinhole_radtan4_project(cam_params, p_Ci, z_hat);
  /* -- Residual */
  real_t r[2] = {0};
  r[0] = factor->z[0] - z_hat[0];
  r[1] = factor->z[1] - z_hat[1];
  /* -- Weighted residual */
  dot(factor->sqrt_info, 2, 2, r, 2, 1, r_out);

  /* Calculate jacobians */
  if (J_out == NULL) {
    return 0;
  }
  /* -- Form: -1 * sqrt_info */
  real_t neg_sqrt_info[2 * 2] = {0};
  mat_copy(factor->sqrt_info, 2, 2, neg_sqrt_info);
  mat_scale(neg_sqrt_info, 2, 2, -1.0);
  /* -- Form: Jh_weighted = -1 * sqrt_info * Jh */
  real_t Jh[2 * 3] = {0};
  real_t Jh_weighted[2 * 3] = {0};
  pinhole_radtan4_project_jacobian(cam_params, p_Ci, Jh);
  dot(neg_sqrt_info, 2, 2, Jh, 2, 3, Jh_weighted);
  /* -- Form: J_cam_params */
  real_t J_cam_params[2 * 8] = {0};
  pinhole_radtan4_params_jacobian(cam_params, p_Ci, J_cam_params);
  /* -- Fill jacobians */
  ba_factor_pose_jacobian(Jh_weighted, T_WCi, p_W, J_out[0], J_out[1]);
  ba_factor_feature_jacobian(Jh_weighted, T_WCi, J_out[2]);
  ba_factor_camera_jacobian(neg_sqrt_info, J_cam_params, J_out[3]);

  return 0;
}

/**
 * Evaluate bundle adjustment factor (wrapper for ceres-solver)
 * @returns 0 for success, -1 for failure
 */
int ba_factor_ceres_eval(void *factor,
                         double **params,
                         double *r_out,
                         double **J_out) {
  assert(factor != NULL);
  assert(params != NULL);
  assert(r_out != NULL);

  return 0;
}

// CAMERA FACTOR ///////////////////////////////////////////////////////////////

/**
 * Setup camera factor
 */
void cam_factor_setup(cam_factor_t *factor,
                      const pose_t *pose,
                      const extrinsics_t *extrinsics,
                      const feature_t *feature,
                      const camera_params_t *camera,
                      const real_t z[2],
                      const real_t var[2]) {
  assert(factor != NULL);
  assert(pose != NULL);
  assert(extrinsics != NULL);
  assert(feature != NULL);
  assert(camera != NULL);
  assert(var != NULL);

  /* Parameters */
  factor->pose = pose;
  factor->extrinsics = extrinsics;
  factor->feature = feature;
  factor->camera = camera;
  factor->nb_params = 4;

  /* Measurement covariance matrix */
  factor->covar[0] = 1.0 / (var[0] * var[0]);
  factor->covar[1] = 0.0;
  factor->covar[2] = 0.0;
  factor->covar[3] = 1.0 / (var[1] * var[1]);

  /* Square-root information matrix */
  factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);
  factor->sqrt_info[1] = 0.0;
  factor->sqrt_info[2] = 0.0;
  factor->sqrt_info[3] = sqrt(1.0 / factor->covar[3]);

  /* Measurement */
  factor->z[0] = z[0];
  factor->z[1] = z[1];
}

/**
 * Pose jacobian
 */
static void cam_factor_pose_jacobian(const real_t Jh_weighted[2 * 3],
                                     const real_t T_WB[3 * 3],
                                     const real_t T_BC[3 * 3],
                                     const real_t p_W[3],
                                     real_t J_pos[2 * 3],
                                     real_t J_rot[2 * 3]) {
  if (J_pos == NULL || J_rot == NULL) {
    return;
  }
  assert(Jh_weighted != NULL);
  assert(T_BC != NULL);
  assert(T_WB != NULL);
  assert(p_W != NULL);
  assert(J_pos != NULL);
  assert(J_rot != NULL);

  /* Jh_weighted = -1 * sqrt_info * Jh; */
  /* J_pos = Jh_weighted * C_CB * -C_BW; */
  /* J_rot = Jh_weighted * C_CB * C_BW * skew(p_W - r_WB) * -C_WB; */
  /* J = [J_pos, J_rot]; */

  /* Setup */
  real_t C_WB[3 * 3] = {0};
  real_t C_BC[3 * 3] = {0};
  real_t C_BW[3 * 3] = {0};
  real_t C_CB[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};

  tf_rot_get(T_WB, C_WB);
  tf_rot_get(T_BC, C_BC);
  mat_transpose(C_WB, 3, 3, C_BW);
  mat_transpose(C_BC, 3, 3, C_CB);
  dot(C_CB, 3, 3, C_BW, 3, 3, C_CW);

  /* Form: -C_BW */
  real_t neg_C_BW[3 * 3] = {0};
  mat_copy(C_BW, 3, 3, neg_C_BW);
  mat_scale(neg_C_BW, 3, 3, -1.0);

  /* Form: -C_CW */
  real_t neg_C_CW[3 * 3] = {0};
  dot(C_CB, 3, 3, neg_C_BW, 3, 3, neg_C_CW);

  /* Form: -C_WB */
  real_t neg_C_WB[3 * 3] = {0};
  mat_copy(C_WB, 3, 3, neg_C_WB);
  mat_scale(neg_C_WB, 3, 3, -1.0);

  /* Form: C_CB * -C_BW * skew(p_W - r_WB) * -C_WB */
  real_t r_WB[3] = {0};
  real_t p[3] = {0};
  real_t S[3 * 3] = {0};
  tf_trans_get(T_WB, r_WB);
  vec_sub(p_W, r_WB, p, 3);
  skew(p, S);

  real_t A[3 * 3] = {0};
  real_t B[3 * 3] = {0};
  dot(neg_C_CW, 3, 3, S, 3, 3, A);
  dot(A, 3, 3, neg_C_WB, 3, 3, B);

  /* Form: J_pos = Jh_weighted * C_CB * -C_BW; */
  dot(Jh_weighted, 2, 3, neg_C_CW, 3, 3, J_pos);

  /* Form: J_rot = Jh_weighted * C_CB * -C_BW * skew(p_W - r_WB) * -C_WB; */
  dot(Jh_weighted, 2, 3, B, 3, 3, J_rot);
}

/**
 * Body-camera extrinsics jacobian
 */
static void cam_factor_extrinsics_jacobian(const real_t Jh_weighted[2 * 3],
                                           const real_t T_BC[3 * 3],
                                           const real_t p_C[3],
                                           real_t J_pos[2 * 3],
                                           real_t J_rot[2 * 3]) {
  if (J_pos == NULL || J_rot == NULL) {
    return;
  }
  assert(Jh_weighted != NULL);
  assert(T_BC != NULL);
  assert(p_C != NULL);
  assert(J_pos != NULL);
  assert(J_rot != NULL);

  /* Jh_weighted = -1 * sqrt_info * Jh; */
  /* J_pos = Jh_weighted * -C_CB; */
  /* J_rot = Jh_weighted * C_CB * skew(C_BC * p_C); */

  /* Setup */
  real_t C_BC[3 * 3] = {0};
  real_t C_CB[3 * 3] = {0};
  real_t C_BW[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};

  tf_rot_get(T_BC, C_BC);
  mat_transpose(C_BC, 3, 3, C_CB);
  dot(C_CB, 3, 3, C_BW, 3, 3, C_CW);

  /* Form: -C_CB */
  real_t neg_C_CB[3 * 3] = {0};
  mat_copy(C_CB, 3, 3, neg_C_CB);
  mat_scale(neg_C_CB, 3, 3, -1.0);

  /* Form: -C_BC */
  real_t neg_C_BC[3 * 3] = {0};
  mat_copy(C_BC, 3, 3, neg_C_BC);
  mat_scale(neg_C_BC, 3, 3, -1.0);

  /* Form: -C_CB * skew(C_BC * p_C) * -C_BC */
  real_t p[3] = {0};
  real_t S[3 * 3] = {0};
  dot(C_BC, 3, 3, p_C, 3, 1, p);
  skew(p, S);

  real_t A[3 * 3] = {0};
  real_t B[3 * 3] = {0};
  dot(neg_C_CB, 3, 3, S, 3, 3, A);
  dot(A, 3, 3, neg_C_BC, 3, 3, B);

  /* Form: J_rot = Jh_weighted * -C_CB; */
  dot(Jh_weighted, 2, 3, neg_C_CB, 3, 3, J_pos);

  /* Form: J_pos = Jh_weighted * -C_CB * skew(C_BC * p_C) * -C_BC; */
  dot(Jh_weighted, 2, 3, B, 3, 3, J_rot);
}

/**
 * Camera parameters jacobian
 */
static void cam_factor_camera_jacobian(const real_t neg_sqrt_info[2 * 2],
                                       const real_t J_cam_params[2 * 8],
                                       real_t J[2 * 8]) {
  assert(neg_sqrt_info != NULL);
  assert(J_cam_params != NULL);
  assert(J != NULL);

  /* J = -1 * sqrt_info * J_cam_params; */
  dot(neg_sqrt_info, 2, 2, J_cam_params, 2, 8, J);
}

/**
 * Feature jacobian
 */
static void cam_factor_feature_jacobian(const real_t Jh_weighted[2 * 3],
                                        const real_t T_WB[4 * 4],
                                        const real_t T_BC[4 * 4],
                                        real_t J[2 * 3]) {
  if (J == NULL) {
    return;
  }
  assert(Jh_weighted != NULL);
  assert(T_WB != NULL);
  assert(T_BC != NULL);
  assert(J != NULL);

  /* Jh_weighted = -1 * sqrt_info * Jh; */
  /* J = Jh_weighted * C_CW; */

  /* Setup */
  real_t T_WC[4 * 4] = {0};
  real_t C_WC[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};
  dot(T_WB, 4, 4, T_BC, 4, 4, T_WC);
  tf_rot_get(T_WC, C_WC);
  mat_transpose(C_WC, 3, 3, C_CW);

  /* Form: J = -1 * sqrt_info * Jh * C_CW; */
  dot(Jh_weighted, 2, 3, C_CW, 3, 3, J);
}

/**
 * Evaluate camera factor
 */
int cam_factor_eval(cam_factor_t *factor,
                    real_t **params,
                    real_t *r_out,
                    real_t **J_out) {
  assert(factor != NULL);
  assert(factor->pose);
  assert(factor->extrinsics);
  assert(factor->feature);
  assert(factor->camera);
  assert(params != NULL);
  assert(r_out != NULL);

  /* Map params */
  const real_t *r_WB = params[0];
  const real_t *q_WB = params[1];
  const real_t *r_BCi = params[2];
  const real_t *q_BCi = params[3];
  const real_t *cam_params = params[4];
  const real_t *p_W = params[5];

  /* Body pose */
  real_t T_WB[4 * 4] = {0};
  eye(T_WB, 4, 4);
  tf_trans_set(T_WB, r_WB);
  tf_quat_set(T_WB, q_WB);

  /* Body-camera extrinsics */
  real_t T_BCi[4 * 4] = {0};
  eye(T_BCi, 4, 4);
  tf_trans_set(T_BCi, r_BCi);
  tf_quat_set(T_BCi, q_BCi);

  /* Form camera pose */
  real_t T_WCi[4 * 4] = {0};
  real_t T_CiW[4 * 4] = {0};
  dot(T_WB, 4, 4, T_BCi, 4, 4, T_WCi);
  tf_inv(T_WCi, T_CiW);

  /* Transform feature from world to camera frame */
  real_t p_Ci[3] = {0};
  tf_point(T_CiW, p_W, p_Ci);

  /* Calculate residuals */
  /* -- Project point from world to image plane */
  real_t z_hat[2];
  pinhole_radtan4_project(cam_params, p_Ci, z_hat);
  /* -- Residual */
  real_t r[2] = {0};
  r[0] = factor->z[0] - z_hat[0];
  r[1] = factor->z[1] - z_hat[1];
  /* -- Weighted residual */
  dot(factor->sqrt_info, 2, 2, r, 2, 1, r_out);

  /* Calculate jacobians */
  if (J_out == NULL) {
    return 0;
  }
  /* -- Form: -1 * sqrt_info */
  real_t neg_sqrt_info[2 * 2] = {0};
  mat_copy(factor->sqrt_info, 2, 2, neg_sqrt_info);
  mat_scale(neg_sqrt_info, 2, 2, -1.0);
  /* -- Form: Jh_ = -1 * sqrt_info * Jh */
  real_t Jh[2 * 3] = {0};
  real_t Jh_[2 * 3] = {0};
  pinhole_radtan4_project_jacobian(cam_params, p_Ci, Jh);
  dot(neg_sqrt_info, 2, 2, Jh, 2, 3, Jh_);
  /* -- Form: J_cam_params */
  real_t J_cam_params[2 * 8] = {0};
  pinhole_radtan4_params_jacobian(cam_params, p_Ci, J_cam_params);
  /* -- Fill jacobians */
  cam_factor_pose_jacobian(Jh_, T_WB, T_BCi, p_W, J_out[0], J_out[1]);
  cam_factor_extrinsics_jacobian(Jh_, T_BCi, p_Ci, J_out[2], J_out[3]);
  cam_factor_camera_jacobian(neg_sqrt_info, J_cam_params, J_out[4]);
  cam_factor_feature_jacobian(Jh_, T_WB, T_BCi, J_out[5]);

  return 0;
}

/**
 * Evaluate camera factor (ceres-solver wrapper)
 */
int cam_factor_ceres_eval(void *factor,
                          double **params,
                          double *r_out,
                          double **J_out) {
  assert(factor != NULL);
  assert(params != NULL);
  assert(r_out != NULL);

  real_t J0[2 * 3] = {0};
  real_t J1[2 * 4] = {0};
  real_t J2[2 * 3] = {0};
  real_t J3[2 * 4] = {0};
  real_t J4[2 * 8] = {0};
  real_t J5[2 * 3] = {0};
  real_t *factor_jacs[6] = {J0, J1, J2, J3, J4, J5};
  cam_factor_t *cam_factor = (cam_factor_t *) factor;
  int retval = cam_factor_eval(cam_factor, params, r_out, factor_jacs);

  if (J_out == NULL) {
    return retval;
  }

  if (J_out[0]) {
    mat_copy(factor_jacs[0], 2, 3, J_out[0]);
  }

  if (J_out[1]) {
    zeros(J_out[1], 2, 3);
    mat_block_set(J_out[1], 4, 0, 0, 1, 3, factor_jacs[1]);
  }

  if (J_out[2]) {
    mat_copy(factor_jacs[2], 2, 3, J_out[2]);
  }

  if (J_out[3]) {
    zeros(J_out[3], 2, 3);
    mat_block_set(J_out[3], 4, 0, 0, 1, 3, factor_jacs[3]);
  }

  if (J_out[4]) {
    mat_copy(factor_jacs[4], 2, 8, J_out[4]);
  }

  if (J_out[5]) {
    mat_copy(factor_jacs[5], 2, 3, J_out[5]);
  }

  return retval;
}

// IMU FACTOR //////////////////////////////////////////////////////////////////

/**
 * Setup IMU buffer
 */
void imu_buf_setup(imu_buf_t *imu_buf) {
  for (int k = 0; k < MAX_IMU_BUF_SIZE; k++) {
    imu_buf->ts[k] = 0.0;

    imu_buf->acc[k][0] = 0.0;
    imu_buf->acc[k][1] = 0.0;
    imu_buf->acc[k][2] = 0.0;

    imu_buf->gyr[k][0] = 0.0;
    imu_buf->gyr[k][1] = 0.0;
    imu_buf->gyr[k][2] = 0.0;
  }

  imu_buf->size = 0;
}

/**
 * Print IMU buffer
 */
void imu_buf_print(const imu_buf_t *imu_buf) {
  for (int k = 0; k < imu_buf->size; k++) {
    const real_t *acc = imu_buf->acc[k];
    const real_t *gyr = imu_buf->gyr[k];

    printf("ts: %ld ", imu_buf->ts[k]);
    printf("acc: [%.2f, %.2f, %.2f] ", acc[0], acc[1], acc[2]);
    printf("gyr: [%.2f, %.2f, %.2f] ", gyr[0], gyr[1], gyr[2]);
    printf("\n");
  }
}

/**
 * Add measurement to IMU buffer
 */
void imu_buf_add(imu_buf_t *imu_buf,
                 const timestamp_t ts,
                 const real_t acc[3],
                 const real_t gyr[3]) {
  assert(imu_buf->size < MAX_IMU_BUF_SIZE);
  const int k = imu_buf->size;
  imu_buf->ts[k] = ts;
  imu_buf->acc[k][0] = acc[0];
  imu_buf->acc[k][1] = acc[1];
  imu_buf->acc[k][2] = acc[2];
  imu_buf->gyr[k][0] = gyr[0];
  imu_buf->gyr[k][1] = gyr[1];
  imu_buf->gyr[k][2] = gyr[2];
  imu_buf->size++;
}

/**
 * Clear IMU buffer
 */
void imu_buf_clear(imu_buf_t *imu_buf) {
  for (int k = 0; k < imu_buf->size; k++) {
    timestamp_t *ts = &imu_buf->ts[k];
    real_t *acc = imu_buf->acc[k];
    real_t *gyr = imu_buf->gyr[k];

    *ts = 0;

    acc[0] = 0.0;
    acc[1] = 0.0;
    acc[2] = 0.0;

    gyr[0] = 0.0;
    gyr[1] = 0.0;
    gyr[2] = 0.0;
  }
  imu_buf->size = 0;
}

/**
 * Copy IMU buffer
 */
void imu_buf_copy(const imu_buf_t *src, imu_buf_t *dst) {
  dst->size = 0;
  for (int k = 0; k < src->size; k++) {
    dst->ts[k] = src->ts[k];

    dst->acc[k][0] = src->acc[k][0];
    dst->acc[k][1] = src->acc[k][1];
    dst->acc[k][2] = src->acc[k][2];

    dst->gyr[k][0] = src->gyr[k][0];
    dst->gyr[k][1] = src->gyr[k][1];
    dst->gyr[k][2] = src->gyr[k][2];
  }
  dst->size = src->size;
}

/* void imu_factor_setup(imu_factor_t *factor, */
/*                       imu_params_t *imu_params, */
/*                       imu_buf_t *imu_buf, */
/*                       pose_t *pose_i, */
/*                       speed_biases_t *sb_i, */
/*                       pose_t *pose_j, */
/*                       speed_biases_t *sb_j) { */
/*   #<{(| Parameters |)}># */
/*   factor->imu_params = imu_params; */
/*   imu_buf_copy(imu_buf, &factor->imu_buf); */
/*   factor->pose_i = pose_i; */
/*   factor->sb_i = sb_i; */
/*   factor->pose_j = pose_j; */
/*   factor->sb_j = sb_j; */
/*  */
/*   #<{(| Covariance and residuals |)}># */
/*   zeros(factor->covar, 15, 15); */
/*   zeros(factor->r, 15, 1); */
/*   factor->r_size = 15; */
/*  */
/*   #<{(| Jacobians |)}># */
/*   factor->jacs[0] = factor->J0; */
/*   factor->jacs[1] = factor->J1; */
/*   factor->jacs[2] = factor->J2; */
/*   factor->jacs[3] = factor->J3; */
/*   factor->nb_params = 4; */
/*  */
/*   #<{(| Pre-integration variables |)}># */
/*   factor->Dt = 0.0; */
/*   eye(factor->F, 15, 15);   #<{(| State jacobian |)}># */
/*   zeros(factor->P, 15, 15); #<{(| State covariance |)}># */
/*  */
/*   #<{(| -- Noise matrix |)}># */
/*   const real_t n_a = imu_params->n_a; */
/*   const real_t n_g = imu_params->n_g; */
/*   const real_t n_ba = imu_params->n_aw; */
/*   const real_t n_bg = imu_params->n_gw; */
/*   const real_t n_a_sq = n_a * n_a; */
/*   const real_t n_g_sq = n_g * n_g; */
/*   const real_t n_ba_sq = n_ba * n_ba; */
/*   const real_t n_bg_sq = n_bg * n_bg; */
/*   real_t Q_diag[12] = {0}; */
/*   Q_diag[0] = n_a_sq; */
/*   Q_diag[1] = n_a_sq; */
/*   Q_diag[2] = n_a_sq; */
/*   Q_diag[3] = n_g_sq; */
/*   Q_diag[4] = n_g_sq; */
/*   Q_diag[5] = n_g_sq; */
/*   Q_diag[6] = n_ba_sq; */
/*   Q_diag[7] = n_ba_sq; */
/*   Q_diag[8] = n_ba_sq; */
/*   Q_diag[9] = n_bg_sq; */
/*   Q_diag[10] = n_bg_sq; */
/*   Q_diag[11] = n_bg_sq; */
/*   zeros(factor->Q, 12, 12); */
/*   mat_diag_set(factor->Q, 12, 12, Q_diag); */
/*  */
/*   #<{(| -- Setup relative position, velocity and rotation |)}># */
/*   real_t dr[3] = {0}; */
/*   real_t dv[3] = {0}; */
/*   real_t dC[3 * 3] = {0}; */
/*   real_t ba[3] = {0}; */
/*   real_t bg[3] = {0}; */
/*  */
/*   zeros(factor->dr, 3, 1); */
/*   zeros(factor->dv, 3, 1); */
/*   eye(factor->dC, 3, 3); */
/*  */
/*   ba[0] = sb_i->data[3]; */
/*   ba[1] = sb_i->data[4]; */
/*   ba[2] = sb_i->data[5]; */
/*  */
/*   bg[0] = sb_i->data[6]; */
/*   bg[1] = sb_i->data[7]; */
/*   bg[2] = sb_i->data[8]; */
/*  */
/*   #<{(| Pre-integrate imu measuremenets |)}># */
/*   for (int k = 0; k < imu_buf->size; k++) { */
/*     #<{(| Euler integration |)}># */
/*     const real_t ts_i = imu_buf->ts[k]; */
/*     const real_t ts_j = imu_buf->ts[k + 1]; */
/*     const real_t *a = imu_buf->acc[k]; */
/*     const real_t *w = imu_buf->gyr[k]; */
/*     const real_t a_t[3] = {a[0] - ba[0], a[1] - ba[1], a[2] - ba[2]}; */
/*     const real_t w_t[3] = {w[0] - bg[0], w[1] - bg[1], w[2] - bg[2]}; */
/*  */
/*     #<{(| Propagate IMU state using Euler method |)}># */
/*     const real_t dt = ts_j - ts_i; */
/*     const real_t dt_sq = dt * dt; */
/*  */
/*     #<{(| dr = dr + (dv * dt) + (0.5 * dC * a_t * dt_sq); |)}># */
/*     real_t vel_int[3] = {dv[0], dv[1], dv[2]}; */
/*     vec_scale(vel_int, 3, dt); */
/*  */
/*     real_t acc_dint[3] = {0}; */
/*     dot(dC, 3, 3, a_t, 3, 1, acc_dint); */
/*     vec_scale(acc_dint, 3, 0.5 * dt_sq); */
/*  */
/*     dr[0] += vel_int[0] + acc_dint[0]; */
/*     dr[1] += vel_int[1] + acc_dint[1]; */
/*     dr[2] += vel_int[2] + acc_dint[2]; */
/*  */
/*     #<{(| dv = dv + dC * a_t * dt; |)}># */
/*     real_t dv_update[3] = {0}; */
/*     real_t acc_int[3] = {a_t[0] * dt, a_t[1] * dt, a_t[2] * dt}; */
/*     dot(dC, 3, 3, acc_int, 3, 1, dv_update); */
/*  */
/*     dv[0] += dv_update[0]; */
/*     dv[1] += dv_update[1]; */
/*     dv[2] += dv_update[2]; */
/*  */
/*     #<{(| dC = dC * Exp((w_t) * dt); |)}># */
/*     real_t dC_old[3 * 3] = {0}; */
/*     real_t C_update[3 * 3] = {0}; */
/*     real_t w_int[3] = {w_t[0] * dt, w_t[1] * dt, w_t[2] * dt}; */
/*     mat_copy(dC, 3, 3, dC_old); */
/*     dot(dC_old, 3, 3, C_update, 3, 3, dC); */
/*  */
/*     #<{(| ba = ba; |)}># */
/*     ba[0] = ba[0]; */
/*     ba[1] = ba[1]; */
/*     ba[2] = ba[2]; */
/*  */
/*     #<{(| bg = bg; |)}># */
/*     bg[0] = bg[0]; */
/*     bg[1] = bg[1]; */
/*     bg[2] = bg[2]; */
/*  */
/*     #<{(| Continuous time transition matrix F |)}># */
/*     real_t F[15 * 15] = {0}; */
/*     F[0] = 1.0; */
/*     F[3] = 1.0; */
/*     F[6] = 1.0; */
/*     #<{(| F(0:3, 3:6) = eye(3); |)}># */
/*     #<{(| F(4:6, 7:9) = -dC * skewa_t; |)}># */
/*     #<{(| F(4:6, 10:12) = -dC; |)}># */
/*     #<{(| F(7:9, 7:9) = -skew(w_t); |)}># */
/*     #<{(| F(7:9, 13:15) = -eye(3); |)}># */
/*  */
/*     #<{(| Continuous time input jacobian G |)}># */
/*     real_t G[15 * 12] = {0}; */
/*     G[0] = 1.0; */
/*     #<{(| G(4 : 6, 1 : 3) = -dC; |)}># */
/*     #<{(| G(7 : 9, 4 : 6) = -eye(3); |)}># */
/*     #<{(| G(10 : 12, 7 : 9) = eye(3); |)}># */
/*     #<{(| G(13 : 15, 10 : 12) = eye(3); |)}># */
/*  */
/*     #<{(| Update |)}># */
/*     #<{(| G_dt = G * dt; |)}># */
/*     #<{(| I_F_dt = eye(15) + F * dt; |)}># */
/*     #<{(| factor.state_F = I_F_dt * factor.state_F; |)}># */
/*     #<{(| factor.state_P = |)}># */
/*     #<{(|     I_F_dt * factor.state_P * I_F_dt ' + G_dt * factor.Q * G_dt';
 * |)}># */
/*     #<{(| factor.Dt += dt; |)}># */
/*   } */
/* } */

/**
 * Reset IMU Factor
 */
void imu_factor_reset(imu_factor_t *factor) {
  zeros(factor->r, 15, 1);
  zeros(factor->J0, 2, 6);
  zeros(factor->J1, 2, 9);
  zeros(factor->J2, 2, 6);
  zeros(factor->J3, 2, 9);

  zeros(factor->dr, 3, 1); /* Relative position */
  zeros(factor->dv, 3, 1); /* Relative velocity */
  eye(factor->dC, 3, 3);   /* Relative rotation */
  zeros(factor->ba, 3, 1); /* Accel bias */
  zeros(factor->bg, 3, 1); /* Gyro bias */
}

// GRAPH //////////////////////////////////////////////////////////////////////

void graph_setup(graph_t *graph) {
  assert(graph);
  graph->H = NULL;
  graph->g = NULL;
  graph->x = NULL;
  graph->x_size = 0;
  graph->r_size = 0;
}

void graph_print(graph_t *graph) {
  printf("graph:\n");
  printf("r_size: %d\n", graph->r_size);
  printf("x_size: %d\n", graph->x_size);
}

void graph_evaluator(graph_t *graph,
                     int **param_orders,
                     int *param_sizes,
                     int nb_params,
                     real_t *r,
                     int r_size,
                     real_t **jacs) {
  real_t *H = graph->H;
  int H_size = graph->x_size;
  real_t *g = graph->g;

  for (int i = 0; i < nb_params; i++) {
    int *idx_i = param_orders[i];
    int size_i = param_sizes[i];
    const real_t *J_i = jacs[i];

    real_t *J_i_trans = {0};
    mat_transpose(J_i, r_size, size_i, J_i_trans);

    for (int j = i; j < nb_params; j++) {
      int *idx_j = param_orders[j];
      int size_j = param_sizes[i];
      const real_t *J_j = jacs[j];

      real_t *H_ij = {0};
      dot(J_i_trans, size_i, r_size, J_j, r_size, size_j, H_ij);

      /* Fill Hessian H */
      /* H_ij = J_i' * J_j */
      /* H_ji = H_ij' */
      int stride = H_size;
      int rs = *idx_i;
      int cs = *idx_j;
      int re = rs + size_i;
      int ce = cs + size_j;
      if (i == j) {
        mat_block_set(H, stride, rs, cs, re, ce, H_ij);
      } else {
        real_t *H_ji = {0};
        mat_transpose(H_ij, size_i, size_j, H_ji);
        mat_block_set(H, stride, rs, cs, re, ce, H_ij);
        mat_block_set(H, stride, cs, rs, ce, re, H_ij);
      }

      /* Fill in the R.H.S of H dx = g */
      /* g = -J_i * r */
      mat_scale(J_i_trans, H_size, r_size, -1);
      dot(J_i_trans, H_size, r_size, r, r_size, 1, g);
    }
  }

  /* Update parameter order */
  for (int i = 0; i < nb_params; i++) {
    param_orders[i] = param_orders[i] + param_sizes[i];
  }
}

int graph_eval(graph_t *graph) {
  assert(graph != NULL);

  /* int pose_idx = 0; */
  /* int lmks_idx = graph->nb_poses * 6; */
  /* int exts_idx = lmks_idx + graph->nb_features * 3; */
  /* int cams_idx = exts_idx + graph->nb_exts * 6; */

  /* #<{(| Evaluate camera factors |)}># */
  /* for (int i = 0; i < graph->nb_cam_factors; i++) { */
  /*   cam_factor_t *factor = &graph->cam_factors[i]; */
  /*   #<{(| cam_factor_eval(factor); |)}># */
  /*  */
  /*   int *param_orders[4] = {&pose_idx, &exts_idx, &cams_idx, &lmks_idx}; */
  /*   int param_sizes[4] = {6, 6, 8, 3}; */
  /*   int nb_params = 4; */
  /*  */
  /*   #<{(| graph_evaluator(graph, |)}># */
  /*   #<{(|                 param_orders, |)}># */
  /*   #<{(|                 param_sizes, |)}># */
  /*   #<{(|                 nb_params, |)}># */
  /*   #<{(|                 factor->r, |)}># */
  /*   #<{(|                 factor->r_size, |)}># */
  /*   #<{(|                 factor->jacs); |)}># */
  /* } */

  return 0;
}

/* int graph_optimize(graph_t *graph) { */
/*   struct timespec solve_tic = tic(); */
/*   real_t lambda_k = 1e-4; */
/*  */
/*   int iter = 0; */
/*   int max_iter = 10; */
/*   int verbose = 1; */
/*  */
/*   for (iter = 0; iter < max_iter; iter++) { */
/*     #<{(| Cost k |)}># */
/*     #<{(| x = graph_get_state(graph); |)}># */
/*     #<{(| graph_eval(graph, H, g, &marg_size, &remain_size); |)}># */
/*     #<{(| const matx_t H_diag = (H.diagonal().asDiagonal()); |)}># */
/*     #<{(| H = H + lambda_k * H_diag; |)}># */
/*     #<{(| dx = H.ldlt().solve(g); |)}># */
/*     #<{(| e = graph_residuals(graph); |)}># */
/*     #<{(| cost = 0.5 * e.transpose() * e; |)}># */
/*  */
/*     #<{(| Cost k+1 |)}># */
/*     #<{(| graph_update(graph, dx); |)}># */
/*     #<{(| e = graph_residuals(graph); |)}># */
/*     const real_t cost_k = 0.5 * e.transpose() * e; */
/*  */
/*     const real_t cost_delta = cost_k - cost; */
/*     const real_t solve_time = toc(&solve_tic); */
/*     const real_t iter_time = (iter == 0) ? 0 : (solve_time / iter); */
/*  */
/*     if (verbose) { */
/*       printf("iter[%d] ", iter); */
/*       printf("cost[%.2e] ", cost); */
/*       printf("cost_k[%.2e] ", cost_k); */
/*       printf("cost_delta[%.2e] ", cost_delta); */
/*       printf("lambda[%.2e] ", lambda_k); */
/*       printf("iter_time[%.4f] ", iter_time); */
/*       printf("solve_time[%.4f]  ", solve_time); */
/*       printf("\n"); */
/*  */
/*       // // Calculate reprojection error */
/*       // size_t nb_keypoints = e.size() / 2.0; */
/*       // real_t sse = 0.0; */
/*       // for (size_t i = 0; i < nb_keypoints; i++) { */
/*       //   sse += pow(e.segment(i * 2, 2).norm(), 2); */
/*       // } */
/*       // const real_t rmse = sqrt(sse / nb_keypoints); */
/*       // printf("rmse reproj error: %.2f\n", rmse); */
/*     } */
/*  */
/*     #<{(| Determine whether to accept update |)}># */
/*     if (cost_k < cost) { */
/*       #<{(| Accept update |)}># */
/*       lambda_k /= update_factor; */
/*       cost = cost_k; */
/*     } else { */
/*       #<{(| Reject update |)}># */
/*       #<{(| graph_set_state(graph, x); // Restore state |)}># */
/*       lambda_k *= update_factor; */
/*     } */
/*  */
/*     #<{(| Termination criterias |)}># */
/*     if (fabs(cost_delta) < cost_change_threshold) { */
/*       break; */
/*     } else if ((solve_time + iter_time) > time_limit) { */
/*       break; */
/*     } */
/*   } */
/*  */
/*   #<{(| solve_time = toc(&solve_tic); |)}># */
/*   #<{(| if (verbose) { |)}># */
/*   #<{(|   printf("cost: %.2e\t", cost); |)}># */
/*   #<{(|   printf("graph took: %.4fs\n", solve_time); |)}># */
/*   #<{(| } |)}># */
/* } */

/******************************************************************************
 * DATASET
 ******************************************************************************/

static int
parse_pose_data(const int i, const int j, const char *entry, pose_t *poses) {
  switch (j) {
    /* Timestamp */
    case 0:
      poses[i].ts = strtol(entry, NULL, 0);
      break;
    /* Translation */
    case 1:
    case 2:
    case 3:
      poses[i].pos[j - 1] = strtod(entry, NULL);
      break;
    /* Orientation */
    case 4:
    case 5:
    case 6:
    case 7:
      poses[i].quat[j - 4] = strtod(entry, NULL);
      break;
    default:
      return -1;
  }

  return 0;
}

/**
 * Load poses from file `fp`. The number of poses in file will be outputted to
 * `nb_poses`.
 */
pose_t *load_poses(const char *fp, int *nb_poses) {
  assert(fp != NULL);
  assert(nb_poses != NULL);

  /* Obtain number of rows and columns in dsv data */
  int nb_rows = dsv_rows(fp);
  int nb_cols = dsv_cols(fp, ',');
  if (nb_rows == -1 || nb_cols == -1) {
    return NULL;
  }

  /* Initialize memory for pose data */
  *nb_poses = nb_rows;
  pose_t *poses = malloc(sizeof(pose_t) * nb_rows);

  /* Load file */
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    fclose(infile);
    free(poses);
    return NULL;
  }

  /* Loop through data */
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;

  /* Loop through data line by line */
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    /* Ignore if comment line */
    if (line[0] == '#') {
      continue;
    }

    /* Iterate through values in line separated by commas */
    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        if (parse_pose_data(row_idx, col_idx, entry, poses) != 0) {
          return NULL;
        }
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;

      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  /* Clean up */
  fclose(infile);

  return poses;
}

/**
 * Associate pose data
 */
int **assoc_pose_data(pose_t *gnd_poses,
                      size_t nb_gnd_poses,
                      pose_t *est_poses,
                      size_t nb_est_poses,
                      double threshold,
                      size_t *nb_matches) {
  assert(gnd_poses != NULL);
  assert(est_poses != NULL);
  assert(nb_gnd_poses != 0);
  assert(nb_est_poses != 0);

  size_t gnd_idx = 0;
  size_t est_idx = 0;
  size_t k_end = (nb_gnd_poses > nb_est_poses) ? nb_est_poses : nb_gnd_poses;

  size_t match_idx = 0;
  int **matches = malloc(sizeof(int *) * k_end);

  while ((gnd_idx + 1) < nb_gnd_poses && (est_idx + 1) < nb_est_poses) {
    /* Calculate time difference between ground truth and estimate */
    double gnd_k_time = ts2sec(gnd_poses[gnd_idx].ts);
    double est_k_time = ts2sec(est_poses[est_idx].ts);
    double t_k_diff = fabs(gnd_k_time - est_k_time);

    /* Check to see if next ground truth timestamp forms a smaller time diff */
    double t_kp1_diff = threshold;
    if ((gnd_idx + 1) < nb_gnd_poses) {
      double gnd_kp1_time = ts2sec(gnd_poses[gnd_idx + 1].ts);
      t_kp1_diff = fabs(gnd_kp1_time - est_k_time);
    }

    /* Conditions to call this pair (ground truth and estimate) a match */
    int threshold_met = t_k_diff < threshold;
    int smallest_diff = t_k_diff < t_kp1_diff;

    /* Mark pairs as a match or increment appropriate indicies */
    if (threshold_met && smallest_diff) {
      matches[match_idx] = malloc(sizeof(int) * 2);
      matches[match_idx][0] = gnd_idx;
      matches[match_idx][1] = est_idx;
      match_idx++;

      gnd_idx++;
      est_idx++;

    } else if (gnd_k_time > est_k_time) {
      est_idx++;

    } else if (gnd_k_time < est_k_time) {
      gnd_idx++;
    }
  }

  /* Clean up */
  if (match_idx == 0) {
    free(matches);
    matches = NULL;
  }

  *nb_matches = match_idx;
  return matches;
}

/******************************************************************************
 * SIM
 ******************************************************************************/

// SIM FEATURES ////////////////////////////////////////////////////////////////

/**
 * Load simulation feature data
 */
sim_features_t *load_sim_features(const char *csv_path) {
  sim_features_t *features_data = malloc(sizeof(sim_features_t));
  int nb_rows = 0;
  int nb_cols = 0;
  features_data->features = csv_data(csv_path, &nb_rows, &nb_cols);
  features_data->nb_features = nb_rows;
  return features_data;
}

/**
 * Free simulation feature data
 */
void free_sim_features(sim_features_t *feature_data) {
  /* Pre-check */
  if (feature_data == NULL) {
    return;
  }

  /* Free data */
  for (int i = 0; i < feature_data->nb_features; i++) {
    free(feature_data->features[i]);
  }
  free(feature_data->features);
  free(feature_data);
}

// SIM IMU DATA ////////////////////////////////////////////////////////////////

/**
 * Load simulation imu data
 */
sim_imu_data_t *load_sim_imu_data(const char *csv_path) {
  sim_imu_data_t *imu_data = malloc(sizeof(sim_imu_data_t));

  int nb_rows = 0;
  int nb_cols = 0;
  imu_data->data = csv_data(csv_path, &nb_rows, &nb_cols);
  imu_data->nb_measurements = nb_rows;

  return imu_data;
}

/**
 * Free simulation imu data
 */
void free_sim_imu_data(sim_imu_data_t *imu_data) {
  /* Pre-check */
  if (imu_data == NULL) {
    return;
  }

  /* Free data */
  for (int i = 0; i < imu_data->nb_measurements; i++) {
    free(imu_data->data[i]);
  }
  free(imu_data->data);
  free(imu_data);
}

// SIM CAMERA DATA /////////////////////////////////////////////////////////////

/**
 * Extract timestamp from path
 */
static timestamp_t ts_from_path(const char *path) {
  char fname[128] = {0};
  char fext[128] = {0};
  path_file_name(path, fname);
  path_file_ext(path, fext);

  char ts_str[128] = {0};
  strncpy(ts_str, fname, strlen(fname) - strlen(fext) - 1);

  char *ptr;
  return strtol(ts_str, &ptr, 10);
}

/**
 * Load simulated camera frame
 */
sim_cam_frame_t *load_sim_cam_frame(const char *csv_path) {
  /* Check if file exists */
  if (file_exists(csv_path) == 0) {
    return NULL;
  }

  /* Load csv data */
  int nb_rows = 0;
  int nb_cols = 0;
  real_t **data = csv_data(csv_path, &nb_rows, &nb_cols);

  /* Create sim_cam_frame_t */
  sim_cam_frame_t *frame_data = malloc(sizeof(sim_cam_frame_t));
  frame_data->ts = ts_from_path(csv_path);
  frame_data->feature_ids = malloc(sizeof(int) * nb_rows);
  frame_data->keypoints = malloc(sizeof(real_t *) * nb_rows);
  frame_data->nb_measurements = nb_rows;
  for (int i = 0; i < nb_rows; i++) {
    frame_data->feature_ids[i] = (int) data[i][0];
    frame_data->keypoints[i] = malloc(sizeof(real_t) * 2);
    frame_data->keypoints[i][0] = data[i][1];
    frame_data->keypoints[i][1] = data[i][2];
  }

  /* Clean up */
  csv_free(data, nb_rows);

  return frame_data;
}

/**
 * Print camera frame
 */
void print_sim_cam_frame(sim_cam_frame_t *frame_data) {
  printf("ts: %ld\n", frame_data->ts);
  printf("nb_frames: %d\n", frame_data->nb_measurements);
  for (int i = 0; i < frame_data->nb_measurements; i++) {
    const int feature_id = frame_data->feature_ids[i];
    const real_t *kp = frame_data->keypoints[i];
    printf("- ");
    printf("feature_id: [%d], ", feature_id);
    printf("kp: [%.2f, %.2f]\n", kp[0], kp[1]);
  }
  printf("\n");
}

/**
 * Free simulated camera frame
 */
void free_sim_cam_frame(sim_cam_frame_t *frame_data) {
  /* Pre-check */
  if (frame_data == NULL) {
    return;
  }

  /* Free data */
  free(frame_data->feature_ids);
  for (int i = 0; i < frame_data->nb_measurements; i++) {
    free(frame_data->keypoints[i]);
  }
  free(frame_data->keypoints);
  free(frame_data);
}

/**
 * Load simulated camera data
 */
sim_cam_data_t *load_sim_cam_data(const char *dir_path) {
  assert(dir_path != NULL);

  /* Form csv file path */
  char *csv_path = path_join(dir_path, "/data.csv");
  if (file_exists(csv_path) == 0) {
    free(csv_path);
    return NULL;
  }

  /* Check number of rows */
  const int nb_rows = dsv_rows(csv_path);
  if (nb_rows == 0) {
    free(csv_path);
    return NULL;
  }

  /* Open csv file */
  FILE *csv_file = fopen(csv_path, "r");
  if (csv_file == NULL) {
    free(csv_path);
    return NULL;
  }

  /* Form sim_cam_data_t */
  sim_cam_data_t *cam_data = malloc(sizeof(sim_cam_data_t));
  cam_data->frames = malloc(sizeof(sim_cam_frame_t *) * nb_rows);
  cam_data->nb_frames = nb_rows;
  cam_data->ts = malloc(sizeof(timestamp_t) * nb_rows);
  cam_data->poses = malloc(sizeof(real_t *) * nb_rows);

  int line_idx = 0;
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, csv_file) != NULL) {
    /* Skip line if its a comment */
    if (line[0] == '#') {
      continue;
    }

    /* Parse line */
    timestamp_t ts;
    double r[3] = {0};
    double q[4] = {0};
    sscanf(line,
           "%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
           &ts,
           &r[0],
           &r[1],
           &r[2],
           &q[0],
           &q[1],
           &q[2],
           &q[3]);

    /* Add camera frame to sim_cam_data_t */
    char fname[128] = {0};
    sprintf(fname, "/data/%ld.csv", ts);
    char *frame_csv = path_join(dir_path, fname);
    cam_data->frames[line_idx] = load_sim_cam_frame(frame_csv);
    free(frame_csv);

    /* Add pose to sim_cam_data_t */
    cam_data->ts[line_idx] = ts;
    cam_data->poses[line_idx] = malloc(sizeof(real_t) * 7);
    cam_data->poses[line_idx][0] = r[0];
    cam_data->poses[line_idx][1] = r[1];
    cam_data->poses[line_idx][2] = r[2];
    cam_data->poses[line_idx][3] = q[0];
    cam_data->poses[line_idx][4] = q[1];
    cam_data->poses[line_idx][5] = q[2];
    cam_data->poses[line_idx][6] = q[3];

    /* Update */
    line_idx++;
  }

  /* Clean up */
  free(csv_path);
  fclose(csv_file);

  return cam_data;
}

/**
 * Free simulated camera data
 */
void free_sim_cam_data(sim_cam_data_t *cam_data) {
  /* Pre-check */
  if (cam_data == NULL) {
    return;
  }

  /* Free data */
  for (int k = 0; k < cam_data->nb_frames; k++) {
    free_sim_cam_frame(cam_data->frames[k]);
    free(cam_data->poses[k]);
  }
  free(cam_data->frames);
  free(cam_data->ts);
  free(cam_data->poses);
  free(cam_data);
}

/******************************************************************************
 * GUI
 *****************************************************************************/

// OPENGL UTILS ////////////////////////////////////////////////////////////////

GLfloat gl_deg2rad(const GLfloat d) {
  return d * M_PI / 180.0f;
}

GLfloat gl_rad2deg(const GLfloat r) {
  return r * 180.0f / M_PI;
}

void gl_print_vector(const char *prefix, const GLfloat *x, const int length) {
  printf("%s: [", prefix);
  for (int i = 0; i < length; i++) {
    printf("%f", x[i]);
    if ((i + 1) != length) {
      printf(", ");
    }
  }
  printf("]\n");
}

void gl_print_matrix(const char *prefix,
                     const GLfloat *A,
                     const int nb_rows,
                     const int nb_cols) {
  printf("%s:\n", prefix);
  for (int i = 0; i < nb_rows; i++) {
    for (int j = 0; j < nb_cols; j++) {
      printf("%f", A[i + (j * nb_rows)]);
      if ((j + 1) != nb_cols) {
        printf(", ");
      }
    }
    printf("\n");
  }
  printf("\n");
}

void gl_zeros(GLfloat *A, const int nb_rows, const int nb_cols) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    A[i] = 0.0f;
  }
}

void gl_ones(GLfloat *A, const int nb_rows, const int nb_cols) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    A[i] = 1.0f;
  }
}

void gl_eye(GLfloat *A, const int nb_rows, const int nb_cols) {
  int idx = 0;
  for (int j = 0; j < nb_cols; j++) {
    for (int i = 0; i < nb_rows; i++) {
      A[idx++] = (i == j) ? 1.0f : 0.0f;
    }
  }
}

void gl_vec2f(GLfloat *v, const GLfloat x, const GLfloat y) {
  v[0] = x;
  v[1] = y;
}

void gl_vec3f(GLfloat *v, const GLfloat x, const GLfloat y, const GLfloat z) {
  v[0] = x;
  v[1] = y;
  v[2] = z;
}

void gl_vec4f(GLfloat *v,
              const GLfloat x,
              const GLfloat y,
              const GLfloat z,
              const GLfloat w) {
  v[0] = x;
  v[1] = y;
  v[2] = z;
  v[3] = w;
}

int gl_equals(const GLfloat *A,
              const GLfloat *B,
              const int nb_rows,
              const int nb_cols,
              const GLfloat tol) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    if (fabs(A[i] - B[i]) > tol) {
      return 0;
    }
  }

  return 1;
}

void gl_matf_set(GLfloat *A,
                 const int m,
                 const int n,
                 const int i,
                 const int j,
                 const GLfloat val) {
  UNUSED(n);
  A[i + (j * m)] = val;
}

GLfloat gl_matf_val(
    const GLfloat *A, const int m, const int n, const int i, const int j) {
  UNUSED(n);
  return A[i + (j * m)];
}

void gl_copy(const GLfloat *src, const int m, const int n, GLfloat *dest) {
  for (int i = 0; i < (m * n); i++) {
    dest[i] = src[i];
  }
}

void gl_transpose(const GLfloat *A, size_t m, size_t n, GLfloat *A_t) {
  assert(A != NULL && A != A_t);
  assert(m > 0 && n > 0);

  int idx = 0;
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      A_t[idx++] = gl_matf_val(A, m, n, i, j);
    }
  }
}

void gl_vec3f_cross(const GLfloat u[3], const GLfloat v[3], GLfloat n[3]) {
  assert(u);
  assert(v);
  assert(n);

  n[0] = u[1] * v[2] - u[2] * v[1];
  n[1] = u[2] * v[0] - u[0] * v[2];
  n[2] = u[0] * v[1] - u[1] * v[0];
}

void gl_add(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    C[i] = A[i] + B[i];
  }
}

void gl_sub(const GLfloat *A,
            const GLfloat *B,
            const int nb_rows,
            const int nb_cols,
            GLfloat *C) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    C[i] = A[i] - B[i];
  }
}

void gl_dot(const GLfloat *A,
            const int A_m,
            const int A_n,
            const GLfloat *B,
            const int B_m,
            const int B_n,
            GLfloat *C) {
  assert(A != C && B != C);
  assert(A_n == B_m);

  int m = A_m;
  int n = B_n;

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      for (int k = 0; k < A_n; k++) {
        C[i + (j * n)] += A[i + (k * A_n)] * B[k + (j * B_n)];
      }
    }
  }
}

void gl_scale(GLfloat factor,
              GLfloat *A,
              const int nb_rows,
              const int nb_cols) {
  for (int i = 0; i < (nb_rows * nb_cols); i++) {
    A[i] *= factor;
  }
}

GLfloat gl_norm(const GLfloat *x, const int size) {
  GLfloat sum_sq = 0.0f;
  for (int i = 0; i < size; i++) {
    sum_sq += x[i] * x[i];
  }

  return sqrt(sum_sq);
}

void gl_normalize(GLfloat *x, const int size) {
  const GLfloat n = gl_norm(x, size);
  for (int i = 0; i < size; i++) {
    x[i] /= n;
  }
}

void gl_perspective(const GLfloat fov,
                    const GLfloat aspect,
                    const GLfloat near,
                    const GLfloat far,
                    GLfloat P[4 * 4]) {
  const GLfloat f = 1.0f / tan(fov * 0.5f);

  gl_zeros(P, 4, 4);
  P[0] = f / aspect;
  P[1] = 0.0f;
  P[2] = 0.0f;
  P[3] = 0.0f;

  P[4] = 0.0f;
  P[5] = f;
  P[6] = 0.0f;
  P[7] = 0.0f;

  P[8] = 0.0f;
  P[9] = 0.0f;
  P[10] = (far + near) / (near - far);
  P[11] = -1;

  P[12] = 0.0f;
  P[13] = 0.0f;
  P[14] = (2 * far * near) / (near - far);
  P[15] = 0.0f;
}

void gl_lookat(const GLfloat eye[3],
               const GLfloat at[3],
               const GLfloat up[3],
               GLfloat V[4 * 4]) {
  /* Z-axis: Camera forward */
  GLfloat z[3] = {0};
  gl_sub(at, eye, 3, 1, z);
  gl_normalize(z, 3);

  /* X-axis: Camera right */
  GLfloat x[3] = {0};
  gl_vec3f_cross(z, up, x);
  gl_normalize(x, 3);

  /* Y-axis: Camera up */
  GLfloat y[3] = {0};
  gl_vec3f_cross(x, z, y);

  /* Negate z-axis */
  gl_scale(-1.0f, z, 3, 1);

  /* Form rotation component */
  GLfloat R[4 * 4] = {0};
  R[0] = x[0];
  R[1] = y[0];
  R[2] = z[0];
  R[3] = 0.0f;

  R[4] = x[1];
  R[5] = y[1];
  R[6] = z[1];
  R[7] = 0.0f;

  R[8] = x[2];
  R[9] = y[2];
  R[10] = z[2];
  R[11] = 0.0f;

  R[12] = 0.0f;
  R[13] = 0.0f;
  R[14] = 0.0f;
  R[15] = 1.0f;

  /* Form translation component */
  GLfloat T[4 * 4] = {0};
  T[0] = 1.0f;
  T[1] = 0.0f;
  T[2] = 0.0f;
  T[3] = 0.0f;

  T[4] = 0.0f;
  T[5] = 1.0f;
  T[6] = 0.0f;
  T[7] = 0.0f;

  T[8] = 0.0f;
  T[9] = 0.0f;
  T[10] = 1.0f;
  T[11] = 0.0f;

  T[12] = -eye[0];
  T[13] = -eye[1];
  T[14] = -eye[2];
  T[15] = 1.0f;

  /* Form view matrix */
  gl_zeros(V, 4, 4);
  gl_dot(R, 4, 4, T, 4, 4, V);
}

// SHADER //////////////////////////////////////////////////////////////////////

GLuint shader_compile(const char *shader_src, const int type) {
  if (shader_src == NULL) {
    LOG_ERROR("Shader source is NULL!");
    return GL_FALSE;
  }

  const GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &shader_src, NULL);
  glCompileShader(shader);

  GLint retval = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &retval);
  if (retval == GL_FALSE) {
    char log[9046] = {0};
    glGetShaderInfoLog(shader, 9046, NULL, log);
    LOG_ERROR("Failed to compile shader:\n%s", log);
    return retval;
  }

  return shader;
}

GLuint shaders_link(const GLuint vertex_shader,
                    const GLuint fragment_shader,
                    const GLuint geometry_shader) {
  // Attach shaders to link
  GLuint program = glCreateProgram();
  glAttachShader(program, vertex_shader);
  glAttachShader(program, fragment_shader);
  if (geometry_shader != GL_FALSE) {
    glAttachShader(program, geometry_shader);
  }
  glLinkProgram(program);

  // Link program
  GLint success = 0;
  char log[9046];
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (success == GL_FALSE) {
    glGetProgramInfoLog(program, 9046, NULL, log);
    LOG_ERROR("Failed to link shaders:\nReason: %s\n", log);
    return GL_FALSE;
  }

  // Delete shaders
  glDeleteShader(vertex_shader);
  glDeleteShader(fragment_shader);
  if (geometry_shader == GL_FALSE) {
    glDeleteShader(geometry_shader);
  }

  return program;
}

// GL PROGRAM //////////////////////////////////////////////////////////////////

GLuint gl_prog_setup(const char *vs_src,
                     const char *fs_src,
                     const char *gs_src) {
  GLuint vs = GL_FALSE;
  GLuint fs = GL_FALSE;
  GLuint gs = GL_FALSE;

  if (vs_src) {
    vs = shader_compile(vs_src, GL_VERTEX_SHADER);
  }

  if (fs_src) {
    fs = shader_compile(fs_src, GL_FRAGMENT_SHADER);
  }

  if (gs_src) {
    gs = shader_compile(gs_src, GL_GEOMETRY_SHADER);
  }

  const GLuint program_id = shaders_link(vs, fs, gs);
  return program_id;
}

int gl_prog_set_int(const GLint id, const char *k, const GLint v) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform1i(location, v);
  return 0;
}

int gl_prog_set_vec2i(const GLint id, const char *k, const GLint v[2]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform2i(location, v[0], v[1]);
  return 0;
}

int gl_prog_set_vec3i(const GLint id, const char *k, const GLint v[3]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform3i(location, v[0], v[1], v[2]);
  return 0;
}

int gl_prog_set_vec4i(const GLint id, const char *k, const GLint v[4]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform4i(location, v[0], v[1], v[2], v[3]);
  return 0;
}

int gl_prog_set_float(const GLint id, const char *k, const GLfloat v) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform1f(location, v);
  return 0;
}

int gl_prog_set_vec2f(const GLint id, const char *k, const GLfloat v[2]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform2f(location, v[0], v[1]);
  return 0;
}

int gl_prog_set_vec3f(const GLint id, const char *k, const GLfloat v[3]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform3f(location, v[0], v[1], v[2]);
  return 0;
}

int gl_prog_set_vec4f(const GLint id, const char *k, const GLfloat v[4]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniform4f(location, v[0], v[1], v[2], v[3]);
  return 0;
}

int gl_prog_set_mat2f(const GLint id, const char *k, const GLfloat v[2 * 2]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix2fv(location, 1, GL_FALSE, v);
  return 0;
}

int gl_prog_set_mat3f(const GLint id, const char *k, const GLfloat v[3 * 3]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix3fv(location, 1, GL_FALSE, v);
  return 0;
}

int gl_prog_set_mat4f(const GLint id, const char *k, const GLfloat v[4 * 4]) {
  const GLint location = glGetUniformLocation(id, k);
  if (location == -1) {
    return -1;
  }

  glUniformMatrix4fv(location, 1, GL_FALSE, v);
  return 0;
}

// GL-CAMERA ///////////////////////////////////////////////////////////////////

void gl_camera_setup(gl_camera_t *camera,
                     int *window_width,
                     int *window_height) {
  camera->window_width = window_width;
  camera->window_height = window_height;

  gl_zeros(camera->focal, 3, 1);
  gl_vec3f(camera->world_up, 0.0f, 1.0f, 0.0f);
  gl_vec3f(camera->position, 0.0f, 0.0f, 0.0f);
  gl_vec3f(camera->right, -1.0f, 0.0f, 0.0f);
  gl_vec3f(camera->up, 0.0f, 1.0f, 0.0f);
  gl_vec3f(camera->front, 0.0f, 0.0f, -1.0f);
  camera->yaw = gl_deg2rad(0.0f);
  camera->pitch = gl_deg2rad(0.0f);
  camera->radius = 10.0f;

  camera->fov = gl_deg2rad(60.0f);
  camera->near = 0.1f;
  camera->far = 100.0f;

  gl_camera_update(camera);
}

void gl_camera_update(gl_camera_t *camera) {
  /* Front vector */
  camera->front[0] = sin(camera->yaw) * cos(camera->pitch);
  camera->front[1] = sin(camera->pitch);
  camera->front[2] = cos(camera->yaw) * cos(camera->pitch);
  gl_normalize(camera->front, 3);

  /* Right vector */
  gl_vec3f_cross(camera->front, camera->world_up, camera->right);
  gl_normalize(camera->right, 3);

  /* Up vector */
  gl_vec3f_cross(camera->right, camera->front, camera->up);
  gl_normalize(camera->up, 3);

  /* Projection matrix */
  const float width = (float) *(camera->window_width);
  const float height = (float) *(camera->window_height);
  const float aspect = width / height;
  gl_perspective(camera->fov, aspect, camera->near, camera->far, camera->P);

  /* View matrix */
  GLfloat eye[3] = {0};
  eye[0] = camera->focal[0] + camera->radius * sin(camera->yaw);
  eye[1] = camera->focal[1] + camera->radius * cos(camera->pitch);
  eye[2] = camera->focal[2] + camera->radius * cos(camera->yaw);
  gl_lookat(eye, camera->focal, camera->world_up, camera->V);
}

void gl_camera_rotate(gl_camera_t *camera,
                      const float factor,
                      const float dx,
                      const float dy) {
  /* Update yaw and pitch */
  float pitch = camera->pitch;
  float yaw = camera->yaw;
  yaw -= dx * factor;
  pitch += dy * factor;

  /* Constrain pitch and yaw */
  pitch = (pitch <= (-M_PI / 2.0) + 1e-5) ? (-M_PI / 2.0) + 1e-5 : pitch;
  pitch = (pitch > 0.0) ? 0.0 : pitch;
  yaw = (yaw > M_PI) ? yaw - 2 * M_PI : yaw;
  yaw = (yaw < -M_PI) ? yaw + 2 * M_PI : yaw;

  /* Update camera attitude */
  camera->pitch = pitch;
  camera->yaw = yaw;
  gl_camera_update(camera);
}

void gl_camera_pan(gl_camera_t *camera,
                   const float factor,
                   const float dx,
                   const float dy) {
  /* camera->focal -= (dy * mouse_sensitivity) * camera->front; */
  /* camera->focal += (dx * mouse_sensitivity) * camera->right; */
  const GLfloat dx_scaled = dx * factor;
  const GLfloat dy_scaled = dy * factor;
  GLfloat front[3] = {camera->front[0], camera->front[1], camera->front[2]};
  GLfloat right[3] = {camera->right[0], camera->right[1], camera->right[2]};
  gl_scale(dy_scaled, front, 3, 1);
  gl_scale(dx_scaled, right, 3, 1);
  gl_sub(camera->focal, front, 3, 1, camera->focal);
  gl_add(camera->focal, right, 3, 1, camera->focal);

  /* limit focal point y-axis */
  camera->focal[1] = (camera->focal[1] < 0) ? 0 : camera->focal[1];
  gl_camera_update(camera);
}

void gl_camera_zoom(gl_camera_t *camera,
                    const float factor,
                    const float dx,
                    const float dy) {
  UNUSED(factor);
  UNUSED(dx);

  if (camera->fov >= gl_deg2rad(0.5f) && camera->fov <= gl_deg2rad(90.0f)) {
    camera->fov -= dy * 0.1;
  }

  if (camera->fov <= gl_deg2rad(0.5f)) {
    camera->fov = gl_deg2rad(5.0f);
  } else if (camera->fov >= gl_deg2rad(90.0f)) {
    camera->fov = gl_deg2rad(90.0f);
  }

  gl_camera_update(camera);
}

// GL-PRIMITIVES ///////////////////////////////////////////////////////////////

/* GL CUBE *******************************************************************/

void gl_cube_setup(gl_entity_t *entity, GLfloat pos[3]) {
  // Entity transform
  gl_eye(entity->T, 4, 4);
  entity->T[12] = pos[0];
  entity->T[13] = pos[1];
  entity->T[14] = pos[2];

  /* Shader program */
  char *vs = file_read("./shaders/cube.vert");
  char *fs = file_read("./shaders/cube.frag");
  entity->program_id = gl_prog_setup(vs, fs, NULL);
  free(vs);
  free(fs);
  if (entity->program_id == GL_FALSE) {
    FATAL("Failed to create shaders to draw cube!");
  }

  // Vertices
  // clang-format off
  const float color[3] = {0.9, 0.4, 0.2};
  const float cube_size = 0.5;
  const float r = color[0];
  const float g = color[1];
  const float b = color[2];
  GLfloat vertices[12 * 3 * 6] = {
    // Triangle 1
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    // Triangle 2
    cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 3
    cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 4
    cube_size, cube_size, -cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 5
    -cube_size, -cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 6
    cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, -cube_size, r, g, b,
    // Triangle 7
    -cube_size, cube_size, cube_size, r, g, b,
    -cube_size, -cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b,
    // Triangle 8
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 9
    cube_size, -cube_size, -cube_size, r, g, b,
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b,
    // Triangle 10
    cube_size, cube_size, cube_size, r, g, b,
    cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    // Triangle 11
    cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, -cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    // Triangle 12
    cube_size, cube_size, cube_size, r, g, b,
    -cube_size, cube_size, cube_size, r, g, b,
    cube_size, -cube_size, cube_size, r, g, b
    // Triangle 12 : end
  };
  const size_t nb_triangles = 12;
  const size_t vertices_per_triangle = 3;
  const size_t nb_vertices = vertices_per_triangle * nb_triangles;
  const size_t vertex_buffer_size = sizeof(float) * 6 * nb_vertices;
  // clang-format on

  // VAO
  glGenVertexArrays(1, &entity->vao);
  glBindVertexArray(entity->vao);

  // VBO
  glGenBuffers(1, &entity->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, entity->vbo);
  glBufferData(GL_ARRAY_BUFFER, vertex_buffer_size, vertices, GL_STATIC_DRAW);
  // -- Position attribute
  size_t vertex_size = 6 * sizeof(float);
  void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, pos_offset);
  glEnableVertexAttribArray(0);
  // -- Color attribute
  void *color_offset = (void *) (3 * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, vertex_size, color_offset);
  glEnableVertexAttribArray(1);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
}

void gl_cube_cleanup(const gl_entity_t *entity) {
  glDeleteVertexArrays(1, &entity->vao);
  glDeleteBuffers(1, &entity->vbo);
}

void gl_cube_draw(const gl_entity_t *entity, const gl_camera_t *camera) {
  glUseProgram(entity->program_id);
  gl_prog_set_mat4f(entity->program_id, "projection", camera->P);
  gl_prog_set_mat4f(entity->program_id, "view", camera->V);
  gl_prog_set_mat4f(entity->program_id, "model", entity->T);

  // 12 x 3 indices starting at 0 -> 12 triangles -> 6 squares
  glBindVertexArray(entity->vao);
  glDrawArrays(GL_TRIANGLES, 0, 36);
  glBindVertexArray(0); // Unbind VAO
}

/* GL CAMERA FRAME ***********************************************************/

void gl_camera_frame_setup(gl_entity_t *entity) {
  /* Entity transform */
  gl_eye(entity->T, 4, 4);

  /* Shader program */
  char *vs = file_read("./shaders/camera_frame.vert");
  char *fs = file_read("./shaders/camera_frame.frag");
  entity->program_id = gl_prog_setup(vs, fs, NULL);
  free(vs);
  free(fs);
  if (entity->program_id == GL_FALSE) {
    FATAL("Failed to create shaders to draw cube!");
  }

  // Form the camera fov frame
  GLfloat fov = gl_deg2rad(60.0);
  GLfloat hfov = fov / 2.0f;
  GLfloat scale = 1.0f;
  GLfloat z = scale;
  GLfloat hwidth = z * tan(hfov);
  const GLfloat lb[3] = {-hwidth, hwidth, z};  // Left bottom
  const GLfloat lt[3] = {-hwidth, -hwidth, z}; // Left top
  const GLfloat rt[3] = {hwidth, -hwidth, z};  // Right top
  const GLfloat rb[3] = {hwidth, hwidth, z};   // Right bottom

  // Rectangle frame
  // clang-format off
  const GLfloat vertices[] = {
    // -- Left bottom to left top
    lb[0], lb[1], lb[2], lt[0], lt[1], lt[2],
    // -- Left top to right top
    lt[0], lt[1], lt[2], rt[0], rt[1], rt[2],
    // -- Right top to right bottom
    rt[0], rt[1], rt[2], rb[0], rb[1], rb[2],
    // -- Right bottom to left bottom
    rb[0], rb[1], rb[2], lb[0], lb[1], lb[2],
    // Rectangle frame to origin
    // -- Origin to left bottom
    0.0f, 0.0f, 0.0f, lb[0], lb[1], lb[2],
    // -- Origin to left top
    0.0f, 0.0f, 0.0f, lt[0], lt[1], lt[2],
    // -- Origin to right top
    0.0f, 0.0f, 0.0f, rt[0], rt[1], rt[2],
    // -- Origin to right bottom
    0.0f, 0.0f, 0.0f, rb[0], rb[1], rb[2]
  };
  // clang-format on
  const size_t nb_lines = 8;
  const size_t nb_vertices = nb_lines * 2;
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;

  // VAO
  glGenVertexArrays(1, &entity->vao);
  glBindVertexArray(entity->vao);

  // VBO
  glGenBuffers(1, &entity->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, entity->vbo);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  glVertexAttribPointer(0,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        3 * sizeof(float),
                        (void *) 0);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
}

void gl_camera_frame_cleanup(const gl_entity_t *entity) {
  glDeleteVertexArrays(1, &entity->vao);
  glDeleteBuffers(1, &entity->vbo);
}

void gl_camera_frame_draw(const gl_entity_t *entity,
                          const gl_camera_t *camera) {
  glUseProgram(entity->program_id);
  gl_prog_set_mat4f(entity->program_id, "projection", camera->P);
  gl_prog_set_mat4f(entity->program_id, "view", camera->V);
  gl_prog_set_mat4f(entity->program_id, "model", entity->T);

  // Store original line width
  GLfloat original_line_width = 0.0f;
  glGetFloatv(GL_LINE_WIDTH, &original_line_width);

  // Set line width
  GLfloat line_width = 2.0f;
  glLineWidth(line_width);

  // Draw frame
  const size_t nb_lines = 8;
  const size_t nb_vertices = nb_lines * 2;
  glBindVertexArray(entity->vao);
  glDrawArrays(GL_LINES, 0, nb_vertices);
  glBindVertexArray(0); // Unbind VAO

  // Restore original line width
  glLineWidth(original_line_width);
}

/* GL AXIS FRAME **************************************************************/

void gl_axis_frame_setup(gl_entity_t *entity) {
  /* Entity transform */
  gl_eye(entity->T, 4, 4);

  /* Shader program */
  char *vs = file_read("./shaders/axis_frame.vert");
  char *fs = file_read("./shaders/axis_frame.frag");
  entity->program_id = gl_prog_setup(vs, fs, NULL);
  free(vs);
  free(fs);
  if (entity->program_id == GL_FALSE) {
    FATAL("Failed to create shaders to draw cube!");
  }

  // Vertices
  // clang-format off
  static const GLfloat vertices[] = {
    // Line 1 : x-axis + color
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    // Line 2 : y-axis + color
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    // Line 3 : z-axis + color
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f
  };
  const size_t nb_vertices = 6;
  const size_t buffer_size = sizeof(GLfloat) * 6 * nb_vertices;
  // clang-format on

  // VAO
  glGenVertexArrays(1, &entity->vao);
  glBindVertexArray(entity->vao);

  // VBO
  glGenBuffers(1, &entity->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, entity->vbo);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  // -- Position attribute
  size_t vertex_size = 6 * sizeof(float);
  void *pos_offset = (void *) 0;
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, pos_offset);
  glEnableVertexAttribArray(0);
  // -- Color attribute
  void *color_offset = (void *) (3 * sizeof(float));
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, vertex_size, color_offset);
  glEnableVertexAttribArray(1);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
}

void gl_axis_frame_cleanup(const gl_entity_t *entity) {
  glDeleteVertexArrays(1, &entity->vao);
  glDeleteBuffers(1, &entity->vbo);
}

void gl_axis_frame_draw(const gl_entity_t *entity, const gl_camera_t *camera) {
  glUseProgram(entity->program_id);
  gl_prog_set_mat4f(entity->program_id, "projection", camera->P);
  gl_prog_set_mat4f(entity->program_id, "view", camera->V);
  gl_prog_set_mat4f(entity->program_id, "model", entity->T);

  // Store original line width
  GLfloat original_line_width = 0.0f;
  glGetFloatv(GL_LINE_WIDTH, &original_line_width);

  // Set line width
  GLfloat line_width = 6.0f;
  glLineWidth(line_width);

  // Draw frame
  glBindVertexArray(entity->vao);
  glDrawArrays(GL_LINES, 0, 6);
  glBindVertexArray(0); // Unbind VAO

  // Restore original line width
  glLineWidth(original_line_width);
}

static GLfloat *glgrid_create_vertices(int grid_size) {
  // Allocate memory for vertices
  int nb_lines = (grid_size + 1) * 2;
  int nb_vertices = nb_lines * 2;
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;
  GLfloat *vertices = (GLfloat *) malloc(buffer_size);

  // Setup
  const float min_x = -1.0f * (float) grid_size / 2.0f;
  const float max_x = (float) grid_size / 2.0f;
  const float min_z = -1.0f * (float) grid_size / 2.0f;
  const float max_z = (float) grid_size / 2.0f;

  // Row vertices
  float z = min_z;
  int vert_idx = 0;
  for (int i = 0; i < ((grid_size + 1) * 2); i++) {
    if ((i % 2) == 0) {
      vertices[(vert_idx * 3)] = min_x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = z;
    } else {
      vertices[(vert_idx * 3)] = max_x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = z;
      z += 1.0f;
    }
    vert_idx++;
  }

  // Column vertices
  float x = min_x;
  for (int j = 0; j < ((grid_size + 1) * 2); j++) {
    if ((j % 2) == 0) {
      vertices[(vert_idx * 3)] = x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = min_z;
    } else {
      vertices[(vert_idx * 3)] = x;
      vertices[(vert_idx * 3) + 1] = 0.0f;
      vertices[(vert_idx * 3) + 2] = max_z;
      x += 1.0f;
    }
    vert_idx++;
  }

  return vertices;
}

/* GL GRID ********************************************************************/

void gl_grid_setup(gl_entity_t *entity) {
  /* Entity transform */
  gl_eye(entity->T, 4, 4);

  /* Shader program */
  char *vs = file_read("./shaders/grid.vert");
  char *fs = file_read("./shaders/grid.frag");
  entity->program_id = gl_prog_setup(vs, fs, NULL);
  free(vs);
  free(fs);
  if (entity->program_id == GL_FALSE) {
    FATAL("Failed to create shaders to draw cube!");
  }

  // Create vertices
  const int grid_size = 10;
  const int nb_lines = (grid_size + 1) * 2;
  const int nb_vertices = nb_lines * 2;
  GLfloat *vertices = glgrid_create_vertices(grid_size);
  const size_t buffer_size = sizeof(GLfloat) * nb_vertices * 3;

  // VAO
  glGenVertexArrays(1, &entity->vao);
  glBindVertexArray(entity->vao);

  // VBO
  glGenBuffers(1, &entity->vbo);
  glBindBuffer(GL_ARRAY_BUFFER, entity->vbo);
  glBufferData(GL_ARRAY_BUFFER, buffer_size, vertices, GL_STATIC_DRAW);
  glVertexAttribPointer(0,
                        3,
                        GL_FLOAT,
                        GL_FALSE,
                        3 * sizeof(float),
                        (void *) 0);
  glEnableVertexAttribArray(0);

  // Clean up
  glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbind VBO
  glBindVertexArray(0);             // Unbind VAO
  free(vertices);
}

void gl_grid_cleanup(const gl_entity_t *entity) {
  glDeleteVertexArrays(1, &entity->vao);
  glDeleteBuffers(1, &entity->vbo);
}

void gl_grid_draw(const gl_entity_t *entity, const gl_camera_t *camera) {
  glUseProgram(entity->program_id);
  gl_prog_set_mat4f(entity->program_id, "projection", camera->P);
  gl_prog_set_mat4f(entity->program_id, "view", camera->V);
  gl_prog_set_mat4f(entity->program_id, "model", entity->T);

  const int grid_size = 10;
  const int nb_lines = (grid_size + 1) * 2;
  const int nb_vertices = nb_lines * 2;

  glBindVertexArray(entity->vao);
  glDrawArrays(GL_LINES, 0, nb_vertices);
  glBindVertexArray(0); // Unbind VAO
}

// GUI /////////////////////////////////////////////////////////////////////////

void gui_window_callback(gui_t *gui, const SDL_Event event) {
  if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
    const int width = event.window.data1;
    const int height = event.window.data2;
    gui->screen_width = width;
    gui->screen_height = (height > 0) ? height : 1;
    glViewport(0, 0, (GLsizei) gui->screen_width, (GLsizei) gui->screen_height);
  }
}

void gui_keyboard_callback(gui_t *gui, const SDL_Event event) {
  if (event.type == SDL_KEYDOWN) {
    switch (event.key.keysym.sym) {
      case SDLK_ESCAPE:
        gui->loop = 0;
        break;
      case SDLK_q:
        gui->loop = 0;
        break;
    }
  }
}

void gui_mouse_callback(gui_t *gui, const SDL_Event event) {
  const float x = event.motion.x;
  const float y = event.motion.y;
  const float dx = x - gui->last_cursor_x;
  const float dy = y - gui->last_cursor_y;
  gui->last_cursor_x = x;
  gui->last_cursor_y = y;

  gui->left_click = (event.button.button == SDL_BUTTON_LEFT);
  gui->right_click = (event.button.button == SDL_BUTTON_RIGHT ||
                      event.button.button == SDL_BUTTON_X1);

  if (gui->left_click) {
    /* Rotate camera */
    if (gui->last_cursor_set == 0) {
      gui->last_cursor_set = 1;
    } else if (gui->last_cursor_set) {
      gl_camera_rotate(&gui->camera, gui->mouse_sensitivity, dx, dy);
    }
  } else if (gui->right_click) {
    /* Pan camera */
    if (gui->last_cursor_set == 0) {
      gui->last_cursor_set = 1;
    } else if (gui->last_cursor_set) {
      gl_camera_pan(&gui->camera, gui->mouse_sensitivity, dx, dy);
    }
  } else if (event.wheel.type == SDL_MOUSEWHEEL && event.wheel.y) {
    gl_camera_zoom(&gui->camera, gui->mouse_sensitivity, 0, event.wheel.y);
  } else {
    /* Reset cursor */
    gui->left_click = 0;
    gui->right_click = 0;
    gui->last_cursor_set = 0;
    gui->last_cursor_x = 0.0;
    gui->last_cursor_y = 0.0;
  }
}

void gui_event_handler(gui_t *gui) {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_WINDOWEVENT:
        gui_window_callback(gui, event);
        break;
      case SDL_KEYUP:
      case SDL_KEYDOWN:
        gui_keyboard_callback(gui, event);
        break;
      case SDL_MOUSEMOTION:
      case SDL_MOUSEBUTTONDOWN:
      case SDL_MOUSEBUTTONUP:
      case SDL_MOUSEWHEEL:
        gui_mouse_callback(gui, event);
        break;
    }
  }
}

void gui_setup(gui_t *gui) {
  /* SDL init */
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    FATAL("SDL_Init Error: %s/n", SDL_GetError());
  }

  /* Get display size */
  SDL_DisplayMode disp_mode;
  SDL_GetCurrentDisplayMode(0, &disp_mode);
  const int disp_w = disp_mode.w;
  const int disp_h = disp_mode.h;

  /* Window */
  const char *title = "Hello World!";
  const int w = 640;
  const int h = 480;
  const int x = disp_w / 2 - w / 2;
  const int y = disp_h / 2 - h / 2;
  const uint32_t flags = SDL_WINDOW_OPENGL;
  gui->window = SDL_CreateWindow(title, x, y, w, h, flags);
  if (gui->window == NULL) {
    FATAL("SDL_CreateWindow Error: %s/n", SDL_GetError());
  }
  SDL_SetWindowResizable(gui->window, 1);

  /* OpenGL context */
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
  SDL_GLContext context = SDL_GL_CreateContext(gui->window);
  SDL_GL_SetSwapInterval(1);
  UNUSED(context);

  /* GLEW */
  GLenum err = glewInit();
  if (err != GLEW_OK) {
    FATAL("glewInit failed: %s", glewGetErrorString(err));
  }

  /* Camera */
  gl_camera_setup(&gui->camera, &gui->window_width, &gui->window_height);
  gui->movement_speed = 50.0f;
  gui->mouse_sensitivity = 0.02f;

  /* Cursor */
  gui->left_click = 0;
  gui->right_click = 0;
  gui->last_cursor_set = 0;
  gui->last_cursor_x = 0.0f;
  gui->last_cursor_y = 0.0f;
}

void gui_reset(gui_t *gui) {
  /* Camera */
  gui->movement_speed = 50.0f;
  gui->mouse_sensitivity = 0.02f;

  /* Cursor */
  gui->left_click = 0;
  gui->right_click = 0;
  gui->last_cursor_set = 0;
  gui->last_cursor_x = 0.0f;
  gui->last_cursor_y = 0.0f;
}

void gui_loop(gui_t *gui) {
  gl_entity_t cube;
  GLfloat cube_pos[3] = {0.0, 0.0, 0.0};
  gl_cube_setup(&cube, cube_pos);

  gl_entity_t cube2;
  GLfloat cube2_pos[3] = {2.0, 0.0, 0.0};
  gl_cube_setup(&cube2, cube2_pos);

  gl_entity_t cube3;
  GLfloat cube3_pos[3] = {-2.0, 0.0, 0.0};
  gl_cube_setup(&cube3, cube3_pos);

  gl_entity_t cf;
  gl_camera_frame_setup(&cf);

  gl_entity_t frame;
  gl_axis_frame_setup(&frame);

  gl_entity_t grid;
  gl_grid_setup(&grid);

  gui->loop = 1;
  while (gui->loop) {
    glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    gl_cube_draw(&cube, &gui->camera);
    gl_cube_draw(&cube2, &gui->camera);
    gl_cube_draw(&cube3, &gui->camera);

    gl_camera_frame_draw(&cf, &gui->camera);
    gl_axis_frame_draw(&frame, &gui->camera);
    gl_grid_draw(&grid, &gui->camera);

    gui_event_handler(gui);
    SDL_GL_SwapWindow(gui->window);
    SDL_Delay(1);
  }

  gl_cube_cleanup(&cube);
  gl_cube_cleanup(&cube2);
  gl_cube_cleanup(&cube3);
  gl_camera_frame_cleanup(&cf);
  gl_grid_cleanup(&grid);

  SDL_DestroyWindow(gui->window);
  SDL_Quit();
}

// IMSHOW //////////////////////////////////////////////////////////////////////

void imshow_window_callback(imshow_t *imshow, const SDL_Event event) {
  if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
    SDL_Surface *screen_surface = SDL_GetWindowSurface(imshow->window);

    Uint32 color = SDL_MapRGB(screen_surface->format, 0, 0, 0);
    SDL_FillRect(screen_surface, NULL, color);

    SDL_Rect stretch;
    stretch.x = 0;
    stretch.y = 0;
    stretch.w = event.window.data1;
    stretch.h = event.window.data2;
    SDL_BlitScaled(imshow->image_surface, NULL, screen_surface, &stretch);
  }
}

void imshow_keyboard_callback(imshow_t *imshow, const SDL_Event event) {
  if (event.type == SDL_KEYDOWN) {
    switch (event.key.keysym.sym) {
      case SDLK_ESCAPE:
        imshow->loop = 0;
        break;
      case SDLK_q:
        imshow->loop = 0;
        break;
    }
  }
}

void imshow_event_handler(imshow_t *imshow) {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
      case SDL_WINDOWEVENT:
        imshow_window_callback(imshow, event);
        break;
      case SDL_KEYUP:
      case SDL_KEYDOWN:
        imshow_keyboard_callback(imshow, event);
        break;
    }
  }
}

void draw_circle(SDL_Renderer *renderer,
                 const int cx,
                 const int cy,
                 const int radius,
                 const SDL_Color color) {
  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
  for (int x = cx - radius; x <= cx + radius; x++) {
    for (int y = cy - radius; y <= cy + radius; y++) {
      if ((pow(cy - y, 2) + pow(cx - x, 2)) <= pow(radius, 2)) {
        SDL_RenderDrawPoint(renderer, x, y);
      }
    }
  }
}

/* void draw_circle(SDL_Renderer *renderer, */
/*                  int32_t centreX, */
/*                  int32_t centreY, */
/*                  int32_t radius, */
/*                  const SDL_Color color) { */
/*   SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a); */
/*   const int32_t diameter = (radius * 2); */
/*  */
/*   int32_t x = (radius - 1); */
/*   int32_t y = 0; */
/*   int32_t tx = 1; */
/*   int32_t ty = 1; */
/*   int32_t error = (tx - diameter); */
/*  */
/*   while (x >= y) { */
/*     //  Each of the following renders an octant of the circle */
/*     SDL_RenderDrawPoint(renderer, centreX + x, centreY - y); */
/*     SDL_RenderDrawPoint(renderer, centreX + x, centreY + y); */
/*     SDL_RenderDrawPoint(renderer, centreX - x, centreY - y); */
/*     SDL_RenderDrawPoint(renderer, centreX - x, centreY + y); */
/*     SDL_RenderDrawPoint(renderer, centreX + y, centreY - x); */
/*     SDL_RenderDrawPoint(renderer, centreX + y, centreY + x); */
/*     SDL_RenderDrawPoint(renderer, centreX - y, centreY - x); */
/*     SDL_RenderDrawPoint(renderer, centreX - y, centreY + x); */
/*  */
/*     if (error <= 0) { */
/*       ++y; */
/*       error += ty; */
/*       ty += 2; */
/*     } */
/*  */
/*     if (error > 0) { */
/*       --x; */
/*       tx += 2; */
/*       error += (tx - diameter); */
/*     } */
/*   } */
/* } */

void imshow_setup(imshow_t *im, const char *fp) {
  /* SDL init */
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    FATAL("SDL_Init Error: %s/n", SDL_GetError());
  }

  /* Load image */
  im->image_surface = IMG_Load(fp);
  if (im->image_surface == NULL) {
    FATAL("Failed to load image [%s]!", fp);
  }
  const int img_w = im->image_surface->w;
  const int img_h = im->image_surface->h;

  /* Get display size */
  SDL_DisplayMode disp_mode;
  SDL_GetCurrentDisplayMode(0, &disp_mode);
  const int disp_w = disp_mode.w;
  const int disp_h = disp_mode.h;

  /* Create window */
  const int x = disp_w / 2 - img_w / 2;
  const int y = disp_h / 2 - img_h / 2;
  const int w = img_w;
  const int h = img_h;
  if (SDL_CreateWindowAndRenderer(w, h, 0, &im->window, &im->renderer) != 0) {
    FATAL("Failed to create window: %s\n", SDL_GetError());
  }
  SDL_SetWindowTitle(im->window, im->window_title);
  SDL_SetWindowPosition(im->window, x, y);
  SDL_SetWindowResizable(im->window, 1);

  /* Clear render */
  SDL_SetRenderDrawColor(im->renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
  SDL_RenderClear(im->renderer);

  /* Show image */
  SDL_Texture *texture =
      SDL_CreateTextureFromSurface(im->renderer, im->image_surface);
  SDL_RenderCopy(im->renderer, texture, NULL, NULL);
  SDL_RenderPresent(im->renderer);

  /* Draw circles */
  const int x_min = 0;
  const int y_min = 0;
  const int x_max = img_w;
  const int y_max = img_h;
  for (int i = 0; i < 200; i++) {
    const int x = (rand() % (x_max + 1 - x_min)) + x_min;
    const int y = (rand() % (y_max + 1 - y_min)) + y_min;
    const int radius = 5;
    SDL_Color color;
    color.r = 255;
    color.g = 0;
    color.b = 0;
    color.a = 255;
    draw_circle(im->renderer, x, y, radius, color);
  }
  SDL_RenderPresent(im->renderer);

  /* Cursor */
  im->left_click = 0;
  im->right_click = 0;
  im->last_cursor_set = 0;
  im->last_cursor_x = 0.0f;
  im->last_cursor_y = 0.0f;
}

void imshow_reset(imshow_t *imshow) {
  /* Camera */
  imshow->movement_speed = 50.0f;
  imshow->mouse_sensitivity = 0.02f;

  /* Cursor */
  imshow->left_click = 0;
  imshow->right_click = 0;
  imshow->last_cursor_set = 0;
  imshow->last_cursor_x = 0.0f;
  imshow->last_cursor_y = 0.0f;
}

void imshow_loop(imshow_t *imshow) {
  imshow->loop = 1;
  while (imshow->loop) {
    imshow_event_handler(imshow);
    SDL_UpdateWindowSurface(imshow->window);
    SDL_Delay(1);
  }

  SDL_DestroyWindow(imshow->window);
  SDL_Quit();
}
