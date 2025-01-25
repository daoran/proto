#include "data.h"

/**
 * String copy from `src` to `dst`.
 */
size_t string_copy(char *dst, const char *src) {
  dst[0] = '\0';
  memcpy(dst, src, strlen(src));
  dst[strlen(src)] = '\0'; // Null terminate
  return strlen(dst);
}

/**
 * Copy a substring from `src` to `dst` where `s` and `n` are the start index
 * and length.
 */
void string_subcopy(char *dst, const char *src, const int s, const int n) {
  assert(s >= 0);
  assert(n > 0);

  dst[0] = '\0';
  for (int i = 0; i < n; i++) {
    dst[i] = src[s + i];
  }
}

/**
 * Concatenate string from `src` to `dst`.
 */
void string_cat(char *dst, const char *src) {
  size_t dst_len = strlen(dst);
  strcat(dst + dst_len, src);
  dst[dst_len + strlen(src)] = '\0'; // strncat does not null terminate
}

/**
 * Allocate heap memory for string `s`.
 */
char *string_malloc(const char *s) {
  assert(s != NULL);
  char *retval = MALLOC(char, strlen(s) + 1);
  memcpy(retval, s, strlen(s));
  retval[strlen(s)] = '\0'; // Null terminate
  return retval;
}

/**
 * Strip whitespace from string `s`.
 */
char *string_strip(char *s) {
  char *end;

  // Trim leading space
  while (*s == ' ') {
    s++;
  }

  if (*s == 0) { // All spaces?
    return s;
  }

  // Trim trailing space
  end = s + strlen(s) - 1;
  while (end > s && (*end == ' ' || *end == '\n')) {
    end--;
  }

  // Write new null terminator character
  end[1] = '\0';

  return s;
}

/**
 * Strip specific character `c` from string `s`.
 */
char *string_strip_char(char *s, const char c) {
  char *end;

  // Trim leading space
  while (*s == c) {
    s++;
  }

  if (*s == 0) { // All spaces?
    return s;
  }

  // Trim trailing space
  end = s + strlen(s) - 1;
  while (end > s && *end == c) {
    end--;
  }

  // Write new null terminator character
  end[1] = '\0';

  return s;
}

/**
 * Split string `s` by delimiter `d`
 */
char **string_split(char *a_str, const char a_delim, size_t *n) {
  char **result = 0;
  char *tmp = a_str;
  char *last_comma = 0;
  char delim[2];
  delim[0] = a_delim;
  delim[1] = 0;

  /* Count how many elements will be extracted. */
  while (*tmp) {
    if (a_delim == *tmp) {
      (*n)++;
      last_comma = tmp;
    }
    tmp++;
  }

  /* Add space for trailing token. */
  *n += last_comma < (a_str + strlen(a_str) - 1);

  /* Add space for terminating null string so caller
     knows where the list of returned strings ends. */
  (*n)++;

  result = malloc(sizeof(char *) * *n);

  if (result) {
    size_t idx = 0;
    char *token = strtok(a_str, delim);

    while (token) {
      assert(idx < *n);
      *(result + idx++) = strdup(token);
      token = strtok(0, delim);
    }
    assert(idx == *n - 1);
    *(result + idx) = 0;
  }

  // Return results
  (*n)--;

  return result;
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
        data = CALLOC(int, array_size + 1);
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
int **load_iarrays(const char *csv_path, int *num_arrays) {
  assert(csv_path != NULL);
  FILE *csv_file = fopen(csv_path, "r");
  *num_arrays = dsv_rows(csv_path);
  int **array = CALLOC(int *, *num_arrays);

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
static double *parse_darray_line(char *line) {
  assert(line != NULL);
  char entry[MAX_LINE_LENGTH] = {0};
  int index = 0;
  double *data = NULL;

  for (size_t i = 0; i < strlen(line); i++) {
    char c = line[i];
    if (c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      if (data == NULL) {
        size_t array_size = strtod(entry, NULL);
        data = CALLOC(double, array_size);
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
 * Parse 2D real arrays from csv file at `csv_path`, on success `num_arrays`
 * will return number of arrays.
 * @returns
 * - List of 1D vector of reals
 * - NULL for failure
 */
double **load_darrays(const char *csv_path, int *num_arrays) {
  assert(csv_path != NULL);
  assert(num_arrays != NULL);
  FILE *csv_file = fopen(csv_path, "r");
  *num_arrays = dsv_rows(csv_path);
  double **array = CALLOC(double *, *num_arrays);

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
  int *i = MALLOC(int, 1);
  *i = val;
  return i;
}

/**
 * Allocate heap memory for float `val`.
 */
float *float_malloc(const float val) {
  float *f = MALLOC(float, 1);
  *f = val;
  return f;
}

/**
 * Allocate heap memory for double `val`.
 */
double *double_malloc(const double val) {
  double *d = MALLOC(double, 1);
  *d = val;
  return d;
}

/**
 * Allocate heap memory for vector `vec` with length `N`.
 */
double *vector_malloc(const double *vec, const double N) {
  double *retval = MALLOC(double, N);
  for (int i = 0; i < N; i++) {
    retval[i] = vec[i];
  }
  return retval;
}

/**
 * Get number of rows in a delimited file at `fp`.
 * @returns
 * - Number of rows
 * - -1 for failure
 */
int dsv_rows(const char *fp) {
  assert(fp != NULL);

  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    return -1;
  }

  // Loop through lines
  int num_rows = 0;
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      num_rows++;
    }
  }

  // Cleanup
  fclose(infile);

  return num_rows;
}

/**
 * Get number of columns in a delimited file at `fp`.
 * @returns
 * - Number of columns
 * - -1 for failure
 */
int dsv_cols(const char *fp, const char delim) {
  assert(fp != NULL);

  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    return -1;
  }

  // Get line that isn't the header
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      break;
    }
  }

  // Parse line to obtain number of elements
  int num_elements = 1;
  int found_separator = 0;
  for (size_t i = 0; i < MAX_LINE_LENGTH; i++) {
    if (line[i] == delim) {
      found_separator = 1;
      num_elements++;
    }
  }

  // Cleanup
  fclose(infile);

  return (found_separator) ? num_elements : -1;
}

/**
 * Get the fields of the delimited file at `fp`, where `delim` is the value
 * separated symbol and `num_fields` returns the length of the fields returned.
 * @returns
 * - List of field strings
 * - NULL for failure
 */
char **dsv_fields(const char *fp, const char delim, int *num_fields) {
  assert(fp != NULL);

  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    return NULL;
  }

  // Get last header line
  char field_line[MAX_LINE_LENGTH] = {0};
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    if (line[0] != '#') {
      break;
    } else {
      memcpy(field_line, line, strlen(line));
    }
  }

  // Parse fields
  *num_fields = dsv_cols(fp, delim);
  char **fields = MALLOC(char *, *num_fields);
  int field_idx = 0;
  char field_name[100] = {0};

  for (size_t i = 0; i < strlen(field_line); i++) {
    char c = field_line[i];

    // Ignore # and ' '
    if (c == '#' || c == ' ') {
      continue;
    }

    if (c == ',' || c == '\n') {
      // Add field name to fields
      fields[field_idx] = string_malloc(field_name);
      memset(field_name, '\0', 100);
      field_idx++;
    } else {
      // Append field name
      field_name[strlen(field_name)] = c;
    }
  }

  // Cleanup
  fclose(infile);

  return fields;
}

/**
 * Load delimited separated value data as a matrix.
 * @returns
 * - Matrix of DSV data
 * - NULL for failure
 */
double **
dsv_data(const char *fp, const char delim, int *num_rows, int *num_cols) {
  assert(fp != NULL);

  // Obtain number of rows and columns in dsv data
  *num_rows = dsv_rows(fp);
  *num_cols = dsv_cols(fp, delim);
  if (*num_rows == -1 || *num_cols == -1) {
    return NULL;
  }

  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    return NULL;
  }

  // Loop through data
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;

  // Loop through data line by line
  double **data = MALLOC(double *, *num_rows);
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    // Ignore if comment line
    if (line[0] == '#') {
      continue;
    }

    // Iterate through values in line separated by commas
    data[row_idx] = MALLOC(double, *num_cols);
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

  // Clean up
  fclose(infile);

  return data;
}

/**
 * Free DSV data.
 */
void dsv_free(double **data, const int num_rows) {
  assert(data != NULL);
  for (int i = 0; i < num_rows; i++) {
    free(data[i]);
  }
  free(data);
}

/**
 * Load comma separated data as a matrix, where `fp` is the csv file path, on
 * success `num_rows` and `num_cols` will be filled.
 * @returns
 * - Matrix of CSV data
 * - NULL for failure
 */
double **csv_data(const char *fp, int *num_rows, int *num_cols) {
  assert(fp != NULL);
  return dsv_data(fp, ',', num_rows, num_cols);
}

/**
 * Free CSV data.
 */
void csv_free(double **data, const int num_rows) {
  for (int i = 0; i < num_rows; i++) {
    free(data[i]);
  }
  free(data);
}

/**
 * Extract filename from `path` to `fname`.
 */
void path_file_name(const char *path, char *fname) {
  assert(path != NULL);
  assert(fname != NULL);

  char path_copy[9046] = {0};
  memcpy(path_copy, path, strlen(path));

  char *base = strrchr(path_copy, '/');
  base = base ? base + 1 : path_copy;

  memcpy(fname, base, strlen(base));
}

/**
 * Extract file extension from `path` to `fext`.
 */
void path_file_ext(const char *path, char *fext) {
  assert(path != NULL);
  assert(fext != NULL);

  char path_copy[9046] = {0};
  memcpy(path_copy, path, strlen(path));

  char *base = strrchr(path_copy, '.');
  if (base) {
    base = base ? base + 1 : path_copy;
    memcpy(fext, base, strlen(base));
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
  memcpy(path_copy, path, strlen(path));

  char *base = strrchr(path_copy, '/');
  memcpy(dir_name, path_copy, base - path_copy);
}

/**
 * Join two paths `x` and `y`
 */
char *path_join(const char *x, const char *y) {
  assert(x != NULL && y != NULL);

  char *retval = NULL;
  if (x[strlen(x) - 1] == '/') {
    retval = MALLOC(char, (strlen(x) + strlen(y)) + 1);
    string_copy(retval, x);
    string_copy(retval + strlen(retval), (y[0] == '/') ? y + 1 : y);
  } else {
    retval = MALLOC(char, (strlen(x) + strlen(y)) + 2);
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
  int num_files = scandir(path, &namelist, 0, alphasort);
  if (num_files < 0) {
    return NULL;
  }

  // The first two are '.' and '..'
  free(namelist[0]);
  free(namelist[1]);

  // Allocate memory for list of files
  char **files = MALLOC(char *, num_files - 2);
  *n = 0;

  // Create list of files
  for (int i = 2; i < num_files; i++) {
    char fp[9046] = {0};
    const char *c = (path[strlen(path) - 1] == '/') ? "" : "/";
    string_cat(fp, path);
    string_cat(fp, c);
    string_cat(fp, namelist[i]->d_name);

    files[*n] = MALLOC(char, strlen(fp) + 1);
    memcpy(files[*n], fp, strlen(fp));
    files[*n][strlen(fp)] = '\0'; // strncpy does not null terminate
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

  char *buf = MALLOC(char, len + 1);
  if (buf == NULL) {
    fclose(f);
    return NULL;
  }
  const ssize_t read = fread(buf, 1, len, f);
  if (read != len) {
    FATAL("Failed to read file [%s]\n", fp);
  }
  buf[len] = '\0';
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
int file_exists(const char *fp) { return (access(fp, F_OK) == 0) ? 1 : 0; }

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
    return -1;
  }

  // Obtain number of lines
  int num_rows = 0;
  char *line = NULL;
  size_t len = 0;
  while (getline(&line, &len, file) != -1) {
    num_rows++;
  }
  free(line);

  // Clean up
  fclose(file);

  return num_rows;
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
    return -1;
  }

  FILE *dst_file = fopen(dst, "wb");
  if (dst_file == NULL) {
    fclose(src_file);
    return -2;
  }

  char *line = NULL;
  size_t len = 0;
  ssize_t read = 0;
  while ((read = getline(&line, &len, src_file)) != -1) {
    fwrite(line, sizeof(char), read, dst_file);
  }
  if (line) {
    free(line);
  }

  // Clean up
  fclose(src_file);
  fclose(dst_file);

  return 0;
}

