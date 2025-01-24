#include "fs.h"

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

