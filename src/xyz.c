#include "xyz.h"

/*******************************************************************************
 * DATA
 ******************************************************************************/

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

/*******************************************************************************
 * TIME
 ******************************************************************************/

/**
 * Tic, start timer.
 * @returns A timespec encapsulating the time instance when tic() is called
 */
struct timespec tic(void) {
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
timestamp_t time_now(void) {
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);

  const time_t sec = spec.tv_sec;
  const long int ns = spec.tv_nsec;
  const uint64_t BILLION = 1000000000L;

  return (uint64_t) sec * BILLION + (uint64_t) ns;
}

/**
 * Convert string to timestamp
 */
timestamp_t str2ts(const char *ts_str) { return strtoll(ts_str, NULL, 10); }

/**
 * Convert timestamp to seconds
 */
double ts2sec(const timestamp_t ts) { return ts * 1e-9; }

/**
 * Convert seconds to timestamp
 */
timestamp_t sec2ts(const double time_s) { return time_s * 1e9; }

/*******************************************************************************
 * NETWORK
 ******************************************************************************/

/**
 * Return IP and Port info from socket file descriptor `sockfd` to `ip` and
 * `port`. Returns `0` for success and `-1` for failure.
 * @returns
 * - 0 for success
 * - -1 for failure
 */
status_t ip_port_info(const int sockfd, char *ip, int *port) {
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
  memcpy(ip, ipstr, strlen(ipstr));
  ip[strlen(ip)] = '\0';

  return 0;
}

/**
 * Configure TCP server
 */
status_t tcp_server_setup(tcp_server_t *server, const int port) {
  assert(server != NULL);

  // Setup server struct
  server->port = port;
  server->sockfd = -1;
  server->conn = -1;

  // Create socket
  server->sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (server->sockfd == -1) {
    LOG_ERROR("Socket creation failed...");
    return -1;
  }

  // Socket options
  const int en = 1;
  const size_t int_sz = sizeof(int);
  if (setsockopt(server->sockfd, SOL_SOCKET, SO_REUSEADDR, &en, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEADDR) failed");
  }
  if (setsockopt(server->sockfd, SOL_SOCKET, SO_REUSEPORT, &en, int_sz) < 0) {
    LOG_ERROR("setsockopt(SO_REUSEPORT) failed");
  }

  // Assign IP, PORT
  struct sockaddr_in addr;
  bzero(&addr, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(server->port);

  // Bind newly created socket to given IP
  int retval = bind(server->sockfd, (struct sockaddr *) &addr, sizeof(addr));
  if (retval != 0) {
    LOG_ERROR("Socket bind failed: %s", strerror(errno));
    return -1;
  }

  return 0;
}

/**
 * Loop TCP server
 * @returns `0` for success, `-1` for failure
 */
status_t tcp_server_loop(tcp_server_t *server) {
  assert(server != NULL);

  // Server is ready to listen
  if ((listen(server->sockfd, 5)) != 0) {
    LOG_ERROR("Listen failed...");
    return -1;
  }

  // Accept the data packet from client and verification
  DEBUG("Server ready!");
  while (1) {
    // Accept incomming connections
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
status_t tcp_client_setup(tcp_client_t *client,
                          const char *server_ip,
                          const int server_port) {
  assert(client != NULL);
  assert(server_ip != NULL);

  // Setup client struct
  string_copy(client->server_ip, server_ip);
  client->server_port = server_port;
  client->sockfd = -1;

  // Create socket
  client->sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (client->sockfd == -1) {
    LOG_ERROR("Socket creation failed!");
    return -1;
  }

  // Assign IP, PORT
  struct sockaddr_in server = {.sin_family = AF_INET,
                               .sin_addr.s_addr = inet_addr(client->server_ip),
                               .sin_port = htons(client->server_port)};

  // Connect to server
  if (connect(client->sockfd, (struct sockaddr *) &server, sizeof(server)) !=
      0) {
    LOG_ERROR("Failed to connect to server!");
    return -1;
  }
  DEBUG("Connected to the server!");

  return 0;
}

/**
 * Loop TCP client
 */
status_t tcp_client_loop(tcp_client_t *client) {
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

/*******************************************************************************
 * MATH
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
 * Generate random vector of size n where each element is between a and b.
 */
void randvec(const real_t a, const real_t b, const size_t n, real_t *v) {
  for (size_t i = 0; i < n; i++) {
    v[i] = randf(a, b);
  }
}

/**
 * Degrees to radians.
 * @returns Radians
 */
real_t deg2rad(const real_t d) { return d * (M_PI / 180.0); }

/**
 * Radians to degrees.
 * @returns Degrees
 */
real_t rad2deg(const real_t r) { return r * (180.0 / M_PI); }

/**
 * Wrap angle `d` in degrees to +- 180 degrees.
 */
real_t wrap_180(const real_t d) {
  real_t x = fmod(d + 180, 360);
  if (x < 0) {
    x += 360;
  }

  return x - 180;
}

/**
 * Wrap angle `d` in degrees to 0 to 360 degrees.
 */
real_t wrap_360(const real_t d) {
  real_t x = fmod(d, 360);
  if (x < 0) {
    x += 360;
  }
  return x;
}

/**
 * Wrap angle `r` in radians to +- pi radians.
 */
real_t wrap_pi(const real_t r) { return deg2rad(wrap_180(rad2deg(r))); }

/**
 * Wrap angle `r` in radians to 0 to 2pi radians.
 */
real_t wrap_2pi(const real_t r) { return deg2rad(wrap_360(rad2deg(r))); }

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
 * Check if reals are equal.
 * @returns 1 if x == y, 0 if x != y.
 */
int flteqs(const real_t x, const real_t y) {
  return (fltcmp(x, y) == 0) ? 1 : 0;
}

/**
 * Check if strings are equal.
 * @returns 1 if x == y, 0 if x != y.
 */
int streqs(const char *x, const char *y) { return (strcmp(x, y) == 0) ? 1 : 0; }

/**
 * Cumulative Sum.
 */
void cumsum(const real_t *x, const size_t n, real_t *s) {
  s[0] = x[0];
  for (size_t i = 1; i < n; i++) {
    s[i] = x[i];
    s[i] = s[i] + s[i - 1];
  }
}

/**
 * Logspace. Generates `n` points between decades `10^a` and `10^b`.
 */
void logspace(const real_t a, const real_t b, const size_t n, real_t *x) {
  const real_t h = (b - a) / (n - 1);

  real_t c = a;
  for (size_t i = 0; i < n; i++) {
    x[i] = pow(10, c);
    c += h;
  }
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
 * Clip value `x` to be between `val_min` and `val_max`.
 */
real_t clip_value(const real_t x, const real_t vmin, const real_t vmax) {
  real_t x_tmp = x;
  x_tmp = (x_tmp > vmax) ? vmax : x_tmp;
  x_tmp = (x_tmp < vmin) ? vmin : x_tmp;
  return x_tmp;
}

/**
 * Clip vector `x` to be between `val_min` and `val_max`.
 */
void clip(real_t *x, const size_t n, const real_t vmin, const real_t vmax) {
  for (size_t i = 0; i < n; i++) {
    x[i] = (x[i] > vmax) ? vmax : x[i];
    x[i] = (x[i] < vmin) ? vmin : x[i];
  }
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

  // Make a copy of the original input vector x
  real_t *vals = MALLOC(real_t, n);
  for (size_t i = 0; i < n; i++) {
    vals[i] = x[i];
  }

  // Sort the values
  qsort(vals, n, sizeof(real_t), fltcmp2);

  // Get median value
  real_t median_value = 0.0;
  if ((n % 2) == 0) {
    const int bwd_idx = (int) (n - 1) / 2.0;
    const int fwd_idx = (int) (n + 1) / 2.0;
    median_value = (vals[bwd_idx] + vals[fwd_idx]) / 2.0;
  } else {
    const int midpoint_idx = n / 2.0;
    median_value = vals[midpoint_idx];
  }

  // Clean up
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

/*******************************************************************************
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
      // printf("%.4f  ", A[idx]);
      printf("%e  ", A[idx]);
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

  printf("%s: ", prefix);
  for (size_t i = 0; i < n; i++) {
    printf("%e ", v[i]);
    // printf("%f ", v[i]);
    // printf("%.4f ", v[i]);
    // printf("%.10f ", v[i]);
  }
  printf("\n");
}

/**
 * Print float array.
 */
void print_float_array(const char *prefix, const float *arr, const size_t n) {
  assert(prefix != NULL);
  assert(arr != NULL);
  assert(n != 0);

  printf("%s: ", prefix);
  for (size_t i = 0; i < n; i++) {
    printf("%.4f ", arr[i]);
  }
  printf("\n");
}

/**
 * Print double array.
 */
void print_double_array(const char *prefix, const double *arr, const size_t n) {
  assert(prefix != NULL);
  assert(arr != NULL);
  assert(n != 0);

  printf("%s: ", prefix);
  for (size_t i = 0; i < n; i++) {
    printf("%.4f ", arr[i]);
  }
  printf("\n");
}

/**
 * Convert vector string
 */
void vec2str(const real_t *v, const int n, char *s) {
  s[0] = '[';
  for (int i = 0; i < n; i++) {
    sprintf(s + strlen(s), "%f", v[i]);
    if (i < (n - 1)) {
      strcat(s + strlen(s), ", ");
    }
  }
  strcat(s + strlen(s), "]");
}

/**
 * Convert vector string
 */
void vec2csv(const real_t *v, const int n, char *s) {
  for (int i = 0; i < n; i++) {
    sprintf(s + strlen(s), "%f", v[i]);
    if (i < (n - 1)) {
      strcat(s + strlen(s), ", ");
    }
  }
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
 * Create skew-symmetric matrix `A` from a 3x1 vector `x`.
 */
void hat(const real_t x[3], real_t A[3 * 3]) {
  assert(x != NULL);
  assert(A != NULL);

  // First row
  A[0] = 0.0;
  A[1] = -x[2];
  A[2] = x[1];

  // Second row
  A[3] = x[2];
  A[4] = 0.0;
  A[5] = -x[0];

  // Third row
  A[6] = -x[1];
  A[7] = x[0];
  A[8] = 0.0;
}

/**
 * Opposite of the skew-symmetric matrix
 */
void vee(const real_t A[3 * 3], real_t x[3]) {
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
 * Enforce semi-positive definite. This function assumes the matrix `A` is
 * square where number of rows `m` and columns `n` is equal, and symmetric.
 */
void enforce_spd(real_t *A, const int m, const int n) {
  assert(A != NULL);
  assert(m == n);

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      const real_t a = A[(i * n) + j];
      const real_t b = A[(j * n) + i];
      A[(i * n) + j] = (a + b) / 2.0;
    }
  }
}

/**
 * Malloc matrix of size `m x n`.
 */
real_t *mat_malloc(const size_t m, const size_t n) {
  assert(m > 0);
  assert(n > 0);
  return CALLOC(real_t, m * n);
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
 * @returns 1 if A == B or 0 if A != B
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
        return 0;
      }
      index++;
    }
  }

  return 1;
}

// /**
//  * Save matrix `A` of size `m x n` to `save_path`.
//  * @returns `0` for success, `-1` for failure
//  */
// int mat_save(const char *save_path, const real_t *A, const int m, const int n) {
//   assert(save_path != NULL);
//   assert(A != NULL);
//   assert(m > 0);
//   assert(n > 0);
//
//   FILE *csv_file = fopen(save_path, "w");
//   if (csv_file == NULL) {
//     return -1;
//   }
//
//   int idx = 0;
//   for (int i = 0; i < m; i++) {
//     for (int j = 0; j < n; j++) {
//       fprintf(csv_file, "%.18e", A[idx]);
//       idx++;
//       if ((j + 1) != n) {
//         fprintf(csv_file, ",");
//       }
//     }
//     fprintf(csv_file, "\n");
//   }
//   fclose(csv_file);
//
//   return 0;
// }
//
// /**
//  * Load matrix from file in `mat_path`, on success `num_rows` and `num_cols`
//  * will be set respectively.
//  */
// real_t *mat_load(const char *mat_path, int *num_rows, int *num_cols) {
//   assert(mat_path != NULL);
//   assert(num_rows != NULL);
//   assert(num_cols != NULL);
//
//   // Obtain number of rows and columns in csv data
//   *num_rows = dsv_rows(mat_path);
//   *num_cols = dsv_cols(mat_path, ',');
//   if (*num_rows == -1 || *num_cols == -1) {
//     return NULL;
//   }
//
//   // Initialize memory for csv data
//   real_t *A = MALLOC(real_t, *num_rows * *num_cols);
//
//   // Load file
//   FILE *infile = fopen(mat_path, "r");
//   if (infile == NULL) {
//     free(A);
//     return NULL;
//   }
//
//   // Loop through data
//   char line[MAX_LINE_LENGTH] = {0};
//   int row_idx = 0;
//   int col_idx = 0;
//   int idx = 0;
//
//   // Loop through data line by line
//   while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
//     // Ignore if comment line
//     if (line[0] == '#') {
//       continue;
//     }
//
//     // Iterate through values in line separated by commas
//     char entry[100] = {0};
//     for (size_t i = 0; i < strlen(line); i++) {
//       char c = line[i];
//       if (c == ' ') {
//         continue;
//       }
//
//       if (c == ',' || c == '\n') {
//         A[idx] = strtod(entry, NULL);
//         idx++;
//
//         memset(entry, '\0', sizeof(char) * 100);
//         col_idx++;
//       } else {
//         entry[strlen(entry)] = c;
//       }
//     }
//
//     col_idx = 0;
//     row_idx++;
//   }
//
//   // Clean up
//   fclose(infile);
//
//   return A;
// }

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
  for (size_t i = 0; i < stride; i++) {
    A[(stride * row_idx) + i] = x[vec_idx++];
  }
}

/**
 * Set matrix column.
 */
void mat_col_set(real_t *A,
                 const size_t stride,
                 const int num_rows,
                 const int col_idx,
                 const real_t *x) {
  int vec_idx = 0;
  for (int i = 0; i < num_rows; i++) {
    A[i * stride + col_idx] = x[vec_idx++];
  }
}

/**
 * Get matrix column.
 */
void mat_col_get(const real_t *A,
                 const int m,
                 const int n,
                 const int col_idx,
                 real_t *x) {
  int vec_idx = 0;
  for (int i = 0; i < m; i++) {
    x[vec_idx++] = A[i * n + col_idx];
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
                   const size_t re,
                   const size_t cs,
                   const size_t ce,
                   real_t *block) {
  assert(A != NULL);
  assert(block != NULL);
  assert(A != block);
  assert(stride != 0);

  size_t idx = 0;
  for (size_t i = rs; i <= re; i++) {
    for (size_t j = cs; j <= ce; j++) {
      // block[idx] = mat_val(A, stride, i, j);
      block[idx] = A[(i * stride) + j];
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
                   const size_t re,
                   const size_t cs,
                   const size_t ce,
                   const real_t *block) {
  assert(A != NULL);
  assert(block != NULL);
  assert(A != block);
  assert(stride != 0);

  size_t idx = 0;
  for (size_t i = rs; i <= re; i++) {
    for (size_t j = cs; j <= ce; j++) {
      A[(i * stride) + j] = block[idx];
      idx++;
    }
  }
}

/**
 * Add to matrix sub-block in `A` with `block` from row and column start `rs`
 * and `cs`, to row and column end `re` and `ce`.
 */
void mat_block_add(real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t re,
                   const size_t cs,
                   const size_t ce,
                   const real_t *block) {
  assert(A != NULL);
  assert(block != NULL);
  assert(A != block);
  assert(stride != 0);

  size_t idx = 0;
  for (size_t i = rs; i <= re; i++) {
    for (size_t j = cs; j <= ce; j++) {
      A[(i * stride) + j] += block[idx];
      idx++;
    }
  }
}

/**
 * Subtract matrix sub-block in `A` with `block` from row and column start `rs`
 * and `cs`, to row and column end `re` and `ce`.
 */
void mat_block_sub(real_t *A,
                   const size_t stride,
                   const size_t rs,
                   const size_t re,
                   const size_t cs,
                   const size_t ce,
                   const real_t *block) {
  assert(A != NULL);
  assert(block != NULL);
  assert(A != block);
  assert(stride != 0);

  size_t idx = 0;
  for (size_t i = rs; i <= re; i++) {
    for (size_t j = cs; j <= ce; j++) {
      A[(i * stride) + j] -= block[idx];
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
      A_t[(j * m) + i] = A[(i * n) + j];
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
 * Copy 3x3 matrix from `src` to `dst`.
 */
void mat3_copy(const real_t src[3 * 3], real_t dst[3 * 3]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];

  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];

  dst[6] = src[6];
  dst[7] = src[7];
  dst[8] = src[8];
}

/**
 * Add 3x3 matrix `A + B = C`.
 */
void mat3_add(const real_t A[3 * 3], const real_t B[3 * 3], real_t C[3 * 3]) {
  C[0] = A[0] + B[0];
  C[1] = A[1] + B[1];
  C[2] = A[2] + B[2];

  C[3] = A[3] + B[3];
  C[4] = A[4] + B[4];
  C[5] = A[5] + B[5];

  C[6] = A[6] + B[6];
  C[7] = A[7] + B[7];
  C[8] = A[8] + B[8];
}

/**
 * Subtract 3x3 matrix `A - B = C`.
 */
void mat3_sub(const real_t A[3 * 3], const real_t B[3 * 3], real_t C[3 * 3]) {
  C[0] = A[0] - B[0];
  C[1] = A[1] - B[1];
  C[2] = A[2] - B[2];

  C[3] = A[3] - B[3];
  C[4] = A[4] - B[4];
  C[5] = A[5] - B[5];

  C[6] = A[6] - B[6];
  C[7] = A[7] - B[7];
  C[8] = A[8] - B[8];
}

/**
 * Create new vector of length `n` in heap memory.
 * @returns Heap allocated vector
 */
real_t *vec_malloc(const real_t *x, const size_t n) {
  assert(n > 0);
  real_t *vec = CALLOC(real_t, n);
  for (size_t i = 0; i < n; i++) {
    vec[i] = x[i];
  }

  return vec;
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
 * Get minimal value in vector `x` of length `n`.
 */
real_t vec_min(const real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  real_t y = x[0];
  for (size_t i = 1; i < n; i++) {
    y = (x[i] < y) ? x[i] : y;
  }

  return y;
}

/**
 * Get minimal value in vector `x` of length `n`.
 */
real_t vec_max(const real_t *x, const size_t n) {
  assert(x != NULL);
  assert(n > 0);

  real_t y = x[0];
  for (size_t i = 1; i < n; i++) {
    y = (x[i] > y) ? x[i] : y;
  }

  return y;
}

/**
 * Get minimal, maximum value in vector `x` of length `n` as `vmin`, `vmax` as well as the range `r`.
 */
void vec_range(const real_t *x,
               const size_t n,
               real_t *vmin,
               real_t *vmax,
               real_t *r) {
  assert(x != NULL);
  assert(n > 0);
  assert(vmin != NULL);
  assert(vmax != NULL);
  assert(r != NULL);

  *vmin = x[0];
  *vmax = x[0];
  for (size_t i = 1; i < n; i++) {
    *vmin = (x[i] < *vmin) ? x[i] : *vmin;
    *vmax = (x[i] > *vmax) ? x[i] : *vmax;
  }
  *r = vmax - vmin;
}

// /**
//  * Load vector.
//  *
//  * @param vec_path Path to csv containing vector values
//  * @param m Number of rows
//  * @param n Number of cols
//  *
//  * @returns Vector or `NULL` for failure
//  */
// real_t *vec_load(const char *vec_path, int *m, int *n) {
//   assert(vec_path != NULL);
//   assert(m != NULL);
//   assert(n != NULL);
//
//   // Obtain number of rows and columns in csv data
//   *m = dsv_rows(vec_path);
//   *n = dsv_cols(vec_path, ',');
//   if (*m > 0 && *n == -1) {
//     // Load file
//     FILE *infile = fopen(vec_path, "r");
//     if (infile == NULL) {
//       return NULL;
//     }
//
//     // Loop through data line by line
//     char line[MAX_LINE_LENGTH] = {0};
//     while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
//       // Ignore if comment line
//       if (line[0] == '#') {
//         continue;
//       }
//
//       if (strlen(line) == 0) {
//         fclose(infile);
//         return NULL;
//       }
//     }
//
//     *n = 1;
//   } else if (*m == -1 || *n == -1) {
//     return NULL;
//   }
//
//   // Initialize memory for csv data
//   real_t *x = MALLOC(real_t, *m * *n);
//
//   // Load file
//   FILE *infile = fopen(vec_path, "r");
//   if (infile == NULL) {
//     free(x);
//     return NULL;
//   }
//
//   // Loop through data
//   char line[MAX_LINE_LENGTH] = {0};
//   int row_idx = 0;
//   int col_idx = 0;
//   int idx = 0;
//
//   // Loop through data line by line
//   while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
//     // Ignore if comment line
//     if (line[0] == '#') {
//       continue;
//     }
//
//     // Iterate through values in line separated by commas
//     char entry[100] = {0};
//     for (size_t i = 0; i < strlen(line); i++) {
//       char c = line[i];
//       if (c == ' ') {
//         continue;
//       }
//
//       if (c == ',' || c == '\n') {
//         x[idx] = strtod(entry, NULL);
//         idx++;
//
//         memset(entry, '\0', sizeof(char) * 100);
//         col_idx++;
//       } else {
//         entry[strlen(entry)] = c;
//       }
//     }
//
//     col_idx = 0;
//     row_idx++;
//   }
//
//   // Clean up
//   fclose(infile);
//
//   return x;
// }

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
 * Copy vector of size 3 from `src` to `dst`.
 */
void vec3_copy(const real_t src[3], real_t dst[3]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
}

/**
 * Add vector of size 3 `x + y = z`.
 */
void vec3_add(const real_t x[3], const real_t y[3], real_t z[3]) {
  z[0] = x[0] + y[0];
  z[1] = x[1] + y[1];
  z[2] = x[2] + y[2];
}

/**
 * Subtract vector of size 3 `x - y = z`.
 */
void vec3_sub(const real_t x[3], const real_t y[3], real_t z[3]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
}

/**
 * Cross product between vector `a` and `b`, output is written to `c`.
 */
void vec3_cross(const real_t a[3], const real_t b[3], real_t c[3]) {
  assert(a != b);
  assert(a != c);

  // cx = ay * bz - az * by
  // cy = az * bx - ax * bz
  // cz = ax * by - ay * bx
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

/**
 * Calculate the norm of vector `x` of size 3.
 */
real_t vec3_norm(const real_t x[3]) {
  return sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
}

/**
 * Normalize vector `x` of size 3.
 */
void vec3_normalize(real_t x[3]) {
  const real_t n = vec3_norm(x);
  x[0] = x[0] / n;
  x[1] = x[1] / n;
  x[2] = x[2] / n;
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

#ifdef USE_CBLAS
#if PRECISION == 1
  cblas_sgemm(CblasRowMajor, // Matrix data arrangement
              CblasNoTrans,  // Transpose A
              CblasNoTrans,  // Transpose B
              A_m,           // Number of rows in A and C
              B_n,           // Number of cols in B and C
              A_n,           // Number of cols in A
              1.0,           // Scaling factor for the product of A and B
              A,             // Matrix A
              A_n,           // First dimension of A
              B,             // Matrix B
              B_n,           // First dimension of B
              0.0,           // Scale factor for C
              C,             // Output
              B_n);          // First dimension of C
#elif PRECISION == 2
  cblas_dgemm(CblasRowMajor, // Matrix data arrangement
              CblasNoTrans,  // Transpose A
              CblasNoTrans,  // Transpose B
              A_m,           // Number of rows in A and C
              B_n,           // Number of cols in B and C
              A_n,           // Number of cols in A
              1.0,           // Scaling factor for the product of A and B
              A,             // Matrix A
              A_n,           // First dimension of A
              B,             // Matrix B
              B_n,           // First dimension of B
              0.0,           // Scale factor for C
              C,             // Output
              B_n);          // First dimension of C
#endif
#else
  size_t m = A_m;
  size_t n = B_n;

  memset(C, 0, sizeof(real_t) * A_m * B_n);
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      for (size_t k = 0; k < A_n; k++) {
        C[(i * n) + j] += A[(i * A_n) + k] * B[(k * B_n) + j];
      }
    }
  }

#endif
}

/**
 * Dot product of two matrices or vectors `A` and `B` of size `A_m x A_n` and
 * `B_m x B_n`. Results are written to `C`.
 */
void dot3(const real_t *A,
          const size_t A_m,
          const size_t A_n,
          const real_t *B,
          const size_t B_m,
          const size_t B_n,
          const real_t *C,
          const size_t C_m,
          const size_t C_n,
          real_t *D) {
  real_t *AB = MALLOC(real_t, A_m * B_n);
  dot(A, A_m, A_n, B, B_m, B_n, AB);
  dot(AB, A_m, B_m, C, C_m, C_n, D);
  free(AB);
}

/**
 * Mulitply Y = X' * A * X
 */
void dot_XtAX(const real_t *X,
              const size_t X_m,
              const size_t X_n,
              const real_t *A,
              const size_t A_m,
              const size_t A_n,
              real_t *Y) {
  assert(X != NULL);
  assert(A != NULL);
  assert(Y != NULL);
  assert(X_m == A_m);

  real_t *XtA = MALLOC(real_t, (X_m * A_m));
  real_t *Xt = MALLOC(real_t, (X_m * X_n));

  mat_transpose(X, X_m, X_n, Xt);
  dot(Xt, X_n, X_m, A, A_m, A_n, XtA);
  dot(XtA, X_m, A_m, Xt, X_n, X_m, Y);

  free(Xt);
  free(XtA);
}

/**
 * Mulitply Y = X * A * X'
 */
void dot_XAXt(const real_t *X,
              const size_t X_m,
              const size_t X_n,
              const real_t *A,
              const size_t A_m,
              const size_t A_n,
              real_t *Y) {
  assert(X != NULL);
  assert(A != NULL);
  assert(Y != NULL);
  assert(X_n == A_m);

  real_t *Xt = MALLOC(real_t, (X_m * X_n));
  real_t *XA = MALLOC(real_t, (X_m * A_n));

  mat_transpose(X, X_m, X_n, Xt);
  dot(X, X_m, X_n, A, A_m, A_n, XA);
  dot(XA, X_m, A_n, Xt, X_n, X_m, Y);

  free(XA);
  free(Xt);
}

/**
 * Invert a block diagonal matrix.
 */
void bdiag_inv(const real_t *A, const int m, const int bs, real_t *A_inv) {
  real_t *A_sub = MALLOC(real_t, bs * bs);
  real_t *A_sub_inv = MALLOC(real_t, bs * bs);
  zeros(A_inv, m, m);

  for (int idx = 0; idx < m; idx += bs) {
    const int rs = idx;
    const int re = idx + bs - 1;
    const int cs = idx;
    const int ce = idx + bs - 1;
    mat_block_get(A, m, rs, re, cs, ce, A_sub);

    // Invert using SVD
    // pinv(A_sub, bs, bs, A_sub_inv);

    // Inverse using Eigen-decomp
    eig_inv(A_sub, bs, bs, 0, A_sub_inv);

    mat_block_set(A_inv, m, rs, re, cs, ce, A_sub_inv);
  }

  free(A_sub);
  free(A_sub_inv);
}

/**
 * Invert a sub block diagonal matrix.
 */
void bdiag_inv_sub(const real_t *A,
                   const int stride,
                   const int m,
                   const int bs,
                   real_t *A_inv) {
  real_t *A_sub = MALLOC(real_t, bs * bs);
  real_t *A_sub_inv = MALLOC(real_t, bs * bs);
  zeros(A_inv, m, m);

  for (int idx = 0; idx < m; idx += bs) {
    const int rs = idx;
    const int re = idx + bs - 1;
    const int cs = idx;
    const int ce = idx + bs - 1;
    mat_block_get(A, stride, rs, re, cs, ce, A_sub);

    // Invert using SVD
    // pinv(A_sub, bs, bs, A_sub_inv);

    // Inverse using Eigen-decomp
    eig_inv(A_sub, bs, bs, 0, A_sub_inv);

    mat_block_set(A_inv, m, rs, re, cs, ce, A_sub_inv);
  }

  free(A_sub);
  free(A_sub_inv);
}

/**
 * Dot product of A * x = b, where A is a block diagonal matrix, x is a vector
 * and b is the result.
 */
void bdiag_dot(const real_t *A,
               const int m,
               const int n,
               const int bs,
               const real_t *x,
               real_t *b) {
  real_t *A_sub = MALLOC(real_t, bs * bs);
  real_t *x_sub = MALLOC(real_t, bs);

  for (int idx = 0; idx < m; idx += bs) {
    const int rs = idx;
    const int re = idx + bs - 1;
    const int cs = idx;
    const int ce = idx + bs - 1;
    mat_block_get(A, m, rs, re, cs, ce, A_sub);
    vec_copy(x + rs, bs, x_sub);
    dot(A_sub, bs, bs, x_sub, bs, 1, b + rs);
  }

  free(A_sub);
  free(x_sub);
}

/**
 * Check inverted matrix A by multiplying by its inverse.
 * @returns `0` for succces, `-1` for failure.
 */
int check_inv(const real_t *A, const real_t *A_inv, const int m) {
  const real_t tol = 1e-2;
  real_t *inv_check = CALLOC(real_t, m * m);
  dot(A, m, m, A_inv, m, m, inv_check);
  // print_matrix("inv_check", inv_check, m, m);
  // gnuplot_matshow(inv_check, m, m);
  // exit(0);

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < m; j++) {
      const real_t target = (i == j) ? 1.0 : 0.0;
      const real_t val = inv_check[i * m + j];
      const real_t diff = fabs(val - target);

      if ((diff > tol) != 0) {
        printf("[%d, %d] got %e, diff: %e\n", i, j, val, diff);
        free(inv_check);
        return -1;
      }
    }
  }

  free(inv_check);
  return 0;
}

/**
 * Return the linear least squares norm.
 */
real_t check_Axb(const real_t *A,
                 const real_t *x,
                 const real_t *b,
                 const int m,
                 const int n) {
  real_t *b_est = MALLOC(real_t, m);
  real_t *diff = MALLOC(real_t, m);
  real_t r_sq = 0.0;

  dot(A, m, n, x, n, 1, b_est);
  vec_sub(b_est, b, diff, m);
  dot(diff, 1, m, diff, m, 1, &r_sq);

  free(b_est);
  free(diff);

  return sqrt(r_sq);
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

  // Check if any of the values are beyond the tol
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < n; j++) {
      if (fabs(mat_val(delta, n, i, j)) >= tol) {
        ok = 0;
      }
    }
  }

  // Print result
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
      print_matrix("analytical jac", jac, m, n);
      print_matrix("num diff jac", fdiff, m, n);
      print_matrix("difference matrix", delta, m, n);
    }
    retval = 0;
  }

  // Clean up
  free(delta);

  return retval;
}

/////////
// SVD //
/////////

#ifdef USE_LAPACK

// LAPACK fortran prototypes
extern void sgesdd_(char *jobz,
                    int *m,
                    int *n,
                    float *a,
                    int *lda,
                    float *s,
                    float *u,
                    int *ldu,
                    float *vt,
                    int *ldvt,
                    float *work,
                    int *lwork,
                    int *iwork,
                    int *info);
extern void dgesdd_(char *jobz,
                    int *m,
                    int *n,
                    double *a,
                    int *lda,
                    double *s,
                    double *u,
                    int *ldu,
                    double *vt,
                    int *ldvt,
                    double *work,
                    int *lwork,
                    int *iwork,
                    int *info);

/**
 * Decompose matrix A with SVD
 */
int __lapack_svd(real_t *A, int m, int n, real_t *s, real_t *U, real_t *Vt) {
  // Transpose matrix A because LAPACK is column major
  real_t *At = MALLOC(real_t, m * n);
  mat_transpose(A, m, n, At);

  // Query and allocate optimal workspace
  int lda = m;
  int lwork = -1;
  int info = 0;
  real_t work_size;
  real_t *work = &work_size;
  int num_sv = (m < n) ? m : n;
  int *iwork = MALLOC(int, 8 * num_sv);

#if PRECISION == 1
  sgesdd_("A", &m, &n, At, &lda, s, U, &m, Vt, &n, work, &lwork, iwork, &info);
#else
  dgesdd_("A", &m, &n, At, &lda, s, U, &m, Vt, &n, work, &lwork, iwork, &info);
#endif
  lwork = work_size;
  work = MALLOC(real_t, lwork);

  // Compute SVD
#if PRECISION == 1
  sgesdd_("A", &m, &n, At, &lda, s, U, &m, Vt, &n, work, &lwork, iwork, &info);
#else
  dgesdd_("A", &m, &n, At, &lda, s, U, &m, Vt, &n, work, &lwork, iwork, &info);
#endif
  if (info > 0) {
    LOG_ERROR("Failed to compute svd!\n");
  }

  // Clean up
  free(At);
  free(iwork);
  free(work);

  return (info == 0) ? 0 : -1;
}

#endif // USE_LAPACK

/**
 * Singular Value Decomposition
 *
 * Given a matrix A of size m x n, compute the singular value decomposition of
 * A = U * W * Vt, where the input A is replaced by U, the diagonal matrix W is
 * output as a vector w of size n, and the matrix V (not V transpose) is of
 * size n x n.
 *
 * Source (Singular-Value-Decomposition: page 59-70):
 *
 *   Press, William H., et al. "Numerical recipes in C++." The art of
 *   scientific computing 2 (2007): 1002.
 *
 * @returns 0 for success, -1 for failure
 */
int __svd(real_t *A, const int m, const int n, real_t *w, real_t *V) {
  int flag, i, its, j, jj, k, l, nm;
  double anorm, c, f, g, h, s, scale, x, y, z, *rv1;
  l = 0;

  rv1 = MALLOC(double, n);
  if (rv1 == NULL) {
    printf("svd(): Unable to allocate vector\n");
    return (-1);
  }

  g = scale = anorm = 0.0;
  for (i = 0; i < n; i++) {
    l = i + 1;
    rv1[i] = scale * g;
    g = s = scale = 0.0;
    if (i < m) {
      for (k = i; k < m; k++) {
        scale += fabs(A[k * n + i]);
      }
      if (scale) {
        for (k = i; k < m; k++) {
          A[k * n + i] /= scale;
          s += A[k * n + i] * A[k * n + i];
        }
        f = A[i * n + i];
        g = -SIGN2(sqrt(s), f);
        h = f * g - s;
        A[i * n + i] = f - g;
        for (j = l; j < n; j++) {
          for (s = 0.0, k = i; k < m; k++) {
            s += A[k * n + i] * A[k * n + j];
          }
          f = s / h;
          for (k = i; k < m; k++) {
            A[k * n + j] += f * A[k * n + i];
          }
        }
        for (k = i; k < m; k++) {
          A[k * n + i] *= scale;
        }
      }
    }
    w[i] = scale * g;
    g = s = scale = 0.0;
    if (i < m && i != n - 1) {
      for (k = l; k < n; k++) {
        scale += fabs(A[i * n + k]);
      }
      if (scale) {
        for (k = l; k < n; k++) {
          A[i * n + k] /= scale;
          s += A[i * n + k] * A[i * n + k];
        }
        f = A[i * n + l];
        g = -SIGN2(sqrt(s), f);
        h = f * g - s;
        A[i * n + l] = f - g;
        for (k = l; k < n; k++) {
          rv1[k] = A[i * n + k] / h;
        }
        for (j = l; j < m; j++) {
          for (s = 0.0, k = l; k < n; k++) {
            s += A[j * n + k] * A[i * n + k];
          }
          for (k = l; k < n; k++) {
            A[j * n + k] += s * rv1[k];
          }
        }
        for (k = l; k < n; k++) {
          A[i * n + k] *= scale;
        }
      }
    }
    anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
  }

  for (i = n - 1; i >= 0; i--) {
    if (i < n - 1) {
      if (g) {
        for (j = l; j < n; j++) {
          V[j * n + i] = (A[i * n + j] / A[i * n + l]) / g;
        }
        for (j = l; j < n; j++) {
          for (s = 0.0, k = l; k < n; k++) {
            s += A[i * n + k] * V[k * n + j];
          }
          for (k = l; k < n; k++) {
            V[k * n + j] += s * V[k * n + i];
          }
        }
      }
      for (j = l; j < n; j++) {
        V[i * n + j] = V[j * n + i] = 0.0;
      }
    }
    V[i * n + i] = 1.0;
    g = rv1[i];
    l = i;
  }

  for (i = MIN(m, n) - 1; i >= 0; i--) {
    l = i + 1;
    g = w[i];
    for (j = l; j < n; j++) {
      A[i * n + j] = 0.0;
    }
    if (g) {
      g = 1.0 / g;
      for (j = l; j < n; j++) {
        for (s = 0.0, k = l; k < m; k++) {
          s += A[k * n + i] * A[k * n + j];
        }
        f = (s / A[i * n + i]) * g;
        for (k = i; k < m; k++) {
          A[k * n + j] += f * A[k * n + i];
        }
      }
      for (j = i; j < m; j++) {
        A[j * n + i] *= g;
      }
    } else
      for (j = i; j < m; j++) {
        A[j * n + i] = 0.0;
      }
    ++A[i * n + i];
  }

  for (k = n - 1; k >= 0; k--) {
    for (its = 0; its < 30; its++) {
      flag = 1;
      for (l = k; l >= 0; l--) {
        nm = l - 1;
        if ((fabs(rv1[l]) + anorm) == anorm) {
          flag = 0;
          break;
        }
        if ((fabs(w[nm]) + anorm) == anorm) {
          break;
        }
      }
      if (flag) {
        c = 0.0;
        s = 1.0;
        for (i = l; i <= k; i++) {
          f = s * rv1[i];
          rv1[i] = c * rv1[i];
          if ((fabs(f) + anorm) == anorm) {
            break;
          }
          g = w[i];
          h = pythag(f, g);
          w[i] = h;
          h = 1.0 / h;
          c = g * h;
          s = -f * h;
          for (j = 0; j < m; j++) {
            y = A[j * n + nm];
            z = A[j * n + i];
            A[j * n + nm] = y * c + z * s;
            A[j * n + i] = z * c - y * s;
          }
        }
      }
      z = w[k];
      if (l == k) {
        if (z < 0.0) {
          w[k] = -z;
          for (j = 0; j < n; j++) {
            V[j * n + k] = -V[j * n + k];
          }
        }
        break;
      }
      if (its == 29) {
        printf("no convergence in 30 svd iterations\n");
      }
      x = w[l];
      nm = k - 1;
      y = w[nm];
      g = rv1[nm];
      h = rv1[k];
      f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
      g = pythag(f, 1.0);
      f = ((x - z) * (x + z) + h * ((y / (f + SIGN2(g, f))) - h)) / x;
      c = s = 1.0;
      for (j = l; j <= nm; j++) {
        i = j + 1;
        g = rv1[i];
        y = w[i];
        h = s * g;
        g = c * g;
        z = pythag(f, h);
        rv1[j] = z;
        c = f / z;
        s = h / z;
        f = x * c + g * s;
        g = g * c - x * s;
        h = y * s;
        y *= c;
        for (jj = 0; jj < n; jj++) {
          x = V[jj * n + j];
          z = V[jj * n + i];
          V[jj * n + j] = x * c + z * s;
          V[jj * n + i] = z * c - x * s;
        }
        z = pythag(f, h);
        w[j] = z;
        if (z) {
          z = 1.0 / z;
          c = f * z;
          s = h * z;
        }
        f = c * g + s * y;
        x = c * y - s * g;
        for (jj = 0; jj < m; jj++) {
          y = A[jj * n + j];
          z = A[jj * n + i];
          A[jj * n + j] = y * c + z * s;
          A[jj * n + i] = z * c - y * s;
        }
      }
      rv1[l] = 0.0;
      rv1[k] = f;
      w[k] = x;
    }
  }
  free(rv1);

  return 0;
}

/**
 * Decompose matrix A with SVD
 */
int svd(const real_t *A,
        const int m,
        const int n,
        real_t *U,
        real_t *s,
        real_t *V) {
#ifdef USE_LAPACK
  real_t *A_copy = MALLOC(real_t, m * n);
  real_t *U_ = MALLOC(real_t, m * m);
  real_t *Ut_ = MALLOC(real_t, m * m);
  real_t *Vt = MALLOC(real_t, n * n);

  mat_copy(A, m, n, A_copy);
  const int retval = __lapack_svd(A_copy, m, n, s, U_, Vt);

  mat_transpose(U_, m, m, Ut_);
  mat_block_get(Ut_, m, 0, m - 1, 0, n - 1, U);
  mat_copy(Vt, n, n, V);

  free(U_);
  free(Ut_);
  free(Vt);
  free(A_copy);

  return retval;
#else
  mat_copy(A, m, n, U);
  return __svd(U, m, n, s, V);
#endif // USE_LAPACK
}

/**
 * Pseudo inverse of matrix A with SVD
 */
void pinv(const real_t *A, const int m, const int n, real_t *A_inv) {
  assert(m == n);

  // Decompose A = U * S * Vt
  const int diag_size = (m < n) ? m : n;
  real_t *s = CALLOC(real_t, diag_size);
  real_t *U = CALLOC(real_t, m * n);
  real_t *V = CALLOC(real_t, n * n);
  svd(A, m, n, U, s, V);

  // Form Sinv diagonal matrix
  real_t *Si = CALLOC(real_t, m * n);
  zeros(Si, n, m);
  for (int idx = 0; idx < m; idx++) {
    const int diag_idx = idx * n + idx;

    if (s[idx] > 1e-24) {
      Si[diag_idx] = 1.0 / s[idx];
    } else {
      Si[diag_idx] = 0.0;
    }
  }

  // A_inv = Vt * Si * U
  // real_t *V_Si = MALLOC(real_t, m * m);
  // zeros(A_inv, m, n);
  // dot(V, m, n, Si, n, m, V_Si);
  // dot(V_Si, m, m, Ut, m, n, A_inv);

  // A_inv = U * Si * Ut
  real_t *Ut = CALLOC(real_t, m * n);
  real_t *Si_Ut = CALLOC(real_t, diag_size * n);
  mat_transpose(U, m, n, Ut);
  dot(Si, diag_size, diag_size, Ut, m, n, Si_Ut);
  dot(V, m, n, Si_Ut, diag_size, n, A_inv);

  // Clean up
  free(s);
  free(U);
  free(V);

  free(Si);

  free(Ut);
  free(Si_Ut);
}

/**
 * Use SVD to find the determinant of matrix `A` of size `m` x `n`.
 *
 * WARNING: This method assumes the matrix `A` is invertible, additionally the
 * returned determinant is the **absolute** value, the sign is not returned.
 */
int svd_det(const real_t *A, const int m, const int n, real_t *det) {
  assert(m == n);

  // Decompose matrix A with SVD
  const int k = (m < n) ? m : n;
  real_t *U = MALLOC(real_t, m * n);
  real_t *s = MALLOC(real_t, k);
  real_t *V = MALLOC(real_t, k * k);
  int retval = svd(A, m, n, U, s, V);

  // Calculate determinant by via product of the diagonal singular values
  *det = s[0];
  for (int i = 1; i < k; i++) {
    *det *= s[i];
  }

  // Clean up
  free(U);
  free(s);
  free(V);

  return retval;
}

/**
 * Calculate matrix rank of `A` of size `m` x `n`.
 */
int svd_rank(const real_t *A, const int m, const int n, real_t tol) {
  // Decompose matrix A with SVD
  const int k = (m < n) ? m : n;
  real_t *U = MALLOC(real_t, m * n);
  real_t *s = MALLOC(real_t, k);
  real_t *V = MALLOC(real_t, k * k);
  int retval = svd(A, m, n, U, s, V);
  if (retval != 0) {
    free(U);
    free(s);
    free(V);
    return -1;
  }

  // Calculate determinant by via product of the diagonal singular values
  int rank = 0;
  for (int i = 0; i < k; i++) {
    if (s[i] >= tol) {
      rank++;
    }
  }

  // Clean up
  free(U);
  free(s);
  free(V);

  return rank;
}

//////////
// CHOL //
//////////

#ifdef USE_LAPACK

// LAPACK fortran prototypes
extern int spotrf_(char *uplo, int *n, float *A, int *lda, int *info);
extern int spotrs_(char *uplo,
                   int *n,
                   int *nrhs,
                   float *A,
                   int *lda,
                   float *B,
                   int *ldb,
                   int *info);
extern int dpotrf_(char *uplo, int *n, double *A, int *lda, int *info);
extern int dpotrs_(char *uplo,
                   int *n,
                   int *nrhs,
                   double *A,
                   int *lda,
                   double *B,
                   int *ldb,
                   int *info);

/**
 * Decompose matrix A to lower triangular matrix L
 */
void __lapack_chol(const real_t *A, const size_t m, real_t *L) {
  assert(A != NULL);
  assert(m > 0);
  assert(L != NULL);

  // Cholesky Decomposition
  int lda = m;
  int n = m;
  char uplo = 'L';
  int info = 0;
  mat_copy(A, m, m, L);

#if PRECISION == 1
  spotrf_(&uplo, &n, L, &lda, &info);
#elif PRECISION == 2
  dpotrf_(&uplo, &n, L, &lda, &info);
#endif
  if (info != 0) {
    fprintf(stderr, "Failed to decompose A using Cholesky Decomposition!\n");
  }

  // Transpose and zero upper triangular result
  for (size_t i = 0; i < m; i++) {
    for (size_t j = i; j < m; j++) {
      if (i != j) {
        L[(j * m) + i] = L[(i * m) + j];
        L[(i * m) + j] = 0;
      }
    }
  }
}

/**
 * Solve Ax = b using LAPACK's implementation of Cholesky decomposition, where
 * `A` is a square matrix, `b` is a vector and `x` is the solution vector of
 * size `n`.
 */
void __lapack_chol_solve(const real_t *A,
                         const real_t *b,
                         real_t *x,
                         const size_t m) {
  assert(A != NULL);
  assert(b != NULL);
  assert(x != NULL);
  assert(m > 0);

  // Cholesky Decomposition
  int info = 0;
  int lda = m;
  int n = m;
  char uplo = 'U';
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

  // Solve Ax = b using Cholesky decomposed A from above
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
#endif // USE_LAPACK

/**
 * Cholesky decomposition. Takes a `m x m` matrix `A` and decomposes it into a
 * lower and upper triangular matrix `L` and `U` with Cholesky decomposition.
 * This function only returns the `L` triangular matrix.
 */
void __chol(const real_t *A, const size_t m, real_t *L) {
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
void __chol_solve(const real_t *A, const real_t *b, real_t *x, const size_t n) {
  assert(A != NULL);
  assert(b != NULL);
  assert(x != NULL);
  assert(n > 0);

  // Allocate memory
  real_t *L = CALLOC(real_t, n * n);
  real_t *Lt = CALLOC(real_t, n * n);
  real_t *y = CALLOC(real_t, n);

  // Cholesky decomposition
  chol(A, n, L);
  mat_transpose(L, n, n, Lt);

  // Forward substitution
  // Ax = b -> LLt x = b.
  // Let y = Lt x, L y = b (Solve for y)
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

  // Backward substitution
  // Now we have y, we can go back to (Lt x = y) and solve for x
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

  // Clean up
  free(y);
  free(L);
  free(Lt);
}

/**
 * Cholesky decomposition. Takes a `m x m` matrix `A` and decomposes it into a
 * lower and upper triangular matrix `L` and `U` with Cholesky decomposition.
 * This function only returns the `L` triangular matrix.
 */
void chol(const real_t *A, const size_t m, real_t *L) {
#ifdef USE_LAPACK
  __lapack_chol(A, m, L);
#else
  __chol(A, m, L);
#endif // USE_LAPACK
}

/**
 * Solve `Ax = b` using Cholesky decomposition, where `A` is a square matrix,
 * `b` is a vector and `x` is the solution vector of size `n`.
 */
void chol_solve(const real_t *A, const real_t *b, real_t *x, const size_t n) {
#ifdef USE_LAPACK
  __lapack_chol_solve(A, b, x, n);
#else
  __chol_solve(A, b, x, n);
#endif // USE_LAPACK
}

////////
// QR //
////////

#ifdef USE_LAPACK

// LAPACK fortran prototypes
void sgeqrf_(const int *M,
             const int *N,
             float *A,
             const int *lda,
             float *TAU,
             float *work,
             const int *lwork,
             int *info);
void dgeqrf_(const int *M,
             const int *N,
             double *A,
             const int *lda,
             double *TAU,
             double *work,
             const int *lwork,
             int *info);

void __lapack_qr(real_t *A, int m, int n, real_t *R) {
  // Transpose matrix A because LAPACK is column major
  real_t *At = MALLOC(real_t, m * n);
  mat_transpose(A, m, n, At);

  // Query and allocate optimal workspace
  int lda = m;
  int lwork = -1;
  int info = 0;
  real_t work_size;
  real_t *work = &work_size;
  real_t *tau = MALLOC(real_t, (m < n) ? m : n);

#if PRECISION == 1
  sgeqrf_(&m, &n, At, &lda, tau, work, &lwork, &info);
#else
  dgeqrf_(&m, &n, At, &lda, tau, work, &lwork, &info);
#endif
  lwork = work_size;
  work = MALLOC(real_t, lwork);

  // Compute QR
#if PRECISION == 1
  sgeqrf_(&m, &n, At, &lda, tau, work, &lwork, &info);
#else
  dgeqrf_(&m, &n, At, &lda, tau, work, &lwork, &info);
#endif
  // mat_transpose(At, m, n, R);

  // Transpose result and zero lower triangular
  for (size_t i = 0; i < m; i++) {
    for (size_t j = 0; j < m; j++) {
      if (i <= j) {
        R[(i * m) + j] = At[(j * m) + i];
      } else {
        R[(i * m) + j] = 0;
      }
    }
  }

  // Recover matrix Q
  // From the LAPACK documentation:
  //
  // The matrix Q is represented as a product of elementary reflectors
  //
  //   Q = H(1) H(2) . . . H(k), where k = min(m, n).
  //
  // Each H(i) has the form
  //
  //   H(i) = I - tau * v * v**T
  //
  // where tau is a real scalar, and v is a real vector with v(1:i-1) = 0 and
  // v(i) = 1; v(i+1:m) is stored on exit in A(i+1:m,i), and tau in (i).
  //
  // Q = eye(6); % Initial
  // v = [ 0 0 0 0 0 0];
  // m = 6;
  // for i = 1:4
  //   v(1:i-1) = 0;
  //   v(i) = 1;
  //   v(i+1:m) = A(i+1:m,i);
  //   A(i+1:m,i)
  //   Q = Q*(eye(6) - tau(i)*v'*v);
  // end
  // real_t *Q = CALLOC(real_t, m * m);
  // real_t *v = CALLOC(real_t, m);
  // for (int i = 0; i < n; i++) {
  //   for (int ii = 0; ii < i; i++) {
  //     v[ii] = 0.0;
  //   }
  //   v[i] = 1.0;
  //   for (int ii = i+1; ii < m; ii++) {
  //     v[ii] = At[(i + 1) * m + i];
  //   }

  // }
  // free(Q);
  // free(v);

  // print_matrix("R", R, m, n);
  // print_vector("tau", tau, m);

  // Clean up
  free(At);
  free(work);
  free(tau);
}
#endif // USE_LAPACK

void qr(real_t *A, const int m, const int n, real_t *R) {
#ifdef USE_LAPACK
  __lapack_qr(A, m, n, R);
#else
  FATAL("NOT IMPLEMENTED!");
#endif // USE_LAPACK
}

/////////
// EIG //
/////////

#ifdef USE_LAPACK

// LAPACK fortran prototypes
void ssyev_(char *jobz,
            char *uplo,
            int *n,
            float *a,
            int *lda,
            float *w,
            float *work,
            int *lwork,
            int *info);
void dsyev_(char *jobz,
            char *uplo,
            int *n,
            double *a,
            int *lda,
            double *w,
            double *work,
            int *lwork,
            int *info);

int __lapack_eig(const real_t *A,
                 const int m,
                 const int n,
                 real_t *V,
                 real_t *w) {
  assert(A != NULL);
  assert(m > 0 && n > 0);
  assert(m == n);
  int n_ = n;
  int lda = n;

  // Copy matrix A to output matrix V
  mat_triu(A, n, V);

  // Query and allocate the optimal workspace
  int lwork = -1;
  int info = 0;
  real_t wkopt;
#if PRECISION == 1
  ssyev_("Vectors", "Lower", &n_, V, &lda, w, &wkopt, &lwork, &info);
#elif PRECISION == 2
  dsyev_("Vectors", "Lower", &n_, V, &lda, w, &wkopt, &lwork, &info);
#endif // Precision
  lwork = (int) wkopt;
  real_t *work = MALLOC(double, lwork);

  // Solve eigenproblem
#if PRECISION == 1
  ssyev_("Vectors", "Lower", &n_, V, &lda, w, work, &lwork, &info);
#elif PRECISION == 2
  dsyev_("Vectors", "Lower", &n_, V, &lda, w, work, &lwork, &info);
#endif // Precision
  if (info > 0) {
    LOG_ERROR("The algorithm failed to compute eigenvalues.\n");
    free(work);
    return -1;
  }

  // Clean up
  real_t *Vt = MALLOC(real_t, n * n);
  mat_transpose(V, n, n, Vt);
  mat_copy(Vt, n, n, V);
  free(Vt);
  free(work);

  return 0;
}
#endif // USE_LAPACK

/**
 * Perform Eigen-Decomposition of a symmetric matrix `A` of size `m` x `n`.
 */
int eig_sym(const real_t *A, const int m, const int n, real_t *V, real_t *w) {
  assert(A != NULL);
  assert(m > 0 && n > 0);
  assert(m == n);
  assert(V != NULL && w != NULL);
#ifdef USE_LAPACK
  return __lapack_eig(A, m, n, V, w);
#else
  FATAL("Not implemented!");
  return 0;
#endif // USE_LAPACK
}

/**
 * Invert matrix `A` of size `m` x `n` with Eigen-decomposition.
 */
int eig_inv(real_t *A, const int m, const int n, const int c, real_t *A_inv) {
  assert(A != NULL);
  assert(m == n);
  assert(A_inv != NULL);

  // Enforce Symmetric Positive Definite
  if (c) {
    enforce_spd(A, m, m);
  }

  // Invert matrix via Eigen-decomposition
  real_t *V = MALLOC(real_t, m * m);
  real_t *Vt = MALLOC(real_t, m * m);
  real_t *Lambda_inv = MALLOC(real_t, m * m);
  real_t *w = MALLOC(real_t, m);

  eig_sym(A, m, m, V, w);
  for (int i = 0; i < m; i++) {
    w[i] = 1.0 / w[i];
  }
  mat_diag_set(Lambda_inv, m, m, w);
  mat_transpose(V, m, m, Vt);
  dot3(V, m, m, Lambda_inv, m, m, Vt, m, m, A_inv);

  // Clean up
  free(V);
  free(Vt);
  free(Lambda_inv);
  free(w);

  return 0;
}

/**
 * Calculate matrix rank of `A` of size `m` x `n`.
 */
int eig_rank(const real_t *A, const int m, const int n, const real_t tol) {
  assert(A != NULL);
  assert(m > 0 && n > 0);
  assert(m == n);

  real_t *V = MALLOC(real_t, m * n);
  real_t *w = MALLOC(real_t, m);
  int retval = eig_sym(A, m, n, V, w);
  if (retval != 0) {
    free(V);
    free(w);
    return -1;
  }

  int rank = 0;
  for (int i = 0; i < m; i++) {
    if (w[i] >= tol) {
      rank++;
    }
  }

  // Clean up
  free(V);
  free(w);

  return rank;
}

/*******************************************************************************
 * SUITE-SPARSE
 ******************************************************************************/

#define CHOLMOD_NZERO_EPS 1e-12

/**
 * Allocate memory and form a sparse matrix
 *
 * @param c Cholmod workspace
 * @param A Matrix A
 * @param m Number of rows in A
 * @param n Number of cols in A
 * @param stype
 *
 *   0:  matrix is "unsymmetric": use both upper and lower triangular parts
 *       (the matrix may actually be symmetric in pattern and value, but
 *       both parts are explicitly stored and used).  May be square or
 *       rectangular.
 *   >0: matrix is square and symmetric, use upper triangular part.
 *       Entries in the lower triangular part are ignored.
 *   <0: matrix is square and symmetric, use lower triangular part.
 *       Entries in the upper triangular part are ignored.
 *
 *   Note that stype>0 and stype<0 are different for cholmod_sparse and
 *   cholmod_triplet.  See the cholmod_triplet data structure for more
 *   details.
 *
 * @returns A suite-sparse sparse matrix
 */
cholmod_sparse *cholmod_sparse_malloc(cholmod_common *c,
                                      const real_t *A,
                                      const int m,
                                      const int n,
                                      const int stype) {
  assert(c != NULL);
  assert(A != NULL);
  assert(m > 0 && n > 0);

  // Count number of non-zeros
  size_t nzmax = 0;
  for (long int idx = 0; idx < (m * n); idx++) {
    if (fabs(A[idx]) > CHOLMOD_NZERO_EPS) {
      nzmax++;
    }
  }

  // Allocate memory for sparse matrix
  cholmod_sparse *A_cholmod =
      cholmod_allocate_sparse(m, n, nzmax, 1, 1, stype, CHOLMOD_REAL, c);
  assert(A_cholmod);

  // Fill sparse matrix
  int *row_ind = A_cholmod->i;
  int *col_ptr = A_cholmod->p;
  real_t *values = A_cholmod->x;
  size_t row_it = 0;
  size_t col_it = 1;
  for (long int j = 0; j < n; ++j) {
    for (long int i = 0; i < m; ++i) {
      if (fabs(A[(i * n) + j]) > CHOLMOD_NZERO_EPS) {
        values[row_it] = A[(i * n) + j];
        row_ind[row_it] = i;
        row_it++;
      }
    }
    col_ptr[col_it] = row_it;
    col_it++;
  }

  return A_cholmod;
}

/**
 * Allocate memory and form a dense vector
 *
 * @param c Cholmod workspace
 * @param x Vector x
 * @param n Length of vector x
 *
 * @returns A dense suite-sparse vector
 */
cholmod_dense *cholmod_dense_malloc(cholmod_common *c,
                                    const real_t *x,
                                    const int n) {
  assert(c != NULL);
  assert(x != NULL);

  cholmod_dense *out = cholmod_allocate_dense(n, 1, n, CHOLMOD_REAL, c);
  assert(out != NULL);

  double *out_x = out->x;
  for (size_t i = 0; i < n; i++) {
    out_x[i] = x[i];
  }

  return out;
}

/**
 * Extract dense suite-sparse vector of length `n` from `src` to `dst`.
 */
void cholmod_dense_raw(const cholmod_dense *src, real_t *dst, const int n) {
  assert(src != NULL);
  assert(dst != NULL);
  assert(n > 0);

  double *data = src->x;
  for (int i = 0; i < n; i++) {
    dst[i] = data[i];
  }
}

/**
 * Solve Ax = b with Suite-Sparse's CHOLMOD package
 *
 * @param c Cholmod workspace
 * @param A Matrix A
 * @param A_m Number of rows in A
 * @param A_n Number of cols in A
 * @param b Vector b
 * @param b_m Number of rows in A
 * @param x Vector x
 *
 * @returns the residual norm of (Ax - b)
 */
real_t suitesparse_chol_solve(cholmod_common *c,
                              const real_t *A,
                              const int A_m,
                              const int A_n,
                              const real_t *b,
                              const int b_m,
                              real_t *x) {
  assert(c != NULL);
  assert(A != NULL && A_m > 0 && A_n > 0);
  assert(b != NULL && b_m > 0);
  assert(A_n == b_m);
  assert(x != NULL);

  // Setup
  cholmod_sparse *A_sparse = cholmod_sparse_malloc(c, A, A_m, A_n, 1);
  cholmod_dense *b_dense = cholmod_dense_malloc(c, b, b_m);
  assert(A_sparse);
  assert(b_dense);
  assert(cholmod_check_sparse(A_sparse, c) != -1);
  assert(cholmod_check_dense(b_dense, c) != -1);

  // Analyze and factorize
  cholmod_factor *L_factor = cholmod_analyze(A_sparse, c);
  cholmod_factorize(A_sparse, L_factor, c);
  assert(cholmod_check_factor(L_factor, c) != -1);

  // Solve A * x = b
  cholmod_dense *x_dense = cholmod_solve(CHOLMOD_A, L_factor, b_dense, c);
  cholmod_dense_raw(x_dense, x, b_m);

  // r = r - A * x
  double m1[2] = {-1, 0};
  double one[2] = {1, 0};
  cholmod_dense *r_dense = cholmod_copy_dense(b_dense, c);
  cholmod_sdmult(A_sparse, 0, m1, one, x_dense, r_dense, c);
  const real_t norm = cholmod_norm_dense(r_dense, 0, c);

  // Clean up
  cholmod_free_sparse(&A_sparse, c);
  cholmod_free_dense(&b_dense, c);
  cholmod_free_factor(&L_factor, c);
  cholmod_free_dense(&x_dense, c);
  cholmod_free_dense(&r_dense, c);

  return norm;
}

/*******************************************************************************
 * LIE
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

  hat(phi, phi_skew);
  dot(phi_skew, 3, 3, phi_skew, 3, 3, phi_skew_sq);

  if (phi_norm < 1e-3) {
    // C = eye(3) + hat(phi);
    eye(C, 3, 3);
    mat_add(C, phi_skew, C, 3, 3);
  } else {
    // C = eye(3);
    // C += (sin(phi_norm) / phi_norm) * phi_skew;
    // C += ((1 - cos(phi_norm)) / phi_norm ^ 2) * phi_skew_sq;
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
   * vec = vee(C - C') / (2 * sin(phi));
   * rvec = phi * vec;
   */
  const real_t tr = C[0] + C[4] + C[8];
  const real_t phi = acos((tr - 1.0) / 2.0);

  real_t C_t[3 * 3] = {0};
  real_t dC[3 * 3] = {0};
  mat_transpose(C, 3, 3, C_t);
  mat_sub(C, C_t, dC, 3, 3);
  real_t u[3] = {0};
  vee(dC, u);
  const real_t s = 1.0 / (2 * sin(phi));
  const real_t vec[3] = {s * u[0], s * u[1], s * u[2]};

  rvec[0] = phi * vec[0];
  rvec[1] = phi * vec[1];
  rvec[2] = phi * vec[2];
}

/**
 * Box-plus operator:
 *
 *   C_new = C [+] alpha
 *
 */
void box_plus(const real_t C[3 * 3],
              const real_t alpha[3],
              real_t C_new[3 * 3]) {
  real_t dC[3 * 3] = {0};
  lie_Exp(alpha, dC);
  dot(C, 3, 3, dC, 3, 3, C_new);
}

/**
 * Box-minus operator:
 *
 *   alpha = C_a [-] C_b
 *
 */
void box_minus(const real_t Ca[3 * 3],
               const real_t Cb[3 * 3],
               real_t alpha[3]) {
  real_t Cbt[3 * 3] = {0};
  real_t dC[3 * 3] = {0};
  mat_transpose(Cb, 3, 3, Cbt);
  dot(Cbt, 3, 3, Ca, 3, 3, dC);
  lie_Log(dC, alpha);
}

/*******************************************************************************
 * TRANSFORMS
 ******************************************************************************/

/** Form rotation matrix around x axis **/
void rotx(const real_t theta, real_t C[3 * 3]) {
  C[0] = 1.0;
  C[1] = 0.0;
  C[2] = 0.0;

  C[3] = 0.0;
  C[4] = cos(theta);
  C[5] = -sin(theta);

  C[6] = 0.0;
  C[7] = sin(theta);
  C[8] = cos(theta);
}

/** Form rotation matrix around y axis */
void roty(const real_t theta, real_t C[3 * 3]) {
  C[0] = cos(theta);
  C[1] = 0.0;
  C[2] = sin(theta);

  C[3] = 0.0;
  C[4] = 1.0;
  C[5] = 0.0;

  C[6] = -sin(theta);
  C[7] = 0.0;
  C[8] = cos(theta);
}

/** Form rotation matrix around z axis */
void rotz(const real_t theta, real_t C[3 * 3]) {
  C[0] = cos(theta);
  C[1] = -sin(theta);
  C[2] = 0.0;

  C[3] = sin(theta);
  C[4] = cos(theta);
  C[5] = 0.0;

  C[6] = 0.0;
  C[7] = 0.0;
  C[8] = 1.0;
}

/**
 * Form 4x4 homogeneous transformation matrix `T` from a 7x1 pose vector
 * `params`.
 *
 *    pose = (translation, rotation)
 *    pose = (rx, ry, rz, qw, qx, qy, qz)
 */
void tf(const real_t params[7], real_t T[4 * 4]) {
  assert(params != NULL);
  assert(T != NULL);

  const real_t r[3] = {params[0], params[1], params[2]};
  const real_t q[4] = {params[3], params[4], params[5], params[6]};

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
 * Form 4x4 homogeneous transformation matrix `T` from a rotation matrix `C`
 * and translation vector `r`.
 */
void tf_cr(const real_t C[3 * 3], const real_t r[3], real_t T[4 * 4]) {
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
 * Form 4x4 homogeneous transformation matrix `T` from a quaternion `q` and
 * translation vector `r`.
 */
void tf_qr(const real_t q[4], const real_t r[3], real_t T[4 * 4]) {
  real_t C[3 * 3] = {0};
  quat2rot(q, C);
  tf_cr(C, r, T);
}

/**
 * Form 4x4 homogeneous transformation matrix `T` from a euler-angles `ypr`
 * (yaw-pitch-roll) and translation vector `r`.
 */
void tf_er(const real_t ypr[3], const real_t r[3], real_t T[4 * 4]) {
  real_t C[3 * 3] = {0};
  euler321(ypr, C);
  tf_cr(C, r, T);
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

  params[3] = q[0];
  params[4] = q[1];
  params[5] = q[2];
  params[6] = q[3];
}

/**
 * Decompose transform `T` into the rotation `C` and translation `r`
 * components.
 */
void tf_decompose(const real_t T[4 * 4], real_t C[3 * 3], real_t r[3]) {
  assert(T != NULL);
  assert(C != NULL);
  assert(r != NULL);

  C[0] = T[0];
  C[1] = T[1];
  C[2] = T[2];
  r[0] = T[3];

  C[3] = T[4];
  C[4] = T[5];
  C[5] = T[6];
  r[1] = T[7];

  C[6] = T[8];
  C[7] = T[9];
  C[8] = T[10];
  r[1] = T[11];
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

  /**
   * Transformation T comprises of rotation C and translation r:
   *
   *   T = [C0, C1, C2, r0]
   *       [C3, C4, C5, r1]
   *       [C6, C7, C8, r2]
   *       [0, 0, 0, 1]
   *
   * The inverse is therefore:
   *
   *   C_inv = C^T
   *   r_inv = -C^T * r
   *
   *   T_inv = [C0, C3, C6, -C0*r0 - C3*r1 - C6*r2]
   *           [C1, C4, C7, -C1*r0 - C4*r1 - C7*r2]
   *           [C2, C5, C8, -C2*r0 - C5*r1 - C8*r2]
   *           [0, 0, 0, 1]
   */

  // Get rotation and translation components
  real_t C[3 * 3] = {0};
  real_t r[3] = {0};
  tf_rot_get(T, C);
  tf_trans_get(T, r);

  // Invert translation
  real_t r_out[3] = {0};
  r_out[0] = -C[0] * r[0] - C[3] * r[1] - C[6] * r[2];
  r_out[1] = -C[1] * r[0] - C[4] * r[1] - C[7] * r[2];
  r_out[2] = -C[2] * r[0] - C[5] * r[1] - C[8] * r[2];

  // First row
  T_inv[0] = C[0];
  T_inv[1] = C[3];
  T_inv[2] = C[6];
  T_inv[3] = r_out[0];

  // Second row
  T_inv[4] = C[1];
  T_inv[5] = C[4];
  T_inv[6] = C[7];
  T_inv[7] = r_out[1];

  // Third row
  T_inv[8] = C[2];
  T_inv[9] = C[5];
  T_inv[10] = C[8];
  T_inv[11] = r_out[2];

  // Fourth row
  T_inv[12] = 0.0;
  T_inv[13] = 0.0;
  T_inv[14] = 0.0;
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

  // Build perturb drvec
  real_t drvec[3] = {0};
  drvec[i] = step_size;

  // Decompose transform to rotation and translation
  real_t C[3 * 3] = {0};
  tf_rot_get(T, C);

  // Perturb rotation
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

  // Build perturb dr
  real_t dr[3] = {0};
  dr[i] = step_size;

  // Decompose transform get translation
  real_t r[3] = {0};
  tf_trans_get(T, r);

  // Perturb translation
  const real_t r_diff[3] = {r[0] + dr[0], r[1] + dr[1], r[2] + dr[2]};
  tf_trans_set(T, r_diff);
}

/**
 * Chain `N` homogeneous transformations `tfs`.
 */
void tf_chain(const real_t **tfs, const int N, real_t T_out[4 * 4]) {
  assert(tfs != NULL);
  assert(T_out != NULL);

  // Initialize T_out with the first transform in tfs
  mat_copy(tfs[0], 4, 4, T_out);

  // Chain transforms
  for (int i = 1; i < N; i++) {
    real_t T_from[4 * 4] = {0};
    mat_copy(T_out, 4, 4, T_from);
    dot(T_from, 4, 4, tfs[i], 4, 4, T_out);
  }
}

/**
 * Chain `N` homogeneous transformations `tfs`.
 */
void tf_chain2(const int n, ...) {
  va_list args;
  real_t T_out[4 * 4] = {0};
  eye(T_out, 4, 4);

  va_start(args, n);
  for (int i = 0; i < n; i++) {
    real_t T_from[4 * 4] = {0};
    mat_copy(T_out, 4, 4, T_from);
    dot(T_from, 4, 4, va_arg(args, real_t *), 4, 4, T_out);
  }
  mat_copy(T_out, 4, 4, va_arg(args, real_t *));
  va_end(args);
}

/**
 * Pose difference between `pose0` and `pose`, returns difference in
 * translation and rotation `diff`.
 */
void tf_diff(const real_t Ti[4 * 4], const real_t Tj[4 * 4], real_t diff[6]) {
  TF_VECTOR(Ti, pose_i);
  TF_VECTOR(Tj, pose_j);
  pose_diff(pose_i, pose_j, diff);
}

/**
 * Find the difference between two transforms, returns difference in
 * translation `dr` and rotation angle `dtheta` in radians.
 */
void tf_diff2(const real_t Ti[4 * 4],
              const real_t Tj[4 * 4],
              real_t dr[3],
              real_t *dtheta) {
  TF_VECTOR(Ti, pose_i);
  TF_VECTOR(Tj, pose_j);
  pose_diff2(pose_i, pose_j, dr, dtheta);
}

/**
 * Return translation vector `r` from pose vector `p`.
 */
void pose_get_trans(const real_t p[7], real_t r[3]) {
  r[0] = p[0];
  r[1] = p[1];
  r[2] = p[2];
}

/**
 * Return Quaternion `q` from pose vector `p`.
 */
void pose_get_quat(const real_t p[7], real_t q[4]) {
  q[0] = p[3];
  q[1] = p[4];
  q[2] = p[5];
  q[3] = p[6];
}

/**
 * Return rotation matrix `C` from pose vector `p`.
 */
void pose_get_rot(const real_t p[7], real_t C[3 * 3]) {
  const real_t q[4] = {p[3], p[4], p[5], p[6]};
  quat2rot(q, C);
}

/**
 * Pose difference between `pose0` and `pose1`, returns difference in
 * translation and rotation `diff`.
 */
void pose_diff(const real_t pose0[7], const real_t pose1[7], real_t diff[6]) {
  assert(pose0 != NULL);
  assert(pose1 != NULL);
  assert(diff != NULL);

  // dr
  diff[0] = pose0[0] - pose1[0];
  diff[1] = pose0[1] - pose1[1];
  diff[2] = pose0[2] - pose1[2];

  // dq = quat_mul(quat_inv(q_meas), q_est);
  const real_t *q0 = pose0 + 3;
  const real_t *q1 = pose1 + 3;
  real_t q0_inv[4] = {0};
  real_t dq[4] = {0};
  quat_inv(q0, q0_inv);
  quat_mul(q0_inv, q1, dq);

  // dtheta = 2 * dq;
  const real_t dtheta[3] = {2.0 * dq[1], 2.0 * dq[2], 2.0 * dq[3]};
  diff[3] = dtheta[0];
  diff[4] = dtheta[1];
  diff[5] = dtheta[2];
}

/**
 * Pose difference between `pose0` and `pose1`, returns difference in
 * translation `dr` and rotation angle `dtheta` in radians.
 */
void pose_diff2(const real_t pose0[7],
                const real_t pose1[7],
                real_t dr[3],
                real_t *dtheta) {
  assert(pose0 != NULL);
  assert(pose1 != NULL);
  assert(dr != NULL);
  assert(dtheta != NULL);

  // dr
  dr[0] = pose0[0] - pose1[0];
  dr[1] = pose0[1] - pose1[1];
  dr[2] = pose0[2] - pose1[2];

  // dC = C0.T * C1
  // dtheta = acos((tr(dC) - 1.0) / 2.0)
  const real_t *q0 = pose0 + 3;
  const real_t *q1 = pose1 + 3;
  real_t C0[3 * 3] = {0};
  real_t C0t[3 * 3] = {0};
  real_t C1[3 * 3] = {0};
  real_t dC[3 * 3] = {0};
  quat2rot(q0, C0);
  quat2rot(q1, C1);
  mat_transpose(C0, 3, 3, C0t);
  dot(C0t, 3, 3, C1, 3, 3, dC);
  const real_t tr = mat_trace(dC, 3, 3);
  if (fabs(tr - 3.0) < 1e-5) {
    *dtheta = 0.0;
  } else {
    *dtheta = acos((tr - 1.0) / 2.0);
  }
}

/**
 * Update pose vector `pose` with update vector `dx`
 */
void pose_update(real_t pose[7], const real_t dx[6]) {
  // Update translation
  pose[0] += dx[0];
  pose[1] += dx[1];
  pose[2] += dx[2];

  // Update rotation
  real_t dq[4] = {0};
  real_t q_new[4] = {0};
  quat_delta(dx + 3, dq);
  quat_mul(pose + 3, dq, q_new);
  pose[3] = q_new[0];
  pose[4] = q_new[1];
  pose[5] = q_new[2];
  pose[6] = q_new[3];
}

/**
 * Perturb pose vector `pose` randomly.
 */
void pose_random_perturb(real_t pose[7],
                         const real_t dtrans,
                         const real_t drot) {
  // Perturb pose position
  pose[0] += randf(-dtrans, dtrans);
  pose[1] += randf(-dtrans, dtrans);
  pose[2] += randf(-dtrans, dtrans);

  // Pertrub pose rotation
  real_t dalpha[3] = {0};
  randvec(-drot, drot, 3, dalpha);
  quat_update(pose + 3, dalpha);
}

/**
 * Print pose vector
 */
void print_pose(const char *prefix, const real_t pose[7]) {
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

void vecs2rot(const real_t acc[3], const real_t gravity[3], real_t *C) {
  // Normalize vectors
  real_t a[3] = {acc[0], acc[1], acc[2]};
  real_t g[3] = {gravity[0], gravity[1], gravity[2]};
  vec3_normalize(a);
  vec3_normalize(g);

  // Create Quaternion from two vectors
  const real_t cos_theta = a[0] * g[0] + a[1] * g[1] + a[2] * g[2];
  const real_t half_cos = sqrt(0.5 * (1.0 + cos_theta));
  const real_t half_sin = sqrt(0.5 * (1.0 - cos_theta));
  real_t w[3] = {0};
  w[0] = a[1] * g[2] - a[2] * g[1];
  w[1] = -a[0] * g[2] + a[2] * g[0];
  w[2] = a[0] * g[1] - a[1] * g[0];
  vec3_normalize(w);

  const real_t qw = half_cos;
  const real_t qx = half_sin * w[0];
  const real_t qy = half_sin * w[1];
  const real_t qz = half_sin * w[2];

  // Convert Quaternion to rotation matrix
  const real_t qx2 = qx * qx;
  const real_t qy2 = qy * qy;
  const real_t qz2 = qz * qz;
  const real_t qw2 = qw * qw;

  C[0] = qw2 + qx2 - qy2 - qz2;
  C[1] = 2 * (qx * qy - qw * qz);
  C[2] = 2 * (qx * qz + qw * qy);

  C[3] = 2 * (qx * qy + qw * qz);
  C[4] = qw2 - qx2 + qy2 - qz2;
  C[5] = 2 * (qy * qz - qw * qx);

  C[6] = 2 * (qx * qz - qw * qy);
  C[7] = 2 * (qy * qz + qw * qx);
  C[8] = qw2 - qx2 - qy2 + qz2;
}

/**
 * Convert rotation vector `rvec` to 3x3 rotation matrix `R`, where `eps` is
 * the tolerance to determine if the rotation is too small.
 */
void rvec2rot(const real_t *rvec, const real_t eps, real_t *R) {
  assert(rvec != NULL);
  assert(eps > 0);
  assert(R != NULL);

  // Magnitude of rvec
  const real_t theta = sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1]);
  // ^ basically norm(rvec), but faster

  // Check if rotation is too small
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

  // Convert rvec to rotation matrix
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

  const real_t cpsi = cos(psi);
  const real_t spsi = sin(psi);

  const real_t ctheta = cos(theta);
  const real_t stheta = sin(theta);

  const real_t cphi = cos(phi);
  const real_t sphi = sin(phi);

  // 1st row
  C[0] = cpsi * ctheta;
  C[1] = cpsi * stheta * sphi - spsi * cphi;
  C[2] = cpsi * stheta * cphi + spsi * sphi;

  // 2nd row
  C[3] = spsi * ctheta;
  C[4] = spsi * stheta * sphi + cpsi * cphi;
  C[5] = spsi * stheta * cphi - cpsi * sphi;

  // 3rd row
  C[6] = -stheta;
  C[7] = ctheta * sphi;
  C[8] = ctheta * cphi;
}

/**
 * Convert Euler angles `ypr` in radians to a Hamiltonian Quaternion.
 */
void euler2quat(const real_t ypr[3], real_t q[4]) {
  const real_t psi = ypr[0];
  const real_t theta = ypr[1];
  const real_t phi = ypr[2];

  const real_t cphi = cos(phi / 2.0);
  const real_t ctheta = cos(theta / 2.0);
  const real_t cpsi = cos(psi / 2.0);
  const real_t sphi = sin(phi / 2.0);
  const real_t stheta = sin(theta / 2.0);
  const real_t spsi = sin(psi / 2.0);

  const real_t qx = sphi * ctheta * cpsi - cphi * stheta * spsi;
  const real_t qy = cphi * stheta * cpsi + sphi * ctheta * spsi;
  const real_t qz = cphi * ctheta * spsi - sphi * stheta * cpsi;
  const real_t qw = cphi * ctheta * cpsi + sphi * stheta * spsi;

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

/**
 * Print Quaternion
 */
void print_quat(const char *prefix, const real_t q[4]) {
  printf("%s: [w: %.10f, x: %.10f, y: %.10f, z: %.10f]\n",
         prefix,
         q[0],
         q[1],
         q[2],
         q[3]);
}

/**
 * Return Quaternion norm
 */
real_t quat_norm(const real_t q[4]) {
  return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

/**
 * Setup Quaternion
 */
void quat_setup(real_t q[4]) {
  q[0] = 1.0;
  q[1] = 0.0;
  q[2] = 0.0;
  q[3] = 0.0;
}

/**
 * Normalize Quaternion
 */
void quat_normalize(real_t q[4]) {
  const real_t n = quat_norm(q);
  q[0] = q[0] / n;
  q[1] = q[1] / n;
  q[2] = q[2] / n;
  q[3] = q[3] / n;
}

/**
 * Normalize Quaternion
 */
void quat_normalize_copy(const real_t q[4], real_t q_normalized[4]) {
  const real_t n = quat_norm(q);
  q_normalized[0] = q[0] / n;
  q_normalized[1] = q[1] / n;
  q_normalized[2] = q[2] / n;
  q_normalized[3] = q[3] / n;
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

  // Homogeneous form
  // -- 1st row
  C[0] = qw2 + qx2 - qy2 - qz2;
  C[1] = 2 * (qx * qy - qw * qz);
  C[2] = 2 * (qx * qz + qw * qy);
  // -- 2nd row
  C[3] = 2 * (qx * qy + qw * qz);
  C[4] = qw2 - qx2 + qy2 - qz2;
  C[5] = 2 * (qy * qz - qw * qx);
  // -- 3rd row
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

  // clang-format off
  left[0]  = qw; left[1]  = -qx; left[2]  = -qy; left[3]  = -qz;
  left[4]  = qx; left[5]  =  qw; left[6]  = -qz; left[7]  =  qy;
  left[8]  = qy; left[9]  =  qz; left[10] =  qw; left[11] = -qx;
  left[12] = qz; left[13] = -qy; left[14] =  qx; left[15] =  qw;
  // clang-format on
}

/**
 * Form Quaternion left multiplication matrix.
 */
void quat_left_xyz(const real_t q[4], real_t left_xyz[3 * 3]) {
  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  // clang-format off
  left_xyz[0] =  qw; left_xyz[1] = -qz; left_xyz[2]  =  qy;
  left_xyz[3] =  qz; left_xyz[4] =  qw;  left_xyz[5] = -qx;
  left_xyz[6] = -qy; left_xyz[7] =  qx;  left_xyz[8] =  qw;
  // clang-format on
}

/**
 * Form Quaternion right multiplication matrix.
 */
void quat_right(const real_t q[4], real_t right[4 * 4]) {
  const real_t qw = q[0];
  const real_t qx = q[1];
  const real_t qy = q[2];
  const real_t qz = q[3];

  // clang-format off
  right[0]  = qw; right[1]  = -qx; right[2]  = -qy; right[3]  = -qz;
  right[4]  = qx; right[5]  =  qw; right[6]  =  qz; right[7]  = -qy;
  right[8]  = qy; right[9]  = -qz; right[10] =  qw; right[11] =  qx;
  right[12] = qz; right[13] =  qy; right[14] = -qx; right[15] =  qw;
  // clang-format on
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

  r[0] = pw * q[0] - px * q[1] - py * q[2] - pz * q[3];
  r[1] = px * q[0] + pw * q[1] - pz * q[2] + py * q[3];
  r[2] = py * q[0] + pz * q[1] + pw * q[2] - px * q[3];
  r[3] = pz * q[0] - py * q[1] + px * q[2] + pw * q[3];
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

  r[0] = qw * q[0] - qx * q[1] - qy * q[2] - qz * q[3];
  r[1] = qx * q[0] + qw * q[1] + qz * q[2] - qy * q[3];
  r[2] = qy * q[0] - qz * q[1] + qw * q[2] + qx * q[3];
  r[3] = qz * q[0] + qy * q[1] - qx * q[2] + qw * q[3];
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
  quat_normalize(dq);
}

/**
 * Update quaternion with small update dalpha.
 */
void quat_update(real_t q[4], const real_t dalpha[3]) {
  const real_t dq[4] = {1.0, 0.5 * dalpha[0], 0.5 * dalpha[1], 0.5 * dalpha[2]};
  real_t q_new[4] = {0};
  quat_mul(q, dq, q_new);
  quat_normalize(q_new);
  q[0] = q_new[0];
  q[1] = q_new[1];
  q[2] = q_new[2];
  q[3] = q_new[3];
}

/**
 * Update quaternion with angular velocity and dt.
 */
void quat_update_dt(real_t q[4], const real_t w[3], const real_t dt) {
  real_t dalpha[3] = {w[0] * dt, w[1] * dt, w[2] * dt};
  quat_update(q, dalpha);
}

/**
 * Perturb quaternion
 */
void quat_perturb(real_t q[4], const int i, const real_t h) {
  assert(i >= 0 && i <= 2);

  // Form small pertubation quaternion dq
  real_t dalpha[3] = {0};
  real_t dq[4] = {0};
  dalpha[i] = h;
  quat_delta(dalpha, dq);

  // Perturb quaternion
  real_t q_[4] = {q[0], q[1], q[2], q[3]};
  quat_mul(q_, dq, q);
  quat_normalize(q);
}

/**
 * Rotate vector `x` with quaternion `q`.
 */
void quat_transform(const real_t q[4], const real_t x[3], real_t y[3]) {
  // y = q * p * q_conj
  const real_t q_conj[4] = {q[0], -q[1], -q[2], -q[3]};
  const real_t p[4] = {0.0, x[0], x[1], x[2]};

  real_t qp[4] = {0};
  real_t p_new[4] = {0};
  quat_mul(q, p, qp);
  quat_mul(qp, q_conj, p_new);

  y[0] = p_new[1];
  y[1] = p_new[2];
  y[2] = p_new[3];
}
