#include "proto.h"

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
  strcpy(path_copy, path);

  char *base = strrchr(path_copy, '/');
  base = base ? base + 1 : path_copy;

  strcpy(fname, base);
}

/**
 * Extract file extension from `path` to `fext`.
 */
void path_file_ext(const char *path, char *fext) {
  assert(path != NULL);
  assert(fext != NULL);

  char path_copy[9046] = {0};
  strcpy(path_copy, path);

  char *base = strrchr(path, '.');
  if (base) {
    base = base ? base + 1 : path_copy;
    strcpy(fext, base);
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
  strcpy(path_copy, path);

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
    strcpy(retval, x);
    strcpy(retval + strlen(retval), (y[0] == '/') ? y + 1 : y);
  } else {
    retval = malloc(sizeof(char) * (strlen(x) + strlen(y)) + 2);
    strcpy(retval, x);
    strcat(retval + strlen(retval), "/");
    strcpy(retval + strlen(retval), (y[0] == '/') ? y + 1 : y);
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
    strcat(fp, path);
    strcat(fp, (fp[strlen(fp) - 1] == '/') ? "" : "/");
    strcat(fp, namelist[i]->d_name);

    files[*n] = malloc(sizeof(char) * (strlen(fp) + 1));
    strcpy(files[*n], fp);
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
 * Allocate heap memory for string `s`.
 */
char *malloc_string(const char *s) {
  assert(s != NULL);
  char *retval = malloc(sizeof(char) * strlen(s) + 1);
  strcpy(retval, s);
  return retval;
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
      strcpy(field_line, line);
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
      fields[field_idx] = malloc_string(field_name);
      memset(field_name, '\0', sizeof(char) * 100);
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

  /* Initialize memory for dsv data */
  real_t **data = malloc(sizeof(real_t *) * *nb_rows);
  for (int i = 0; i < *nb_rows; i++) {
    data[i] = malloc(sizeof(real_t) * *nb_cols);
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
  strcpy(ip, ipstr);

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
  struct sockaddr_in sockaddr;
  bzero(&sockaddr, sizeof(sockaddr));
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  sockaddr.sin_port = htons(server->port);

  /* Bind newly created socket to given IP */
  int retval =
      bind(server->sockfd, (struct sockaddr *) &sockaddr, sizeof(sockaddr));
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
  strcpy(client->server_ip, server_ip);
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
real_t deg2rad(const real_t d) { return d * (M_PI / 180.0); }

/**
 * Radians to degrees.
 * @returns Degrees
 */
real_t rad2deg(const real_t r) { return r * (180.0 / M_PI); }

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

  const real_t diff = (*(real_t *) x - *(real_t *) y);
  if (fabs(diff) < CMP_TOL) {
    return 0;
  } else if (diff > 0) {
    return 1;
  }

  return -1;
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

/**
 * SVD decomposition

 * Takes a `m x n` matrix `A` and decomposes it into `UDV`, where `U`, `V` are
 * left and right orthogonal transformation matrices, and `D` is a diagonal
 * matrix of singular values.
 *
 * The input matrix `A` to be decomposed, gets overwritten with `U`. Where `w`
 * is the singular values of `A` and `V` is the right orthogonal transformation
 * matrix.
 *
 * This routine is adapted from svdecomp.c in XLISP-STAT 2.1 which is code from
 * Numerical Recipes adapted by Luke Tierney and David Betz.
 *
 * @returns
 * - 0 for success
 * - -1 for failure
 */
int svd(real_t *A, const int m, const int n, real_t *w, real_t *V) {
  /* assert(m < n); */
  int flag, i, its, j, jj, k, l, nm;
  real_t c, f, h, s, x, y, z;
  real_t anorm = 0.0, g = 0.0, scale = 0.0;

  /* Householder reduction to bidiagonal form */
  real_t *rv1 = malloc(sizeof(real_t) * n);
  for (i = 0; i < n; i++) {
    /* left-hand reduction */
    l = i + 1;
    rv1[i] = scale * g;
    g = s = scale = 0.0;
    if (i < m) {
      for (k = i; k < m; k++) {
        scale += fabs(A[k * n + i]);
      }

      if (scale) {
        for (k = i; k < m; k++) {
          A[k * n + i] = (A[k * n + i] / scale);
          s += (A[k * n + i] * A[k * n + i]);
        }

        f = A[i * n + i];
        g = -SIGN2(sqrt(s), f);
        h = f * g - s;
        A[i * n + i] = (f - g);

        if (i != n - 1) {
          for (j = l; j < n; j++) {
            for (s = 0.0, k = i; k < m; k++) {
              s += (A[k * n + i] * A[k * n + j]);
            }
            f = s / h;
            for (k = i; k < m; k++) {
              A[k * n + j] += (f * A[k * n + i]);
            }
          }
        }

        for (k = i; k < m; k++) {
          A[k * n + i] = (A[k * n + i] * scale);
        }
      }
    }
    w[i] = (scale * g);

    /* right-hand reduction */
    g = s = scale = 0.0;
    if (i < m && i != n - 1) {
      for (k = l; k < n; k++) {
        scale += fabs(A[i * n + k]);
      }

      if (scale) {
        for (k = l; k < n; k++) {
          A[i * n + k] = (A[i * n + k] / scale);
          s += (A[i * n + k] * A[i * n + k]);
        }

        f = A[i * n + l];
        g = -SIGN2(sqrt(s), f);
        h = f * g - s;
        A[i * n + l] = (f - g);

        for (k = l; k < n; k++) {
          rv1[k] = A[i * n + k] / h;
        }

        if (i != m - 1) {
          for (j = l; j < m; j++) {
            for (s = 0.0, k = l; k < n; k++) {
              s += (A[j * n + k] * A[i * n + k]);
            }
            for (k = l; k < n; k++) {
              A[j * n + k] += (s * rv1[k]);
            }
          }
        }
        for (k = l; k < n; k++)
          A[i * n + k] = (A[i * n + k] * scale);
      }
    }
    anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
  }

  /* Accumulate the right-hand transformation */
  for (i = n - 1; i >= 0; i--) {
    if (i < n - 1) {
      if (g) {
        for (j = l; j < n; j++) {
          V[j * n + i] = ((A[i * n + j] / A[i * n + l]) / g);
        }
        /* real_t division to avoid underflow */
        for (j = l; j < n; j++) {
          for (s = 0.0, k = l; k < n; k++) {
            s += (A[i * n + k] * V[k * n + j]);
          }
          for (k = l; k < n; k++) {
            V[k * n + j] += (s * V[k * n + i]);
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

  /* accumulate the left-hand transformation */
  for (i = n - 1; i >= 0; i--) {
    l = i + 1;
    g = w[i];
    if (i < n - 1) {
      for (j = l; j < n; j++) {
        A[i * n + j] = 0.0;
      }
    }
    if (g) {
      g = 1.0 / g;
      if (i != n - 1) {
        for (j = l; j < n; j++) {
          for (s = 0.0, k = l; k < m; k++) {
            s += (A[k * n + i] * A[k * n + j]);
          }
          f = (s / A[i * n + i]) * g;

          for (k = i; k < m; k++) {
            A[k * n + j] += (f * A[k * n + i]);
          }
        }
      }
      for (j = i; j < m; j++) {
        A[j * n + i] = (A[j * n + i] * g);
      }
    } else {
      for (j = i; j < m; j++) {
        A[j * n + i] = 0.0;
      }
    }
    ++A[i * n + i];
  }

  /* diagonalize the bidiagonal form */
  for (k = n - 1; k >= 0; k--) {     /* loop over singular values */
    for (its = 0; its < 30; its++) { /* loop over allowed iterations */
      flag = 1;
      for (l = k; l >= 0; l--) { /* test for splitting */
        nm = l - 1;
        if (fabs(rv1[l]) + anorm == anorm) {
          flag = 0;
          break;
        }
        if (fabs(w[nm]) + anorm == anorm)
          break;
      }
      if (flag) {
        c = 0.0;
        s = 1.0;
        for (i = l; i <= k; i++) {
          f = s * rv1[i];
          if (fabs(f) + anorm != anorm) {
            g = w[i];
            h = pythag(f, g);
            w[i] = h;
            h = 1.0 / h;
            c = g * h;
            s = (-f * h);
            for (j = 0; j < m; j++) {
              y = A[j * n + nm];
              z = A[j * n + i];
              A[j * n + nm] = y * c + z * s;
              A[j * n + i] = z * c - y * s;
            }
          }
        }
      }
      z = w[k];
      if (l == k) {    /* convergence */
        if (z < 0.0) { /* make singular value nonnegative */
          w[k] = (-z);
          for (j = 0; j < n; j++)
            V[j * n + k] = (-V[j * n + k]);
        }
        break;
      }
      if (its >= 30) {
        free((void *) rv1);
        fprintf(stderr, "No convergence after 30,000! iterations \n");
        return (0);
      }

      /* Shift from bottom 2 x 2 minor */
      x = w[l];
      nm = k - 1;
      y = w[nm];
      g = rv1[nm];
      h = rv1[k];
      f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
      g = pythag(f, 1.0);
      f = ((x - z) * (x + z) + h * ((y / (f + SIGN2(g, f))) - h)) / x;

      /* next QR transformation */
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
        y = y * c;
        for (jj = 0; jj < n; jj++) {
          x = V[(jj * n) + j];
          z = V[(jj * n) + i];
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
        f = (c * g) + (s * y);
        x = (c * y) - (s * g);
        for (jj = 0; jj < m; jj++) {
          y = A[jj * n + j];
          z = A[jj * n + i];
          A[jj * n + j] = (y * c + z * s);
          A[jj * n + i] = (z * c - y * s);
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
  unsigned char *data = stbi_load(file_path, &img_w, &img_h, &img_c, 0);
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

  /* Open csv file */
  FILE *csv_file = fopen(csv_path, "r");
  const int nb_rows = dsv_rows(csv_path);

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

  strcpy(camera->proj_model, proj_model);
  strcpy(camera->dist_model, dist_model);

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
