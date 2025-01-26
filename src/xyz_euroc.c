#include "xyz_euroc.h"

/**
 * Count number of lines in file
 * @returns Number of lines or `-1` for failure
 */
static size_t file_lines(const char *path) {
  FILE *fp = fopen(path, "r");
  size_t lines = 0;

  if (fp == NULL) {
    return -1;
  }

  while (EOF != (fscanf(fp, "%*[^\n]"), fscanf(fp, "%*c"))) {
    ++lines;
  }

  return lines;
}

/**
 * Print vector
 */
static void print_int_vector(const char *prefix, const int *v, const int n) {
  printf("%s: [", prefix);
  for (int i = 0; i < n; i++) {
    printf("%d ", v[i]);
  }
  printf("\b]\n");
}

/**
 * Print YAML Token
 */
inline void yaml_print_token(const yaml_token_t token) {
  switch (token.type) {
    case YAML_NO_TOKEN:
      printf("YAML_NO_TOKEN\n");
      break;
    case YAML_STREAM_START_TOKEN:
      printf("YAML_STREAM_START_TOKEN\n");
      break;
    case YAML_STREAM_END_TOKEN:
      printf("YAML_STREAM_END_TOKEN\n");
      break;

    case YAML_VERSION_DIRECTIVE_TOKEN:
      printf("YAML_VERSION_DIRECTIVE_TOKEN\n");
      break;
    case YAML_TAG_DIRECTIVE_TOKEN:
      printf("YAML_TAG_DIRECTIVE_TOKEN\n");
      break;
    case YAML_DOCUMENT_START_TOKEN:
      printf("YAML_DOCUMENT_START_TOKEN\n");
      break;
    case YAML_DOCUMENT_END_TOKEN:
      printf("YAML_DOCUMENT_END_TOKEN\n");
      break;

    case YAML_BLOCK_SEQUENCE_START_TOKEN:
      printf("YAML_BLOCK_SEQUENCE_START_TOKEN\n");
      break;
    case YAML_BLOCK_MAPPING_START_TOKEN:
      printf("YAML_BLOCK_MAPPING_START_TOKEN\n");
      break;
    case YAML_BLOCK_END_TOKEN:
      printf("YAML_BLOCK_END_TOKEN\n");
      break;

    case YAML_FLOW_SEQUENCE_START_TOKEN:
      printf("YAML_FLOW_SEQUENCE_START_TOKEN\n");
      break;
    case YAML_FLOW_SEQUENCE_END_TOKEN:
      printf("YAML_FLOW_SEQUENCE_END_TOKEN\n");
      break;
    case YAML_FLOW_MAPPING_START_TOKEN:
      printf("YAML_FLOW_MAPPING_START_TOKEN\n");
      break;
    case YAML_FLOW_MAPPING_END_TOKEN:
      printf("YAML_FLOW_MAPPING_END_TOKEN\n");
      break;

    case YAML_BLOCK_ENTRY_TOKEN:
      printf("YAML_BLOCK_ENTRY_TOKEN\n");
      break;
    case YAML_FLOW_ENTRY_TOKEN:
      printf("YAML_FLOW_ENTRY_TOKEN\n");
      break;
    case YAML_KEY_TOKEN:
      printf("YAML_KEY_TOKEN\n");
      break;
    case YAML_VALUE_TOKEN:
      printf("YAML_VALUE_TOKEN\n");
      break;

    case YAML_ALIAS_TOKEN:
      printf("YAML_ALIAS_TOKEN\n");
      break;
    case YAML_ANCHOR_TOKEN:
      printf("YAML_ANCHOR_TOKEN\n");
      break;
    case YAML_TAG_TOKEN:
      printf("YAML_TAG_TOKEN\n");
      break;
    case YAML_SCALAR_TOKEN:
      printf("YAML_SCALAR_TOKEN [%s]\n", token.data.scalar.value);
      break;

    default:
      printf("-\n");
      break;
  }
}

/**
 * Get key-value from yaml file
 */
static int
yaml_get(const char *yaml_file, const char *key, char value_type, void *value) {
  // Load calibration data
  yaml_parser_t parser;
  yaml_token_t token;

  // Open sensor file
  FILE *fp = fopen(yaml_file, "r");
  if (fp == NULL) {
    EUROC_FATAL("YAML file [%s] not found!\n", yaml_file);
    return -1;
  }

  // Initialize YAML parser
  yaml_parser_initialize(&parser);
  yaml_parser_set_input_file(&parser, fp);

  // Parse YAML data
  int state = 0;
  int match = 0;
  int found = 0;
  do {
    yaml_parser_scan(&parser, &token);

    switch (token.type) {
      case YAML_KEY_TOKEN:
        state = 0;
        break;
      case YAML_VALUE_TOKEN:
        state = 1;
        break;
      case YAML_SCALAR_TOKEN: {
        char *tk = (char *) token.data.scalar.value;

        // Check key
        if (state == 0 && strcmp(tk, key) == 0) {
          match = 1;
        }

        // Parse value
        if (state == 1 && match == 1) {
          if (value_type == 'd') {
            *(double *) value = strtod(tk, NULL);
          } else if (value_type == 's') {
            strcpy((char *) value, tk);
          } else if (value_type == 'i') {
            *(int *) value = strtol(tk, NULL, 10);
          } else {
            EUROC_FATAL("Unrecognized value type: '%c'!\n", value_type);
          }
          found = 1;
        }
        break;
      }
      default:
        break;
    }

    if (token.type != YAML_STREAM_END_TOKEN) {
      yaml_token_delete(&token);
    }
  } while (token.type != YAML_STREAM_END_TOKEN && found == 0);

  // Clean up
  yaml_token_delete(&token);
  yaml_parser_delete(&parser);
  fclose(fp);

  return (found) ? 0 : -1;
}

/**
 * Get vector from yaml file
 */
static int yaml_get_vector(const char *yaml_file,
                           const char *key,
                           const char value_type,
                           const int n,
                           void *v) {
  // Load calibration data
  yaml_parser_t parser;
  yaml_token_t token;

  // Open sensor file
  FILE *fp = fopen(yaml_file, "r");
  if (fp == NULL) {
    return -1;
  }

  // Initialize YAML parser
  yaml_parser_initialize(&parser);
  yaml_parser_set_input_file(&parser, fp);

  // Parse YAML data
  int done = 0;
  int state = 0;
  int match_key = 0;
  int found_key = 0;
  int v_idx = 0;

  do {
    yaml_parser_scan(&parser, &token);

    switch (token.type) {
      case YAML_KEY_TOKEN:
        state = 0;
        break;
      case YAML_VALUE_TOKEN:
        state = 1;
        break;
      case YAML_SCALAR_TOKEN: {
        char *tk = (char *) token.data.scalar.value;

        // Check key
        if (state == 0 && strcmp(tk, key) == 0) {
          match_key = 1;
          found_key = 1;
        }

        // Parse data
        if (match_key == 1 && state == 1) {
          if (value_type == 'd') {
            ((double *) v)[v_idx++] = strtod(tk, NULL);
          } else if (value_type == 'i') {
            ((int *) v)[v_idx++] = strtol(tk, NULL, 10);
          } else {
            EUROC_FATAL("Unrecognized value type: '%c'!\n", value_type);
          }
        }
        break;
      }
      default:
        break;
    }

    if (token.type != YAML_STREAM_END_TOKEN) {
      yaml_token_delete(&token);
    }
  } while (token.type != YAML_STREAM_END_TOKEN && done == 0);

  // Clean up
  yaml_token_delete(&token);
  yaml_parser_delete(&parser);
  fclose(fp);

  return (found_key && (n == v_idx)) ? 0 : -1;
}

/**
 * Get matrix from yaml file
 */
static int yaml_get_matrix(const char *yaml_file,
                           const char *key,
                           const int m,
                           const int n,
                           double *A) {
  // Load calibration data
  yaml_parser_t parser;
  yaml_token_t token;

  // Open sensor file
  FILE *fp = fopen(yaml_file, "r");
  if (fp == NULL) {
    return -1;
  }

  // Initialize YAML parser
  yaml_parser_initialize(&parser);
  yaml_parser_set_input_file(&parser, fp);

  // Parse YAML data
  int done = 0;
  int state = 0;
  int match_key = 0;
  int parse_rows = 0;
  int parse_cols = 0;
  int parse_data = 0;

  int num_rows = 0;
  int num_cols = 0;
  int tf_idx = 0;

  do {
    yaml_parser_scan(&parser, &token);
    // yaml_print_token(token);

    switch (token.type) {
      case YAML_KEY_TOKEN:
        state = 0;
        break;
      case YAML_VALUE_TOKEN:
        state = 1;
        break;
      case YAML_SCALAR_TOKEN: {
        char *tk = (char *) token.data.scalar.value;

        // Check key
        if (state == 0 && strcmp(tk, key) == 0) {
          match_key = 1;
        }

        // Parse rows
        if (match_key == 1 && state == 0 && strcmp(tk, "rows") == 0) {
          parse_rows = 1;
        } else if (match_key == 1 && state == 1 && parse_rows == 1) {
          num_rows = strtol(tk, NULL, 10);
          parse_rows = 0;
        }

        // Parse cols
        if (match_key == 1 && state == 0 && strcmp(tk, "cols") == 0) {
          parse_cols = 1;
        } else if (match_key == 1 && state == 1 && parse_cols == 1) {
          num_cols = strtol(tk, NULL, 10);
          parse_cols = 0;
        }

        // Parse data
        if (match_key == 1 && state == 0 && strcmp(tk, "data") == 0) {
          parse_data = 1;
        } else if (match_key == 1 && state == 1 && parse_data == 1) {
          // Pre-check
          if (num_rows != m || num_cols != n) {
            EUROC_LOG("Number of rows or columns expected != parsed\n");
            EUROC_LOG("rows expected: %d, got: %d\n", m, num_rows);
            EUROC_LOG("cols expected: %d, got: %d\n", m, num_cols);
          }

          // Set matrix
          A[tf_idx++] = strtod(tk, NULL);
          if (tf_idx >= (num_rows * num_cols)) {
            parse_data = 0;
            done = 1;
          }
        }
        break;
      }
      default:
        break;
    }

    if (token.type != YAML_STREAM_END_TOKEN) {
      yaml_token_delete(&token);
    }
  } while (token.type != YAML_STREAM_END_TOKEN && done == 0);

  // Clean up
  yaml_token_delete(&token);
  yaml_parser_delete(&parser);
  fclose(fp);

  return ((num_rows * num_cols) == tf_idx) ? 0 : -1;
}

/*****************************************************************************
 * euroc_imu_t
 ****************************************************************************/

/**
 * Load EuRoC IMU data
 */
euroc_imu_t *euroc_imu_load(const char *data_dir) {
  // Setup
  euroc_imu_t *data = MALLOC(euroc_imu_t, 1);

  // Form data and sensor paths
  char data_path[1024] = {0};
  char conf[1024] = {0};
  strcat(data_path, data_dir);
  strcat(data_path, "/data.csv");
  strcat(conf, data_dir);
  strcat(conf, "/sensor.yaml");

  // Open file for loading
  const size_t num_rows = file_lines(data_path);
  FILE *fp = fopen(data_path, "r");
  if (fp == NULL) {
    EUROC_FATAL("Failed to open [%s]!\n", data_path);
  }

  // Malloc
  assert(num_rows > 0);
  data->num_timestamps = 0;
  data->timestamps = MALLOC(timestamp_t, num_rows);
  data->w_B = MALLOC(double *, num_rows);
  data->a_B = MALLOC(double *, num_rows);

  // Parse file
  for (size_t i = 0; i < num_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double w[3] = {0};
    double a[3] = {0};
    int retval = fscanf(fp,
                        "%" SCNd64 ",%lf,%lf,%lf,%lf,%lf,%lf",
                        &ts,
                        &w[0],
                        &w[1],
                        &w[2],
                        &a[0],
                        &a[1],
                        &a[2]);
    if (retval != 7) {
      EUROC_FATAL("Failed to parse line in [%s]\n", data_path);
    }

    // Add data
    data->timestamps[data->num_timestamps] = ts;
    data->w_B[data->num_timestamps] = vec_malloc(w, 3);
    data->a_B[data->num_timestamps] = vec_malloc(a, 3);
    data->num_timestamps++;
  }
  fclose(fp);

  // Load sensor configuration
  // clang-format off
  yaml_get(conf, "sensor_type", 's', &data->sensor_type);
  yaml_get(conf, "comment", 's', &data->comment);
  yaml_get_matrix(conf, "T_BS", 4, 4, data->T_BS);
  yaml_get(conf, "rate_hz", 'd', &data->rate_hz);
  yaml_get(conf, "gyroscope_noise_density", 'd', &data->gyro_noise_density);
  yaml_get(conf, "gyroscope_random_walk", 'd', &data->gyro_random_walk);
  yaml_get(conf, "accelerometer_noise_density", 'd', &data->accel_noise_density);
  yaml_get(conf, "accelerometer_random_walk", 'd', &data->accel_random_walk);
  // clang-format on

  return data;
}

/**
 * Free EuRoC IMU data
 */
void euroc_imu_free(euroc_imu_t *data) {
  assert(data != NULL);
  free(data->timestamps);
  for (size_t k = 0; k < data->num_timestamps; k++) {
    free(data->w_B[k]);
    free(data->a_B[k]);
  }
  free(data->w_B);
  free(data->a_B);
  free(data);
}

/**
 * Print EuRoC IMU data
 */
void euroc_imu_print(const euroc_imu_t *data) {
  printf("sensor_type: %s\n", data->sensor_type);
  printf("comment: %s\n", data->comment);
  print_matrix("T_BS", data->T_BS, 4, 4);
  printf("rate_hz: %f\n", data->rate_hz);
  printf("gyroscope_noise_density: %f\n", data->gyro_noise_density);
  printf("gyroscope_random_walk: %f\n", data->gyro_random_walk);
  printf("accelerometer_noise_density: %f\n", data->accel_noise_density);
  printf("accelerometer_random_walk: %f\n", data->accel_random_walk);
}

/*****************************************************************************
 * euroc_camera_t
 ****************************************************************************/

/**
 * Load EuRoC camera data
 */
euroc_camera_t *euroc_camera_load(const char *data_dir, int is_calib_data) {
  // Setup
  euroc_camera_t *data = MALLOC(euroc_camera_t, 1);
  data->is_calib_data = is_calib_data;

  // Form data and sensor paths
  char data_path[1024] = {0};
  char conf[1024] = {0};
  strcat(data_path, data_dir);
  strcat(data_path, "/data.csv");
  strcat(conf, data_dir);
  strcat(conf, "/sensor.yaml");

  // Open file for loading
  const size_t num_rows = file_lines(data_path);
  FILE *fp = fopen(data_path, "r");
  if (fp == NULL) {
    EUROC_FATAL("Failed to open [%s]!\n", data_path);
  }

  // Malloc
  assert(num_rows > 0);
  data->num_timestamps = 0;
  data->timestamps = MALLOC(timestamp_t, num_rows);
  data->image_paths = MALLOC(char *, num_rows);

  // Parse file
  for (size_t i = 0; i < num_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    char filename[50] = {0};
    int retval = fscanf(fp, "%" SCNd64 ",%s", &ts, filename);
    if (retval != 2) {
      EUROC_FATAL("Failed to parse line in [%s]\n", data_path);
    }

    // Check if file exists
    char image_path[9046] = {0};
    strcat(image_path, data_dir);
    strcat(image_path, "/data/");
    strcat(image_path, filename);
    if (file_exists(image_path) == 0) {
      EUROC_FATAL("File [%s] does not exist!\n", image_path);
    }

    // Add data
    const int k = data->num_timestamps;
    data->timestamps[k] = ts;
    data->image_paths[k] = string_malloc(image_path);
    data->num_timestamps++;
  }
  fclose(fp);

  // Load sensor configuration
  yaml_get(conf, "sensor_type", 's', &data->sensor_type);
  yaml_get(conf, "comment", 's', &data->comment);
  yaml_get_matrix(conf, "T_BS", 4, 4, data->T_BS);
  yaml_get(conf, "rate_hz", 'd', &data->rate_hz);
  yaml_get_vector(conf, "resolution", 'i', 2, data->resolution);

  if (is_calib_data) {
    // Camera data is calibration data, thus there are no calibration
    // parameters
    data->camera_model[0] = '\0';
    memset(data->intrinsics, 0, 4 * sizeof(double));
    data->distortion_model[0] = '\0';
    memset(data->distortion_coefficients, 0, 4 * sizeof(double));

  } else {
    // Camera data is calibrated
    yaml_get(conf, "camera_model", 's', &data->camera_model);
    yaml_get_vector(conf, "intrinsics", 'd', 4, data->intrinsics);
    yaml_get(conf, "distortion_model", 's', &data->distortion_model);
    yaml_get_vector(conf, "distortion_coefficients", 'd', 4, data->intrinsics);
  }

  return data;
}

/**
 * Free EuRoC camera data
 */
void euroc_camera_free(euroc_camera_t *data) {
  free(data->timestamps);
  for (size_t k = 0; k < data->num_timestamps; k++) {
    free(data->image_paths[k]);
  }
  free(data->image_paths);
  free(data);
}

/**
 * EuRoC camera to output stream
 */
void euroc_camera_print(const euroc_camera_t *data) {
  printf("sensor_type: %s\n", data->sensor_type);
  printf("comment: %s\n", data->comment);
  print_matrix("T_BS", data->T_BS, 4, 4);
  printf("rate_hz: %f\n", data->rate_hz);
  print_int_vector("resolution", data->resolution, 2);
  if (data->is_calib_data == 0) {
    printf("camera_model: %s\n", data->camera_model);
    print_vector("intrinsics", data->intrinsics, 4);
    printf("distortion_model: %s\n", data->distortion_model);
    print_vector("distortion_coefficients", data->distortion_coefficients, 4);
  }
}

/*****************************************************************************
 * euroc_ground_truth_t
 ****************************************************************************/

/**
 * Load EuRoC ground truth data
 */
euroc_ground_truth_t *euroc_ground_truth_load(const char *data_dir) {
  // Setup
  euroc_ground_truth_t *data = MALLOC(euroc_ground_truth_t, 1);

  // Form data path
  char data_path[9046] = {0};
  strcat(data_path, data_dir);
  strcat(data_path, "/data.csv");

  // Open file for loading
  const size_t num_rows = file_lines(data_path);
  FILE *fp = fopen(data_path, "r");
  if (fp == NULL) {
    EUROC_FATAL("Failed to open [%s]!\n", data_path);
  }

  // Malloc
  assert(num_rows > 0);
  data->num_timestamps = 0;
  data->timestamps = MALLOC(timestamp_t, num_rows);
  data->p_RS_R = MALLOC(double *, num_rows);
  data->q_RS = MALLOC(double *, num_rows);
  data->v_RS_R = MALLOC(double *, num_rows);
  data->b_w_RS_S = MALLOC(double *, num_rows);
  data->b_a_RS_S = MALLOC(double *, num_rows);

  // Parse file
  char str_format[9046] = {0};
  strcat(str_format, "%" SCNd64 ",");     // Timestamp
  strcat(str_format, "%lf,%lf,%lf,");     // Position
  strcat(str_format, "%lf,%lf,%lf,%lf,"); // Quaternion
  strcat(str_format, "%lf,%lf,%lf,");     // Velocity
  strcat(str_format, "%lf,%lf,%lf,");     // Gyro bias
  strcat(str_format, "%lf,%lf,%lf");      // Accel bias

  for (size_t i = 0; i < num_rows; i++) {
    // Skip first line
    if (i == 0) {
      skip_line(fp);
      continue;
    }

    // Parse line
    timestamp_t ts = 0;
    double p[3] = {0};
    double q[4] = {0};
    double v[3] = {0};
    double w[3] = {0};
    double a[3] = {0};
    int retval = fscanf(fp,
                        str_format,
                        &ts,
                        &p[0],
                        &p[1],
                        &p[2],
                        &q[0],
                        &q[1],
                        &q[2],
                        &q[3],
                        &v[0],
                        &v[1],
                        &v[2],
                        &w[0],
                        &w[1],
                        &w[2],
                        &a[0],
                        &a[1],
                        &a[2]);
    if (retval != 17) {
      EUROC_FATAL("Failed to parse line in [%s]", data_path);
    }

    // Add data
    const int k = data->num_timestamps;
    data->timestamps[k] = ts;
    data->p_RS_R[k] = vec_malloc(p, 3);
    data->q_RS[k] = vec_malloc(q, 4);
    data->v_RS_R[k] = vec_malloc(v, 3);
    data->b_w_RS_S[k] = vec_malloc(w, 3);
    data->b_a_RS_S[k] = vec_malloc(a, 3);
    data->num_timestamps++;
  }
  fclose(fp);

  return data;
}

/**
 * Free EuRoC ground truth data
 */
void euroc_ground_truth_free(euroc_ground_truth_t *data) {
  free(data->timestamps);

  for (size_t k = 0; k < data->num_timestamps; k++) {
    free(data->p_RS_R[k]);
    free(data->q_RS[k]);
    free(data->v_RS_R[k]);
    free(data->b_w_RS_S[k]);
    free(data->b_a_RS_S[k]);
  }
  free(data->p_RS_R);
  free(data->q_RS);
  free(data->v_RS_R);
  free(data->b_w_RS_S);
  free(data->b_a_RS_S);

  free(data);
}

/*****************************************************************************
 * euroc_timeline_t
 ****************************************************************************/

static void timestamps_insertion_sort(timestamp_t *timestamps, const size_t n) {
  for (size_t i = 1; i < n; i++) {
    timestamp_t key = timestamps[i];
    size_t j = i - 1;

    while (j >= 0 && timestamps[j] > key) {
      timestamps[j + 1] = timestamps[j];
      j = j - 1;
    }
    timestamps[j + 1] = key;
  }
}

static void timestamps_unique(timestamp_t *set,
                              size_t *set_len,
                              const timestamp_t *timestamps,
                              const size_t num_timestamps) {
  for (size_t i = 0; i < num_timestamps; i++) {
    const timestamp_t ts_k = timestamps[i];

    // Check duplicate in set
    int dup = 0;
    for (size_t j = 0; j < *set_len; j++) {
      if (set[j] == ts_k) {
        dup = 1;
        break;
      }
    }

    // Add to set if no duplicate
    if (dup == 0) {
      set[*set_len] = ts_k;
      (*set_len)++;
    }
  }

  // Sort timestamps (just to be sure)
  // struct timespec t_start = tic();
  timestamps_insertion_sort(set, *set_len);
  // printf("sort: %f\n", toc(&t_start));
}

/**
 * Create EuRoC timeline
 */
euroc_timeline_t *euroc_timeline_create(const euroc_imu_t *imu0_data,
                                        const euroc_camera_t *cam0_data,
                                        const euroc_camera_t *cam1_data) {
  // Determine unique set of timestamps
  int max_len = 0;
  max_len += imu0_data->num_timestamps;
  max_len += cam0_data->num_timestamps;
  max_len += cam1_data->num_timestamps;

  size_t ts_set_len = 0;
  timestamp_t *ts_set = MALLOC(timestamp_t, max_len);
  timestamps_unique(ts_set,
                    &ts_set_len,
                    imu0_data->timestamps,
                    imu0_data->num_timestamps);
  timestamps_unique(ts_set,
                    &ts_set_len,
                    cam0_data->timestamps,
                    cam0_data->num_timestamps);
  timestamps_unique(ts_set,
                    &ts_set_len,
                    cam1_data->timestamps,
                    cam1_data->num_timestamps);

  // Create timeline
  euroc_timeline_t *timeline = MALLOC(euroc_timeline_t, 1);
  timeline->num_timestamps = ts_set_len;
  timeline->timestamps = ts_set;
  timeline->events = MALLOC(euroc_event_t, ts_set_len);

  size_t imu0_idx = 0;
  size_t cam0_idx = 0;
  size_t cam1_idx = 0;

  for (size_t k = 0; k < timeline->num_timestamps; k++) {
    const timestamp_t ts = timeline->timestamps[k];

    // imu0 event
    int has_imu0 = 0;
    double *acc = NULL;
    double *gyr = NULL;
    for (size_t i = imu0_idx; i < imu0_data->num_timestamps; i++) {
      if (imu0_data->timestamps[i] == ts) {
        has_imu0 = 1;
        acc = imu0_data->a_B[imu0_idx];
        gyr = imu0_data->w_B[imu0_idx];
        imu0_idx++;
        break;
      }
    }

    // cam0 event
    int has_cam0 = 0;
    char *cam0_image = NULL;
    for (size_t i = cam0_idx; i < cam0_data->num_timestamps; i++) {
      if (cam0_data->timestamps[i] == ts) {
        has_cam0 = 1;
        cam0_image = cam0_data->image_paths[cam0_idx];
        cam0_idx++;
        break;
      }
    }

    // cam1 event
    int has_cam1 = 0;
    char *cam1_image = NULL;
    for (size_t i = cam1_idx; i < cam1_data->num_timestamps; i++) {
      if (cam1_data->timestamps[i] == ts) {
        has_cam1 = 1;
        cam1_image = cam1_data->image_paths[cam1_idx];
        cam1_idx++;
        break;
      }
    }

    // Add to event
    euroc_event_t *event = &timeline->events[k];
    event->has_imu0 = has_imu0;
    event->has_cam0 = has_cam0;
    event->has_cam1 = has_cam1;
    event->ts = ts;
    event->imu0_idx = imu0_idx - 1;
    event->acc = acc;
    event->gyr = gyr;
    event->cam0_idx = cam0_idx - 1;
    event->cam0_image = cam0_image;
    event->cam1_idx = cam1_idx - 1;
    event->cam1_image = cam1_image;
  }

  return timeline;
}

/**
 * Free EuRoC timeline
 */
void euroc_timeline_free(euroc_timeline_t *timeline) {
  free(timeline->timestamps);
  free(timeline->events);
  free(timeline);
}

/*****************************************************************************
 * euroc_data_t
 ****************************************************************************/

/**
 * Load EuRoC data
 */
euroc_data_t *euroc_data_load(const char *data_path) {
  // Setup
  euroc_data_t *data = MALLOC(euroc_data_t, 1);

  // Load IMU data
  char imu0_path[9046] = {0};
  strcat(imu0_path, data_path);
  strcat(imu0_path, "/mav0/imu0");
  data->imu0_data = euroc_imu_load(imu0_path);

  // Load cam0 data
  char cam0_path[9046] = {0};
  strcat(cam0_path, data_path);
  strcat(cam0_path, "/mav0/cam0");
  data->cam0_data = euroc_camera_load(cam0_path, 0);

  // Load cam1 data
  char cam1_path[9046] = {0};
  strcat(cam1_path, data_path);
  strcat(cam1_path, "/mav0/cam1");
  data->cam1_data = euroc_camera_load(cam1_path, 0);

  // Load ground truth
  char gnd_path[9046] = {0};
  strcat(gnd_path, data_path);
  strcat(gnd_path, "/mav0/state_groundtruth_estimate0");
  data->ground_truth = euroc_ground_truth_load(gnd_path);

  // Create timeline
  data->timeline =
      euroc_timeline_create(data->imu0_data, data->cam0_data, data->cam1_data);

  return data;
}

/**
 * Free EuRoC data
 */
void euroc_data_free(euroc_data_t *data) {
  assert(data != NULL);
  euroc_imu_free(data->imu0_data);
  euroc_camera_free(data->cam0_data);
  euroc_camera_free(data->cam1_data);
  euroc_ground_truth_free(data->ground_truth);
  euroc_timeline_free(data->timeline);
  free(data);
}

/*****************************************************************************
 * euroc_calib_target_t
 ****************************************************************************/

/**
 * Load EuRoC calibration target configuration
 */
euroc_calib_target_t *euroc_calib_target_load(const char *conf) {
  euroc_calib_target_t *data = MALLOC(euroc_calib_target_t, 1);
  yaml_get(conf, "target_type", 's', &data->type);
  yaml_get(conf, "tagRows", 'i', &data->tag_rows);
  yaml_get(conf, "tagCols", 'i', &data->tag_cols);
  yaml_get(conf, "tagSize", 'd', &data->tag_size);
  yaml_get(conf, "tagSpacing", 'd', &data->tag_spacing);
  return data;
}

/**
 * Free EuRoC calibration target
 */
void euroc_calib_target_free(euroc_calib_target_t *target) {
  free(target);
}

/**
 * EuRoC calibration target to output stream
 */
void euroc_calib_target_print(const euroc_calib_target_t *target) {
  printf("target_type: %s\n", target->type);
  printf("tag_rows: %d\n", target->tag_rows);
  printf("tag_cols: %d\n", target->tag_cols);
  printf("tag_size: %f\n", target->tag_size);
  printf("tag_spacing: %f\n", target->tag_spacing);
}

/*****************************************************************************
 * euroc_calib_t
 ****************************************************************************/

/**
 * Load EuRoC calibration data
 */
euroc_calib_t *euroc_calib_load(const char *data_path) {
  // Setup
  euroc_calib_t *data = MALLOC(euroc_calib_t, 1);

  // Load IMU data
  char imu0_path[9046] = {0};
  strcat(imu0_path, data_path);
  strcat(imu0_path, "/mav0/imu0");
  data->imu0_data = euroc_imu_load(imu0_path);

  // Load cam0 data
  char cam0_path[9046] = {0};
  strcat(cam0_path, data_path);
  strcat(cam0_path, "/mav0/cam0");
  data->cam0_data = euroc_camera_load(cam0_path, 0);

  // Load cam1 data
  char cam1_path[9046] = {0};
  strcat(cam1_path, data_path);
  strcat(cam1_path, "/mav0/cam1");
  data->cam1_data = euroc_camera_load(cam1_path, 0);

  // Load calibration target data
  char target_path[9046] = {0};
  strcat(target_path, data_path);
  strcat(target_path, "/april_6x6.yaml");
  data->calib_target = euroc_calib_target_load(target_path);

  // Create timeline
  data->timeline =
      euroc_timeline_create(data->imu0_data, data->cam0_data, data->cam1_data);

  return data;
}

/**
 * Free EuRoC calibration data
 */
void euroc_calib_free(euroc_calib_t *data) {
  euroc_imu_free(data->imu0_data);
  euroc_camera_free(data->cam0_data);
  euroc_camera_free(data->cam1_data);
  euroc_calib_target_free(data->calib_target);
  euroc_timeline_free(data->timeline);
  free(data);
}
