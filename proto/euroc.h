#ifndef EUROC_H
#define EUROC_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>
#include <unistd.h>

#include <yaml.h>

#ifndef timestamp_t
#define timestamp_t int64_t
#endif

/** Macro that adds the ability to switch between C / C++ style mallocs */
#ifdef __cplusplus
#ifndef MALLOC
#define MALLOC(TYPE, N) (TYPE *) malloc(sizeof(TYPE) * (N));
#endif // MALLOC

#ifndef CALLOC
#define CALLOC(TYPE, N) (TYPE *) calloc((N), sizeof(TYPE));
#endif // CALLOC

#else

#ifndef MALLOC
#define MALLOC(TYPE, N) malloc(sizeof(TYPE) * (N));
#endif // MALLOC

#ifndef CALLOC
#define CALLOC(TYPE, N) calloc((N), sizeof(TYPE));
#endif // CALLOC
#endif // __cplusplus

/**
 * Fatal
 *
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifndef FATAL
#define FATAL(...)                                                             \
  do {                                                                         \
    fprintf(stderr, "[FATAL] [%s:%d:%s()]: ", __FILE__, __LINE__, __func__);   \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0);                                                                 \
  exit(-1)
#endif

/**
 * EuRoC IMU data
 */
typedef struct euroc_imu_t {
  // Data
  int num_timestamps;
  timestamp_t *timestamps;
  double **w_B;
  double **a_B;

  // Sensor properties
  char sensor_type[100];
  char comment[9046];
  double T_BS[4 * 4];
  double rate_hz;
  double gyro_noise_density;
  double gyro_random_walk;
  double accel_noise_density;
  double accel_random_walk;
} euroc_imu_t;

euroc_imu_t *euroc_imu_load(const char *data_dir);
void euroc_imu_free(euroc_imu_t *data);
void euroc_imu_print(const euroc_imu_t *data);

/*****************************************************************************
 * euroc_camera_t
 ****************************************************************************/

/**
 * EuRoC camera data
 */
typedef struct euroc_camera_t {
  // Data
  int num_timestamps;
  timestamp_t *timestamps;
  char **image_paths;

  // Sensor properties
  char *sensor_type;
  char *comment;
  double T_BS[4 * 4];
  double rate_hz;
  int resolution[2];
  char *camera_model;
  double *intrinsics;
  char *distortion_model;
  double *distortion_coefficients;
} euroc_camera_t;

euroc_camera_t *euroc_camera_load(const char *data_dir, int is_calib_data);
void euroc_camera_free(euroc_camera_t *data);
void euroc_camera_print(const euroc_camera_t *data);

/*****************************************************************************
 * euroc_ground_truth_t
 ****************************************************************************/

/**
 * EuRoC ground truth
 */
typedef struct euroc_ground_truth_t {
  // Data
  int num_timestamps;
  timestamp_t *timestamps;
  double **p_RS_R;
  double **q_RS;
  double **v_RS_R;
  double **b_w_RS_S;
  double **b_a_RS_S;

} euroc_ground_truth_t;

euroc_ground_truth_t *euroc_ground_truth_load(const char *data_dir);
void euroc_ground_truth_free(euroc_ground_truth_t *data);

/*****************************************************************************
 * euroc_data_t
 ****************************************************************************/

/**
 * EuRoC data
 */
typedef struct euroc_data_t {
  euroc_imu_t *imu_data;
  euroc_camera_t *cam0_data;
  euroc_camera_t *cam1_data;
  euroc_ground_truth_t *ground_truth;
  int image_size[2];

  timestamp_t ts_start;
  timestamp_t ts_end;
  timestamp_t ts_now;
  size_t time_index;
  size_t imu_index;
  size_t frame_index;

  // std::set<timestamp_t> timestamps;
  // std::map<timestamp_t, double> time;
  // std::multimap<timestamp_t, timeline_event_t> timeline;
} euroc_data_t;

euroc_data_t *euroc_data_load(const char *data_path);
void euroc_data_free(euroc_data_t *data);

//   void reset() {
//     ts_start = min_timestamp();
//     ts_end = max_timestamp();
//     ts_now = ts_start;
//     time_index = 0;
//     imu_index = 0;
//     frame_index = 0;
//   }

//   timestamp_t min_timestamp() {
//     const timestamp_t cam0_ts0 = cam0_data.timestamps.front();
//     const timestamp_t imu_ts0 = imu_data.timestamps.front();

//     timestamps_t first_ts{cam0_ts0, imu_ts0};
//     auto ts0 = std::min_element(first_ts.begin(), first_ts.end());
//     const size_t first_ts_index = std::distance(first_ts.begin(), ts0);

//     return first_ts[first_ts_index];
//   }

//   timestamp_t max_timestamp() {
//     const timestamp_t cam0_last_ts = cam0_data.timestamps.back();
//     const timestamp_t imu_last_ts = imu_data.timestamps.back();

//     timestamps_t last_ts{cam0_last_ts, imu_last_ts};
//     auto last_result = std::max_element(last_ts.begin(), last_ts.end());
//     const timestamp_t last_ts_index =
//         std::distance(last_ts.begin(), last_result);
//     const timestamp_t max_ts = last_ts[last_ts_index];

//     return max_ts;
//   }
// };

/*****************************************************************************
 * euroc_target_t
 ****************************************************************************/

/**
 * EuRoC calibration target
 */
typedef struct euroc_target_t {
  char type[100];
  int tag_rows;
  int tag_cols;
  double tag_size;
  double tag_spacing;
} euroc_target_t;

euroc_target_t *euroc_target_load(const char *conf);
void euroc_target_free(euroc_target_t *target);
void euroc_target_print(const euroc_target_t *target);

/*****************************************************************************
 * euroc_calib_t
 ****************************************************************************/

/**
 * EuRoC calibration data
 */
typedef struct euroc_calib_t {
  euroc_imu_t imu_data;
  euroc_camera_t cam0_data;
  euroc_camera_t cam1_data;
  euroc_target_t calib_target;
  // cv::Size image_size;
} euroc_calib_t;

euroc_calib_t *euroc_calib_load(const char *data_path);

//   timeline_t timeline() {
//     // Create timeline
//     timeline_t timeline;

//     // -- Add cam0 events
//     for (size_t i = 0; i < cam0_data.timestamps.size(); i++) {
//       const auto ts = cam0_data.timestamps[i];
//       const auto img_path = cam0_data.image_paths[i];
//       const timeline_event_t event{ts, 0, img_path};
//       timeline.add(event);
//     }
//     // -- Add cam1 events
//     for (size_t i = 0; i < cam1_data.timestamps.size(); i++) {
//       const auto ts = cam1_data.timestamps[i];
//       const auto img_path = cam1_data.image_paths[i];
//       const timeline_event_t event{ts, 1, img_path};
//       timeline.add(event);
//     }
//     // -- Add imu events
//     for (size_t i = 0; i < imu_data.timestamps.size(); i++) {
//       const auto ts = imu_data.timestamps[i];
//       const auto a_B = imu_data.a_B[i];
//       const auto w_B = imu_data.w_B[i];
//       const timeline_event_t event{ts, a_B, w_B};
//       timeline.add(event);
//     }

//     return timeline;
//   }
// };

#endif // EUROC_H

//////////////////////////////////////////////////////////////////////////////
//                             IMPLEMENTATION                               //
//////////////////////////////////////////////////////////////////////////////

#ifdef EUROC_IMPLEMENTATION

/**
 * Skip line in file.
 */
static void skip_line(FILE *fp) {
  assert(fp != NULL);

  char header[BUFSIZ];
  char *retval = fgets(header, BUFSIZ, fp);
  if (retval == NULL) {
    FATAL("Failed to skip line!\n");
  }
}

/**
 * Count number of lines in file
 * @returns Number of lines or `-1` for failure
 */
size_t file_lines(const char *path) {
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
 * Check if file exists.
 * @returns
 * - 1 File exists
 * - 0 File does not exist
 */
int file_exists(const char *fp) {
  return (access(fp, F_OK) == 0) ? 1 : 0;
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
 * Create new vector of length `n` in heap memory.
 * @returns Heap allocated vector
 */
double *vec_malloc(const double *x, const size_t n) {
  assert(n > 0);
  double *vec = CALLOC(double, n);
  for (size_t i = 0; i < n; i++) {
    vec[i] = x[i];
  }

  return vec;
}

/**
 * Print YAML Token
 */
void yaml_print_token(const yaml_token_t token) {
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
int yaml_get(const char *yaml_file,
             const char *key,
             char *value_type,
             void *value) {
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
          if (strcmp(value_type, "d") == 0) {
            *(double *) value = strtod(tk, NULL);
          } else if (strcmp(value_type, "s") == 0) {
            strcpy((char *) value, tk);
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
 * Get key-value from yaml file
 */
int yaml_get_tf(const char *yaml_file, const char *key, double *value) {
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

        // // Parse value
        // if (state == 1 && match == 1) {
        //   if (strcmp(value_type, "d") == 0) {
        //     *(double *) value = strtod(tk, NULL);
        //   } else if (strcmp(value_type, "s") == 0) {
        //     strcpy((char *) value, tk);
        //   }
        //   found = 1;
        // }
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
    FATAL("Failed to open [%s]!\n", data_path);
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
      FATAL("Failed to parse line in [%s]\n", data_path);
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
  yaml_get(conf, "sensor_type", "s", &data->sensor_type);
  yaml_get(conf, "comment", "s", &data->comment);
  // yaml_get_tf(conf, "T_BS", &data->T_BS);
  yaml_get(conf, "rate_hz", "d", &data->rate_hz);
  yaml_get(conf, "gyroscope_noise_density", "d", &data->gyro_noise_density);
  yaml_get(conf, "gyroscope_random_walk", "d", &data->gyro_random_walk);
  yaml_get(conf, "accelerometer_noise_density", "d", &data->accel_noise_density);
  yaml_get(conf, "accelerometer_random_walk", "d", &data->accel_random_walk);
  // clang-format on

  return data;
}

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
  // printf("T_BS:\n", data->T_BS);
  printf("rate_hz: %f\n", data->rate_hz);
  printf("gyroscope_noise_density: %f\n", data->gyro_noise_density);
  printf("gyroscope_random_walk: %f\n", data->gyro_random_walk);
  printf("accelerometer_noise_density: %f\n", data->accel_noise_density);
  printf("accelerometer_random_walk: %f\n", data->accel_random_walk);
}

/*****************************************************************************
 * euroc_camera_t
 ****************************************************************************/

euroc_camera_t *euroc_camera_load(const char *data_dir, int is_calib_data) {
  // Setup
  euroc_camera_t *data = MALLOC(euroc_camera_t, 1);

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
    FATAL("Failed to open [%s]!\n", data_path);
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
      FATAL("Failed to parse line in [%s]\n", data_path);
    }

    // Check if file exists
    char image_path[9046] = {0};
    strcat(image_path, data_dir);
    strcat(image_path, "/data/");
    strcat(image_path, filename);
    if (file_exists(image_path) == 0) {
      FATAL("File [%s] does not exist!\n", image_path);
    }

    // Add data
    const int k = data->num_timestamps;
    data->timestamps[k] = ts;
    data->image_paths[k] = string_malloc(image_path);
    data->num_timestamps++;
  }
  fclose(fp);

  // Load sensor configuration
  yaml_get(conf, "sensor_type", "s", &data->sensor_type);
  yaml_get(conf, "comment", "s", &data->comment);
  //  parse(config, "T_BS", T_BS);
  yaml_get(conf, "rate_hz", "d", &data->rate_hz);
  //  parse(config, "resolution", resolution);
  //  parse(config, "camera_model", camera_model, is_calib_data);
  yaml_get(conf, "camera_model", "s", &data->camera_model);
  //  parse(config, "intrinsics", intrinsics, is_calib_data);
  yaml_get(conf, "distortion_model", "s", &data->distortion_model);
  //  parse(config,
  //        "distortion_coefficients",
  //        distortion_coefficients,
  //        is_calib_data);

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

  free(data->sensor_type);
  free(data->comment);
  free(data->camera_model);
  free(data->intrinsics);
  free(data->distortion_model);
  free(data->distortion_coefficients);
  free(data);
}

/**
 * EuRoC camera to output stream
 */
void euroc_camera_print(const euroc_camera_t *data) {
  // clang-format off
  printf("sensor_type: %s\n", data->sensor_type);
  printf("comment: %s\n", data->comment);
  // printf("T_BS:\n", data->T_BS);
  printf("rate_hz: %f\n", data->rate_hz);
  // printf("resolution: %s\n", data->resolution.transpose());
  printf("camera_model: %s\n", data->camera_model);
  // printf("intrinsics: %s\n", data->intrinsics.transpose());
  printf("distortion_model: %s\n", data->distortion_model);
  // printf("distortion_coefficients: %s\n" <<
  // data.distortion_coefficients.transpose() << std::endl;
  // clang-format on
}

/*****************************************************************************
 * euroc_ground_truth_t
 ****************************************************************************/

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
    FATAL("Failed to open [%s]!\n", data_path);
  }

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
                        &q[1],
                        &q[2],
                        &q[3],
                        &q[0],
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
      FATAL("Failed to parse line in [%s]", data_path);
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
 * euroc_data_t
 ****************************************************************************/

euroc_data_t *euroc_data_load(const char *data_path) {
  //  // Load IMU data
  //  imu_data = euroc_imu_t{data_path + "/mav0/imu0"};
  //  for (size_t i = 0; i < imu_data.timestamps.size(); i++) {
  //    const timestamp_t ts = imu_data.timestamps[i];
  //    const vec3_t a_B = imu_data.a_B[i];
  //    const vec3_t w_B = imu_data.w_B[i];
  //    const auto imu_event = timeline_event_t{ts, a_B, w_B};
  //    timeline.insert({ts, imu_event});
  //  }

  //  // Load camera data
  //  // -- Load cam0 data
  //  const auto cam0_path = data_path + "/mav0/cam0";
  //  cam0_data = euroc_camera_t{cam0_path};
  //  for (size_t i = 0; i < cam0_data.timestamps.size(); i++) {
  //    const timestamp_t ts = cam0_data.timestamps[i];
  //    const auto image_path = cam0_data.image_paths[i];
  //    const auto cam0_event = timeline_event_t(ts, 0, image_path);
  //    timeline.insert({ts, cam0_event});
  //  }
  //  // -- Load cam1 data
  //  const auto cam1_path = data_path + "/mav0/cam1";
  //  cam1_data = euroc_camera_t{cam0_path};
  //  for (size_t i = 0; i < cam1_data.timestamps.size(); i++) {
  //    const timestamp_t ts = cam1_data.timestamps[i];
  //    const auto image_path = cam1_data.image_paths[i];
  //    const auto cam1_event = timeline_event_t(ts, 1, image_path);
  //    timeline.insert({ts, cam1_event});
  //  }
  //  // -- Set camera image size
  //  cv::Mat image = cv::imread(cam0_data.image_paths[0]);
  //  image_size = cv::Size(image.size());

  //  // Load ground truth
  //  const auto gt_path = data_path + "/mav0/state_groundtruth_estimate0";
  //  ground_truth = euroc_ground_truth_t{gt_path};

  //  // Process timestamps
  //  ts_start = min_timestamp();
  //  ts_end = max_timestamp();
  //  ts_now = ts_start;

  //  // Get timestamps and calculate relative time
  //  auto it = timeline.begin();
  //  auto it_end = timeline.end();
  //  while (it != it_end) {
  //    const timestamp_t ts = it->first;
  //    timestamps.insert(ts);
  //    time[ts] = ((double) ts - ts_start) * 1e-9;

  //    // Advance to next non-duplicate entry.
  //    do {
  //      ++it;
  //    } while (ts == it->first);
  //  }
  return NULL;
}

/*****************************************************************************
 * euroc_target_t
 ****************************************************************************/

/**
 * Load EuRoC calibration target configuration
 */
euroc_target_t *euroc_target_load(const char *conf) {
  euroc_target_t *data = MALLOC(euroc_target_t, 1);
  yaml_get(conf, "target_type", "s", &data->type);
  yaml_get(conf, "tagRows", "i", &data->tag_rows);
  yaml_get(conf, "tagCols", "i", &data->tag_cols);
  yaml_get(conf, "tagSize", "d", &data->tag_size);
  yaml_get(conf, "tagSpacing", "d", &data->tag_spacing);
  return data;
}

/**
 * Free EuRoC calibration target
 */
void euroc_target_free(euroc_target_t *target) {
  free(target);
}

/**
 * EuRoC calibration target to output stream
 */
void euroc_target_print(const euroc_target_t *target) {
  printf("target_type: %s\n", target->type);
  printf("tag_rows: %d\n", target->tag_rows);
  printf("tag_cols: %d\n", target->tag_cols);
  printf("tag_size: %f\n", target->tag_size);
  printf("tag_spacing: %f\n", target->tag_spacing);
}

/*****************************************************************************
 * euroc_calib_t
 ****************************************************************************/

euroc_calib_t *euroc_calib_load(const char *data_path) {
  // Setup
  euroc_calib_t *data = MALLOC(euroc_calib_t, 1);

  //  // Load IMU data
  //  const std::string imu_data_dir = data_path + "/mav0/imu0";
  //  imu_data = euroc_imu_t{imu_data_dir};

  //  // Load cam0 data
  //  const std::string cam0_dir = data_path + "/mav0/cam0";
  //  cam0_data = euroc_camera_t{cam0_dir, true};

  //  // Load cam1 data
  //  const std::string cam1_dir = data_path + "/mav0/cam1";
  //  cam1_data = euroc_camera_t{cam1_dir, true};

  //  // Check if cam0 has same amount of images as cam1
  //  const size_t cam0_nb_images = cam0_data.image_paths.size();
  //  const size_t cam1_nb_images = cam1_data.image_paths.size();
  //  if (cam0_nb_images != cam1_nb_images) {
  //    if (cam0_nb_images > cam1_nb_images) {
  //      cam0_data.timestamps.pop_back();
  //      cam0_data.image_paths.pop_back();
  //    } else if (cam0_nb_images < cam1_nb_images) {
  //      cam1_data.timestamps.pop_back();
  //      cam1_data.image_paths.pop_back();
  //    }
  //  }

  //  // Get image size
  //  const cv::Mat image = cv::imread(cam0_data.image_paths[0]);
  //  image_size = cv::Size(image.size());

  //  // Load calibration target data
  //  const std::string target_path = data_path + "/april_6x6.yaml";
  //  calib_target = euroc_target_t{target_path};

  return data;
}

#endif // EUROC_IMPLEMENTATION

//////////////////////////////////////////////////////////////////////////////
//                                UNITTESTS                                 //
//////////////////////////////////////////////////////////////////////////////

#ifdef EUROC_UNITTEST

#include <stdio.h>
#include <math.h>

// UNITESTS GLOBAL VARIABLES
static int nb_tests = 0;
static int nb_passed = 0;
static int nb_failed = 0;

#define ENABLE_TERM_COLORS 0
#if ENABLE_TERM_COLORS == 1
#define TERM_RED "\x1B[1;31m"
#define TERM_GRN "\x1B[1;32m"
#define TERM_WHT "\x1B[1;37m"
#define TERM_NRM "\x1B[1;0m"
#else
#define TERM_RED
#define TERM_GRN
#define TERM_WHT
#define TERM_NRM
#endif

/**
 * Run unittests
 * @param[in] test_name Test name
 * @param[in] test_ptr Pointer to unittest
 */
void run_test(const char *test_name, int (*test_ptr)()) {
  if ((*test_ptr)() == 0) {
    printf("-> [%s] " TERM_GRN "OK!\n" TERM_NRM, test_name);
    fflush(stdout);
    nb_passed++;
  } else {
    printf(TERM_RED "FAILED!\n" TERM_NRM);
    fflush(stdout);
    nb_failed++;
  }
  nb_tests++;
}

/**
 * Add unittest
 * @param[in] TEST Test function
 */
#define TEST(TEST_FN) run_test(#TEST_FN, TEST_FN);

/**
 * Unit-test assert
 * @param[in] TEST Test condition
 */
#define TEST_ASSERT(TEST)                                                      \
  do {                                                                         \
    if ((TEST) == 0) {                                                         \
      printf(TERM_RED "ERROR!" TERM_NRM " [%s:%d] %s FAILED!\n",               \
             __func__,                                                         \
             __LINE__,                                                         \
             #TEST);                                                           \
      return -1;                                                               \
    }                                                                          \
  } while (0)

/**
 * Compare floats
 */
int fltcmp(const float x, const float y) {
  if (fabs(x - y) < 1e-10) {
    return 0;
  } else if (x > y) {
    return 1;
  }

  return -1;
}

int test_euroc_imu_load() {
  const char *data_dir = "/data/euroc/imu_april/mav0/imu0";
  euroc_imu_t *data = euroc_imu_load(data_dir);
  euroc_imu_print(data);
  euroc_imu_free(data);
  return 0;
}

int main(int argc, char *argv[]) {
  TEST(test_euroc_imu_load);
  return (nb_failed) ? -1 : 0;
}

#endif // EUROC_UNITTEST
