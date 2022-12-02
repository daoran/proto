#ifndef EUROC_H
#define EUROC_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>

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

//   euroc_ground_truth_t(const std::string &data_dir_) : data_dir{data_dir_} {
//     // Open file for loading
//     const std::string data_path = data_dir + "/data.csv";
//     int nb_rows = 0;
//     FILE *fp = file_open(data_path, "r", &nb_rows);
//     if (fp == NULL) {
//       FATAL("Failed to open [%s]!", data_path.c_str());
//     }

//     // Parse file
//     std::string str_format;
//     str_format += "%" SCNu64 ",";     // Timestamp
//     str_format += "%lf,%lf,%lf,";     // Position
//     str_format += "%lf,%lf,%lf,%lf,"; // Quaternion
//     str_format += "%lf,%lf,%lf,";     // Velocity
//     str_format += "%lf,%lf,%lf,";     // Gyro bias
//     str_format += "%lf,%lf,%lf";      // Accel bias

//     for (int i = 0; i < nb_rows; i++) {
//       // Skip first line
//       if (i == 0) {
//         skip_line(fp);
//         continue;
//       }

//       // Parse line
//       timestamp_t ts = 0;
//       double p_x, p_y, p_z = 0.0;
//       double q_x, q_y, q_z, q_w = 0.0;
//       double v_x, v_y, v_z = 0.0;
//       double b_w_x, b_w_y, b_w_z = 0.0;
//       double b_a_x, b_a_y, b_a_z = 0.0;
//       int retval = fscanf(fp,
//                           str_format.c_str(),
//                           &ts,
//                           &p_x,
//                           &p_y,
//                           &p_z,
//                           &q_x,
//                           &q_y,
//                           &q_z,
//                           &q_w,
//                           &v_x,
//                           &v_y,
//                           &v_z,
//                           &b_w_x,
//                           &b_w_y,
//                           &b_w_z,
//                           &b_a_x,
//                           &b_a_y,
//                           &b_a_z);
//       if (retval != 17) {
//         FATAL("Failed to parse line in [%s]", data_path.c_str());
//       }

//       timestamps.push_back(ts);
//       p_RS_R.emplace_back(p_x, p_y, p_z);
//       q_RS.emplace_back(q_w, q_x, q_y, q_z);
//       v_RS_R.emplace_back(v_x, v_y, v_z);
//       b_w_RS_S.emplace_back(b_w_x, b_w_y, b_w_z);
//       b_a_RS_S.emplace_back(b_a_x, b_a_y, b_a_z);
//     }
//     fclose(fp);

//     ok = true;
//   }

//   ~euroc_ground_truth_t() {
//   }
// };

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
  // cv::Size image_size;

  timestamp_t ts_start;
  timestamp_t ts_end;
  timestamp_t ts_now;
  long time_index;
  long imu_index;
  long frame_index;

  // std::set<timestamp_t> timestamps;
  // std::map<timestamp_t, double> time;
  // std::multimap<timestamp_t, timeline_event_t> timeline;
} euroc_data_t;

//   euroc_data_t(const std::string &data_path)
//       : data_path{strip_end(data_path, "/")} {
//     // Load IMU data
//     imu_data = euroc_imu_t{data_path + "/mav0/imu0"};
//     for (size_t i = 0; i < imu_data.timestamps.size(); i++) {
//       const timestamp_t ts = imu_data.timestamps[i];
//       const vec3_t a_B = imu_data.a_B[i];
//       const vec3_t w_B = imu_data.w_B[i];
//       const auto imu_event = timeline_event_t{ts, a_B, w_B};
//       timeline.insert({ts, imu_event});
//     }

//     // Load camera data
//     // -- Load cam0 data
//     const auto cam0_path = data_path + "/mav0/cam0";
//     cam0_data = euroc_camera_t{cam0_path};
//     for (size_t i = 0; i < cam0_data.timestamps.size(); i++) {
//       const timestamp_t ts = cam0_data.timestamps[i];
//       const auto image_path = cam0_data.image_paths[i];
//       const auto cam0_event = timeline_event_t(ts, 0, image_path);
//       timeline.insert({ts, cam0_event});
//     }
//     // -- Load cam1 data
//     const auto cam1_path = data_path + "/mav0/cam1";
//     cam1_data = euroc_camera_t{cam0_path};
//     for (size_t i = 0; i < cam1_data.timestamps.size(); i++) {
//       const timestamp_t ts = cam1_data.timestamps[i];
//       const auto image_path = cam1_data.image_paths[i];
//       const auto cam1_event = timeline_event_t(ts, 1, image_path);
//       timeline.insert({ts, cam1_event});
//     }
//     // -- Set camera image size
//     cv::Mat image = cv::imread(cam0_data.image_paths[0]);
//     image_size = cv::Size(image.size());

//     // Load ground truth
//     const auto gt_path = data_path + "/mav0/state_groundtruth_estimate0";
//     ground_truth = euroc_ground_truth_t{gt_path};

//     // Process timestamps
//     ts_start = min_timestamp();
//     ts_end = max_timestamp();
//     ts_now = ts_start;

//     // Get timestamps and calculate relative time
//     auto it = timeline.begin();
//     auto it_end = timeline.end();
//     while (it != it_end) {
//       const timestamp_t ts = it->first;
//       timestamps.insert(ts);
//       time[ts] = ((double) ts - ts_start) * 1e-9;

//       // Advance to next non-duplicate entry.
//       do {
//         ++it;
//       } while (ts == it->first);
//     }

//     ok = true;
//   }

//   ~euroc_data_t() {
//   }

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

// /*****************************************************************************
//  * euroc_target_t
//  ****************************************************************************/

// /**
//  * EuRoC calibration target
//  */
// struct euroc_target_t {
//   bool ok = false;
//   std::string file_path;

//   std::string type;
//   int tag_rows = 0;
//   int tag_cols = 0;
//   double tag_size = 0.0;
//   double tag_spacing = 0.0;

//   euroc_target_t() {
//   }

//   euroc_target_t(const std::string &target_file) : file_path{target_file} {
//     config_t config{target_file};
//     if (config.ok != true) {
//       FATAL("Failed to load target file [%s]!", target_file.c_str());
//     }
//     parse(config, "target_type", type);
//     parse(config, "tagRows", tag_rows);
//     parse(config, "tagCols", tag_cols);
//     parse(config, "tagSize", tag_size);
//     parse(config, "tagSpacing", tag_spacing);

//     ok = true;
//   }

//   ~euroc_target_t() {
//   }
// };

// /**
//  * EuRoC target to output stream
//  */
// std::ostream &operator<<(std::ostream &os, const euroc_target_t &target) {
//   os << "target_type: " << target.type << std::endl;
//   os << "tag_rows: " << target.tag_rows << std::endl;
//   os << "tag_cols: " << target.tag_cols << std::endl;
//   os << "tag_size: " << target.tag_size << std::endl;
//   os << "tag_spacing: " << target.tag_spacing << std::endl;
//   return os;
// }

// /*****************************************************************************
//  * euroc_calib_t
//  ****************************************************************************/

// /**
//  * EuRoC calibration data
//  */
// struct euroc_calib_t {
//   bool ok = false;

//   // Settings
//   std::string data_path;
//   bool imshow = false;

//   // Data
//   euroc_imu_t imu_data;
//   euroc_camera_t cam0_data;
//   euroc_camera_t cam1_data;
//   euroc_target_t calib_target;
//   cv::Size image_size;

//   euroc_calib_t(const std::string &data_path)
//       : data_path{strip_end(data_path, "/")} {
//     // Load IMU data
//     const std::string imu_data_dir = data_path + "/mav0/imu0";
//     imu_data = euroc_imu_t{imu_data_dir};

//     // Load cam0 data
//     const std::string cam0_dir = data_path + "/mav0/cam0";
//     cam0_data = euroc_camera_t{cam0_dir, true};

//     // Load cam1 data
//     const std::string cam1_dir = data_path + "/mav0/cam1";
//     cam1_data = euroc_camera_t{cam1_dir, true};

//     // Check if cam0 has same amount of images as cam1
//     const size_t cam0_nb_images = cam0_data.image_paths.size();
//     const size_t cam1_nb_images = cam1_data.image_paths.size();
//     if (cam0_nb_images != cam1_nb_images) {
//       if (cam0_nb_images > cam1_nb_images) {
//         cam0_data.timestamps.pop_back();
//         cam0_data.image_paths.pop_back();
//       } else if (cam0_nb_images < cam1_nb_images) {
//         cam1_data.timestamps.pop_back();
//         cam1_data.image_paths.pop_back();
//       }
//     }

//     // Get image size
//     const cv::Mat image = cv::imread(cam0_data.image_paths[0]);
//     image_size = cv::Size(image.size());

//     // Load calibration target data
//     const std::string target_path = data_path + "/april_6x6.yaml";
//     calib_target = euroc_target_t{target_path};

//     ok = true;
//   }

//   ~euroc_calib_t() {
//   }

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
        char *tk = token.data.scalar.value;

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
int yaml_get_tf(const char *yaml_file, const char *key, void *value) {
  // Load calibration data
  yaml_parser_t parser;
  yaml_token_t token;

  // -- Open sensor file
  FILE *fp = fopen(yaml_file, "r");
  if (fp == NULL) {
    return -1;
  }

  // -- Initialize YAML parser
  yaml_parser_initialize(&parser);
  yaml_parser_set_input_file(&parser, fp);

  int found_rows = 0;
  int found_cols = 0;
  // int found_data = 0;

  // -- Parse YAML data
  do {
    yaml_parser_scan(&parser, &token);

    switch (token.type) {
      case YAML_KEY_TOKEN:
        // Key token
        yaml_token_delete(&token);

        // Key
        yaml_parser_scan(&parser, &token);
        if (strcmp((char *) token.data.scalar.value, key) != 0) {
          break;
        }
        yaml_token_delete(&token);

        // Value token
        yaml_parser_scan(&parser, &token);
        yaml_token_delete(&token);

        // Block mapping
        yaml_parser_scan(&parser, &token);
        yaml_print_token(token);
        yaml_token_delete(&token);

        // field 'rows'
        // -- Key token
        yaml_parser_scan(&parser, &token);
        yaml_token_delete(&token);
        // -- Key
        yaml_parser_scan(&parser, &token);
        if (strcmp((char *) token.data.scalar.value, "rows") == 0) {
          yaml_token_delete(&token);
          // -- Value token
          yaml_parser_scan(&parser, &token);
          yaml_token_delete(&token);
          // -- Value
          yaml_parser_scan(&parser, &token);
          printf("rows: %s\n", (char *) token.data.scalar.value);
          yaml_token_delete(&token);
          found_rows = 1;
          break;
        }

        // field 'cols'
        // -- Key token
        yaml_parser_scan(&parser, &token);
        yaml_token_delete(&token);
        // -- Key
        yaml_parser_scan(&parser, &token);
        if (strcmp((char *) token.data.scalar.value, "cols") == 0) {
          yaml_token_delete(&token);
          // -- Value token
          yaml_parser_scan(&parser, &token);
          yaml_token_delete(&token);
          // -- Value
          yaml_parser_scan(&parser, &token);
          printf("cols: %s\n", (char *) token.data.scalar.value);
          yaml_token_delete(&token);
          found_cols = 1;
          break;
        }

        // // field 'data'
        // // -- Key token
        // yaml_parser_scan(&parser, &token);
        // yaml_token_delete(&token);
        // // -- Key
        // yaml_parser_scan(&parser, &token);
        // if (strcmp((char *) token.data.scalar.value, "data") != 0) {
        //   return -1;
        // }
        // yaml_token_delete(&token);
        // // -- Value token
        // yaml_parser_scan(&parser, &token);
        // yaml_token_delete(&token);
        // // -- Value
        // yaml_parser_scan(&parser, &token);
        // printf("rows: %s\n", (char *) token.data.scalar.value);
        // yaml_token_delete(&token);

        // for (int i = 0; i < 36; i++) {
        //   // Get next value
        //   yaml_parser_scan(&parser, &token);
        //   yaml_print_token(token);
        //   yaml_token_delete(&token);
        // }

        break;
      default:
        // Do nothing
        break;
    }

    // Clean up token
    if (token.type != YAML_STREAM_END_TOKEN) {
      yaml_token_delete(&token);
    }

  } while (token.type != YAML_STREAM_END_TOKEN);

  // Clean up
  yaml_token_delete(&token);
  yaml_parser_delete(&parser);
  fclose(fp);

  return 0;
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
    double wx, wy, wz = 0.0;
    double ax, ay, az = 0.0;
    int retval = fscanf(fp,
                        "%" SCNd64 ",%lf,%lf,%lf,%lf,%lf,%lf",
                        &ts,
                        &wx,
                        &wy,
                        &wz,
                        &ax,
                        &ay,
                        &az);
    if (retval != 7) {
      FATAL("Failed to parse line in [%s]\n", data_path);
    }

    // Add data
    data->timestamps[data->num_timestamps] = ts;
    data->w_B[data->num_timestamps] = MALLOC(double, 3);
    data->w_B[data->num_timestamps][0] = wx;
    data->w_B[data->num_timestamps][1] = wy;
    data->w_B[data->num_timestamps][2] = wz;
    data->a_B[data->num_timestamps] = MALLOC(double, 3);
    data->a_B[data->num_timestamps][0] = ax;
    data->a_B[data->num_timestamps][1] = ay;
    data->a_B[data->num_timestamps][2] = az;
    data->num_timestamps++;
  }
  fclose(fp);

  // clang-format off
  yaml_get(conf, "sensor_type", "s", &data->sensor_type);
  yaml_get(conf, "comment", "s", &data->comment);
  // // yaml_get_tf(conf, "T_BS", &data->T_BS);
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
  // // printf("T_BS:\n", data->T_BS);
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

  // // Parse file
  // for (size_t i = 0; i < num_rows; i++) {
  //   // Skip first line
  //   if (i == 0) {
  //     skip_line(fp);
  //     continue;
  //   }

  //   // Parse line
  //   timestamp_t ts = 0;
  //   char filename[50] = {0};
  //   int retval = fscanf(fp, "%" SCNu64 ",%s", &ts, filename);
  //   if (retval != 2) {
  //     FATAL("Failed to parse line in [%s]", data_path.c_str());
  //   }

  //   // // Check if file exists
  //   // const std::string image_file{filename};
  //   // const auto image_path = data_dir + "/data/" + image_file;
  //   // if (file_exists(image_path) == false) {
  //   //   FATAL("File [%s] does not exist!", image_path.c_str());
  //   // }

  //   // Add data
  //   timestamps.emplace_back(ts);
  //   image_paths.emplace_back(image_path);
  // }
  // fclose(fp);

  //     // Load calibration data
  //     config_t config{sensor_path};
  //     if (config.ok == false) {
  //       FATAL("Failed to load senor file [%s]!", sensor_path.c_str());
  //     }
  //     parse(config, "sensor_type", sensor_type);
  //     parse(config, "comment", comment);
  //     parse(config, "T_BS", T_BS);
  //     parse(config, "rate_hz", rate_hz);
  //     parse(config, "resolution", resolution);
  //     parse(config, "camera_model", camera_model, is_calib_data);
  //     parse(config, "intrinsics", intrinsics, is_calib_data);
  //     parse(config, "distortion_model", distortion_model, is_calib_data);
  //     parse(config,
  //           "distortion_coefficients",
  //           distortion_coefficients,
  //           is_calib_data);

  return data;
}

/**
 * EuRoC camera to output stream
 */
void euroc_camera_print(const euroc_camera_t *data) {
  // clang-format off
  printf("sensor_type: %s\n", data->sensor_type);
  printf("comment: %s\n", data->comment);
  // printf("T_BS:\n" << data.T_BS);
  // printf("rate_hz: %s\n" << data.rate_hz);
  // printf("resolution: %s\n" << data.resolution.transpose());
  // printf("camera_model: %s\n" << data.camera_model);
  // printf("intrinsics: %s\n" << data.intrinsics.transpose());
  // printf("distortion_model: %s\n" << data.distortion_model);
  // printf("distortion_coefficients: %s\n" <<
  // data.distortion_coefficients.transpose() << std::endl;
  // clang-format on
}

/*****************************************************************************
 * euroc_ground_truth_t
 ****************************************************************************/

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
