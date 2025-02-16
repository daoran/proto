#include "xyz_kitti.h"

/**
 * Fatal
 *
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifndef KITTI_FATAL
#define KITTI_FATAL(...)                                                       \
  do {                                                                         \
    fprintf(stderr,                                                            \
            "[KITTI_FATAL] [%s:%d:%s()]: ",                                    \
            __FILE__,                                                          \
            __LINE__,                                                          \
            __func__);                                                         \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0);                                                                 \
  exit(-1)
#endif

#ifndef KITTI_LOG
#define KITTI_LOG(...)                                                         \
  do {                                                                         \
    fprintf(stderr,                                                            \
            "[KITTI_LOG] [%s:%d:%s()]: ",                                      \
            __FILE__,                                                          \
            __LINE__,                                                          \
            __func__);                                                         \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0);
#endif

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
 * Parse date time string to timestamp in nanoseconds (Unix time)
 */
static timestamp_t parse_dateline(const char *dt_str) {
  // Parse
  struct tm tm;
  int fractional_seconds;
  memset(&tm, 0, sizeof(struct tm));
  sscanf(dt_str,
         "%d-%d-%d %d:%d:%d.%d",
         &tm.tm_year,
         &tm.tm_mon,
         &tm.tm_mday,
         &tm.tm_hour,
         &tm.tm_min,
         &tm.tm_sec,
         &fractional_seconds);
  tm.tm_year -= 1900; // Adjust for tm_year (years since 1900)
  tm.tm_mon -= 1;     // Adjust for 0-based months

  // Convert to time_t (Unix timestamp)
  time_t timestamp = mktime(&tm);
  if (timestamp == -1) {
    KITTI_FATAL("Failed to convert time to timestamp");
  }

  // Convert to uint64_t: Combine seconds and fractional part
  return (uint64_t) timestamp * 1000000000LL + fractional_seconds;
}

/**
 * Parse line
 */
static void
parse_value(FILE *fp, const char *key, const char *value_type, void *value) {
  assert(fp != NULL);
  assert(key != NULL);
  assert(value_type != NULL);
  assert(value != NULL);

  // Parse line
  const size_t buf_len = 1024;
  char buf[1024] = {0};
  if (fgets(buf, buf_len, fp) == NULL) {
    KITTI_FATAL("Failed to parse [%s]\n", key);
  }

  // Split key-value
  char delim[2] = ":";
  char *key_str = strtok(buf, delim);
  char *value_str = strtok(NULL, "\n");

  // Check key matches
  if (strcmp(key_str, key) != 0) {
    KITTI_FATAL("Failed to parse [%s]\n", key);
  }

  // Typecase value
  if (value_type == NULL) {
    KITTI_FATAL("Value type not set!\n");
  }

  if (strcmp(value_type, "string") == 0) {
    strcpy((char *) value, value_str);
  } else if (strcmp(value_type, "uint64_t") == 0) {
    *(uint64_t *) value = atol(value_str);
  } else if (strcmp(value_type, "int") == 0) {
    *(int *) value = atoi(value_str);
  } else if (strcmp(value_type, "float") == 0) {
    *(float *) value = atof(value_str);
  } else if (strcmp(value_type, "double") == 0) {
    *(double *) value = atof(value_str);
  } else {
    KITTI_FATAL("Invalid value type [%s]\n", value_type);
  }
}

static void
parse_float_array(FILE *fp, const char *key, double *array, int size) {
  assert(fp != NULL);
  assert(key != NULL);
  assert(array != NULL);

  // Parse line
  const size_t buf_len = 1024;
  char buf[1024] = {0};
  if (fgets(buf, buf_len, fp) == NULL) {
    KITTI_FATAL("Failed to parse [%s]\n", key);
  }

  // Split key-value
  char delim[2] = ":";
  char *key_str = strtok(buf, ":");
  char *value_str = strtok(NULL, delim);

  // Check key matches
  if (strcmp(key_str, key) != 0) {
    KITTI_FATAL("Failed to parse [%s]\n", key);
  }

  int i = 0;
  char *token = strtok(value_str, " ");
  while (token != NULL && i < size) {
    array[i++] = strtof(token, NULL);
    token = strtok(NULL, " ");
  }
}

/*****************************************************************************
 * kitti_camera_t
 ****************************************************************************/

kitti_camera_t *kitti_camera_load(const char *data_dir) {
  // Form timestamps path
  char timestamps_path[1024] = {0};
  strcat(timestamps_path, data_dir);
  strcat(timestamps_path, "/timestamps.txt");

  // Open timestamps file
  const size_t num_rows = file_lines(timestamps_path);
  FILE *fp = fopen(timestamps_path, "r");
  if (fp == NULL) {
    KITTI_FATAL("Failed to open [%s]!\n", timestamps_path);
  }

  // Parse
  assert(num_rows > 0);
  kitti_camera_t *data = MALLOC(kitti_camera_t, 1);
  data->num_timestamps = num_rows;
  data->timestamps = MALLOC(timestamp_t, num_rows);
  data->image_paths = MALLOC(char *, num_rows);

  for (int i = 0; i < num_rows; ++i) {
    // Timestamp
    char line[1024] = {0};
    fgets(line, sizeof(line), fp);
    timestamp_t ts = parse_dateline(line);
    data->timestamps[i] = ts;

    // Image path
    char image_path[1024] = {0};
    sprintf(image_path, "%s/%s/%010d.png", data_dir, "data", i);
    data->image_paths[i] = MALLOC(char, strlen(image_path) + 1);
    strcpy(data->image_paths[i], image_path);
  }
  fclose(fp);

  return data;
}

void kitti_camera_free(kitti_camera_t *data) {
  // Timestamps
  free(data->timestamps);

  // Image paths
  for (size_t i = 0; i < data->num_timestamps; ++i) {
    free(data->image_paths[i]);
  }
  free(data->image_paths);

  free(data);
}

/*****************************************************************************
 * kitti_oxts_t
 ****************************************************************************/

kitti_oxts_t *kitti_oxts_load(const char *data_dir) {
  // Get number of measurements
  char timestamps_path[1024] = {0};
  strcat(timestamps_path, data_dir);
  strcat(timestamps_path, "/timestamps.txt");
  const size_t num_rows = file_lines(timestamps_path);

  // Parse setup
  assert(num_rows > 0);
  kitti_oxts_t *data = MALLOC(kitti_oxts_t, 1);
  // -- Timestamps
  data->num_timestamps = num_rows;
  data->timestamps = MALLOC(timestamp_t, num_rows);
  // -- GPS
  data->lat = MALLOC(double, num_rows);
  data->lon = MALLOC(double, num_rows);
  data->alt = MALLOC(double, num_rows);
  // -- Attitude
  data->roll = MALLOC(double, num_rows);
  data->pitch = MALLOC(double, num_rows);
  data->yaw = MALLOC(double, num_rows);
  // -- Velocity
  data->vn = MALLOC(double, num_rows);
  data->ve = MALLOC(double, num_rows);
  data->vf = MALLOC(double, num_rows);
  data->vl = MALLOC(double, num_rows);
  data->vu = MALLOC(double, num_rows);
  // -- Acceleration
  data->ax = MALLOC(double, num_rows);
  data->ay = MALLOC(double, num_rows);
  data->az = MALLOC(double, num_rows);
  data->af = MALLOC(double, num_rows);
  data->al = MALLOC(double, num_rows);
  data->au = MALLOC(double, num_rows);
  // -- Angular velocity
  data->wx = MALLOC(double, num_rows);
  data->wy = MALLOC(double, num_rows);
  data->wz = MALLOC(double, num_rows);
  data->wf = MALLOC(double, num_rows);
  data->wl = MALLOC(double, num_rows);
  data->wu = MALLOC(double, num_rows);
  // -- Satellite tracking
  data->pos_accuracy = MALLOC(double, num_rows);
  data->vel_accuracy = MALLOC(double, num_rows);
  data->navstat = MALLOC(int, num_rows);
  data->numsats = MALLOC(int, num_rows);
  data->posmode = MALLOC(int, num_rows);
  data->velmode = MALLOC(int, num_rows);
  data->orimode = MALLOC(int, num_rows);

  // -- Parse timestamps
  {
    FILE *fp = fopen(timestamps_path, "r");
    if (fp == NULL) {
      KITTI_FATAL("Failed to open [%s]!\n", timestamps_path);
    }
    for (int i = 0; i < num_rows; ++i) {
      char line[1024] = {0};
      fgets(line, sizeof(line), fp);
      timestamp_t ts = parse_dateline(line);
      data->timestamps[i] = ts;
    }
    fclose(fp);
  }

  // -- Parse oxts data
  {
    char format[1024] = {0};
    strcat(format, "%lf %lf %lf ");             // GPS
    strcat(format, "%lf %lf %lf ");             // Attitude
    strcat(format, "%lf %lf %lf %lf %lf ");     // Velocity
    strcat(format, "%lf %lf %lf %lf %lf %lf "); // Acceleration
    strcat(format, "%lf %lf %lf %lf %lf %lf "); // Angular velocity
    strcat(format, "%lf %lf %d %d %d %d %d");   // Satellite tracking

    for (int i = 0; i < num_rows; ++i) {
      // Open oxts entry
      char entry_path[1024] = {0};
      sprintf(entry_path, "%s/%s/%010d.txt", data_dir, "data", i);
      FILE *fp = fopen(entry_path, "r");
      if (fp == NULL) {
        KITTI_FATAL("Failed to open [%s]!\n", timestamps_path);
      }

      // Others
      int retval = fscanf(fp,
                          format,
                          // GPS
                          &data->lat[i],
                          &data->lon[i],
                          &data->alt[i],
                          // Attitude
                          &data->roll[i],
                          &data->pitch[i],
                          &data->yaw[i],
                          // Velocity
                          &data->vn[i],
                          &data->ve[i],
                          &data->vf[i],
                          &data->vl[i],
                          &data->vu[i],
                          // Acceleration
                          &data->ax[i],
                          &data->ay[i],
                          &data->az[i],
                          &data->af[i],
                          &data->al[i],
                          &data->au[i],
                          // Angular velocity
                          &data->wx[i],
                          &data->wy[i],
                          &data->wz[i],
                          &data->wf[i],
                          &data->wl[i],
                          &data->wu[i],
                          // Satellite tracking
                          &data->pos_accuracy[i],
                          &data->vel_accuracy[i],
                          &data->navstat[i],
                          &data->numsats[i],
                          &data->posmode[i],
                          &data->velmode[i],
                          &data->orimode[i]);
      if (retval != 30) {
        KITTI_FATAL("Failed to parse line in [%s]\n", entry_path);
      }
      fclose(fp);
    }
  }

  return data;
}

void kitti_oxts_free(kitti_oxts_t *data) {
  // Timestamps
  free(data->timestamps);

  // GPS
  free(data->lat);
  free(data->lon);
  free(data->alt);

  // Attitude
  free(data->roll);
  free(data->pitch);
  free(data->yaw);

  // Velocity
  free(data->vn);
  free(data->ve);
  free(data->vf);
  free(data->vl);
  free(data->vu);

  // Acceleration
  free(data->ax);
  free(data->ay);
  free(data->az);
  free(data->af);
  free(data->al);
  free(data->au);

  // Angular velocity
  free(data->wx);
  free(data->wy);
  free(data->wz);
  free(data->wf);
  free(data->wl);
  free(data->wu);

  // Satellite tracking
  free(data->pos_accuracy);
  free(data->vel_accuracy);
  free(data->navstat);
  free(data->numsats);
  free(data->posmode);
  free(data->velmode);
  free(data->orimode);

  free(data);
}

/*****************************************************************************
 * kitti_velodyne_t
 ****************************************************************************/

static timestamp_t *load_timestamps(const char *file_path) {
  const size_t num_rows = file_lines(file_path);
  if (num_rows == 0) {
    return NULL;
  }

  FILE *fp = fopen(file_path, "r");
  if (fp == NULL) {
    KITTI_FATAL("Failed to open [%s]!\n", file_path);
  }

  timestamp_t *timestamps = MALLOC(timestamp_t, num_rows);
  for (int i = 0; i < num_rows; ++i) {
    char line[1024] = {0};
    fgets(line, sizeof(line), fp);
    timestamps[i] = parse_dateline(line);
  }

  return timestamps;
}

point_xyzr_t *kitti_load_points(const char *pcd_path) {
  // Load pcd file
  FILE *pcd_file = fopen(pcd_path, "rb");
  if (!pcd_file) {
    KITTI_LOG("Failed to open [%s]", pcd_path);
    return NULL;
  }

  // Get the size of the file to know how many points
  fseek(pcd_file, 0, SEEK_END);
  const long file_size = ftell(pcd_file);
  fseek(pcd_file, 0, SEEK_SET);

  // Allocate memory for the points
  const int num_points = file_size / 16;
  point_xyzr_t *points = MALLOC(point_xyzr_t, num_points);
  if (!points) {
    KITTI_LOG("Failed to allocate memory for points");
    fclose(pcd_file);
    return NULL;
  }

  // Read points from the file
  const size_t point_size = sizeof(point_xyzr_t);
  const size_t read_count = fread(points, point_size, num_points, pcd_file);
  if (read_count != num_points) {
    KITTI_LOG("Failed to read all points");
    free(points);
    fclose(pcd_file);
    return NULL;
  }

  // Clean up
  fclose(pcd_file);

  return points;
}

kitti_velodyne_t *kitti_velodyne_load(const char *data_dir) {
  // Setup timestamp paths
  char timestamps_path[1024] = {0};
  char timestamps_start_path[1024] = {0};
  char timestamps_end_path[1024] = {0};
  sprintf(timestamps_path, "%s/timestamps.txt", data_dir);
  sprintf(timestamps_start_path, "%s/timestamps_start.txt", data_dir);
  sprintf(timestamps_end_path, "%s/timestamps_end.txt", data_dir);

  // Load data
  kitti_velodyne_t *data = MALLOC(kitti_velodyne_t, 1);
  data->num_timestamps = file_lines(timestamps_path);
  data->timestamps = load_timestamps(timestamps_path);
  data->timestamps_start = load_timestamps(timestamps_start_path);
  data->timestamps_end = load_timestamps(timestamps_end_path);
  data->pcd_paths = MALLOC(char *, data->num_timestamps);
  for (int i = 0; i < data->num_timestamps; ++i) {
    char pcd_path[1024] = {0};
    sprintf(pcd_path, "%s/%s/%010d.bin", data_dir, "data", i);
    data->pcd_paths[i] = MALLOC(char, strlen(pcd_path) + 1);
    strcpy(data->pcd_paths[i], pcd_path);
  }

  return data;
}

void kitti_velodyne_free(kitti_velodyne_t *data) {
  free(data->timestamps);
  free(data->timestamps_start);
  free(data->timestamps_end);
  for (int i = 0; i < data->num_timestamps; ++i) {
    free(data->pcd_paths[i]);
  }
  free(data->pcd_paths);
  free(data);
}

/*****************************************************************************
 * kitti_calib_t
 ****************************************************************************/

kitti_calib_t *kitti_calib_load(const char *data_dir) {
  kitti_calib_t *data = MALLOC(kitti_calib_t, 1);

  // Load camera calibrations
  {
    char calib_cam_to_cam_path[1024] = {0};
    sprintf(calib_cam_to_cam_path, "%s/calib_cam_to_cam.txt", data_dir);
    FILE *fp = fopen(calib_cam_to_cam_path, "r");
    if (fp == NULL) {
      KITTI_FATAL("Failed to load [%s]\n", calib_cam_to_cam_path);
    }

    parse_value(fp, "calib_time", "string", &data->calib_time_cam_to_cam);
    parse_value(fp, "corner_dist", "double", &data->corner_dist);

    parse_float_array(fp, "S_00", data->S_00, 2);
    parse_float_array(fp, "K_00", data->K_00, 9);
    parse_float_array(fp, "D_00", data->D_00, 5);
    parse_float_array(fp, "R_00", data->D_00, 9);
    parse_float_array(fp, "T_00", data->T_00, 3);
    parse_float_array(fp, "S_rect_00", data->S_rect_00, 2);
    parse_float_array(fp, "R_rect_00", data->R_rect_00, 9);
    parse_float_array(fp, "P_rect_00", data->P_rect_00, 12);

    parse_float_array(fp, "S_01", data->S_01, 2);
    parse_float_array(fp, "K_01", data->K_01, 9);
    parse_float_array(fp, "D_01", data->D_01, 5);
    parse_float_array(fp, "R_01", data->D_01, 9);
    parse_float_array(fp, "T_01", data->T_01, 3);
    parse_float_array(fp, "S_rect_01", data->S_rect_01, 2);
    parse_float_array(fp, "R_rect_01", data->R_rect_01, 9);
    parse_float_array(fp, "P_rect_01", data->P_rect_01, 12);

    parse_float_array(fp, "S_02", data->S_02, 2);
    parse_float_array(fp, "K_02", data->K_02, 9);
    parse_float_array(fp, "D_02", data->D_02, 5);
    parse_float_array(fp, "R_02", data->D_02, 9);
    parse_float_array(fp, "T_02", data->T_02, 3);
    parse_float_array(fp, "S_rect_02", data->S_rect_02, 2);
    parse_float_array(fp, "R_rect_02", data->R_rect_02, 9);
    parse_float_array(fp, "P_rect_02", data->P_rect_02, 12);

    parse_float_array(fp, "S_03", data->S_03, 2);
    parse_float_array(fp, "K_03", data->K_03, 9);
    parse_float_array(fp, "D_03", data->D_03, 5);
    parse_float_array(fp, "R_03", data->D_03, 9);
    parse_float_array(fp, "T_03", data->T_03, 3);
    parse_float_array(fp, "S_rect_03", data->S_rect_03, 2);
    parse_float_array(fp, "R_rect_03", data->R_rect_03, 9);
    parse_float_array(fp, "P_rect_03", data->P_rect_03, 12);

    fclose(fp);
  }

  // Load IMU to Velodyne extrinsics
  {
    char calib_imu_to_velo_path[1024] = {0};
    sprintf(calib_imu_to_velo_path, "%s/calib_imu_to_velo.txt", data_dir);
    FILE *fp = fopen(calib_imu_to_velo_path, "r");
    if (fp == NULL) {
      KITTI_FATAL("Failed to load [%s]\n", calib_imu_to_velo_path);
    }

    parse_value(fp, "calib_time", "string", &data->calib_time_imu_to_velo);
    parse_float_array(fp, "R", data->R_velo_imu, 9);
    parse_float_array(fp, "T", data->T_velo_imu, 3);
    fclose(fp);
  }

  // Load Velodyne to camera extrinsics
  {
    char calib_velo_to_cam_path[1024] = {0};
    sprintf(calib_velo_to_cam_path, "%s/calib_velo_to_cam.txt", data_dir);
    FILE *fp = fopen(calib_velo_to_cam_path, "r");
    if (fp == NULL) {
      KITTI_FATAL("Failed to load [%s]\n", calib_velo_to_cam_path);
    }

    parse_value(fp, "calib_time", "string", &data->calib_time_velo_to_cam);
    parse_float_array(fp, "R", data->R_cam_velo, 9);
    parse_float_array(fp, "T", data->T_cam_velo, 3);
    parse_float_array(fp, "delta_f", data->delta_f, 2);
    parse_float_array(fp, "delta_c", data->delta_c, 2);
    fclose(fp);
  }

  return data;
}

void kitti_calib_free(kitti_calib_t *data) { free(data); }

void kitti_calib_print(const kitti_calib_t *data) {
  printf("calib_time_cam_to_cam: %s\n", data->calib_time_cam_to_cam);
  printf("calib_time_imu_to_velo: %s\n", data->calib_time_imu_to_velo);
  printf("calib_time_velo_to_cam: %s\n", data->calib_time_velo_to_cam);
  printf("corner_dist: %f\n", data->corner_dist);
  print_double_array("S_00", data->S_00, 2);
  print_double_array("K_00", data->K_00, 9);
  print_double_array("D_00", data->D_00, 5);
  print_double_array("R_00", data->R_00, 9);
  print_double_array("T_00", data->T_00, 3);
  print_double_array("S_rect_00", data->S_rect_00, 2);
  print_double_array("R_rect_00", data->R_rect_00, 9);
  print_double_array("P_rect_00", data->P_rect_00, 12);
  printf("\n");

  print_double_array("S_01", data->S_01, 2);
  print_double_array("K_01", data->K_01, 9);
  print_double_array("D_01", data->D_01, 5);
  print_double_array("R_01", data->R_01, 9);
  print_double_array("T_01", data->T_01, 3);
  print_double_array("S_rect_01", data->S_rect_01, 2);
  print_double_array("R_rect_01", data->R_rect_01, 9);
  print_double_array("P_rect_01", data->P_rect_01, 12);
  printf("\n");

  print_double_array("S_02", data->S_02, 2);
  print_double_array("K_02", data->K_02, 9);
  print_double_array("D_02", data->D_02, 5);
  print_double_array("R_02", data->R_02, 9);
  print_double_array("T_02", data->T_02, 3);
  print_double_array("S_rect_02", data->S_rect_02, 2);
  print_double_array("R_rect_02", data->R_rect_02, 9);
  print_double_array("P_rect_02", data->P_rect_02, 12);
  printf("\n");

  print_double_array("S_03", data->S_03, 2);
  print_double_array("K_03", data->K_03, 9);
  print_double_array("D_03", data->D_03, 5);
  print_double_array("R_03", data->R_03, 9);
  print_double_array("T_03", data->T_03, 3);
  print_double_array("S_rect_03", data->S_rect_03, 2);
  print_double_array("R_rect_03", data->R_rect_03, 9);
  print_double_array("P_rect_03", data->P_rect_03, 12);
  printf("\n");

  print_double_array("R_velo_imu", data->R_velo_imu, 9);
  print_double_array("T_velo_imu", data->T_velo_imu, 3);
  printf("\n");

  print_double_array("R_cam_velo", data->R_cam_velo, 9);
  print_double_array("T_cam_velo", data->T_cam_velo, 3);
  print_double_array("delta_f", data->delta_f, 2);
  print_double_array("delta_c", data->delta_c, 2);
  printf("\n");

}

/*****************************************************************************
 * kitti_raw_t
 ****************************************************************************/

kitti_raw_t *kitti_raw_load(const char *data_dir, const char *seq_name) {
  // Setup paths
  char image_00_path[1024] = {0};
  char image_01_path[1024] = {0};
  char image_02_path[1024] = {0};
  char image_03_path[1024] = {0};
  char oxts_path[1024] = {0};
  char velodyne_points_path[1024] = {0};
  sprintf(image_00_path, "%s/%s/image_00", data_dir, seq_name);
  sprintf(image_01_path, "%s/%s/image_01", data_dir, seq_name);
  sprintf(image_02_path, "%s/%s/image_02", data_dir, seq_name);
  sprintf(image_03_path, "%s/%s/image_03", data_dir, seq_name);
  sprintf(oxts_path, "%s/%s/oxts", data_dir, seq_name);
  sprintf(velodyne_points_path, "%s/%s/velodyne_points", data_dir, seq_name);

  // Load data
  kitti_raw_t *data = MALLOC(kitti_raw_t, 1);
  strcpy(data->seq_name, seq_name);
  data->image_00 = kitti_camera_load(image_00_path);
  data->image_01 = kitti_camera_load(image_01_path);
  data->image_02 = kitti_camera_load(image_02_path);
  data->image_03 = kitti_camera_load(image_03_path);
  data->oxts = kitti_oxts_load(oxts_path);
  data->velodyne_points = kitti_velodyne_load(velodyne_points_path);
  data->calib = kitti_calib_load(data_dir);

  return data;
}

void kitti_raw_free(kitti_raw_t *data) {
  kitti_camera_free(data->image_00);
  kitti_camera_free(data->image_01);
  kitti_camera_free(data->image_02);
  kitti_camera_free(data->image_03);
  kitti_oxts_free(data->oxts);
  kitti_velodyne_free(data->velodyne_points);
  kitti_calib_free(data->calib);
  free(data);
}
