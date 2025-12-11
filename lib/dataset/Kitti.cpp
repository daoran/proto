#include "Kitti.hpp"

namespace xyz {

static int file_lines(const char *fp) {
  FILE *f = fopen(fp, "r");
  int lines = 0;

  if (f == NULL) {
    return -1;
  }

  int ch;
  while ((ch = getc(f)) != EOF) {
    if (ch == '\n') {
      ++lines;
    }
  }

  return lines;
}

static void print_double_array(const char *prefix,
                               const double *arr,
                               const size_t n) {
  assert(prefix != NULL);
  assert(arr != NULL);
  assert(n != 0);

  printf("%s: ", prefix);
  for (size_t i = 0; i < n; i++) {
    printf("%.4f ", arr[i]);
  }
  printf("\n");
}

static int64_t parse_dateline(const char *dt_str) {
  // Parse
  struct tm tm;
  int fractional_seconds;
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
parse_double_array(FILE *fp, const char *key, double *array, int size) {
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

static std::vector<int64_t> load_timestamps(const char *file_path) {
  const int num_rows = file_lines(file_path);
  if (num_rows == 0) {
    return std::vector<int64_t>();
  }

  FILE *fp = fopen(file_path, "r");
  if (fp == NULL) {
    KITTI_FATAL("Failed to open [%s]!\n", file_path);
  }

  std::vector<int64_t> timestamps(num_rows);
  for (int i = 0; i < num_rows; ++i) {
    char line[1024] = {0};
    if (fgets(line, sizeof(line), fp) == NULL) {
      KITTI_FATAL("Failed to parse line %d in [%s]!\n", i, file_path);
    }
    timestamps[i] = parse_dateline(line);
  }

  return timestamps;
}

//////////////////
// KITTI CAMERA //
//////////////////

KittiCamera::KittiCamera(const fs::path &data_dir) {
  // Form timestamps path
  const fs::path timestamps_path = data_dir / "timestamps.txt";

  // Open timestamps file
  const int num_rows = file_lines(timestamps_path.c_str());
  FILE *fp = fopen(timestamps_path.c_str(), "r");
  if (fp == NULL) {
    KITTI_FATAL("Failed to open [%s]!\n", timestamps_path.c_str());
  }

  // Parse
  assert(num_rows > 0);
  for (int i = 0; i < num_rows; ++i) {
    // Timestamp
    char line[1024] = {0};
    if (fgets(line, sizeof(line), fp) == NULL) {
      KITTI_FATAL("Failed to parse line %d in [%s]!\n",
                  i,
                  timestamps_path.c_str());
    }
    timestamps.push_back(parse_dateline(line));

    // Image path
    char image_path[1024] = {0};
    sprintf(image_path, "%s/%s/%010d.png", data_dir.c_str(), "data", i);
    image_paths.push_back(image_path);
  }
  fclose(fp);
}

////////////////
// KITTI OXTS //
////////////////

KittiOxts::KittiOxts(const fs::path &data_dir) {
  // Get number of measurements
  char timestamps_path[1024] = {0};
  sprintf(timestamps_path, "%s/timestamps.txt", data_dir.c_str());
  const int num_rows = file_lines(timestamps_path);

  // Pre-allocate memory
  // -- GPS
  lat.resize(num_rows); // Latitude [deg]
  lon.resize(num_rows); // Longitude [deg]
  alt.resize(num_rows); // Altitude [m]
  // -- Attitude
  roll.resize(num_rows);  // Roll [rad]
  pitch.resize(num_rows); // Pitch [rad]
  yaw.resize(num_rows);   // Yaw [rad]
  // -- Velocity
  vn.resize(num_rows); // Velocity towards north [m/s]
  ve.resize(num_rows); // Velocity towards east [m/s]
  vf.resize(num_rows); // Forward velocity [m/s]
  vl.resize(num_rows); // Leftward velocity [m/s]
  vu.resize(num_rows); // Upward velocity [m/s]
  // -- Acceleration
  ax.resize(num_rows); // Acceleration in x [m/s^2]
  ay.resize(num_rows); // Acceleration in y [m/s^2]
  az.resize(num_rows); // Acceleration in z [m/s^2]
  af.resize(num_rows); // Forward acceleration [m/s^2]
  al.resize(num_rows); // Leftward acceleration [m/s^2]
  au.resize(num_rows); // Upward acceleration [m/s^2]
  // -- Angular velocity
  wx.resize(num_rows); // Angular rate around x [rad/s]
  wy.resize(num_rows); // Angular rate around y [rad/s]
  wz.resize(num_rows); // Angular rate around z [rad/s]
  wf.resize(num_rows); // Angular rate around foward axis [rad/s]
  wl.resize(num_rows); // Angular rate around left axis [rad/s]
  wu.resize(num_rows); // Angular rate around up axis [rad/s]
  // -- Satellite tracking data
  pos_accuracy.resize(num_rows); // Position accuracy [north / east in m]
  vel_accuracy.resize(num_rows); // Velocity accuracy [north / east in m/s]
  navstat.resize(num_rows);      // Navigation status
  numsats.resize(num_rows);      // Number of satelllites tracked by GPS
  posmode.resize(num_rows);      // Position mode
  velmode.resize(num_rows);      // Velocity mode
  orimode.resize(num_rows);      // Orientation mode

  // -- Parse timestamps
  {
    FILE *fp = fopen(timestamps_path, "r");
    if (fp == NULL) {
      KITTI_FATAL("Failed to open [%s]!\n", timestamps_path);
    }
    for (int i = 0; i < num_rows; ++i) {
      char line[1024] = {0};
      if (fgets(line, sizeof(line), fp) == NULL) {
        KITTI_FATAL("Failed to parse line %d in [%s]!\n", i, timestamps_path);
      }
      timestamps.push_back(parse_dateline(line));
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
    strcat(format, "%lf %lf %lf %lf %lf %lf "); // Angular Velocity
    strcat(format, "%lf %lf %s %s %s %s %s");   // Satellite tracking

    for (int i = 0; i < num_rows; ++i) {
      // Open oxts entry
      char entry_path[1024] = {0};
      sprintf(entry_path, "%s/%s/%010d.txt", data_dir.c_str(), "data", i);
      FILE *fp = fopen(entry_path, "r");
      if (fp == NULL) {
        KITTI_FATAL("Failed to open [%s]!\n", timestamps_path);
      }

      // Parse
      char navstat_str[30] = {0}; // Navigation status
      char numsats_str[30] = {0}; // Number of satelllites tracked by GPS
      char posmode_str[30] = {0}; // Position mode
      char velmode_str[30] = {0}; // Velocity mode
      char orimode_str[30] = {0}; // Orientation mode
      int retval = fscanf(fp,
                          format,
                          // GPS
                          &lat[i],
                          &lon[i],
                          &alt[i],
                          // Attitude
                          &roll[i],
                          &pitch[i],
                          &yaw[i],
                          // Velocity
                          &vn[i],
                          &ve[i],
                          &vf[i],
                          &vl[i],
                          &vu[i],
                          // Acceleration
                          &ax[i],
                          &ay[i],
                          &az[i],
                          &af[i],
                          &al[i],
                          &au[i],
                          // Angular velocity
                          &wx[i],
                          &wy[i],
                          &wz[i],
                          &wf[i],
                          &wl[i],
                          &wu[i],
                          // Satellite tracking
                          &pos_accuracy[i],
                          &vel_accuracy[i],
                          navstat_str,
                          numsats_str,
                          posmode_str,
                          velmode_str,
                          orimode_str);

      // There's a bug in the KITTI OXTS data where in should be integer
      // but sometimes its float. Here we are parsing the satellite
      // tracking data as a string and converting it to integers.
      navstat[i] = strtol(navstat_str, NULL, 10);
      numsats[i] = strtol(numsats_str, NULL, 10);
      posmode[i] = strtol(posmode_str, NULL, 10);
      velmode[i] = strtol(velmode_str, NULL, 10);
      orimode[i] = strtol(orimode_str, NULL, 10);

      if (retval != 30) {
        KITTI_FATAL("Failed to parse [%s]\n", entry_path);
      }
      fclose(fp);
    }
  }
}

float *KittiVelodyne::load_points(const fs::path &pcd_path,
                                  size_t *num_points) {
  // Load pcd file
  FILE *pcd_file = fopen(pcd_path.c_str(), "rb");
  if (!pcd_file) {
    KITTI_LOG("Failed to open [%s]", pcd_path.c_str());
    return NULL;
  }

  // Get the size of the file to know how many points
  fseek(pcd_file, 0, SEEK_END);
  const long int file_size = ftell(pcd_file);
  rewind(pcd_file);

  // Allocate memory for the points
  *num_points = file_size / (sizeof(float) * 4);
  float *points = (float *) malloc(sizeof(float) * 4 * *num_points);
  if (!points) {
    KITTI_LOG("Failed to allocate memory for points");
    fclose(pcd_file);
    return NULL;
  }

  // Read points from the file
  const size_t point_size = sizeof(float) * 4;
  const size_t read_count = fread(points, point_size, *num_points, pcd_file);
  if (read_count != *num_points) {
    KITTI_LOG("Failed to read all points");
    free(points);
    fclose(pcd_file);
    return NULL;
  }

  // Clean up
  fclose(pcd_file);

  return points;
}

// float *kitti_lidar_xyz(const char *pcd_path,
//                        const float voxel_size,
//                        size_t *nout) {
//   // Load Kitti LIDAR points [x, y, z, intensity]
//   size_t n = 0;
//   float *raw_points = kitti_load_points(pcd_path, &n);
//
//   // Extract only the relevant parts (x, y, z)
//   float *points_xyz = malloc(sizeof(float) * 3 * n);
//   for (size_t i = 0; i < n; ++i) {
//     points_xyz[i * 3 + 0] = raw_points[i * 4 + 0];
//     points_xyz[i * 3 + 1] = raw_points[i * 4 + 1];
//     points_xyz[i * 3 + 2] = raw_points[i * 4 + 2];
//   }
//
//   // Downsample
//   float *points_out = voxel_grid_downsample(points_xyz, n, voxel_size, nout);
//
//   // Clean up
//   free(raw_points);
//   free(points_xyz);
//
//   return points_out;
// }

///////////////////
// KittiVelodyne //
///////////////////

KittiVelodyne::KittiVelodyne(const fs::path &data_dir) {
  // Setup timestamp paths
  char timestamps_path[1024] = {0};
  char timestamps_start_path[1024] = {0};
  char timestamps_end_path[1024] = {0};
  sprintf(timestamps_path, "%s/timestamps.txt", data_dir.c_str());
  sprintf(timestamps_start_path, "%s/timestamps_start.txt", data_dir.c_str());
  sprintf(timestamps_end_path, "%s/timestamps_end.txt", data_dir.c_str());

  // Load timestamps
  timestamps = load_timestamps(timestamps_path);
  timestamps_start = load_timestamps(timestamps_start_path);
  timestamps_end = load_timestamps(timestamps_end_path);

  // Load data
  const int num_timestamps = timestamps.size();
  for (int i = 0; i < num_timestamps; ++i) {
    char pcd_path[1024] = {0};
    sprintf(pcd_path, "%s/%s/%010d.bin", data_dir.c_str(), "data", i);
    pcd_paths[i] = fs::path{pcd_path};
  }
}

////////////////
// KittiCalib //
////////////////

KittiCalib::KittiCalib(const fs::path &data_dir) {
  // Load camera calibrations
  {
    char calib_cam_to_cam_path[1024] = {0};
    sprintf(calib_cam_to_cam_path, "%s/calib_cam_to_cam.txt", data_dir.c_str());
    FILE *fp = fopen(calib_cam_to_cam_path, "r");
    if (fp == NULL) {
      KITTI_FATAL("Failed to load [%s]\n", calib_cam_to_cam_path);
    }

    char calib_time[100] = {0};
    parse_value(fp, "calib_time", "string", &calib_time);
    parse_value(fp, "corner_dist", "double", &corner_dist);
    calib_time_cam_to_cam = std::string{calib_time};

    parse_double_array(fp, "S_00", S_00, 2);
    parse_double_array(fp, "K_00", K_00, 9);
    parse_double_array(fp, "D_00", D_00, 5);
    parse_double_array(fp, "R_00", D_00, 9);
    parse_double_array(fp, "T_00", T_00, 3);
    parse_double_array(fp, "S_rect_00", S_rect_00, 2);
    parse_double_array(fp, "R_rect_00", R_rect_00, 9);
    parse_double_array(fp, "P_rect_00", P_rect_00, 12);

    parse_double_array(fp, "S_01", S_01, 2);
    parse_double_array(fp, "K_01", K_01, 9);
    parse_double_array(fp, "D_01", D_01, 5);
    parse_double_array(fp, "R_01", D_01, 9);
    parse_double_array(fp, "T_01", T_01, 3);
    parse_double_array(fp, "S_rect_01", S_rect_01, 2);
    parse_double_array(fp, "R_rect_01", R_rect_01, 9);
    parse_double_array(fp, "P_rect_01", P_rect_01, 12);

    parse_double_array(fp, "S_02", S_02, 2);
    parse_double_array(fp, "K_02", K_02, 9);
    parse_double_array(fp, "D_02", D_02, 5);
    parse_double_array(fp, "R_02", D_02, 9);
    parse_double_array(fp, "T_02", T_02, 3);
    parse_double_array(fp, "S_rect_02", S_rect_02, 2);
    parse_double_array(fp, "R_rect_02", R_rect_02, 9);
    parse_double_array(fp, "P_rect_02", P_rect_02, 12);

    parse_double_array(fp, "S_03", S_03, 2);
    parse_double_array(fp, "K_03", K_03, 9);
    parse_double_array(fp, "D_03", D_03, 5);
    parse_double_array(fp, "R_03", D_03, 9);
    parse_double_array(fp, "T_03", T_03, 3);
    parse_double_array(fp, "S_rect_03", S_rect_03, 2);
    parse_double_array(fp, "R_rect_03", R_rect_03, 9);
    parse_double_array(fp, "P_rect_03", P_rect_03, 12);

    fclose(fp);
  }

  // Load IMU to Velodyne extrinsics
  {
    char calib_imu_to_velo_path[1024] = {0};
    sprintf(calib_imu_to_velo_path,
            "%s/calib_imu_to_velo.txt",
            data_dir.c_str());
    FILE *fp = fopen(calib_imu_to_velo_path, "r");
    if (fp == NULL) {
      KITTI_FATAL("Failed to load [%s]\n", calib_imu_to_velo_path);
    }

    char calib_time[100] = {0};
    parse_value(fp, "calib_time", "string", &calib_time);
    parse_double_array(fp, "R", R_velo_imu, 9);
    parse_double_array(fp, "T", T_velo_imu, 3);
    calib_time_imu_to_velo = std::string{calib_time};

    fclose(fp);
  }

  // Load Velodyne to camera extrinsics
  {
    char calib_velo_to_cam_path[1024] = {0};
    sprintf(calib_velo_to_cam_path,
            "%s/calib_velo_to_cam.txt",
            data_dir.c_str());
    FILE *fp = fopen(calib_velo_to_cam_path, "r");
    if (fp == NULL) {
      KITTI_FATAL("Failed to load [%s]\n", calib_velo_to_cam_path);
    }

    char calib_time[100] = {0};
    parse_value(fp, "calib_time", "string", &calib_time);
    parse_double_array(fp, "R", R_cam_velo, 9);
    parse_double_array(fp, "T", T_cam_velo, 3);
    parse_double_array(fp, "delta_f", delta_f, 2);
    parse_double_array(fp, "delta_c", delta_c, 2);
    calib_time_velo_to_cam = std::string{calib_time};

    fclose(fp);
  }
}

void KittiCalib::print() {
  printf("calib_time_cam_to_cam: %s\n", calib_time_cam_to_cam.c_str());
  printf("calib_time_imu_to_velo: %s\n", calib_time_imu_to_velo.c_str());
  printf("calib_time_velo_to_cam: %s\n", calib_time_velo_to_cam.c_str());
  printf("corner_dist: %f\n", corner_dist);
  print_double_array("S_00", S_00, 2);
  print_double_array("K_00", K_00, 9);
  print_double_array("D_00", D_00, 5);
  print_double_array("R_00", R_00, 9);
  print_double_array("T_00", T_00, 3);
  print_double_array("S_rect_00", S_rect_00, 2);
  print_double_array("R_rect_00", R_rect_00, 9);
  print_double_array("P_rect_00", P_rect_00, 12);
  printf("\n");

  print_double_array("S_01", S_01, 2);
  print_double_array("K_01", K_01, 9);
  print_double_array("D_01", D_01, 5);
  print_double_array("R_01", R_01, 9);
  print_double_array("T_01", T_01, 3);
  print_double_array("S_rect_01", S_rect_01, 2);
  print_double_array("R_rect_01", R_rect_01, 9);
  print_double_array("P_rect_01", P_rect_01, 12);
  printf("\n");

  print_double_array("S_02", S_02, 2);
  print_double_array("K_02", K_02, 9);
  print_double_array("D_02", D_02, 5);
  print_double_array("R_02", R_02, 9);
  print_double_array("T_02", T_02, 3);
  print_double_array("S_rect_02", S_rect_02, 2);
  print_double_array("R_rect_02", R_rect_02, 9);
  print_double_array("P_rect_02", P_rect_02, 12);
  printf("\n");

  print_double_array("S_03", S_03, 2);
  print_double_array("K_03", K_03, 9);
  print_double_array("D_03", D_03, 5);
  print_double_array("R_03", R_03, 9);
  print_double_array("T_03", T_03, 3);
  print_double_array("S_rect_03", S_rect_03, 2);
  print_double_array("R_rect_03", R_rect_03, 9);
  print_double_array("P_rect_03", P_rect_03, 12);
  printf("\n");

  print_double_array("R_velo_imu", R_velo_imu, 9);
  print_double_array("T_velo_imu", T_velo_imu, 3);
  printf("\n");

  print_double_array("R_cam_velo", R_cam_velo, 9);
  print_double_array("T_cam_velo", T_cam_velo, 3);
  print_double_array("delta_f", delta_f, 2);
  print_double_array("delta_c", delta_c, 2);
  printf("\n");
}

///////////////
// Kitti Raw //
///////////////

KittiRaw::KittiRaw(const fs::path &data_dir, const std::string &seq_name)
    : seq_name{seq_name} {
  // Setup paths
  const auto image_00_path = data_dir / seq_name / "image_00";
  const auto image_01_path = data_dir / seq_name / "image_01";
  const auto image_02_path = data_dir / seq_name / "image_02";
  const auto image_03_path = data_dir / seq_name / "image_03";
  const auto velodyne_path = data_dir / seq_name / "velodyne_points";
  const auto oxts_path = data_dir / seq_name / "oxts";

  // Load data
  image_00 = std::make_shared<KittiCamera>(image_00_path);
  image_01 = std::make_shared<KittiCamera>(image_01_path);
  image_02 = std::make_shared<KittiCamera>(image_02_path);
  image_03 = std::make_shared<KittiCamera>(image_03_path);
  oxts = std::make_shared<KittiOxts>(oxts_path);
  velodyne = std::make_shared<KittiVelodyne>(velodyne_path);
  calib = std::make_shared<KittiCalib>(data_dir);
}

} // namespace xyz
