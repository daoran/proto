#include "sim.h"

/**
 * Default circle trajectory settings.
 */
void sim_circle_defaults(sim_circle_t *conf) {
  conf->imu_rate = 200.0;
  conf->cam_rate = 10.0;
  conf->circle_r = 5.0;
  conf->circle_v = 1.0;
  conf->theta_init = M_PI;
  conf->yaw_init = M_PI / 2.0;
}

//////////////////
// SIM FEATURES //
//////////////////

/**
 * Load simulation feature data.
 */
sim_features_t *sim_features_load(const char *csv_path) {
  sim_features_t *features_data = MALLOC(sim_features_t, 1);
  int num_rows = 0;
  int num_cols = 0;
  features_data->features = csv_data(csv_path, &num_rows, &num_cols);
  features_data->num_features = num_rows;
  return features_data;
}

/**
 * Free simulation feature data.
 */
void sim_features_free(sim_features_t *feature_data) {
  // Pre-check
  if (feature_data == NULL) {
    return;
  }

  // Free data
  for (int i = 0; i < feature_data->num_features; i++) {
    free(feature_data->features[i]);
  }
  free(feature_data->features);
  free(feature_data);
}

//////////////////
// SIM IMU DATA //
//////////////////

/**
 * Setup sim imu data.
 */
void sim_imu_data_setup(sim_imu_data_t *imu_data) {
  imu_data->num_measurements = 0;
  imu_data->timestamps = NULL;
  imu_data->poses = NULL;
  imu_data->velocities = NULL;
  imu_data->imu_acc = NULL;
  imu_data->imu_gyr = NULL;
}

/**
 * Malloc sim imu data.
 */
sim_imu_data_t *sim_imu_data_malloc(void) {
  sim_imu_data_t *imu_data = MALLOC(sim_imu_data_t, 1);
  sim_imu_data_setup(imu_data);
  return imu_data;
}

/**
 * Free simulation imu data.
 */
void sim_imu_data_free(sim_imu_data_t *imu_data) {
  // Pre-check
  if (imu_data == NULL) {
    return;
  }

  // Free data
  free(imu_data->timestamps);
  free(imu_data->poses);
  free(imu_data->velocities);
  free(imu_data->imu_acc);
  free(imu_data->imu_gyr);
  free(imu_data);
}

/**
 * Load simulation imu data.
 */
sim_imu_data_t *sim_imu_data_load(const char *csv_path) { return NULL; }

/**
 * Simulate IMU circle trajectory.
 */
sim_imu_data_t *sim_imu_circle_trajectory(const sim_circle_t *conf) {
  // Setup
  const int imu_rate = conf->imu_rate;
  const real_t circle_r = conf->circle_r;
  const real_t circle_v = conf->circle_r;
  const real_t theta_init = conf->circle_r;
  const real_t yaw_init = conf->yaw_init;

  // Circle trajectory configurations
  const real_t circle_dist = 2.0 * M_PI * circle_r;
  const real_t time_taken = circle_dist / circle_v;
  const real_t w = -2.0 * M_PI * (1.0 / time_taken);

  // Allocate memory for test data
  sim_imu_data_t *imu_data = sim_imu_data_malloc();
  imu_data->num_measurements = time_taken * imu_rate;
  imu_data->timestamps = CALLOC(real_t, imu_data->num_measurements);
  imu_data->poses = CALLOC(real_t, imu_data->num_measurements * 7);
  imu_data->velocities = CALLOC(real_t, imu_data->num_measurements * 3);
  imu_data->imu_acc = CALLOC(real_t, imu_data->num_measurements * 3);
  imu_data->imu_gyr = CALLOC(real_t, imu_data->num_measurements * 3);

  // Simulate IMU poses
  const real_t dt = 1.0 / imu_rate;
  timestamp_t ts = 0.0;
  real_t theta = theta_init;
  real_t yaw = yaw_init;

  for (size_t k = 0; k < imu_data->num_measurements; k++) {
    // IMU pose
    // -- Position
    const real_t rx = circle_r * cos(theta);
    const real_t ry = circle_r * sin(theta);
    const real_t rz = 0.0;
    // -- Orientation
    const real_t ypr[3] = {yaw, 0.0, 0.0};
    real_t q[4] = {0};
    euler2quat(ypr, q);
    // -- Pose vector
    const real_t pose[7] = {rx, ry, rz, q[0], q[1], q[2], q[3]};
    // print_vector("pose", pose, 7);

    // Velocity
    const real_t vx = -circle_r * w * sin(theta);
    const real_t vy = circle_r * w * cos(theta);
    const real_t vz = 0.0;
    const real_t v_WS[3] = {vx, vy, vz};

    // Acceleration
    const real_t ax = -circle_r * w * w * cos(theta);
    const real_t ay = -circle_r * w * w * sin(theta);
    const real_t az = 0.0;
    const real_t a_WS[3] = {ax, ay, az};

    // Angular velocity
    const real_t wx = 0.0;
    const real_t wy = 0.0;
    const real_t wz = w;
    const real_t w_WS[3] = {wx, wy, wz};

    // IMU measurements
    real_t C_WS[3 * 3] = {0};
    real_t C_SW[3 * 3] = {0};
    quat2rot(q, C_WS);
    mat_transpose(C_WS, 3, 3, C_SW);
    // -- Accelerometer measurement
    real_t acc[3] = {0};
    dot(C_SW, 3, 3, a_WS, 3, 1, acc);
    acc[2] += 9.81;
    // -- Gyroscope measurement
    real_t gyr[3] = {0};
    dot(C_SW, 3, 3, w_WS, 3, 1, gyr);

    // Update
    imu_data->timestamps[k] = ts;
    vec_copy(pose, 7, imu_data->poses + k * 7);
    vec_copy(v_WS, 3, imu_data->velocities + k * 3);
    vec_copy(acc, 3, imu_data->imu_acc + k * 3);
    vec_copy(gyr, 3, imu_data->imu_gyr + k * 3);

    theta += w * dt;
    yaw += w * dt;
    ts += sec2ts(dt);
  }

  return imu_data;
}

void sim_imu_measurements(const sim_imu_data_t *data,
                          const int64_t ts_i,
                          const int64_t ts_j,
                          imu_buffer_t *imu_buf) {
  imu_buffer_setup(imu_buf);

  for (size_t k = 0; k < data->num_measurements; k++) {
    const int64_t ts = data->timestamps[k];
    if (ts <= ts_i) {
      continue;
    } else if (ts >= ts_j) {
      break;
    }

    imu_buffer_add(imu_buf, ts, &data->imu_acc[k * 3], &data->imu_gyr[k * 3]);
  }
}

/////////////////////
// SIM CAMERA DATA //
/////////////////////

/**
 * Simulate 3D features.
 */
void sim_create_features(const real_t origin[3],
                         const real_t dim[3],
                         const int num_features,
                         real_t *features) {
  assert(origin != NULL);
  assert(dim != NULL);
  assert(num_features > 0);
  assert(features != NULL);

  // Setup
  const real_t w = dim[0];
  const real_t l = dim[1];
  const real_t h = dim[2];
  const int features_per_side = num_features / 4.0;
  int feature_idx = 0;

  // Features in the east side
  {
    const real_t x_bounds[2] = {origin[0] - w, origin[0] + w};
    const real_t y_bounds[2] = {origin[1] + l, origin[1] + l};
    const real_t z_bounds[2] = {origin[2] - h, origin[2] + h};
    for (int i = 0; i < features_per_side; i++) {
      features[feature_idx * 3 + 0] = randf(x_bounds[0], x_bounds[1]);
      features[feature_idx * 3 + 1] = randf(y_bounds[0], y_bounds[1]);
      features[feature_idx * 3 + 2] = randf(z_bounds[0], z_bounds[1]);
      feature_idx++;
    }
  }

  // Features in the north side
  {
    const real_t x_bounds[2] = {origin[0] + w, origin[0] + w};
    const real_t y_bounds[2] = {origin[1] - l, origin[1] + l};
    const real_t z_bounds[2] = {origin[2] - h, origin[2] + h};
    for (int i = 0; i < features_per_side; i++) {
      features[feature_idx * 3 + 0] = randf(x_bounds[0], x_bounds[1]);
      features[feature_idx * 3 + 1] = randf(y_bounds[0], y_bounds[1]);
      features[feature_idx * 3 + 2] = randf(z_bounds[0], z_bounds[1]);
      feature_idx++;
    }
  }

  // Features in the west side
  {
    const real_t x_bounds[2] = {origin[0] - w, origin[0] + w};
    const real_t y_bounds[2] = {origin[1] - l, origin[1] - l};
    const real_t z_bounds[2] = {origin[2] - h, origin[2] + h};
    for (int i = 0; i < features_per_side; i++) {
      features[feature_idx * 3 + 0] = randf(x_bounds[0], x_bounds[1]);
      features[feature_idx * 3 + 1] = randf(y_bounds[0], y_bounds[1]);
      features[feature_idx * 3 + 2] = randf(z_bounds[0], z_bounds[1]);
      feature_idx++;
    }
  }

  // Features in the south side
  {
    const real_t x_bounds[2] = {origin[0] - w, origin[0] - w};
    const real_t y_bounds[2] = {origin[1] - l, origin[1] + l};
    const real_t z_bounds[2] = {origin[2] - h, origin[2] + h};
    for (int i = 0; i < features_per_side; i++) {
      features[feature_idx * 3 + 0] = randf(x_bounds[0], x_bounds[1]);
      features[feature_idx * 3 + 1] = randf(y_bounds[0], y_bounds[1]);
      features[feature_idx * 3 + 2] = randf(z_bounds[0], z_bounds[1]);
      feature_idx++;
    }
  }
}

/**
 * Extract timestamp from path.
 */
static timestamp_t path2ts(const char *path) {
  char fname[128] = {0};
  char fext[128] = {0};
  path_file_name(path, fname);
  path_file_ext(path, fext);

  char ts_str[128] = {0};
  memcpy(ts_str, fname, strlen(fname) - strlen(fext) - 1);

  char *ptr;
  return strtol(ts_str, &ptr, 10);
}

/**
 * Setup simulated camera frame.
 */
void sim_camera_frame_setup(sim_camera_frame_t *frame,
                            const timestamp_t ts,
                            const int cam_idx) {
  frame->ts = ts;
  frame->cam_idx = cam_idx;
  frame->n = 0;
  frame->feature_ids = NULL;
  frame->keypoints = NULL;
}

/**
 * Malloc simulated camera frame.
 */
sim_camera_frame_t *sim_camera_frame_malloc(const timestamp_t ts,
                                            const int cam_idx) {
  sim_camera_frame_t *frame = MALLOC(sim_camera_frame_t, 1);
  sim_camera_frame_setup(frame, ts, cam_idx);
  return frame;
}

/**
 * Free simulated camera frame.
 */
void sim_camera_frame_free(sim_camera_frame_t *frame_data) {
  // Pre-check
  if (frame_data == NULL) {
    return;
  }

  // Free data
  free(frame_data->keypoints);
  free(frame_data->feature_ids);
  free(frame_data);
}

/**
 * Add keypoint measurement to camera frame.
 */
void sim_camera_frame_add_keypoint(sim_camera_frame_t *frame,
                                   const size_t feature_id,
                                   const real_t kp[2]) {
  const int N = frame->n + 1;
  frame->n = N;
  frame->feature_ids = REALLOC(frame->feature_ids, real_t, N);
  frame->keypoints = REALLOC(frame->keypoints, real_t, N * 2);
  frame->feature_ids[N - 1] = feature_id;
  frame->keypoints[(N - 1) * 2 + 0] = kp[0];
  frame->keypoints[(N - 1) * 2 + 1] = kp[1];
}

/**
 * Load simulated camera frame.
 */
sim_camera_frame_t *sim_camera_frame_load(const char *csv_path) {
  // Check if file exists
  if (file_exists(csv_path) == 0) {
    return NULL;
  }

  // Load csv data
  int num_rows = 0;
  int num_cols = 0;
  real_t **data = csv_data(csv_path, &num_rows, &num_cols);

  // Create sim_camera_frame_t
  sim_camera_frame_t *frame_data = MALLOC(sim_camera_frame_t, 1);
  frame_data->ts = path2ts(csv_path);
  frame_data->feature_ids = MALLOC(size_t, num_rows);
  frame_data->keypoints = MALLOC(real_t, num_rows * 2);
  frame_data->n = num_rows;
  for (int i = 0; i < num_rows; i++) {
    frame_data->feature_ids[i] = (int) data[i][0];
    frame_data->keypoints[i * 2 + 0] = data[i][1];
    frame_data->keypoints[i * 2 + 1] = data[i][2];
  }

  // Clean up
  csv_free(data, num_rows);

  return frame_data;
}

/**
 * Print camera frame.
 */
void sim_camera_frame_print(const sim_camera_frame_t *frame_data) {
  printf("ts: %ld\n", frame_data->ts);
  printf("num_measurements: %d\n", frame_data->n);
  for (int i = 0; i < frame_data->n; i++) {
    const int feature_id = frame_data->feature_ids[i];
    const real_t *kp = frame_data->keypoints + i * 2;
    printf("- ");
    printf("feature_id: [%d], ", feature_id);
    printf("kp: [%.2f, %.2f]\n", kp[0], kp[1]);
  }
  printf("\n");
}

/**
 * Setup simulated camera frames.
 */
void sim_camera_data_setup(sim_camera_data_t *data) {
  data->frames = NULL;
  data->num_frames = 0;
  data->timestamps = NULL;
  data->poses = NULL;
}

/**
 * Malloc simulated camera frames.
 */
sim_camera_data_t *sim_camerea_data_malloc(void) {
  sim_camera_data_t *data = MALLOC(sim_camera_data_t, 1);
  sim_camera_data_setup(data);
  return data;
}

/**
 * Free simulated camera data.
 */
void sim_camera_data_free(sim_camera_data_t *cam_data) {
  // Pre-check
  if (cam_data == NULL) {
    return;
  }

  // Free data
  for (size_t k = 0; k < cam_data->num_frames; k++) {
    sim_camera_frame_free(cam_data->frames[k]);
  }
  free(cam_data->frames);
  free(cam_data->timestamps);
  free(cam_data->poses);
  free(cam_data);
}

/**
 * Load simulated camera data.
 */
sim_camera_data_t *sim_camera_data_load(const char *dir_path) {
  assert(dir_path != NULL);

  // Form csv file path
  char *csv_path = path_join(dir_path, "/data.csv");
  if (file_exists(csv_path) == 0) {
    free(csv_path);
    return NULL;
  }

  // Check number of rows
  const int num_rows = dsv_rows(csv_path);
  if (num_rows == 0) {
    free(csv_path);
    return NULL;
  }

  // Open csv file
  FILE *csv_file = fopen(csv_path, "r");
  if (csv_file == NULL) {
    free(csv_path);
    return NULL;
  }

  // Form sim_camera_frame_t
  sim_camera_data_t *cam_data = MALLOC(sim_camera_data_t, 1);
  cam_data->frames = MALLOC(sim_camera_frame_t *, num_rows);
  cam_data->num_frames = num_rows;
  cam_data->timestamps = MALLOC(timestamp_t, num_rows);
  cam_data->poses = MALLOC(real_t *, num_rows * 7);

  int line_idx = 0;
  char line[MAX_LINE_LENGTH] = {0};
  while (fgets(line, MAX_LINE_LENGTH, csv_file) != NULL) {
    // Skip line if its a comment
    if (line[0] == '#') {
      continue;
    }

    // Parse line
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

    // Add camera frame to sim_camera_frame_t
    char fname[128] = {0};
    sprintf(fname, "/data/%ld.csv", ts);
    char *frame_csv = path_join(dir_path, fname);
    cam_data->frames[line_idx] = sim_camera_frame_load(frame_csv);
    free(frame_csv);

    // Add pose to sim_camera_frame_t
    cam_data->timestamps[line_idx] = ts;
    cam_data->poses[line_idx * 7 + 0] = r[0];
    cam_data->poses[line_idx * 7 + 1] = r[1];
    cam_data->poses[line_idx * 7 + 2] = r[2];
    cam_data->poses[line_idx * 7 + 3] = q[0];
    cam_data->poses[line_idx * 7 + 4] = q[1];
    cam_data->poses[line_idx * 7 + 5] = q[2];
    cam_data->poses[line_idx * 7 + 6] = q[3];

    // Update
    line_idx++;
  }

  // Clean up
  free(csv_path);
  fclose(csv_file);

  return cam_data;
}

/**
 * Simulate camera going round in a circle.
 */
sim_camera_data_t *
sim_camera_circle_trajectory(const sim_circle_t *conf,
                             const real_t T_BC[4 * 4],
                             const camera_params_t *cam_params,
                             const real_t *features,
                             const int num_features) {
  // Settings
  const real_t cam_rate = conf->cam_rate;
  const real_t circle_r = conf->circle_r;
  const real_t circle_v = conf->circle_v;
  const real_t theta_init = conf->theta_init;
  const real_t yaw_init = conf->yaw_init;

  // Circle trajectory configurations
  const real_t circle_dist = 2.0 * M_PI * circle_r;
  const real_t time_taken = circle_dist / circle_v;
  const real_t w = -2.0 * M_PI * (1.0 / time_taken);

  // Allocate memory for test data
  const int cam_idx = cam_params->cam_idx;
  const int num_frames = time_taken * cam_rate;
  sim_camera_data_t *data = sim_camerea_data_malloc();
  data->cam_idx = cam_idx;
  data->frames = CALLOC(sim_camera_frame_t *, num_frames);
  data->num_frames = num_frames;
  data->timestamps = CALLOC(real_t, data->num_frames);
  data->poses = CALLOC(real_t, data->num_frames * 7);

  // Simulate camera
  const real_t dt = 1.0 / cam_rate;
  timestamp_t ts = 0.0;
  real_t theta = theta_init;
  real_t yaw = yaw_init;

  for (size_t k = 0; k < data->num_frames; k++) {
    // Body pose T_WB
    // -- Position
    const real_t rx = circle_r * cos(theta);
    const real_t ry = circle_r * sin(theta);
    const real_t rz = 0.0;
    const real_t r_WB[3] = {rx, ry, rz};
    // -- Orientation
    const real_t ypr_WB[3] = {yaw, 0.0, 0.0};
    real_t q_WB[4] = {0};
    euler2quat(ypr_WB, q_WB);
    // -- Body Pose
    TF_QR(q_WB, r_WB, T_WB);

    // Camera pose
    TF_CHAIN(T_WC, 2, T_WB, T_BC);
    TF_VECTOR(T_WC, cam_pose);
    TF_INV(T_WC, T_CW);

    // Simulate camera frame
    sim_camera_frame_t *frame = sim_camera_frame_malloc(ts, cam_idx);
    for (size_t feature_id = 0; feature_id < num_features; feature_id++) {
      // Check point is infront of camera
      const real_t *p_W = &features[feature_id * 3];
      TF_POINT(T_CW, p_W, p_C);
      if (p_C[2] < 0) {
        continue;
      }

      // Project image point to image plane
      real_t z[2] = {0};
      pinhole_radtan4_project(cam_params->data, p_C, z);

      // Check projection
      const int x_ok = (z[0] < cam_params->resolution[0] && z[0] > 0);
      const int y_ok = (z[1] < cam_params->resolution[1] && z[1] > 0);
      if (x_ok == 0 || y_ok == 0) {
        continue;
      }

      // Add keypoint to camera frame
      sim_camera_frame_add_keypoint(frame, feature_id, z);
    }
    data->frames[k] = frame;

    // Update
    data->timestamps[k] = ts;
    vec_copy(cam_pose, 7, data->poses + k * 7);

    theta += w * dt;
    yaw += w * dt;
    ts += sec2ts(dt);
  }

  return data;
}

/////////////////////////
// SIM CAMERA IMU DATA //
/////////////////////////

sim_circle_camera_imu_t *sim_circle_camera_imu(void) {
  // Malloc
  sim_circle_camera_imu_t *sim_data = MALLOC(sim_circle_camera_imu_t, 1);

  // Simulate features
  const real_t origin[3] = {0.0, 0.0, 0.0};
  const real_t dim[3] = {5.0, 5.0, 5.0};
  sim_data->num_features = 1000;
  sim_create_features(origin,
                      dim,
                      sim_data->num_features,
                      sim_data->feature_data);

  // Camera configuration
  const int res[2] = {640, 480};
  const real_t fov = 90.0;
  const real_t fx = pinhole_focal(res[0], fov);
  const real_t fy = pinhole_focal(res[0], fov);
  const real_t cx = res[0] / 2.0;
  const real_t cy = res[1] / 2.0;
  const real_t cam_vec[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
  const char *pmodel = "pinhole";
  const char *dmodel = "radtan4";
  camera_params_t *cam0_params = &sim_data->cam0_params;
  camera_params_t *cam1_params = &sim_data->cam1_params;
  camera_params_setup(cam0_params, 0, res, pmodel, dmodel, cam_vec);
  camera_params_setup(cam1_params, 1, res, pmodel, dmodel, cam_vec);

  // IMU-Camera0 extrinsic
  const real_t cam0_ext_ypr[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  const real_t cam0_ext_r[3] = {0.05, 0.0, 0.0};
  const real_t cam1_ext_ypr[3] = {-M_PI / 2.0, 0.0, -M_PI / 2.0};
  const real_t cam1_ext_r[3] = {-0.05, 0.0, 0.0};
  TF_ER(cam0_ext_ypr, cam0_ext_r, T_SC0);
  TF_ER(cam1_ext_ypr, cam1_ext_r, T_SC1);
  tf_vector(T_SC0, sim_data->cam0_ext);
  tf_vector(T_SC1, sim_data->cam1_ext);

  // IMU extrinsic
  TF_IDENTITY(T_BS);
  tf_vector(T_BS, sim_data->imu0_ext);

  // Simulate data
  sim_circle_defaults(&sim_data->conf);
  sim_data->imu_data = sim_imu_circle_trajectory(&sim_data->conf);
  sim_data->cam0_data = sim_camera_circle_trajectory(&sim_data->conf,
                                                     T_SC0,
                                                     &sim_data->cam0_params,
                                                     sim_data->feature_data,
                                                     sim_data->num_features);
  sim_data->cam1_data = sim_camera_circle_trajectory(&sim_data->conf,
                                                     T_SC1,
                                                     &sim_data->cam1_params,
                                                     sim_data->feature_data,
                                                     sim_data->num_features);

  // Form timeline
  const int num_event_types = 1;
  timeline_event_t **events = MALLOC(timeline_event_t *, num_event_types);
  timestamp_t **events_timestamps = MALLOC(timestamp_t *, num_event_types);
  int *events_lengths = CALLOC(int, num_event_types);
  int *events_types = CALLOC(int, num_event_types);
  int type_idx = 0;

  // -- IMU data to timeline
  const size_t num_imu_events = sim_data->imu_data->num_measurements;
  timeline_event_t *imu_events = MALLOC(timeline_event_t, num_imu_events);
  for (size_t k = 0; k < sim_data->imu_data->num_measurements; k++) {
    imu_events[k].type = IMU_EVENT;
    imu_events[k].ts = sim_data->imu_data->timestamps[k];
    imu_events[k].data.imu.ts = sim_data->imu_data->timestamps[k];
    imu_events[k].data.imu.acc[0] = sim_data->imu_data->imu_acc[k * 3 + 0];
    imu_events[k].data.imu.acc[1] = sim_data->imu_data->imu_acc[k * 3 + 1];
    imu_events[k].data.imu.acc[2] = sim_data->imu_data->imu_acc[k * 3 + 2];
    imu_events[k].data.imu.gyr[0] = sim_data->imu_data->imu_gyr[k * 3 + 0];
    imu_events[k].data.imu.gyr[1] = sim_data->imu_data->imu_gyr[k * 3 + 1];
    imu_events[k].data.imu.gyr[2] = sim_data->imu_data->imu_gyr[k * 3 + 2];
  }
  events[type_idx] = imu_events;
  events_timestamps[type_idx] = CALLOC(timestamp_t, num_imu_events);
  for (int k = 0; k < num_imu_events; k++) {
    events_timestamps[type_idx][k] = events[type_idx][k].ts;
  }
  events_lengths[type_idx] = num_imu_events;
  events_types[type_idx] = IMU_EVENT;
  type_idx++;

  // -- Add to timeline
  sim_data->timeline = timeline_malloc();
  sim_data->timeline->num_imus = 1;
  sim_data->timeline->num_event_types = num_event_types;
  sim_data->timeline->events = events;
  sim_data->timeline->events_timestamps = events_timestamps;
  sim_data->timeline->events_lengths = events_lengths;
  sim_data->timeline->events_types = events_types;

  // -- Form timeline
  timeline_form_timeline(sim_data->timeline);

  return sim_data;
}

void sim_circle_camera_imu_free(sim_circle_camera_imu_t *sim_data) {
  sim_imu_data_free(sim_data->imu_data);
  sim_camera_data_free(sim_data->cam0_data);
  sim_camera_data_free(sim_data->cam1_data);
  timeline_free(sim_data->timeline);
  free(sim_data);
}

// /////////////////////
// // SIM GIMBAL DATA //
// /////////////////////
//
// /**
//  * Malloc a simulated gimbal view.
//  */
// sim_gimbal_view_t *sim_gimbal_view_malloc(const int max_corners) {
//   sim_gimbal_view_t *view = MALLOC(sim_gimbal_view_t, 1);
//
//   view->num_measurements = 0;
//   view->tag_ids = MALLOC(int, max_corners);
//   view->corner_indices = MALLOC(int, max_corners);
//   view->object_points = MALLOC(real_t, max_corners * 3);
//   view->keypoints = MALLOC(real_t, max_corners * 2);
//
//   assert(view->tag_ids != NULL);
//   assert(view->corner_indices != NULL);
//   assert(view->object_points != NULL);
//   assert(view->keypoints != NULL);
//
//   return view;
// }
//
// /**
//  * Free simulated gimbal view.
//  */
// void sim_gimbal_view_free(sim_gimbal_view_t *view) {
//   free(view->tag_ids);
//   free(view->corner_indices);
//   free(view->object_points);
//   free(view->keypoints);
//   free(view);
// }
//
// /**
//  * Print simulated gimbal view.
//  */
// void sim_gimbal_view_print(const sim_gimbal_view_t *view) {
//   printf("num_measurements: %d\n", view->num_measurements);
//   for (size_t i = 0; i < view->num_measurements; i++) {
//     printf("%d ", view->tag_ids[i]);
//     printf("%d ", view->corner_indices[i]);
//     printf("(%.2f, %.2f, %.2f) ",
//            view->object_points[i + 0],
//            view->object_points[i + 1],
//            view->object_points[i + 2]);
//     printf("(%.2f, %.2f) ", view->keypoints[i + 0], view->keypoints[i + 1]);
//     printf("\n");
//   }
// }
//
// /**
//  * Malloc gimbal simulation.
//  */
// sim_gimbal_t *sim_gimbal_malloc(void) {
//   sim_gimbal_t *sim = MALLOC(sim_gimbal_t, 1);
//
//   // Aprilgrid
//   int num_rows = 6;
//   int num_cols = 6;
//   double tag_size = 0.088;
//   double tag_spacing = 0.3;
//   sim->grid = aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);
//
//   // Fiducial pose
//   const real_t ypr_WF[3] = {-M_PI / 2.0, 0.0, M_PI / 2.0};
//   const real_t r_WF[3] = {0.5, 0.0, 0.0};
//   POSE_ER(ypr_WF, r_WF, fiducial_ext);
//   fiducial_setup(&sim->fiducial_ext, fiducial_ext);
//
//   // Gimbal pose
//   real_t x = 0.0;
//   real_t y = 0.0;
//   aprilgrid_center(sim->grid, &x, &y);
//   const real_t r_WB[3] = {0, -y, 0};
//   const real_t ypr_WB[3] = {0, 0, 0};
//   POSE_ER(ypr_WB, r_WB, gimbal_pose);
//   pose_setup(&sim->gimbal_pose, 0, gimbal_pose);
//
//   // Gimbal extrinsic (body to gimbal)
//   const real_t ypr_BM0[3] = {0.01, 0.01, 0.01};
//   const real_t r_BM0[3] = {0.001, 0.001, 0.001};
//   POSE_ER(ypr_BM0, r_BM0, gimbal_ext);
//   extrinsic_setup(&sim->gimbal_ext, gimbal_ext);
//
//   // Links
//   sim->num_links = 2;
//   sim->gimbal_links = MALLOC(extrinsic_t, sim->num_links);
//   // -- Roll link
//   const real_t ypr_L0M1[3] = {0.0, M_PI / 2, 0.0};
//   const real_t r_L0M1[3] = {-0.1, 0.0, 0.15};
//   POSE_ER(ypr_L0M1, r_L0M1, link_roll);
//   extrinsic_setup(&sim->gimbal_links[0], link_roll);
//   // -- Pitch link
//   const real_t ypr_L1M2[3] = {0.0, 0.0, -M_PI / 2.0};
//   const real_t r_L1M2[3] = {0.0, -0.05, 0.1};
//   POSE_ER(ypr_L1M2, r_L1M2, link_pitch);
//   extrinsic_setup(&sim->gimbal_links[1], link_pitch);
//
//   // Joints
//   sim->num_joints = 3;
//   sim->gimbal_joints = MALLOC(joint_t, sim->num_joints);
//   joint_setup(&sim->gimbal_joints[0], 0, 0, 0.0);
//   joint_setup(&sim->gimbal_joints[1], 0, 1, 0.0);
//   joint_setup(&sim->gimbal_joints[2], 0, 2, 0.0);
//
//   // Setup cameras
//   sim->num_cams = 2;
//   // -- Camera extrinsic
//   sim->cam_exts = MALLOC(extrinsic_t, sim->num_cams);
//   // ---- cam0 extrinsic
//   const real_t ypr_M2eC0[3] = {-M_PI / 2.0, M_PI / 2.0, 0.0};
//   const real_t r_M2eC0[3] = {0.0, -0.05, 0.12};
//   POSE_ER(ypr_M2eC0, r_M2eC0, cam0_exts);
//   extrinsic_setup(&sim->cam_exts[0], cam0_exts);
//   // ---- cam1 extrinsic
//   const real_t ypr_M2eC1[3] = {-M_PI / 2.0, M_PI / 2.0, 0.0};
//   const real_t r_M2eC1[3] = {0.0, -0.05, -0.12};
//   POSE_ER(ypr_M2eC1, r_M2eC1, cam1_exts);
//   extrinsic_setup(&sim->cam_exts[1], cam1_exts);
//   // -- Camera parameters
//   const int cam_res[2] = {640, 480};
//   const real_t fov = 120.0;
//   const real_t fx = pinhole_focal(cam_res[0], fov);
//   const real_t fy = pinhole_focal(cam_res[0], fov);
//   const real_t cx = cam_res[0] / 2.0;
//   const real_t cy = cam_res[1] / 2.0;
//   const char *proj_model = "pinhole";
//   const char *dist_model = "radtan4";
//   const real_t data[8] = {fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0};
//   sim->cam_params = MALLOC(camera_params_t, sim->num_cams);
//   camera_params_setup(&sim->cam_params[0],
//                       0,
//                       cam_res,
//                       proj_model,
//                       dist_model,
//                       data);
//   camera_params_setup(&sim->cam_params[1],
//                       1,
//                       cam_res,
//                       proj_model,
//                       dist_model,
//                       data);
//
//   return sim;
// }
//
// /**
//  * Free gimbal simulation.
//  */
// void sim_gimbal_free(sim_gimbal_t *sim) {
//   if (sim == NULL) {
//     return;
//   }
//
//   aprilgrid_free(sim->grid);
//   FREE(sim->gimbal_links);
//   FREE(sim->gimbal_joints);
//   FREE(sim->cam_exts);
//   FREE(sim->cam_params);
//   FREE(sim);
// }
//
// /**
//  * Print gimbal simulation.
//  */
// void sim_gimbal_print(const sim_gimbal_t *sim) {
//   // Configuration file
//   for (int cam_idx = 0; cam_idx < sim->num_cams; cam_idx++) {
//     camera_params_print(&sim->cam_params[cam_idx]);
//     printf("\n");
//   }
//   for (int cam_idx = 0; cam_idx < sim->num_cams; cam_idx++) {
//     char cam_str[20] = {0};
//     sprintf(cam_str, "cam%d_ext", cam_idx);
//     extrinsic_print(cam_str, &sim->cam_exts[cam_idx]);
//   }
//   for (int link_idx = 0; link_idx < sim->num_links; link_idx++) {
//     char link_str[20] = {0};
//     sprintf(link_str, "link%d_ext", link_idx);
//     extrinsic_print(link_str, &sim->gimbal_links[link_idx]);
//   }
//   extrinsic_print("gimbal_ext", &sim->gimbal_ext);
//   fiducial_print("fiducial_ext", &sim->fiducial_ext);
// }
//
// /**
//  * Set gimbal joint.
//  */
// void sim_gimbal_set_joint(sim_gimbal_t *sim,
//                           const int joint_idx,
//                           const real_t angle) {
//   sim->gimbal_joints[joint_idx].data[0] = angle;
// }
//
// /**
//  * Get gimbal joint.
//  */
// void sim_gimbal_get_joints(sim_gimbal_t *sim,
//                            const int num_joints,
//                            real_t *angles) {
//   for (int i = 0; i < num_joints; i++) {
//     angles[i] = sim->gimbal_joints[i].data[0];
//   }
// }
//
// /**
//  * Simulate 3-axis gimbal view.
//  */
// sim_gimbal_view_t *sim_gimbal3_view(const aprilgrid_t *grid,
//                                     const timestamp_t ts,
//                                     const int view_idx,
//                                     const real_t fiducial_pose[7],
//                                     const real_t body_pose[7],
//                                     const real_t gimbal_ext[7],
//                                     const real_t gimbal_link0[7],
//                                     const real_t gimbal_link1[7],
//                                     const real_t gimbal_joint0,
//                                     const real_t gimbal_joint1,
//                                     const real_t gimbal_joint2,
//                                     const int cam_idx,
//                                     const int cam_res[2],
//                                     const real_t cam_params[8],
//                                     const real_t cam_ext[7]) {
//   // Form: T_CiF
//   TF(fiducial_pose, T_WF);
//   TF(body_pose, T_WB);
//   TF(gimbal_ext, T_BM0);
//   TF(gimbal_link0, T_L0M1);
//   TF(gimbal_link1, T_L1M2);
//   GIMBAL_JOINT_TF(gimbal_joint0, T_M0L0);
//   GIMBAL_JOINT_TF(gimbal_joint1, T_M1L1);
//   GIMBAL_JOINT_TF(gimbal_joint2, T_M2L2);
//   TF(cam_ext, T_L2Ci);
//   TF_CHAIN(T_BCi, 7, T_BM0, T_M0L0, T_L0M1, T_M1L1, T_L1M2, T_M2L2, T_L2Ci);
//   TF_INV(T_BCi, T_CiB);
//   TF_INV(T_WB, T_BW);
//   TF_CHAIN(T_CiF, 3, T_CiB, T_BW, T_WF);
//
//   const int max_tags = grid->num_rows * grid->num_cols;
//   const int max_corners = max_tags * 4;
//   sim_gimbal_view_t *view = sim_gimbal_view_malloc(max_corners);
//
//   for (int tag_id = 0; tag_id < max_tags; tag_id++) {
//     for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
//       // Transform fiducial point to camera frame
//       real_t p_FFi[3] = {0};
//       aprilgrid_object_point(grid, tag_id, corner_idx, p_FFi);
//       TF_POINT(T_CiF, p_FFi, p_CiFi);
//
//       // Check point is infront of camera
//       if (p_CiFi[2] < 0) {
//         continue;
//       }
//
//       // Project image point to image plane
//       real_t z[2] = {0};
//       pinhole_radtan4_project(cam_params, p_CiFi, z);
//
//       // Check projection
//       const int x_ok = (z[0] < cam_res[0] && z[0] > 0);
//       const int y_ok = (z[1] < cam_res[1] && z[1] > 0);
//       if (x_ok == 0 || y_ok == 0) {
//         continue;
//       }
//
//       // Add to measurements
//       view->tag_ids[view->num_measurements] = tag_id;
//       view->corner_indices[view->num_measurements] = corner_idx;
//       view->object_points[view->num_measurements * 3] = p_FFi[0];
//       view->object_points[view->num_measurements * 3 + 1] = p_FFi[1];
//       view->object_points[view->num_measurements * 3 + 2] = p_FFi[2];
//       view->keypoints[view->num_measurements * 2] = z[0];
//       view->keypoints[view->num_measurements * 2 + 1] = z[1];
//       view->num_measurements++;
//     }
//   }
//
//   return view;
// }
//
// /**
//  * Simulate 3-axis gimbal view.
//  */
// sim_gimbal_view_t *sim_gimbal_view(const sim_gimbal_t *sim,
//                                    const timestamp_t ts,
//                                    const int view_idx,
//                                    const int cam_idx,
//                                    const real_t body_pose[7]) {
//   return sim_gimbal3_view(sim->grid,
//                           ts,
//                           view_idx,
//                           sim->fiducial_ext.data,
//                           body_pose,
//                           sim->gimbal_ext.data,
//                           sim->gimbal_links[0].data,
//                           sim->gimbal_links[1].data,
//                           sim->gimbal_joints[0].data[0],
//                           sim->gimbal_joints[1].data[0],
//                           sim->gimbal_joints[2].data[0],
//                           cam_idx,
//                           sim->cam_params->resolution,
//                           sim->cam_params->data,
//                           sim->cam_exts[cam_idx].data);
// }
