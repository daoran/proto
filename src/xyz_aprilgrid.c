#include "xyz_aprilgrid.h"

// APRILGRID /////////////////////////////////////////////////////////////////

/**
 * Malloc AprilGrid.
 *
 * @param ts Timestamp
 * @param grid AprilGrid
 */
aprilgrid_t *aprilgrid_malloc(const int num_rows,
                              const int num_cols,
                              const real_t tag_size,
                              const real_t tag_spacing) {
  aprilgrid_t *grid = malloc(sizeof(aprilgrid_t));
  grid->num_rows = num_rows;
  grid->num_cols = num_cols;
  grid->tag_size = tag_size;
  grid->tag_spacing = tag_spacing;

  // Grid data
  grid->timestamp = 0;
  const int max_corners = (num_rows * num_cols * 4);
  grid->corners_detected = 0;
  grid->data = calloc(max_corners * 6, sizeof(real_t));

  return grid;
}

void aprilgrid_free(aprilgrid_t *grid) {
  if (grid) {
    free(grid->data);
    free(grid);
  }
}

/**
 * Clear Aprilgrid
 */
void aprilgrid_clear(aprilgrid_t *grid) {
  assert(grid != NULL);

  grid->timestamp = 0;
  grid->corners_detected = 0;
  const int max_corners = (grid->num_rows * grid->num_cols * 4);
  for (int i = 0; i < (max_corners * 6); i++) {
    grid->data[i] = 0;
  }
}

/**
 * Reset Aprilgrid
 */
void aprilgrid_reset(aprilgrid_t *grid) {
  assert(grid != NULL);

  grid->timestamp = 0;
  grid->corners_detected = 0;
  const int max_corners = (grid->num_rows * grid->num_cols * 4);
  for (int i = 0; i < (max_corners * 6); i++) {
    grid->data[i] = 0;
  }

  grid->num_rows = 0;
  grid->num_cols = 0;
  grid->tag_size = 0;
  grid->tag_spacing = 0;
}

/**
 * Copy AprilGrid
 */
void aprilgrid_copy(const aprilgrid_t *src, aprilgrid_t *dst) {
  dst->timestamp = src->timestamp;
  dst->num_rows = src->num_rows;
  dst->num_cols = src->num_cols;
  dst->tag_size = src->tag_size;
  dst->tag_spacing = src->tag_spacing;

  dst->corners_detected = src->corners_detected;
  for (size_t i = 0; i < (dst->num_rows * dst->num_cols * 4); i++) {
    for (size_t j = 0; j < 6; j++) {
      dst->data[i * 6 + j] = src->data[i * 6 + j];
    }
  }
}

/**
 * Check AprilGrids are equal
 */
int aprilgrid_equals(const aprilgrid_t *grid0, const aprilgrid_t *grid1) {
  APRILGRID_CHECK(grid0->timestamp == grid1->timestamp);
  APRILGRID_CHECK(grid0->num_rows == grid1->num_rows);
  APRILGRID_CHECK(grid0->num_cols == grid1->num_cols);
  APRILGRID_CHECK(fabs(grid0->tag_size - grid1->tag_size) < 1e-8);
  APRILGRID_CHECK(fabs(grid0->tag_spacing - grid1->tag_spacing) < 1e-8);
  APRILGRID_CHECK(grid0->corners_detected == grid1->corners_detected);

  for (size_t i = 0; i < (grid0->num_rows * grid0->num_cols * 4); i++) {
    for (size_t j = 0; j < 6; j++) {
      APRILGRID_CHECK(fabs(grid0->data[i * 6 + j] - grid1->data[i * 6 + j]) <
                      1e-8);
    }
  }

  return 1;
error:
  return 0;
}

/**
 * Print Aprilgrid
 */
void aprilgrid_print(const aprilgrid_t *grid) {
  assert(grid != NULL);

  printf("timestamp: %ld\n", grid->timestamp);
  printf("num_rows: %d\n", grid->num_rows);
  printf("num_cols: %d\n", grid->num_cols);
  printf("tag_size: %f\n", grid->tag_size);
  printf("tag_spacing: %f\n", grid->tag_spacing);
  printf("\n");
  printf("corners_detected: %d\n", grid->corners_detected);
  printf("#tag_id, corner_idx, kp_x, kp_y, p_x, p_y, p_z\n");
  const int max_corners = (grid->num_rows * grid->num_cols * 4);
  for (int i = 0; i < max_corners; i++) {
    if (grid->data[i * 6 + 0] <= 0) {
      continue;
    }

    const int tag_id = i / 4;
    const int corner_idx = i % 4;
    printf("%d, ", tag_id);
    printf("%d, ", corner_idx);
    printf("%.2f, ", grid->data[i * 6 + 1]);
    printf("%.2f, ", grid->data[i * 6 + 2]);
    printf("%.2f, ", grid->data[i * 6 + 3]);
    printf("%.2f, ", grid->data[i * 6 + 4]);
    printf("%.2f", grid->data[i * 6 + 5]);
    printf("\n");
  }
}

/**
 * Return center of AprilGrid
 */
void aprilgrid_center(const aprilgrid_t *grid, real_t *cx, real_t *cy) {
  assert(grid != NULL);
  assert(cx != NULL);
  assert(cy != NULL);

  *cx = ((grid->num_cols / 2.0) * grid->tag_size);
  *cx += (((grid->num_cols / 2.0) - 1) * grid->tag_spacing * grid->tag_size);
  *cx += (0.5 * grid->tag_spacing * grid->tag_size);

  *cy = ((grid->num_rows / 2.0) * grid->tag_size);
  *cy += (((grid->num_rows / 2.0) - 1) * grid->tag_spacing * grid->tag_size);
  *cy += (0.5 * grid->tag_spacing * grid->tag_size);
}

/**
 * Return AprilTag grid index within the AprilGrid based on tag id
 */
void aprilgrid_grid_index(const aprilgrid_t *grid,
                          const int tag_id,
                          int *i,
                          int *j) {
  assert(grid != NULL);
  assert(tag_id >= 0 && tag_id <= (grid->num_rows * grid->num_cols - 1));
  assert(i != NULL);
  assert(j != NULL);

  if (tag_id > (grid->num_rows * grid->num_cols)) {
    APRILGRID_FATAL("tag_id > (num_rows * num_cols)!\n");
  } else if (tag_id < 0) {
    APRILGRID_FATAL("tag_id < 0!\n");
  }

  *i = (int) (tag_id / grid->num_cols);
  *j = (int) (tag_id % grid->num_cols);
}

/**
 * Return AprilGrid object point from tag id and corner index
 */
void aprilgrid_object_point(const aprilgrid_t *grid,
                            const int tag_id,
                            const int corner_idx,
                            real_t object_point[3]) {
  assert(grid != NULL);
  assert(tag_id >= 0 && tag_id <= (grid->num_rows * grid->num_cols - 1));
  assert(corner_idx >= 0 && corner_idx <= 3);
  assert(object_point != NULL);

  // Calculate the AprilGrid index using tag id
  int i = 0;
  int j = 0;
  aprilgrid_grid_index(grid, tag_id, &i, &j);

  // Caculate the x and y of the tag origin (bottom left corner of tag)
  // relative to grid origin (bottom left corner of entire grid)
  const real_t x = j * (grid->tag_size + grid->tag_size * grid->tag_spacing);
  const real_t y = i * (grid->tag_size + grid->tag_size * grid->tag_spacing);

  // Calculate the x and y of each corner
  switch (corner_idx) {
    case 0: // Bottom left
      object_point[0] = x;
      object_point[1] = y;
      object_point[2] = 0;
      break;
    case 1: // Bottom right
      object_point[0] = x + grid->tag_size;
      object_point[1] = y;
      object_point[2] = 0;
      break;
    case 2: // Top right
      object_point[0] = x + grid->tag_size;
      object_point[1] = y + grid->tag_size;
      object_point[2] = 0;
      break;
    case 3: // Top left
      object_point[0] = x;
      object_point[1] = y + grid->tag_size;
      object_point[2] = 0;
      break;
    default:
      APRILGRID_FATAL("Incorrect corner id [%d]!\n", corner_idx);
      break;
  }
}

/**
 * Add AprilGrid corner measurement
 */
void aprilgrid_add_corner(aprilgrid_t *grid,
                          const int tag_id,
                          const int corner_idx,
                          const real_t kp[2]) {
  assert(grid != NULL);
  assert(tag_id >= 0 && tag_id <= (grid->num_rows * grid->num_cols - 1));
  assert(corner_idx >= 0 && corner_idx <= 3);

  // Set AprilGrid as detected
  grid->corners_detected++;

  // Push tag_id and keypoints
  const int data_row = (tag_id * 4) + corner_idx;
  real_t p[3] = {0};
  aprilgrid_object_point(grid, tag_id, corner_idx, p);

  grid->data[data_row * 6 + 0] = 1;
  grid->data[data_row * 6 + 1] = kp[0];
  grid->data[data_row * 6 + 2] = kp[1];
  grid->data[data_row * 6 + 3] = p[0];
  grid->data[data_row * 6 + 4] = p[1];
  grid->data[data_row * 6 + 5] = p[2];
}

/**
 * Remove AprilGrid corner measurement
 */
void aprilgrid_remove_corner(aprilgrid_t *grid,
                             const int tag_id,
                             const int corner_idx) {
  assert(grid != NULL);
  assert(tag_id >= 0 && tag_id <= (grid->num_rows * grid->num_cols - 1));
  assert(corner_idx >= 0 && corner_idx <= 3);

  const int data_row = (tag_id * 4) + corner_idx;
  assert(data_row >= 0);
  grid->data[data_row * 6 + 0] = 0;
  grid->data[data_row * 6 + 1] = 0;
  grid->data[data_row * 6 + 2] = 0;
  grid->data[data_row * 6 + 3] = 0;
  grid->data[data_row * 6 + 4] = 0;
  grid->data[data_row * 6 + 5] = 0;
  grid->corners_detected--;
}

/**
 * Add AprilGrid AprilTag measurement
 */
void aprilgrid_add_tag(aprilgrid_t *grid,
                       const int tag_id,
                       const real_t tag_kps[4][2]) {
  assert(grid != NULL);
  assert(tag_id >= 0 && tag_id <= (grid->num_rows * grid->num_cols - 1));
  assert(tag_kps != NULL);

  for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
    if (tag_kps[corner_idx] == NULL) {
      continue;
    }
    aprilgrid_add_corner(grid, tag_id, corner_idx, tag_kps[corner_idx]);
  }
}

/**
 * Remove AprilGrid AprilTag measurement
 */
void aprilgrid_remove_tag(aprilgrid_t *grid, const int tag_id) {
  assert(grid != NULL);
  assert(tag_id >= 0 && tag_id <= (grid->num_rows * grid->num_cols - 1));

  for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
    aprilgrid_remove_corner(grid, tag_id, corner_idx);
  }
}

/**
 * Return AprilGrid measurements
 */
void aprilgrid_measurements(const aprilgrid_t *grid,
                            int *tag_ids,
                            int *corner_idxs,
                            real_t *tag_kps,
                            real_t *obj_pts) {
  assert(grid != NULL);
  assert(tag_ids != NULL);
  assert(corner_idxs != NULL);
  assert(tag_kps != NULL);
  assert(obj_pts != NULL);

  // Pre-check
  if (grid->corners_detected == 0) {
    return;
  }

  // Get measurements
  int meas_idx = 0;
  for (long i = 0; i < (grid->num_rows * grid->num_cols * 4); i++) {
    if (grid->data[i * 6 + 0] < 1.0) {
      continue;
    }

    const int tag_id = i / 4;
    const int corner_idx = i % 4;
    const real_t kp_x = grid->data[i * 6 + 1];
    const real_t kp_y = grid->data[i * 6 + 2];
    const real_t p_x = grid->data[i * 6 + 3];
    const real_t p_y = grid->data[i * 6 + 4];
    const real_t p_z = grid->data[i * 6 + 5];

    tag_ids[meas_idx] = tag_id;
    corner_idxs[meas_idx] = corner_idx;
    tag_kps[meas_idx * 2] = kp_x;
    tag_kps[meas_idx * 2 + 1] = kp_y;
    obj_pts[meas_idx * 3] = p_x;
    obj_pts[meas_idx * 3 + 1] = p_y;
    obj_pts[meas_idx * 3 + 2] = p_z;
    meas_idx++;
  }
}

/**
 * Save AprilGrid
 */
int aprilgrid_save(const aprilgrid_t *grid, const char *save_path) {
  assert(grid != NULL);
  assert(save_path != NULL);

  // Open file for saving
  FILE *fp = fopen(save_path, "w");
  if (fp == NULL) {
    APRILGRID_LOG("Failed to open [%s] for saving!", save_path);
    return -1;
  }

  // Output header
  // -- Configuration
  fprintf(fp, "timestamp:%ld\n", grid->timestamp);
  fprintf(fp, "num_rows:%d\n", grid->num_rows);
  fprintf(fp, "num_cols:%d\n", grid->num_cols);
  fprintf(fp, "tag_size:%f\n", grid->tag_size);
  fprintf(fp, "tag_spacing:%f\n", grid->tag_spacing);
  fprintf(fp, "\n");
  // -- Data
  fprintf(fp, "corners_detected:%d\n", grid->corners_detected);
  fprintf(fp, "tag_id,corner_idx,kp_x,kp_y,p_x,p_y,p_z\n");

  // Output data
  if (grid->corners_detected) {
    // vec2s_t kps = keypoints();
    for (long i = 0; i < (grid->num_rows * grid->num_cols * 4); i++) {
      const int tag_id = i / 4;
      const int corner_idx = i % 4;

      if (grid->data[i * 6 + 0] > 0) { // Corner detected?
        fprintf(fp, "%d,", tag_id);
        fprintf(fp, "%d,", corner_idx);
        fprintf(fp, "%f,", grid->data[i * 6 + 1]);
        fprintf(fp, "%f,", grid->data[i * 6 + 2]);
        fprintf(fp, "%f,", grid->data[i * 6 + 3]);
        fprintf(fp, "%f,", grid->data[i * 6 + 4]);
        fprintf(fp, "%f", grid->data[i * 6 + 5]);
        fprintf(fp, "\n");
      }
    }
  }

  // Close up
  fclose(fp);

  return 0;
}

static void aprilgrid_parse_line(FILE *fp,
                                 const char *key,
                                 const char *value_type,
                                 void *value) {
  assert(fp != NULL);
  assert(key != NULL);
  assert(value_type != NULL);
  assert(value != NULL);

  // Parse line
  const size_t buf_len = 1024;
  char buf[1024] = {0};
  if (fgets(buf, buf_len, fp) == NULL) {
    APRILGRID_FATAL("Failed to parse [%s]\n", key);
  }

  // Split key-value
  char delim[2] = ":";
  char *key_str = strtok(buf, delim);
  char *value_str = strtok(NULL, delim);

  // Check key matches
  if (strcmp(key_str, key) != 0) {
    APRILGRID_FATAL("Failed to parse [%s]\n", key);
  }

  // Typecase value
  if (value_type == NULL) {
    APRILGRID_FATAL("Value type not set!\n");
  }

  if (strcmp(value_type, "uint64_t") == 0) {
    *(uint64_t *) value = atol(value_str);
  } else if (strcmp(value_type, "int") == 0) {
    *(int *) value = atoi(value_str);
  } else if (strcmp(value_type, "real_t") == 0) {
    *(real_t *) value = atof(value_str);
  } else {
    APRILGRID_FATAL("Invalid value type [%s]\n", value_type);
  }
}

static void aprilgrid_parse_skip_line(FILE *fp) {
  assert(fp != NULL);
  const size_t buf_len = 1024;
  char buf[1024] = {0};
  char *retval = fgets(buf, buf_len, fp);
  APRILGRID_UNUSED(retval);
}

/**
 * Load AprilGrid
 */
aprilgrid_t *aprilgrid_load(const char *data_path) {
  assert(data_path != NULL);

  // Open file for loading
  FILE *fp = fopen(data_path, "r");
  if (fp == NULL) {
    APRILGRID_LOG("Failed to open [%s]!\n", data_path);
    return NULL;
  }

  // Parse configuration
  timestamp_t timestamp;
  int num_rows = 0;
  int num_cols = 0;
  real_t tag_size = 0;
  real_t tag_spacing = 0;
  aprilgrid_parse_line(fp, "timestamp", "uint64_t", &timestamp);
  aprilgrid_parse_line(fp, "num_rows", "int", &num_rows);
  aprilgrid_parse_line(fp, "num_cols", "int", &num_cols);
  aprilgrid_parse_line(fp, "tag_size", "real_t", &tag_size);
  aprilgrid_parse_line(fp, "tag_spacing", "real_t", &tag_spacing);
  aprilgrid_parse_skip_line(fp);
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  // Parse data
  int corners_detected = 0;
  grid->timestamp = timestamp;
  aprilgrid_parse_line(fp, "corners_detected", "int", &corners_detected);
  aprilgrid_parse_skip_line(fp);

#if PRECISION == 1
  const char *scan_format = "%d,%d,%f,%f,%f,%f,%f";
#else
  const char *scan_format = "%d,%d,%lf,%lf,%lf,%lf,%lf";
#endif
  for (int i = 0; i < corners_detected; i++) {
    // Parse data line
    int tag_id = 0;
    int corner_idx = 0;
    real_t kp[2] = {0};
    real_t p[3] = {0};
    const int retval = fscanf(fp,
                              scan_format,
                              &tag_id,
                              &corner_idx,
                              &kp[0],
                              &kp[1],
                              &p[0],
                              &p[1],
                              &p[2]);
    if (retval != 7) {
      APRILGRID_FATAL("Failed to parse data line in [%s]\n", data_path);
    }

    // Add corner
    aprilgrid_add_corner(grid, tag_id, corner_idx, kp);
  }

  // Clean up
  fclose(fp);

  return grid;
}

// APRILGRID DETECTOR ////////////////////////////////////////////////////////

aprilgrid_detector_t *aprilgrid_detector_malloc(int num_rows,
                                                int num_cols,
                                                real_t tag_size,
                                                real_t tag_spacing) {
  aprilgrid_detector_t *det = malloc(sizeof(aprilgrid_detector_t));
  // det->tf = tagStandard41h12_create();
  det->tf = tag36h11_create();
  det->td = apriltag_detector_create();
  apriltag_detector_add_family_bits(det->td, det->tf, 1);

  det->num_rows = num_rows;
  det->num_cols = num_cols;
  det->tag_size = tag_size;
  det->tag_spacing = tag_spacing;

  return det;
}

void aprilgrid_detector_free(aprilgrid_detector_t *det) {
  assert(det != NULL);
  apriltag_detector_destroy(det->td);
  // tagStandard41h12_destroy(det->tf);
  tag36h11_destroy(det->tf);
  free(det);
  det = NULL;
}

aprilgrid_t *aprilgrid_detector_detect(const aprilgrid_detector_t *det,
                                       const timestamp_t ts,
                                       const int32_t image_width,
                                       const int32_t image_height,
                                       const int32_t image_stride,
                                       uint8_t *image_data) {
  assert(det != NULL);
  assert(image_width > 0);
  assert(image_height > 0);
  assert(image_stride > 0);
  assert(image_data != NULL);

  // Form image_u8_t
  image_u8_t im = {.width = image_width,
                   .height = image_height,
                   .stride = image_stride,
                   .buf = image_data};

  // Detect AprilTags
  aprilgrid_t *grid = aprilgrid_malloc(det->num_rows,
                                       det->num_cols,
                                       det->tag_size,
                                       det->tag_spacing);
  grid->timestamp = ts;
  zarray_t *dets = apriltag_detector_detect(det->td, &im);
  // int num_corners = 0;
  for (int i = 0; i < zarray_size(dets); i++) {
    apriltag_detection_t *det;
    zarray_get(dets, i, &det);

    const real_t p0[2] = {det->p[0][0], det->p[0][1]};
    const real_t p1[2] = {det->p[1][0], det->p[1][1]};
    const real_t p2[2] = {det->p[2][0], det->p[2][1]};
    const real_t p3[2] = {det->p[3][0], det->p[3][1]};

    aprilgrid_add_corner(grid, det->id, 0, p0);
    aprilgrid_add_corner(grid, det->id, 1, p1);
    aprilgrid_add_corner(grid, det->id, 2, p2);
    aprilgrid_add_corner(grid, det->id, 3, p3);
    // num_corners += 4;
  }
  apriltag_detections_destroy(dets);

  // // Return NULL if no apriltag detected
  // if (num_corners == 0) {
  //   aprilgrid_free(grid);
  //   return NULL;
  // }

  return grid;
}
