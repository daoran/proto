#ifndef APRILGRID_H
#define APRILGRID_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <inttypes.h>

#include "apriltag/apriltag.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/common/pjpeg.h"
#include "apriltag/common/image_u8.h"

// CONSTANTS
#ifndef timestamp_t
#define timestamp_t int64_t
#endif

// MACROS
#ifndef APRILGRID_LOG
#define APRILGRID_LOG(...) printf(__VA_ARGS__);
#endif

#ifndef APRILGRID_FATAL
#define APRILGRID_FATAL(...)                                                   \
  printf(__VA_ARGS__);                                                         \
  exit(-1);
#endif

#ifndef APRILGRID_UNUSED
#define APRILGRID_UNUSED(expr)                                                 \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)
#endif

#ifndef APRILGRID_CHECK
#define APRILGRID_CHECK(X)                                                     \
  if (!(X)) {                                                                  \
    APRILGRID_LOG(#X " Failed!\n");                                            \
    goto error;                                                                \
  }
#endif

// APRILGRID /////////////////////////////////////////////////////////////////

#define APRILGRID_NUM_ROWS 6
#define APRILGRID_NUM_COLS 6
#define APRILGRID_MAX_CORNERS APRILGRID_NUM_ROWS *APRILGRID_NUM_COLS * 4
#define APRILGRID_TAG_SIZE 0.10
#define APRILGRID_TAG_SPACING 0.0

typedef struct aprilgrid_t {
  // Grid properties
  timestamp_t timestamp;
  int num_rows;
  int num_cols;
  double tag_size;
  double tag_spacing;

  // Grid data
  int corners_detected;
  double data[APRILGRID_MAX_CORNERS][6];
} aprilgrid_t;

void aprilgrid_setup(const timestamp_t ts, aprilgrid_t *grid);
void aprilgrid_reset(aprilgrid_t *grid);
void aprilgrid_copy(const aprilgrid_t *src, aprilgrid_t *dst);
int aprilgrid_equals(const aprilgrid_t *grid0, const aprilgrid_t *grid1);
void aprilgrid_center(const aprilgrid_t *grid, double *cx, double *cy);
void aprilgrid_grid_index(const aprilgrid_t *grid,
                          const int tag_id,
                          int *i,
                          int *j);
void aprilgrid_object_point(const aprilgrid_t *grid,
                            const int tag_id,
                            const int corner_idx,
                            double object_point[3]);
void aprilgrid_add_corner(aprilgrid_t *grid,
                          const int tag_id,
                          const int corner_idx,
                          const double kp[2]);
void aprilgrid_remove_corner(aprilgrid_t *grid,
                             const int tag_id,
                             const int corner_idx);
void aprilgrid_add_tag(aprilgrid_t *grid,
                       const int tag_id,
                       const double kp[4][2]);
void aprilgrid_remove_tag(aprilgrid_t *grid, const int tag_id);
void aprilgrid_measurements(const aprilgrid_t *grid,
                            int *tag_ids,
                            int *corner_idxs,
                            double *tag_kps,
                            double *obj_pts,
                            int *num_corners);
int aprilgrid_save(const aprilgrid_t *grid, const char *save_path);
int aprilgrid_load(aprilgrid_t *grid, const char *data_path);

// APRILGRID DETECTOR ////////////////////////////////////////////////////////

typedef struct aprilgrid_detector_t {
  apriltag_family_t *tf;
  apriltag_detector_t *td;
} aprilgrid_detector_t;

aprilgrid_detector_t *aprilgrid_detector_malloc();
void aprilgrid_detector_free(aprilgrid_detector_t *det);
int aprilgrid_detector_detect(const aprilgrid_detector_t *det,
                              aprilgrid_t *grid,
                              const int32_t image_width,
                              const int32_t image_height,
                              const int32_t image_stride,
                              uint8_t *image_data);

#endif // APRILGRID_H

//////////////////////////////////////////////////////////////////////////////
//                             IMPLEMENTATION                               //
//////////////////////////////////////////////////////////////////////////////

#ifdef APRILGRID_IMPLEMENTATION

// APRILGRID /////////////////////////////////////////////////////////////////

/**
 * Setup AprilGrid
 *
 * @param ts Timestamp
 * @param grid AprilGrid
 */
void aprilgrid_setup(const timestamp_t ts, aprilgrid_t *grid) {
  assert(grid != NULL);

  grid->timestamp = ts;
  grid->num_rows = APRILGRID_NUM_ROWS;
  grid->num_cols = APRILGRID_NUM_COLS;
  grid->tag_size = APRILGRID_TAG_SIZE;
  grid->tag_spacing = APRILGRID_TAG_SPACING;

  // Grid data
  grid->corners_detected = 0; // Number of corners detected
  for (int i = 0; i < APRILGRID_MAX_CORNERS; i++) {
    for (int j = 0; j < 6; j++) {
      grid->data[i][j] = 0;
    }
  }
}

/**
 * Reset Aprilgrid
 */
void aprilgrid_reset(aprilgrid_t *grid) {
  assert(grid != NULL);
  aprilgrid_setup(grid->timestamp, grid);
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
      dst->data[i][j] = src->data[i][j];
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
      APRILGRID_CHECK(fabs(grid0->data[i][j] - grid1->data[i][j]) < 1e-8);
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
  for (int i = 0; i < APRILGRID_MAX_CORNERS; i++) {
    if (grid->data[i][0] <= 0) {
      continue;
    }

    const int tag_id = i / 4;
    const int corner_idx = i % 4;
    printf("%d, ", tag_id);
    printf("%d, ", corner_idx);
    printf("%.2f, ", grid->data[i][1]);
    printf("%.2f, ", grid->data[i][2]);
    printf("%.2f, ", grid->data[i][3]);
    printf("%.2f, ", grid->data[i][4]);
    printf("%.2f", grid->data[i][5]);
    printf("\n");
  }
}

/**
 * Return center of AprilGrid
 */
void aprilgrid_center(const aprilgrid_t *grid, double *cx, double *cy) {
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
                            double object_point[3]) {
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
  const double x = j * (grid->tag_size + grid->tag_size * grid->tag_spacing);
  const double y = i * (grid->tag_size + grid->tag_size * grid->tag_spacing);

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
                          const double kp[2]) {
  assert(grid != NULL);
  assert(tag_id >= 0 && tag_id <= (grid->num_rows * grid->num_cols - 1));
  assert(corner_idx >= 0 && corner_idx <= 3);

  // Set AprilGrid as detected
  grid->corners_detected++;

  // Push tag_id and keypoints
  const int data_row = (tag_id * 4) + corner_idx;
  double p[3] = {0};
  aprilgrid_object_point(grid, tag_id, corner_idx, p);

  grid->data[data_row][0] = 1;
  grid->data[data_row][1] = kp[0];
  grid->data[data_row][2] = kp[1];
  grid->data[data_row][3] = p[0];
  grid->data[data_row][4] = p[1];
  grid->data[data_row][5] = p[2];
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
  grid->data[data_row][0] = 0;
  grid->data[data_row][1] = 0;
  grid->data[data_row][2] = 0;
  grid->data[data_row][3] = 0;
  grid->data[data_row][4] = 0;
  grid->data[data_row][5] = 0;
  grid->corners_detected--;
}

/**
 * Add AprilGrid AprilTag measurement
 */
void aprilgrid_add_tag(aprilgrid_t *grid,
                       const int tag_id,
                       const double tag_kps[4][2]) {
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
                            double *tag_kps,
                            double *obj_pts,
                            int *num_corners) {
  assert(grid != NULL);
  assert(tag_ids != NULL);
  assert(corner_idxs != NULL);
  assert(tag_kps != NULL);
  assert(obj_pts != NULL);
  assert(num_corners != NULL);

  // Pre-check
  *num_corners = grid->corners_detected;
  if (grid->corners_detected == 0) {
    return;
  }

  // Get measurements
  int meas_idx = 0;
  for (long i = 0; i < (grid->num_rows * grid->num_cols * 4); i++) {
    if (grid->data[i][0] < 1.0) {
      continue;
    }

    const int tag_id = i / 4;
    const int corner_idx = i % 4;
    const double kp_x = grid->data[i][1];
    const double kp_y = grid->data[i][2];
    const double p_x = grid->data[i][3];
    const double p_y = grid->data[i][4];
    const double p_z = grid->data[i][5];

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

      if (grid->data[i][0] > 0) { // Corner detected?
        fprintf(fp, "%d,", tag_id);
        fprintf(fp, "%d,", corner_idx);
        fprintf(fp, "%f,", grid->data[i][1]);
        fprintf(fp, "%f,", grid->data[i][2]);
        fprintf(fp, "%f,", grid->data[i][3]);
        fprintf(fp, "%f,", grid->data[i][4]);
        fprintf(fp, "%f", grid->data[i][5]);
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
  } else if (strcmp(value_type, "double") == 0) {
    *(double *) value = atof(value_str);
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
int aprilgrid_load(aprilgrid_t *grid, const char *data_path) {
  assert(grid != NULL);
  assert(data_path != NULL);

  // Open file for loading
  FILE *fp = fopen(data_path, "r");
  if (fp == NULL) {
    APRILGRID_LOG("Failed to open [%s]!\n", data_path);
    return -1;
  }

  // Parse configuration
  timestamp_t timestamp;
  int num_rows = 0;
  int num_cols = 0;
  double tag_size = 0;
  double tag_spacing = 0;
  aprilgrid_parse_line(fp, "timestamp", "uint64_t", &timestamp);
  aprilgrid_parse_line(fp, "num_rows", "int", &num_rows);
  aprilgrid_parse_line(fp, "num_cols", "int", &num_cols);
  aprilgrid_parse_line(fp, "tag_size", "double", &tag_size);
  aprilgrid_parse_line(fp, "tag_spacing", "double", &tag_spacing);
  aprilgrid_parse_skip_line(fp);
  aprilgrid_setup(timestamp, grid);
  grid->num_rows = num_rows;
  grid->num_cols = num_cols;
  grid->tag_size = tag_size;
  grid->tag_spacing = tag_spacing;

  // Parse data
  int corners_detected = 0;
  aprilgrid_parse_line(fp, "corners_detected", "int", &corners_detected);
  aprilgrid_parse_skip_line(fp);

  const char *scan_format = "%d,%d,%lf,%lf,%lf,%lf,%lf";
  for (int i = 0; i < corners_detected; i++) {
    // Parse data line
    int tag_id = 0;
    int corner_idx = 0;
    double kp[2] = {0};
    double p[3] = {0};
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

  return 0;
}

// APRILGRID DETECTOR ////////////////////////////////////////////////////////

aprilgrid_detector_t *aprilgrid_detector_malloc() {
  aprilgrid_detector_t *det = malloc(sizeof(aprilgrid_detector_t));
  det->tf = tagStandard41h12_create();
  det->td = apriltag_detector_create();
  apriltag_detector_add_family_bits(det->td, det->tf, 1);
  return det;
}

void aprilgrid_detector_free(aprilgrid_detector_t *det) {
  assert(det != NULL);
  apriltag_detector_destroy(det->td);
  tagStandard41h12_destroy(det->tf);
  free(det);
  det = NULL;
}

int aprilgrid_detector_detect(const aprilgrid_detector_t *det,
                              aprilgrid_t *grid,
                              const int32_t image_width,
                              const int32_t image_height,
                              const int32_t image_stride,
                              uint8_t *image_data) {
  assert(det != NULL);
  assert(grid != NULL);
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
  zarray_t *dets = apriltag_detector_detect(det->td, &im);
  int num_corners = 0;
  for (int i = 0; i < zarray_size(dets); i++) {
    apriltag_detection_t *det;
    zarray_get(dets, i, &det);
    aprilgrid_add_corner(grid, det->id, 0, det->p[0]);
    aprilgrid_add_corner(grid, det->id, 1, det->p[1]);
    aprilgrid_add_corner(grid, det->id, 2, det->p[2]);
    aprilgrid_add_corner(grid, det->id, 3, det->p[3]);
    num_corners += 4;
  }
  apriltag_detections_destroy(dets);

  return num_corners;
}

#endif // APRILGRID_IMPLEMENTATION

//////////////////////////////////////////////////////////////////////////////
//                                UNITTESTS                                 //
//////////////////////////////////////////////////////////////////////////////

#ifdef APRILGRID_UNITTEST

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

static int fltcmp(const float x, const float y) {
  if (fabs(x - y) < 1e-10) {
    return 0;
  } else if (x > y) {
    return 1;
  }

  return -1;
}

int test_aprilgrid_setup() {
  // Setup
  const timestamp_t ts = 0;
  aprilgrid_t grid;
  aprilgrid_setup(ts, &grid);

  TEST_ASSERT(grid.timestamp == 0);
  TEST_ASSERT(grid.num_rows == APRILGRID_NUM_ROWS);
  TEST_ASSERT(grid.num_cols == APRILGRID_NUM_COLS);
  TEST_ASSERT(fltcmp(grid.tag_size, APRILGRID_TAG_SIZE) == 0);
  TEST_ASSERT(fltcmp(grid.tag_spacing, APRILGRID_TAG_SPACING) == 0);
  TEST_ASSERT(grid.corners_detected == 0);

  return 0;
}

int test_aprilgrid_center() {
  // Setup
  const timestamp_t ts = 0;
  aprilgrid_t grid;
  aprilgrid_setup(ts, &grid);
  grid.num_rows = 5;
  grid.num_cols = 2;
  grid.tag_size = 0.1;
  grid.tag_spacing = 0;

  // Aprilgrid center
  double cx = 0.0;
  double cy = 0.0;
  aprilgrid_center(&grid, &cx, &cy);
  TEST_ASSERT(fltcmp(cx, 0.1) == 0);
  TEST_ASSERT(fltcmp(cy, 0.25) == 0);

  return 0;
}

int test_aprilgrid_grid_index() {
  // Setup
  const timestamp_t ts = 0;
  const int num_rows = 6;
  const int num_cols = 6;
  const double tag_size = 0.088;
  const double tag_spacing = 0.3;
  aprilgrid_t grid;
  aprilgrid_setup(ts, &grid);
  grid.num_rows = num_rows;
  grid.num_cols = num_cols;
  grid.tag_size = tag_size;
  grid.tag_spacing = tag_spacing;

  // Get grid index
  int i = 0;
  int j = 0;
  aprilgrid_grid_index(&grid, 0, &i, &j);
  TEST_ASSERT(i == 0);
  TEST_ASSERT(j == 0);

  aprilgrid_grid_index(&grid, 1, &i, &j);
  TEST_ASSERT(i == 0);
  TEST_ASSERT(j == 1);

  aprilgrid_grid_index(&grid, 5, &i, &j);
  TEST_ASSERT(i == 0);
  TEST_ASSERT(j == 5);

  aprilgrid_grid_index(&grid, 7, &i, &j);
  TEST_ASSERT(i == 1);
  TEST_ASSERT(j == 1);

  aprilgrid_grid_index(&grid, 17, &i, &j);
  TEST_ASSERT(i == 2);
  TEST_ASSERT(j == 5);

  return 0;
}

int test_aprilgrid_object_point() {
  // Setup
  const timestamp_t ts = 0;
  const int num_rows = 6;
  const int num_cols = 6;
  const double tag_size = 0.1;
  const double tag_spacing = 0.0;
  aprilgrid_t grid;
  aprilgrid_setup(ts, &grid);
  grid.num_rows = num_rows;
  grid.num_cols = num_cols;
  grid.tag_size = tag_size;
  grid.tag_spacing = tag_spacing;

  // Get object point
  double p[3] = {0};
  aprilgrid_object_point(&grid, 1, 0, p);
  TEST_ASSERT(fltcmp(p[0], tag_size) == 0);
  TEST_ASSERT(fltcmp(p[1], 0) == 0);
  TEST_ASSERT(fltcmp(p[2], 0) == 0);

  aprilgrid_object_point(&grid, 1, 1, p);
  TEST_ASSERT(fltcmp(p[0], tag_size * 2) == 0);
  TEST_ASSERT(fltcmp(p[1], 0) == 0);
  TEST_ASSERT(fltcmp(p[2], 0) == 0);

  aprilgrid_object_point(&grid, 1, 2, p);
  TEST_ASSERT(fltcmp(p[0], tag_size * 2) == 0);
  TEST_ASSERT(fltcmp(p[1], tag_size) == 0);
  TEST_ASSERT(fltcmp(p[2], 0) == 0);

  aprilgrid_object_point(&grid, 1, 3, p);
  TEST_ASSERT(fltcmp(p[0], tag_size) == 0);
  TEST_ASSERT(fltcmp(p[1], tag_size) == 0);
  TEST_ASSERT(fltcmp(p[2], 0) == 0);

  return 0;
}

int test_aprilgrid_add_and_remove_corner() {
  // Setup
  const timestamp_t ts = 0;
  const int num_rows = 6;
  const int num_cols = 6;
  const double tag_size = 0.1;
  const double tag_spacing = 0.0;
  aprilgrid_t grid;
  aprilgrid_setup(ts, &grid);
  grid.num_rows = num_rows;
  grid.num_cols = num_cols;
  grid.tag_size = tag_size;
  grid.tag_spacing = tag_spacing;

  // Add corner
  const int tag_id = 5;
  const int corner_idx = 0;
  const double kp[2] = {1.0, 2.0};
  aprilgrid_add_corner(&grid, tag_id, corner_idx, kp);

  const int data_row = (tag_id * 4) + corner_idx;
  TEST_ASSERT(grid.corners_detected == 1);
  TEST_ASSERT(grid.data[data_row][0] == 1);
  TEST_ASSERT(fltcmp(grid.data[data_row][1], kp[0]) == 0);
  TEST_ASSERT(fltcmp(grid.data[data_row][2], kp[1]) == 0);

  // Remove corner
  aprilgrid_remove_corner(&grid, tag_id, corner_idx);

  TEST_ASSERT(grid.corners_detected == 0);
  TEST_ASSERT(grid.data[data_row][0] == 0);
  TEST_ASSERT(fltcmp(grid.data[data_row][1], 0.0) == 0);
  TEST_ASSERT(fltcmp(grid.data[data_row][2], 0.0) == 0);

  return 0;
}

int test_aprilgrid_add_and_remove_tag() {
  // Setup
  const timestamp_t ts = 0;
  const int num_rows = 6;
  const int num_cols = 6;
  const double tag_size = 0.1;
  const double tag_spacing = 0.0;
  aprilgrid_t grid;
  aprilgrid_setup(ts, &grid);
  grid.num_rows = num_rows;
  grid.num_cols = num_cols;
  grid.tag_size = tag_size;
  grid.tag_spacing = tag_spacing;

  // Add tag
  const int tag_id = 5;
  const double tag_kps[4][2] = {{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}, {7.0, 8.0}};
  aprilgrid_add_tag(&grid, tag_id, tag_kps);

  TEST_ASSERT(grid.corners_detected == 4);
  for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
    const int data_row = (tag_id * 4) + corner_idx;
    TEST_ASSERT(grid.data[data_row][0] == 1);
    TEST_ASSERT(fltcmp(grid.data[data_row][1], tag_kps[corner_idx][0]) == 0);
    TEST_ASSERT(fltcmp(grid.data[data_row][2], tag_kps[corner_idx][1]) == 0);
  }

  // Remove tag
  aprilgrid_remove_tag(&grid, tag_id);

  TEST_ASSERT(grid.corners_detected == 0);
  for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
    const int data_row = (tag_id * 4) + corner_idx;
    TEST_ASSERT(grid.data[data_row][0] == 0);
    TEST_ASSERT(fltcmp(grid.data[data_row][1], 0.0) == 0);
    TEST_ASSERT(fltcmp(grid.data[data_row][2], 0.0) == 0);
  }

  return 0;
}

int test_aprilgrid_save_and_load() {
  // Setup
  const timestamp_t ts = 0;
  aprilgrid_t grid;
  aprilgrid_setup(ts, &grid);

  // Add tag
  const int tag_id = 5;
  const double tag_kps[4][2] = {{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}, {7.0, 8.0}};
  aprilgrid_add_tag(&grid, tag_id, tag_kps);

  TEST_ASSERT(grid.corners_detected == 4);
  for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
    const int data_row = (tag_id * 4) + corner_idx;
    TEST_ASSERT(grid.data[data_row][0] == 1);
    TEST_ASSERT(fltcmp(grid.data[data_row][1], tag_kps[corner_idx][0]) == 0);
    TEST_ASSERT(fltcmp(grid.data[data_row][2], tag_kps[corner_idx][1]) == 0);
  }

  // Save
  const int retval = aprilgrid_save(&grid, "/tmp/test_aprilgrid.dat");
  TEST_ASSERT(retval == 0);

  // Load
  aprilgrid_t grid_load;
  aprilgrid_load(&grid_load, "/tmp/test_aprilgrid.dat");
  TEST_ASSERT(grid_load.num_rows == grid.num_rows);
  TEST_ASSERT(grid_load.num_cols == grid.num_cols);
  TEST_ASSERT(fltcmp(grid_load.tag_size, grid.tag_size) == 0);
  TEST_ASSERT(fltcmp(grid_load.tag_spacing, grid.tag_spacing) == 0);
  TEST_ASSERT(grid_load.corners_detected == grid.corners_detected);
  for (int i = 0; i < APRILGRID_MAX_CORNERS; i++) {
    for (int j = 0; j < 6; j++) {
      TEST_ASSERT(fltcmp(grid_load.data[i][j], grid.data[i][j]) == 0);
    }
  }
  // aprilgrid_print(&grid_load);

  return 0;
}

int test_aprilgrid_detector_detect() {
  // Load test image
  // -- Load JPG
  const char *test_image = "./test_data/images/aprilgrid.jpg";
  int err = 0;
  pjpeg_t *pjpeg = pjpeg_create_from_file(test_image, 0, &err);
  if (pjpeg == NULL) {
    printf("Failed to load [%s]\n", test_image);
    return -1;
  }
  // -- Convert to single channel 8-bit image
  image_u8_t *im = pjpeg_to_u8_baseline(pjpeg);

  // Detect
  const timestamp_t ts = 0;
  const int num_rows = 5;
  const int num_cols = 5;
  const double tag_size = 1.0;
  const double tag_spacing = 0.0;
  aprilgrid_t grid;
  aprilgrid_setup(ts, &grid);
  grid.num_rows = num_rows;
  grid.num_cols = num_cols;
  grid.tag_size = tag_size;
  grid.tag_spacing = tag_spacing;

  aprilgrid_detector_t *det = aprilgrid_detector_malloc();
  int num_corners = aprilgrid_detector_detect(det,
                                              &grid,
                                              im->width,
                                              im->height,
                                              im->stride,
                                              im->buf);
  TEST_ASSERT(num_corners == 100);
  TEST_ASSERT(grid.timestamp == 0);
  TEST_ASSERT(grid.num_rows == num_rows);
  TEST_ASSERT(grid.num_cols == num_cols);
  TEST_ASSERT(fltcmp(grid.tag_size, tag_size) == 0);
  TEST_ASSERT(fltcmp(grid.tag_spacing, tag_spacing) == 0);
  TEST_ASSERT(grid.corners_detected == num_corners);

  // Clean up
  pjpeg_destroy(pjpeg);
  image_u8_destroy(im);
  aprilgrid_detector_free(det);

  return 0;
}

int main(int argc, char *argv[]) {
  TEST(test_aprilgrid_setup);
  TEST(test_aprilgrid_center);
  TEST(test_aprilgrid_grid_index);
  TEST(test_aprilgrid_object_point);
  TEST(test_aprilgrid_add_and_remove_corner);
  TEST(test_aprilgrid_add_and_remove_tag);
  TEST(test_aprilgrid_save_and_load);
  TEST(test_aprilgrid_detector_detect);

  return (nb_failed) ? -1 : 0;
}

#endif // APRILGRID_UNITTEST
