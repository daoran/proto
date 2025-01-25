//////////////////////////////////////////////////////////////////////////////
//                                UNITTESTS                                 //
//////////////////////////////////////////////////////////////////////////////

#ifdef XYZ_APRILGRID_UNITTEST

#include <math.h>
#include <stdio.h>

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
void run_test(const char *test_name, int (*test_ptr)(void)) {
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

int test_aprilgrid_malloc_and_free(void) {
  // Setup
  const int num_rows = 6;
  const int num_cols = 7;
  const real_t tsize = 0.1;
  const real_t tspacing = 0.1;
  aprilgrid_t *g = aprilgrid_malloc(num_rows, num_cols, tsize, tspacing);

  TEST_ASSERT(g->timestamp == 0);
  TEST_ASSERT(g->num_rows == num_rows);
  TEST_ASSERT(g->num_cols == num_cols);
  TEST_ASSERT(fltcmp(g->tag_size, tsize) == 0);
  TEST_ASSERT(fltcmp(g->tag_spacing, tspacing) == 0);
  TEST_ASSERT(g->corners_detected == 0);

  aprilgrid_free(g);

  return 0;
}

int test_aprilgrid_center(void) {
  // Setup
  int num_rows = 5;
  int num_cols = 2;
  real_t tag_size = 0.1;
  real_t tag_spacing = 0;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  // Aprilgrid center
  real_t cx = 0.0;
  real_t cy = 0.0;
  aprilgrid_center(grid, &cx, &cy);
  TEST_ASSERT(fltcmp(cx, 0.1) == 0);
  TEST_ASSERT(fltcmp(cy, 0.25) == 0);

  // Clean up
  aprilgrid_free(grid);

  return 0;
}

int test_aprilgrid_grid_index(void) {
  // Setup
  const int num_rows = 6;
  const int num_cols = 6;
  const real_t tag_size = 0.088;
  const real_t tag_spacing = 0.3;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  // Get grid index
  int i = 0;
  int j = 0;
  aprilgrid_grid_index(grid, 0, &i, &j);
  TEST_ASSERT(i == 0);
  TEST_ASSERT(j == 0);

  aprilgrid_grid_index(grid, 1, &i, &j);
  TEST_ASSERT(i == 0);
  TEST_ASSERT(j == 1);

  aprilgrid_grid_index(grid, 5, &i, &j);
  TEST_ASSERT(i == 0);
  TEST_ASSERT(j == 5);

  aprilgrid_grid_index(grid, 7, &i, &j);
  TEST_ASSERT(i == 1);
  TEST_ASSERT(j == 1);

  aprilgrid_grid_index(grid, 17, &i, &j);
  TEST_ASSERT(i == 2);
  TEST_ASSERT(j == 5);

  // Clean up
  aprilgrid_free(grid);

  return 0;
}

int test_aprilgrid_object_point(void) {
  // Setup
  const int num_rows = 6;
  const int num_cols = 6;
  const real_t tag_size = 0.1;
  const real_t tag_spacing = 0.0;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  // Get object point
  real_t p[3] = {0};
  aprilgrid_object_point(grid, 1, 0, p);
  TEST_ASSERT(fltcmp(p[0], tag_size) == 0);
  TEST_ASSERT(fltcmp(p[1], 0) == 0);
  TEST_ASSERT(fltcmp(p[2], 0) == 0);

  aprilgrid_object_point(grid, 1, 1, p);
  TEST_ASSERT(fltcmp(p[0], tag_size * 2) == 0);
  TEST_ASSERT(fltcmp(p[1], 0) == 0);
  TEST_ASSERT(fltcmp(p[2], 0) == 0);

  aprilgrid_object_point(grid, 1, 2, p);
  TEST_ASSERT(fltcmp(p[0], tag_size * 2) == 0);
  TEST_ASSERT(fltcmp(p[1], tag_size) == 0);
  TEST_ASSERT(fltcmp(p[2], 0) == 0);

  aprilgrid_object_point(grid, 1, 3, p);
  TEST_ASSERT(fltcmp(p[0], tag_size) == 0);
  TEST_ASSERT(fltcmp(p[1], tag_size) == 0);
  TEST_ASSERT(fltcmp(p[2], 0) == 0);

  // Clean up
  aprilgrid_free(grid);

  return 0;
}

int test_aprilgrid_add_and_remove_corner(void) {
  // Setup
  const int num_rows = 6;
  const int num_cols = 6;
  const real_t tag_size = 0.1;
  const real_t tag_spacing = 0.0;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  // Add corner
  const int tag_id = 5;
  const int corner_idx = 0;
  const real_t kp[2] = {1.0, 2.0};
  aprilgrid_add_corner(grid, tag_id, corner_idx, kp);

  const int data_row = (tag_id * 4) + corner_idx;
  TEST_ASSERT(grid->corners_detected == 1);
  TEST_ASSERT(grid->data[data_row * 6 + 0] == 1);
  TEST_ASSERT(fltcmp(grid->data[data_row * 6 + 1], kp[0]) == 0);
  TEST_ASSERT(fltcmp(grid->data[data_row * 6 + 2], kp[1]) == 0);

  // Remove corner
  aprilgrid_remove_corner(grid, tag_id, corner_idx);

  TEST_ASSERT(grid->corners_detected == 0);
  TEST_ASSERT(grid->data[data_row * 6 + 0] == 0);
  TEST_ASSERT(fltcmp(grid->data[data_row * 6 + 1], 0.0) == 0);
  TEST_ASSERT(fltcmp(grid->data[data_row * 6 + 2], 0.0) == 0);

  // Clean up
  aprilgrid_free(grid);

  return 0;
}

int test_aprilgrid_add_and_remove_tag(void) {
  // Setup
  const int num_rows = 6;
  const int num_cols = 6;
  const real_t tag_size = 0.1;
  const real_t tag_spacing = 0.2;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  // Add tag
  const int tag_id = 5;
  const real_t tag_kps[4][2] = {{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}, {7.0, 8.0}};
  aprilgrid_add_tag(grid, tag_id, tag_kps);

  TEST_ASSERT(grid->corners_detected == 4);
  for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
    const int data_row = (tag_id * 4) + corner_idx;
    TEST_ASSERT(grid->data[data_row * 6 + 0] == 1);
    TEST_ASSERT(fltcmp(grid->data[data_row * 6 + 1], tag_kps[corner_idx][0]) ==
                0);
    TEST_ASSERT(fltcmp(grid->data[data_row * 6 + 2], tag_kps[corner_idx][1]) ==
                0);
  }

  // Remove tag
  aprilgrid_remove_tag(grid, tag_id);

  TEST_ASSERT(grid->corners_detected == 0);
  for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
    const int data_row = (tag_id * 4) + corner_idx;
    TEST_ASSERT(grid->data[data_row * 6 + 0] == 0);
    TEST_ASSERT(fltcmp(grid->data[data_row * 6 + 1], 0.0) == 0);
    TEST_ASSERT(fltcmp(grid->data[data_row * 6 + 2], 0.0) == 0);
  }

  // Clean up
  aprilgrid_free(grid);

  return 0;
}

int test_aprilgrid_save_and_load(void) {
  // Setup
  const int num_rows = 6;
  const int num_cols = 6;
  const real_t tag_size = 0.088;
  const real_t tag_spacing = 0.3;
  aprilgrid_t *grid =
      aprilgrid_malloc(num_rows, num_cols, tag_size, tag_spacing);

  // Add tag
  const int tag_id = 5;
  const real_t tag_kps[4][2] = {{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}, {7.0, 8.0}};
  aprilgrid_add_tag(grid, tag_id, tag_kps);

  TEST_ASSERT(grid->corners_detected == 4);
  for (int corner_idx = 0; corner_idx < 4; corner_idx++) {
    const int data_row = (tag_id * 4) + corner_idx;
    TEST_ASSERT(grid->data[data_row * 6 + 0] == 1);
    TEST_ASSERT(fltcmp(grid->data[data_row * 6 + 1], tag_kps[corner_idx][0]) ==
                0);
    TEST_ASSERT(fltcmp(grid->data[data_row * 6 + 2], tag_kps[corner_idx][1]) ==
                0);
  }

  // Save
  const int retval = aprilgrid_save(grid, "/tmp/test_aprilgrid.dat");
  TEST_ASSERT(retval == 0);

  // Load
  aprilgrid_t *grid_load = aprilgrid_load("/tmp/test_aprilgrid.dat");
  TEST_ASSERT(grid_load->num_rows == grid->num_rows);
  TEST_ASSERT(grid_load->num_cols == grid->num_cols);
  TEST_ASSERT(fltcmp(grid_load->tag_size, grid->tag_size) == 0);
  TEST_ASSERT(fltcmp(grid_load->tag_spacing, grid->tag_spacing) == 0);
  TEST_ASSERT(grid_load->corners_detected == grid->corners_detected);
  const int max_corners = (grid->num_rows * grid->num_cols * 4);
  for (int i = 0; i < max_corners; i++) {
    for (int j = 0; j < 6; j++) {
      TEST_ASSERT(fltcmp(grid_load->data[i * 6 + j], grid->data[i * 6 + j]) ==
                  0);
    }
  }
  // aprilgrid_print(grid_load);
  aprilgrid_free(grid_load);
  aprilgrid_free(grid);

  return 0;
}

#if ENABLE_APRILGRID_DETECTOR == 1

int test_aprilgrid_detector_detect(void) {
  // Load test image
  // -- Load JPG
  const char *test_image = "./test_data/images/aprilgrid_tag36h11.jpg";
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
  const int num_rows = 10;
  const int num_cols = 10;
  const real_t tag_size = 1.0;
  const real_t tag_spacing = 0.0;

  aprilgrid_detector_t *det =
      aprilgrid_detector_malloc(num_rows, num_cols, tag_size, tag_spacing);
  aprilgrid_t *grid = aprilgrid_detector_detect(det,
                                                ts,
                                                im->width,
                                                im->height,
                                                im->stride,
                                                im->buf);
  TEST_ASSERT(grid->corners_detected == 400);
  TEST_ASSERT(grid->timestamp == 0);
  TEST_ASSERT(grid->num_rows == num_rows);
  TEST_ASSERT(grid->num_cols == num_cols);
  TEST_ASSERT(fltcmp(grid->tag_size, tag_size) == 0);
  TEST_ASSERT(fltcmp(grid->tag_spacing, tag_spacing) == 0);

  // Clean up
  pjpeg_destroy(pjpeg);
  image_u8_destroy(im);
  aprilgrid_detector_free(det);
  aprilgrid_free(grid);

  return 0;
}

#endif // ENABLE_APRILGRID_DETECTOR

int main(int argc, char *argv[]) {
  TEST(test_aprilgrid_malloc_and_free);
  TEST(test_aprilgrid_center);
  TEST(test_aprilgrid_grid_index);
  TEST(test_aprilgrid_object_point);
  TEST(test_aprilgrid_add_and_remove_corner);
  TEST(test_aprilgrid_add_and_remove_tag);
  TEST(test_aprilgrid_save_and_load);
#if ENABLE_APRILGRID_DETECTOR == 1
  TEST(test_aprilgrid_detector_detect);
#endif

  return (nb_failed) ? -1 : 0;
}

#endif // XYZ_APRILGRID_UNITTEST
