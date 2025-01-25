#pragma once

#include <math.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-prototypes"
#include "apriltag/apriltag.h"
#include "apriltag/common/image_u8.h"
#include "apriltag/common/pjpeg.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tagStandard41h12.h"
#pragma GCC diagnostic pop

#include "macros.h"
#include "logging.h"
#include "data.h"
#include "time.h"

// APRILGRID /////////////////////////////////////////////////////////////////

typedef struct aprilgrid_t {
  // Grid properties
  int num_rows;
  int num_cols;
  real_t tag_size;
  real_t tag_spacing;

  // Grid data
  timestamp_t timestamp;
  int corners_detected;
  real_t *data;
} aprilgrid_t;

aprilgrid_t *aprilgrid_malloc(const int num_rows,
                              const int num_cols,
                              const real_t tag_size,
                              const real_t tag_spacing);
void aprilgrid_free(aprilgrid_t *grid);
void aprilgrid_clear(aprilgrid_t *grid);
void aprilgrid_reset(aprilgrid_t *grid);
void aprilgrid_copy(const aprilgrid_t *src, aprilgrid_t *dst);
int aprilgrid_equals(const aprilgrid_t *grid0, const aprilgrid_t *grid1);
void aprilgrid_center(const aprilgrid_t *grid, real_t *cx, real_t *cy);
void aprilgrid_grid_index(const aprilgrid_t *grid,
                          const int tag_id,
                          int *i,
                          int *j);
void aprilgrid_object_point(const aprilgrid_t *grid,
                            const int tag_id,
                            const int corner_idx,
                            real_t object_point[3]);
void aprilgrid_add_corner(aprilgrid_t *grid,
                          const int tag_id,
                          const int corner_idx,
                          const real_t kp[2]);
void aprilgrid_remove_corner(aprilgrid_t *grid,
                             const int tag_id,
                             const int corner_idx);
void aprilgrid_add_tag(aprilgrid_t *grid,
                       const int tag_id,
                       const real_t kp[4][2]);
void aprilgrid_remove_tag(aprilgrid_t *grid, const int tag_id);
void aprilgrid_measurements(const aprilgrid_t *grid,
                            int *tag_ids,
                            int *corner_idxs,
                            real_t *tag_kps,
                            real_t *obj_pts);
int aprilgrid_save(const aprilgrid_t *grid, const char *save_path);
aprilgrid_t *aprilgrid_load(const char *data_path);

// APRILGRID DETECTOR ////////////////////////////////////////////////////////

typedef struct aprilgrid_detector_t {
  apriltag_family_t *tf;
  apriltag_detector_t *td;

  int num_rows;
  int num_cols;
  real_t tag_size;
  real_t tag_spacing;
} aprilgrid_detector_t;

aprilgrid_detector_t *aprilgrid_detector_malloc(int num_rows,
                                                int num_cols,
                                                real_t tag_size,
                                                real_t tag_spacing);
void aprilgrid_detector_free(aprilgrid_detector_t *det);
aprilgrid_t *aprilgrid_detector_detect(const aprilgrid_detector_t *det,
                                       const timestamp_t ts,
                                       const int32_t image_width,
                                       const int32_t image_height,
                                       const int32_t image_stride,
                                       uint8_t *image_data);
