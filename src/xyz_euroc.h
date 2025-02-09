#pragma once

#include <inttypes.h>

#include <yaml.h>

#include "xyz.h"

/**
 * Fatal
 *
 * @param[in] M Message
 * @param[in] ... Varadic arguments
 */
#ifndef EUROC_FATAL
#define EUROC_FATAL(...)                                                       \
  do {                                                                         \
    fprintf(stderr,                                                            \
            "[EUROC_FATAL] [%s:%d:%s()]: ",                                    \
            __FILE__,                                                          \
            __LINE__,                                                          \
            __func__);                                                         \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0);                                                                 \
  exit(-1)
#endif

#ifndef EUROC_LOG
#define EUROC_LOG(...)                                                         \
  do {                                                                         \
    fprintf(stderr,                                                            \
            "[EUROC_LOG] [%s:%d:%s()]: ",                                      \
            __FILE__,                                                          \
            __LINE__,                                                          \
            __func__);                                                         \
    fprintf(stderr, __VA_ARGS__);                                              \
  } while (0);
#endif

/*****************************************************************************
 * euroc_imu_t
 ****************************************************************************/

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
  int is_calib_data;
  int num_timestamps;
  timestamp_t *timestamps;
  char **image_paths;

  // Sensor properties
  char sensor_type[100];
  char comment[9046];
  double T_BS[4 * 4];
  double rate_hz;
  int resolution[2];
  char camera_model[100];
  double intrinsics[4];
  char distortion_model[100];
  double distortion_coefficients[4];
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
void euroc_ground_truth_print(const euroc_ground_truth_t *data);

/*****************************************************************************
 * euroc_timeline_t
 ****************************************************************************/

typedef struct euroc_event_t {
  int has_imu0;
  int has_cam0;
  int has_cam1;

  timestamp_t ts;

  size_t imu0_idx;
  double *acc;
  double *gyr;

  size_t cam0_idx;
  char *cam0_image;

  size_t cam1_idx;
  char *cam1_image;
} euroc_event_t;

typedef struct euroc_timeline_t {
  int num_timestamps;
  timestamp_t *timestamps;
  euroc_event_t *events;

} euroc_timeline_t;

euroc_timeline_t *euroc_timeline_create(const euroc_imu_t *imu0_data,
                                        const euroc_camera_t *cam0_data,
                                        const euroc_camera_t *cam1_data);
void euroc_timeline_free(euroc_timeline_t *timeline);

/*****************************************************************************
 * euroc_data_t
 ****************************************************************************/

/**
 * EuRoC data
 */
typedef struct euroc_data_t {
  euroc_imu_t *imu0_data;
  euroc_camera_t *cam0_data;
  euroc_camera_t *cam1_data;
  euroc_ground_truth_t *ground_truth;
  euroc_timeline_t *timeline;
} euroc_data_t;

euroc_data_t *euroc_data_load(const char *data_path);
void euroc_data_free(euroc_data_t *data);

/*****************************************************************************
 * euroc_calib_target_t
 ****************************************************************************/

/**
 * EuRoC calibration target
 */
typedef struct euroc_calib_target_t {
  char type[100];
  int tag_rows;
  int tag_cols;
  double tag_size;
  double tag_spacing;
} euroc_calib_target_t;

euroc_calib_target_t *euroc_calib_target_load(const char *conf);
void euroc_calib_target_free(euroc_calib_target_t *target);
void euroc_calib_target_print(const euroc_calib_target_t *target);

/*****************************************************************************
 * euroc_calib_t
 ****************************************************************************/

/**
 * EuRoC calibration data
 */
typedef struct euroc_calib_t {
  euroc_imu_t *imu0_data;
  euroc_camera_t *cam0_data;
  euroc_camera_t *cam1_data;
  euroc_calib_target_t *calib_target;
  euroc_timeline_t *timeline;
} euroc_calib_t;

euroc_calib_t *euroc_calib_load(const char *data_path);
void euroc_calib_free(euroc_calib_t *data);
