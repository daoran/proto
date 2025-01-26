#pragma once

#include <inttypes.h>

#include "xyz.h"


#define CAMERA_EVENT 1
#define IMU_EVENT 2
#define FIDUCIAL_EVENT 3

typedef struct camera_event_t {
  timestamp_t ts;
  int cam_idx;
  char *image_path;

  int num_features;
  size_t *feature_ids;
  real_t *keypoints;
} camera_event_t;

typedef struct imu_event_t {
  timestamp_t ts;
  real_t acc[3];
  real_t gyr[3];
} imu_event_t;

typedef struct fiducial_event_t {
  timestamp_t ts;
  int cam_idx;
  int num_corners;
  int *tag_ids;
  int *corner_indices;
  real_t *object_points;
  real_t *keypoints;
} fiducial_event_t;

union event_data_t {
  camera_event_t camera;
  imu_event_t imu;
  fiducial_event_t fiducial;
};

typedef struct timeline_event_t {
  int type;
  timestamp_t ts;
  union event_data_t data;
} timeline_event_t;

typedef struct timeline_t {
  // Stats
  int num_cams;
  int num_imus;
  int num_event_types;

  // Events
  timeline_event_t **events;
  timestamp_t **events_timestamps;
  int *events_lengths;
  int *events_types;

  // Timeline
  size_t timeline_length;
  timestamp_t *timeline_timestamps;
  timeline_event_t ***timeline_events;
  int *timeline_events_lengths;
} timeline_t;

void print_camera_event(const camera_event_t *event);
void print_imu_event(const imu_event_t *event);
void print_fiducial_event(const fiducial_event_t *event);

timeline_t *timeline_malloc(void); void timeline_free(timeline_t *timeline);
void timeline_form_timeline(timeline_t *tl);
timeline_t *timeline_load_data(const char *data_dir,
                               const int num_cams,
                               const int num_imus);
