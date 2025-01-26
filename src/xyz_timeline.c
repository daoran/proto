#include "xyz_timeline.h"

//////////////
// TIMELINE //
//////////////

/**
 * Sort timestamps `timestamps` of length `n` with insertion sort.
 */
static void timestamps_insertion_sort(timestamp_t *timestamps, const size_t n) {
  for (size_t i = 1; i < n; i++) {
    timestamp_t key = timestamps[i];
    size_t j = i - 1;

    while (j >= 0 && timestamps[j] > key) {
      timestamps[j + 1] = timestamps[j];
      j = j - 1;
    }
    timestamps[j + 1] = key;
  }
}

/**
 * This function only adds unique timestamps to `set` if it does not already exists.
 */
static void timestamps_unique(timestamp_t *set,
                              size_t *set_len,
                              const timestamp_t *timestamps,
                              const size_t num_timestamps) {
  for (size_t i = 0; i < num_timestamps; i++) {
    const timestamp_t ts_k = timestamps[i];

    // Check duplicate in set
    int dup = 0;
    for (size_t j = 0; j < *set_len; j++) {
      if (set[j] == ts_k) {
        dup = 1;
        break;
      }
    }

    // Add to set if no duplicate
    if (dup == 0) {
      set[*set_len] = ts_k;
      (*set_len)++;
    }
  }

  // Sort timestamps (just to be sure)
  timestamps_insertion_sort(set, *set_len);
}

/**
 * Print camera event.
 */
void print_camera_event(const camera_event_t *event) {
  printf("camera_event:\n");
  printf("  ts: %ld\n", event->ts);
  printf("  cam_idx: %d\n", event->cam_idx);
  if (event->image_path) {
    printf("  image_path: %s\n", event->image_path);
  }
  printf("\n");
  printf("  num_features: %d\n", event->num_features);
  printf("  features: [\n");
  for (size_t i = 0; i < event->num_features; i++) {
    const size_t feature_id = event->feature_ids[i];
    const real_t *kps = &event->keypoints[i * 2 + 0];
    printf("    %zu: [%.2f, %.2f]\n", feature_id, kps[0], kps[1]);
  }
  printf("  ]\n");
}

/**
 * Print IMU event.
 */
void print_imu_event(const imu_event_t *event) {
  printf("imu_event:\n");
  printf("  ts: %ld\n", event->ts);
  printf("  acc: [%.4f, %.4f, %.4f]\n",
         event->acc[0],
         event->acc[1],
         event->acc[2]);
  printf("  gyr: [%.4f, %.4f, %.4f]\n",
         event->gyr[0],
         event->gyr[1],
         event->gyr[2]);
  printf("\n");
}

/**
 * Print Fiducial event.
 */
void print_fiducial_event(const fiducial_event_t *event) {
  printf("fiducial_event:\n");
  printf("  ts: %ld\n", event->ts);
  printf("  cam_idx: %d\n", event->cam_idx);
  printf("  num_corners: %d\n", event->num_corners);
  printf("\n");
  printf("  #tag_id, corner_idx, kp_x, kp_y, p_x, p_y, p_z\n");
  for (int i = 0; i < event->num_corners; i++) {
    const int tag_id = event->tag_ids[i];
    const int corner_idx = event->corner_indices[i];
    printf("  ");
    printf("%d, ", tag_id);
    printf("%d, ", corner_idx);
    printf("%.2f, ", event->keypoints[i * 2 + 0]);
    printf("%.2f, ", event->keypoints[i * 2 + 1]);
    printf("%.2f, ", event->object_points[i * 3 + 0]);
    printf("%.2f, ", event->object_points[i * 3 + 1]);
    printf("%.2f", event->object_points[i * 3 + 2]);
    printf("\n");
  }
  printf("\n");
}

/**
 * Malloc timeline.
 */
timeline_t *timeline_malloc(void) {
  timeline_t *timeline = MALLOC(timeline_t, 1);

  // Stats
  timeline->num_cams = 0;
  timeline->num_imus = 0;
  timeline->num_event_types = 0;

  // Events
  timeline->events = NULL;
  timeline->events_timestamps = NULL;
  timeline->events_lengths = NULL;
  timeline->events_types = NULL;

  // Timeline
  timeline->timeline_length = 0;
  timeline->timeline_timestamps = 0;
  timeline->timeline_events = 0;
  timeline->timeline_events_lengths = 0;

  return timeline;
}

/**
 * Free timeline.
 */
void timeline_free(timeline_t *timeline) {
  // Pre-check
  if (timeline == NULL) {
    return;
  }

  // Free events
  for (size_t type_idx = 0; type_idx < timeline->num_event_types; type_idx++) {
    for (int k = 0; k < timeline->events_lengths[type_idx]; k++) {
      timeline_event_t *event = &timeline->events[type_idx][k];
      if (event == NULL) {
        continue;
      }

      switch (event->type) {
        case CAMERA_EVENT:
          free(event->data.camera.image_path);
          free(event->data.camera.keypoints);
          break;
        case IMU_EVENT:
          // Do nothing
          break;
        case FIDUCIAL_EVENT:
          free(event->data.fiducial.tag_ids);
          free(event->data.fiducial.corner_indices);
          free(event->data.fiducial.object_points);
          free(event->data.fiducial.keypoints);
          break;
      }
    }
    free(timeline->events[type_idx]);
    free(timeline->events_timestamps[type_idx]);
  }
  free(timeline->events);
  free(timeline->events_timestamps);
  free(timeline->events_lengths);
  free(timeline->events_types);

  // Free timeline
  free(timeline->timeline_timestamps);
  for (int k = 0; k < timeline->timeline_length; k++) {
    free(timeline->timeline_events[k]);
  }
  free(timeline->timeline_events);
  free(timeline->timeline_events_lengths);

  // Free timeline
  free(timeline);
}

// /**
//  * Load timeline fiducial data.
//  */
// timeline_event_t *timeline_load_fiducial(const char *data_dir,
//                                          const int cam_idx,
//                                          int *num_events) {
//   // Load fiducial files
//   *num_events = 0;
//   char **files = list_files(data_dir, num_events);
//
//   // Exit if no data
//   if (*num_events == 0) {
//     for (int view_idx = 0; view_idx < *num_events; view_idx++) {
//       free(files[view_idx]);
//     }
//     free(files);
//     return NULL;
//   }
//
//   // Load fiducial events
//   timeline_event_t *events = MALLOC(timeline_event_t, *num_events);
//
//   for (int view_idx = 0; view_idx < *num_events; view_idx++) {
//     // Load aprilgrid
//     aprilgrid_t *grid = aprilgrid_load(files[view_idx]);
//
//     // Get aprilgrid measurements
//     const timestamp_t ts = grid->timestamp;
//     const int num_corners = grid->corners_detected;
//     int *tag_ids = MALLOC(int, num_corners);
//     int *corner_indices = MALLOC(int, num_corners);
//     real_t *kps = MALLOC(real_t, num_corners * 2);
//     real_t *pts = MALLOC(real_t, num_corners * 3);
//     aprilgrid_measurements(grid, tag_ids, corner_indices, kps, pts);
//
//     // Create event
//     events[view_idx].type = FIDUCIAL_EVENT;
//     events[view_idx].ts = ts;
//     events[view_idx].data.fiducial.ts = ts;
//     events[view_idx].data.fiducial.cam_idx = cam_idx;
//     events[view_idx].data.fiducial.num_corners = num_corners;
//     events[view_idx].data.fiducial.corner_indices = corner_indices;
//     events[view_idx].data.fiducial.tag_ids = tag_ids;
//     events[view_idx].data.fiducial.object_points = pts;
//     events[view_idx].data.fiducial.keypoints = kps;
//
//     // Clean up
//     free(files[view_idx]);
//     aprilgrid_free(grid);
//   }
//   free(files);
//
//   return events;
// }

/**
 * Load timeline IMU data.
 */
timeline_event_t *timeline_load_imu(const char *csv_path, int *num_events) {
  // Open file for loading
  const int num_rows = file_rows(csv_path);
  FILE *fp = fopen(csv_path, "r");
  if (fp == NULL) {
    FATAL("Failed to open [%s]!\n", csv_path);
  }
  skip_line(fp);

  // Malloc
  assert(num_rows > 0);
  *num_events = num_rows - 1;
  timeline_event_t *events = MALLOC(timeline_event_t, *num_events);

  // Parse file
  for (size_t k = 0; k < *num_events; k++) {
    // Parse line
    timestamp_t ts = 0;
    double w[3] = {0};
    double a[3] = {0};
    int retval = fscanf(fp,
                        "%" SCNd64 ",%lf,%lf,%lf,%lf,%lf,%lf",
                        &ts,
                        &w[0],
                        &w[1],
                        &w[2],
                        &a[0],
                        &a[1],
                        &a[2]);
    if (retval != 7) {
      FATAL("Failed to parse line in [%s]\n", csv_path);
    }

    // Add data
    events[k].type = IMU_EVENT;
    events[k].ts = ts;
    events[k].data.imu.ts = ts;
    events[k].data.imu.acc[0] = a[0];
    events[k].data.imu.acc[1] = a[1];
    events[k].data.imu.acc[2] = a[2];
    events[k].data.imu.gyr[0] = w[0];
    events[k].data.imu.gyr[1] = w[1];
    events[k].data.imu.gyr[2] = w[2];
  }
  fclose(fp);

  return events;
}

/**
 * Load events.
 */
static void timeline_load_events(timeline_t *timeline, const char *data_dir) {
  // Load events
  const int num_event_types = timeline->num_event_types;
  timeline_event_t **events = MALLOC(timeline_event_t *, num_event_types);
  int *events_lengths = CALLOC(int, num_event_types);
  int *events_types = CALLOC(int, num_event_types);
  timestamp_t **events_timestamps = MALLOC(timestamp_t *, num_event_types);
  int type_idx = 0;

  // -- Load fiducial events
  // for (int cam_idx = 0; cam_idx < timeline->num_cams; cam_idx++) {
  //   // Form events
  //   int num_events = 0;
  //   char dir[1024] = {0};
  //   sprintf(dir, "%s/cam%d", data_dir, cam_idx);
  //   events[type_idx] = timeline_load_fiducial(dir, cam_idx, &num_events);
  //   events_lengths[type_idx] = num_events;
  //   events_types[type_idx] = FIDUCIAL_EVENT;
  //
  //   // Form timestamps
  //   events_timestamps[type_idx] = CALLOC(timestamp_t, num_events);
  //   for (int k = 0; k < num_events; k++) {
  //     events_timestamps[type_idx][k] = events[type_idx][k].ts;
  //   }
  //
  //   // Update
  //   type_idx++;
  // }

  // -- Load imu events
  for (int imu_idx = 0; imu_idx < timeline->num_imus; imu_idx++) {
    // Form events
    int num_events = 0;
    char csv_path[1024] = {0};
    sprintf(csv_path, "%s/imu%d/data.csv", data_dir, imu_idx);
    events[type_idx] = timeline_load_imu(csv_path, &num_events);
    events_lengths[type_idx] = num_events;
    events_types[type_idx] = IMU_EVENT;

    // Form timestamps
    events_timestamps[type_idx] = CALLOC(timestamp_t, num_events);
    for (int k = 0; k < num_events; k++) {
      events_timestamps[type_idx][k] = events[type_idx][k].ts;
    }

    // Update
    type_idx++;
  }

  // Set timeline events
  timeline->events = events;
  timeline->events_timestamps = events_timestamps;
  timeline->events_lengths = events_lengths;
  timeline->events_types = events_types;
}

/**
 * Form timeline.
 */
void timeline_form_timeline(timeline_t *tl) {
  // Determine timeline timestamps
  int max_timeline_length = 0;
  for (int type_idx = 0; type_idx < tl->num_event_types; type_idx++) {
    max_timeline_length += tl->events_lengths[type_idx];
  }

  tl->timeline_length = 0;
  tl->timeline_timestamps = CALLOC(timestamp_t, max_timeline_length);
  for (int type_idx = 0; type_idx < tl->num_event_types; type_idx++) {
    timestamps_unique(tl->timeline_timestamps,
                      &tl->timeline_length,
                      tl->events_timestamps[type_idx],
                      tl->events_lengths[type_idx]);
  }

  // Form timeline events
  tl->timeline_events = CALLOC(timeline_event_t **, tl->timeline_length);
  tl->timeline_events_lengths = CALLOC(int, tl->timeline_length);

  int *indices = CALLOC(int, tl->num_event_types);
  for (int k = 0; k < tl->timeline_length; k++) {
    // Allocate memory
    tl->timeline_events[k] = CALLOC(timeline_event_t *, tl->num_event_types);

    // Add events at k
    int k_len = 0; // Number of events at k
    for (int type_idx = 0; type_idx < tl->num_event_types; type_idx++) {
      // Find timestamp index
      int ts_found = 0;
      int ts_idx = 0;
      for (int i = indices[type_idx]; i < tl->events_lengths[type_idx]; i++) {
        timeline_event_t *event = &tl->events[type_idx][i];
        if (event->ts == tl->timeline_timestamps[k]) {
          indices[type_idx] = i;
          ts_found = 1;
          ts_idx = i;
          break;
        } else if (event->ts > tl->timeline_timestamps[k]) {
          break;
        }
      }

      // Add event to timeline
      if (ts_found) {
        tl->timeline_events[k][k_len] = &tl->events[type_idx][ts_idx];
        k_len++;
      }
    }

    // Set number of events at timestep k
    tl->timeline_events_lengths[k] = k_len;
  }

  // Clean-up
  free(indices);
}

/**
 * Load timeline
 */
timeline_t *timeline_load_data(const char *data_dir,
                               const int num_cams,
                               const int num_imus) {
  assert(num_cams >= 0);
  assert(num_imus >= 0 && num_imus <= 1);

  // Form timeline
  timeline_t *timeline = timeline_malloc();
  timeline->num_cams = num_cams;
  timeline->num_imus = num_imus;
  timeline->num_event_types = num_cams + num_imus;
  // -- Events
  timeline_load_events(timeline, data_dir);
  // -- Timeline
  timeline_form_timeline(timeline);

  return timeline;
}
