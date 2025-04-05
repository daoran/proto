#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <time.h>
#include <inttypes.h>

// Settings
#ifndef PRECISION
#define PRECISION 2
#endif

// Float Precision
#ifndef REAL_TYPE
#define REAL_TYPE
#if PRECISION == 1
typedef float real_t;
#elif PRECISION == 2
typedef double real_t;
#else
#error "Floating Point Precision not defined!"
#endif
#endif

// Timestamp Type
#ifndef TIMESTAMP_TYPE
#define TIMESTAMP_TYPE
typedef int64_t timestamp_t;
#endif

/*****************************************************************************
 * kitti_camera_t
 ****************************************************************************/

typedef struct kitti_camera_t {
  int camera_index;
  int num_timestamps;
  timestamp_t *timestamps;
  char **image_paths;
} kitti_camera_t;

kitti_camera_t *kitti_camera_load(const char *data_dir);
void kitti_camera_free(kitti_camera_t *data);

/*****************************************************************************
 * kitti_oxts_t
 ****************************************************************************/

typedef struct kitti_oxts_t {
  // Timestamps
  int num_timestamps;
  timestamp_t *timestamps;

  // GPS
  double *lat; // Latitude [deg]
  double *lon; // Longitude [deg]
  double *alt; // Altitude [m]

  // Attitude
  double *roll;  // Roll [rad]
  double *pitch; // Pitch [rad]
  double *yaw;   // Yaw [rad]

  // Velocity
  double *vn; // Velocity towards north [m/s]
  double *ve; // Velocity towards east [m/s]
  double *vf; // Forward velocity [m/s]
  double *vl; // Leftward velocity [m/s]
  double *vu; // Upward velocity [m/s]

  // Acceleration
  double *ax; // Acceleration in x [m/s^2]
  double *ay; // Acceleration in y [m/s^2]
  double *az; // Acceleration in z [m/s^2]
  double *af; // Forward acceleration [m/s^2]
  double *al; // Leftward acceleration [m/s^2]
  double *au; // Upward acceleration [m/s^2]

  // Angular velocity
  double *wx; // Angular rate around x [rad/s]
  double *wy; // Angular rate around y [rad/s]
  double *wz; // Angular rate around z [rad/s]
  double *wf; // Angular rate around foward axis [rad/s]
  double *wl; // Angular rate around left axis [rad/s]
  double *wu; // Angular rate around up axis [rad/s]

  // Satellite tracking data
  double *pos_accuracy; // Position accuracy [north / east in m]
  double *vel_accuracy; // Velocity accuracy [north / east in m/s]
  int *navstat;         // Navigation status
  int *numsats;         // Number of satelllites tracked by GPS
  int *posmode;         // Position mode
  int *velmode;         // Velocity mode
  int *orimode;         // Orientation mode
} kitti_oxts_t;

kitti_oxts_t *kitti_oxts_load(const char *data_dir);
void kitti_oxts_free(kitti_oxts_t *data);

/*****************************************************************************
 * kitti_velodyne_t
 ****************************************************************************/

typedef struct kitti_velodyne_t {
  int num_timestamps;
  timestamp_t *timestamps;
  timestamp_t *timestamps_start;
  timestamp_t *timestamps_end;
  char **pcd_paths;
} kitti_velodyne_t;

float *kitti_load_points(const char *pcd_path, size_t *num_points);
kitti_velodyne_t *kitti_velodyne_load(const char *data_dir);
void kitti_velodyne_free(kitti_velodyne_t *data);

/*****************************************************************************
 * kitti_calib_t
 ****************************************************************************/

typedef struct kitti_calib_t {
  char calib_time_cam_to_cam[100];
  char calib_time_imu_to_velo[100];
  char calib_time_velo_to_cam[100];
  double corner_dist;

  double S_00[2];       // Image size [pixels]
  double K_00[9];       // Camera 0 intrinsics
  double D_00[5];       // Camera 0 distortion coefficients
  double R_00[9];       // Rotation from camera 0 to camera 0
  double T_00[3];       // Translation from camera 0 to camera 0
  double S_rect_00[2];  // Image size after rectifcation [pixels]
  double R_rect_00[9];  // Rotation after rectification
  double P_rect_00[12]; // Projection matrix after rectification

  double S_01[2];       // Image size [pixels]
  double K_01[9];       // Camera 1 intrinsics
  double D_01[5];       // Camera 1 distortion coefficients
  double R_01[9];       // Rotation from camera 0 to camera 1
  double T_01[3];       // Translation from camera 0 to camera 1
  double S_rect_01[2];  // Image size after rectifcation [pixels]
  double R_rect_01[9];  // Rotation after rectification
  double P_rect_01[12]; // Projection matrix after rectification

  double S_02[2];       // Image size [pixels]
  double K_02[9];       // Camera 2 intrinsics
  double D_02[5];       // Camera 2 distortion coefficients
  double R_02[9];       // Rotation from camera 0 to camera 2
  double T_02[3];       // Translation from camera 0 to camera 2
  double S_rect_02[2];  // Image size after rectifcation [pixels]
  double R_rect_02[9];  // Rotation after rectification
  double P_rect_02[12]; // Projection matrix after rectification

  double S_03[2];       // Image size [pixels]
  double K_03[9];       // Camera 3 intrinsics
  double D_03[5];       // Camera 3 distortion coefficients
  double R_03[9];       // Rotation from camera 0 to camera 3
  double T_03[3];       // Translation from camera 0 to camera 3
  double S_rect_03[2];  // Image size after rectifcation [pixels]
  double R_rect_03[9];  // Rotation after rectification
  double P_rect_03[12]; // Projection matrix after rectification

  double R_velo_imu[9]; // Rotation from imu to velodyne
  double T_velo_imu[3]; // Translation from imu to velodyne

  double R_cam_velo[9]; // Rotation from velodyne to camera
  double T_cam_velo[3]; // Translation from velodyne to camera
  double delta_f[2];
  double delta_c[2];
} kitti_calib_t;

kitti_calib_t *kitti_calib_load(const char *data_dir);
void kitti_calib_free(kitti_calib_t *data);
void kitti_calib_print(const kitti_calib_t *data);

/*****************************************************************************
 * kitti_raw_t
 ****************************************************************************/

typedef struct kitti_raw_t {
  char seq_name[1024];
  kitti_camera_t *image_00;
  kitti_camera_t *image_01;
  kitti_camera_t *image_02;
  kitti_camera_t *image_03;
  kitti_oxts_t *oxts;
  kitti_velodyne_t *velodyne;
  kitti_calib_t *calib;
} kitti_raw_t;

kitti_raw_t *kitti_raw_load(const char *data_dir, const char *seq_name);
void kitti_raw_free(kitti_raw_t *data);
