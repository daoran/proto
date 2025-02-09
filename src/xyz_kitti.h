#pragma once

#include "xyz.h"

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

typedef struct {
  float x;
  float y;
  float z;
  float r;
} point_xyzr_t;

typedef struct kitti_velodyne_t {
  int num_timestamps;
  timestamp_t *timestamps;
  timestamp_t *timestamps_start;
  timestamp_t *timestamps_end;
  char **pcd_paths;
} kitti_velodyne_t;

point_xyzr_t *kitti_load_points(const char *pcd_path);
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

  double S_00[2];
  double K_00[9];
  double D_00[5];
  double R_00[9];
  double T_00[3];
  double S_rect_00[2];
  double R_rect_00[9];
  double P_rect_00[12];

  double S_01[2];
  double K_01[9];
  double D_01[5];
  double R_01[9];
  double T_01[3];
  double S_rect_01[2];
  double R_rect_01[9];
  double P_rect_01[12];

  double S_02[2];
  double K_02[9];
  double D_02[5];
  double R_02[9];
  double T_02[3];
  double S_rect_02[2];
  double R_rect_02[9];
  double P_rect_02[12];

  double S_03[2];
  double K_03[9];
  double D_03[5];
  double R_03[9];
  double T_03[3];
  double S_rect_03[2];
  double R_rect_03[9];
  double P_rect_03[12];

  double R_velo_imu[9];
  double T_velo_imu[3];

  double R_cam_velo[9];
  double T_cam_velo[3];
  double delta_f[2];
  double delta_c[2];
} kitti_calib_t;

kitti_calib_t *kitti_calib_load(const char *data_dir);
void kitti_calib_free(kitti_calib_t *data);

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
  kitti_velodyne_t *velodyne_points;
  kitti_calib_t *calib;
} kitti_raw_t;

kitti_raw_t *kitti_raw_load(const char *data_dir, const char *seq_name);
void kitti_raw_free(kitti_raw_t *data);
