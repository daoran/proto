#pragma once

#include "state_estimation.h"
#include "timeline.h"
#include "aprilgrid.h"
#include "gimbal.h"

/** Sim Circle Trajectory **/
typedef struct sim_circle_t {
  real_t imu_rate;
  real_t cam_rate;
  real_t circle_r;
  real_t circle_v;
  real_t theta_init;
  real_t yaw_init;
} sim_circle_t;

void sim_circle_defaults(sim_circle_t *conf);

//////////////////
// SIM FEATURES //
//////////////////

typedef struct sim_features_t {
  real_t **features;
  int num_features;
} sim_features_t;

sim_features_t *sim_features_load(const char *csv_path);
void sim_features_free(sim_features_t *features_data);

//////////////////
// SIM IMU DATA //
//////////////////

typedef struct sim_imu_data_t {
  size_t num_measurements;
  real_t *timestamps;
  real_t *poses;
  real_t *velocities;
  real_t *imu_acc;
  real_t *imu_gyr;
} sim_imu_data_t;

void sim_imu_data_setup(sim_imu_data_t *imu_data);
sim_imu_data_t *sim_imu_data_malloc(void); void sim_imu_data_free(sim_imu_data_t *imu_data);
sim_imu_data_t *sim_imu_data_load(const char *csv_path);
sim_imu_data_t *sim_imu_circle_trajectory(const sim_circle_t *conf);
void sim_imu_measurements(const sim_imu_data_t *data,
                          const int64_t ts_i,
                          const int64_t ts_j,
                          imu_buffer_t *imu_buf);

/////////////////////
// SIM CAMERA DATA //
/////////////////////

/** Simulation Utils **/
void sim_create_features(const real_t origin[3],
                         const real_t dim[3],
                         const int num_features,
                         real_t *features);

/** Sim Camera Frame **/
typedef struct sim_camera_frame_t {
  timestamp_t ts;
  int cam_idx;
  size_t *feature_ids;
  real_t *keypoints;
  int n;
} sim_camera_frame_t;

void sim_camera_frame_setup(sim_camera_frame_t *frame,
                            const timestamp_t ts,
                            const int cam_idx);
sim_camera_frame_t *sim_camera_frame_malloc(const timestamp_t ts,
                                            const int cam_idx);
void sim_camera_frame_free(sim_camera_frame_t *frame_data);
void sim_camera_frame_add_keypoint(sim_camera_frame_t *frame_data,
                                   const size_t feature_id,
                                   const real_t kp[2]);
sim_camera_frame_t *sim_camera_frame_load(const char *csv_path);
void sim_camera_frame_print(const sim_camera_frame_t *frame_data);

/** Sim Camera Data **/
typedef struct sim_camera_data_t {
  int cam_idx;
  sim_camera_frame_t **frames;
  int num_frames;

  timestamp_t *timestamps;
  real_t *poses;
} sim_camera_data_t;

void sim_camera_data_setup(sim_camera_data_t *data);
sim_camera_data_t *sim_camerea_data_malloc(void); void sim_camera_data_free(sim_camera_data_t *cam_data);
sim_camera_data_t *sim_camera_data_load(const char *dir_path);

sim_camera_data_t *
sim_camera_circle_trajectory(const sim_circle_t *conf,
                             const real_t T_BC[4 * 4],
                             const camera_params_t *cam_params,
                             const real_t *features,
                             const int num_features);

/////////////////////////
// SIM CAMERA IMU DATA //
/////////////////////////

/** Sim Circle Camera-IMU Data **/
typedef struct sim_circle_camera_imu_t {
  sim_circle_t conf;
  sim_imu_data_t *imu_data;
  sim_camera_data_t *cam0_data;
  sim_camera_data_t *cam1_data;

  real_t feature_data[3 * 1000];
  int num_features;

  camera_params_t cam0_params;
  camera_params_t cam1_params;
  real_t cam0_ext[7];
  real_t cam1_ext[7];
  real_t imu0_ext[7];

  timeline_t *timeline;
} sim_circle_camera_imu_t;

sim_circle_camera_imu_t *sim_circle_camera_imu(void); void sim_circle_camera_imu_free(sim_circle_camera_imu_t *sim_data);

// /////////////////////
// // SIM GIMBAL DATA //
// /////////////////////
//
// typedef struct sim_gimbal_view_t {
//   int num_measurements;
//   int *tag_ids;
//   int *corner_indices;
//   real_t *object_points;
//   real_t *keypoints;
// } sim_gimbal_view_t;
//
// typedef struct sim_gimbal_t {
//   aprilgrid_t *grid;
//
//   int num_links;
//   int num_joints;
//   int num_cams;
//
//   fiducial_t fiducial_ext;
//   pose_t gimbal_pose;
//   extrinsic_t gimbal_ext;
//   extrinsic_t *gimbal_links;
//   joint_t *gimbal_joints;
//   extrinsic_t *cam_exts;
//   camera_params_t *cam_params;
// } sim_gimbal_t;
//
// sim_gimbal_view_t *sim_gimbal_view_malloc(const int max_corners);
// void sim_gimbal_view_free(sim_gimbal_view_t *view);
// void sim_gimbal_view_print(const sim_gimbal_view_t *view);
//
// sim_gimbal_t *sim_gimbal_malloc(void); void sim_gimbal_free(sim_gimbal_t *sim);
// void sim_gimbal_print(const sim_gimbal_t *sim);
// void sim_gimbal_set_joint(sim_gimbal_t *sim,
//                           const int joint_idx,
//                           const real_t angle);
// void sim_gimbal_get_joints(sim_gimbal_t *sim,
//                            const int num_joints,
//                            real_t *angles);
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
//                                     const real_t cam_ext[7]);
// sim_gimbal_view_t *sim_gimbal_view(const sim_gimbal_t *sim,
//                                    const timestamp_t ts,
//                                    const int view_idx,
//                                    const int cam_idx,
//                                    const real_t body_pose[7]);
