#pragma once

#include "xyz.h"

// ///////////////////////
// // INERTIAL ODOMETRY //
// ///////////////////////
//
// typedef struct inertial_odometry_t {
//   // IMU Parameters
//   imu_params_t imu_params;
//
//   // Factors
//   int num_factors;
//   imu_factor_t *factors;
//   marg_factor_t *marg;
//
//   // Variables
//   pose_t *poses;
//   velocity_t *vels;
//   imu_biases_t *biases;
// } inertial_odometry_t;
//
// inertial_odometry_t *inertial_odometry_malloc(void);
// void inertial_odometry_free(inertial_odometry_t *odom);
// void inertial_odometry_save(const inertial_odometry_t *odom,
//                             const char *save_path);
// param_order_t *inertial_odometry_param_order(const void *data,
//                                              int *sv_size,
//                                              int *r_size);
// void inertial_odometry_cost(const void *data, real_t *r);
// void inertial_odometry_linearize_compact(const void *data,
//                                          const int sv_size,
//                                          param_order_t *hash,
//                                          real_t *H,
//                                          real_t *g,
//                                          real_t *r);
//
// /////////////////////////////
// // RELATIVE POSE ESTIMATOR //
// /////////////////////////////
//
// int relpose_estimator(const int num_cams,
//                       const camera_params_t **cam_params,
//                       const real_t **cam_exts,
//                       const size_t **fids,
//                       const real_t **kps,
//                       const int *num_kps,
//                       const feature_map_t *feature_map,
//                       const real_t T_WB_km1[4 * 4],
//                       real_t T_WB_k[4 * 4]);

//////////////
// CAMCHAIN //
//////////////

typedef struct {
  timestamp_t key;
  real_t *value;
} camchain_pose_hash_t;

typedef struct {
  int analyzed;
  int num_cams;

  int **adj_list;
  real_t **adj_exts;
  // camchain_pose_hash_t **cam_poses;
} camchain_t;

camchain_t *camchain_malloc(const int num_cams);
void camchain_free(camchain_t *cc);
void camchain_add_pose(camchain_t *cc,
                       const int cam_idx,
                       const timestamp_t ts,
                       const real_t T_CiF[4 * 4]);
void camchain_adjacency(camchain_t *cc);
void camchain_adjacency_print(const camchain_t *cc);
int camchain_find(camchain_t *cc,
                  const int idx_i,
                  const int idx_j,
                  real_t T_CiCj[4 * 4]);

////////////////////////
// CAMERA CALIBRATION //
////////////////////////

typedef struct calib_camera_view_t {
  timestamp_t ts;
  int view_idx;
  int cam_idx;
  int num_corners;

  int *tag_ids;
  int *corner_indices;
  real_t *object_points;
  real_t *keypoints;

  struct calib_camera_factor_t *factors;
} calib_camera_view_t;

typedef struct calib_camera_viewset_t {
  timestamp_t key;
  calib_camera_view_t **value;
} calib_camera_viewset_t;

typedef struct calib_camera_t {
  // Settings
  int fix_cam_params;
  int fix_cam_exts;
  int verbose;
  int max_iter;

  // Flags
  int cams_ok;

  // Counters
  int num_cams;
  int num_views;
  int num_factors;

  // Variables
  timestamp_t *timestamps;
  // pose_hash_t *poses;
  extrinsic_t *cam_exts;
  camera_params_t *cam_params;

  // Factors
  calib_camera_viewset_t *view_sets;
  marg_factor_t *marg;
} calib_camera_t;

calib_camera_view_t *calib_camera_view_malloc(const timestamp_t ts,
                                              const int view_idx,
                                              const int cam_idx,
                                              const int num_corners,
                                              const int *tag_ids,
                                              const int *corner_indices,
                                              const real_t *object_points,
                                              const real_t *keypoints,
                                              pose_t *pose,
                                              extrinsic_t *cam_ext,
                                              camera_params_t *cam_params);
void calib_camera_view_free(calib_camera_view_t *view);

calib_camera_t *calib_camera_malloc(void);
void calib_camera_free(calib_camera_t *calib);
void calib_camera_print(calib_camera_t *calib);
void calib_camera_add_camera(calib_camera_t *calib,
                             const int cam_idx,
                             const int cam_res[2],
                             const char *proj_model,
                             const char *dist_model,
                             const real_t *cam_params,
                             const real_t *cam_ext);
void calib_camera_add_view(calib_camera_t *calib,
                           const timestamp_t ts,
                           const int view_idx,
                           const int cam_idx,
                           const int num_corners,
                           const int *tag_ids,
                           const int *corner_indices,
                           const real_t *object_points,
                           const real_t *keypoints);
void calib_camera_marginalize(calib_camera_t *calib);
int calib_camera_add_data(calib_camera_t *calib,
                          const int cam_idx,
                          const char *data_path);
void calib_camera_errors(calib_camera_t *calib,
                         real_t *reproj_rmse,
                         real_t *reproj_mean,
                         real_t *reproj_median);
int calib_camera_shannon_entropy(calib_camera_t *calib, real_t *entropy);

// param_order_t *calib_camera_param_order(const void *data,
//                                         int *sv_size,
//                                         int *r_size);
// void calib_camera_cost(const void *data, real_t *r);
// void calib_camera_linearize_compact(const void *data,
//                                     const int sv_size,
//                                     param_order_t *hash,
//                                     real_t *H,
//                                     real_t *g,
//                                     real_t *r);
// void calib_camera_linsolve(const void *data,
//                            const int sv_size,
//                            param_order_t *hash,
//                            real_t *H,
//                            real_t *g,
//                            real_t *dx);
// void calib_camera_solve(calib_camera_t *calib);

////////////////////////////
// CAMERA-IMU CALIBRATION //
////////////////////////////

// typedef struct calib_imucam_view_t {
//   timestamp_t ts;
//   int view_idx;
//   int cam_idx;
//   int num_corners;
//
//   int *tag_ids;
//   int *corner_indices;
//   real_t *object_points;
//   real_t *keypoints;
//
//   calib_imucam_factor_t *cam_factors;
// } calib_imucam_view_t;
//
// typedef struct calib_imucam_viewset_t {
//   timestamp_t key;
//   calib_imucam_view_t **value;
// } calib_imucam_viewset_t;
//
// typedef struct imu_factor_hash_t {
//   int64_t key;
//   imu_factor_t *value;
// } imu_factor_hash_t;
//
// typedef struct calib_imucam_t {
//   // Settings
//   int fix_fiducial;
//   int fix_poses;
//   int fix_velocities;
//   int fix_biases;
//   int fix_cam_params;
//   int fix_cam_exts;
//   int fix_time_delay;
//   int verbose;
//   int max_iter;
//
//   // Flags
//   int imu_ok;
//   int cams_ok;
//   int state_initialized;
//
//   // Counters
//   int num_imus;
//   int num_cams;
//   int num_views;
//   int num_cam_factors;
//   int num_imu_factors;
//
//   // Variables
//   timestamp_t *timestamps;
//   pose_hash_t *poses;
//   velocity_hash_t *velocities;
//   imu_biases_hash_t *imu_biases;
//   fiducial_t *fiducial;
//   extrinsic_t *cam_exts;
//   camera_params_t *cam_params;
//   extrinsic_t *imu_ext;
//   time_delay_t *time_delay;
//
//   // Data
//   fiducial_buffer_t *fiducial_buffer;
//   imu_params_t imu_params;
//   imu_buffer_t imu_buf;
//
//   // Views
//   calib_imucam_viewset_t *view_sets;
//   imu_factor_hash_t *imu_factors;
// } calib_imucam_t;
//
// calib_imucam_view_t *calib_imucam_view_malloc(const timestamp_t ts,
//                                               const int view_idx,
//                                               const int cam_idx,
//                                               const int num_corners,
//                                               const int *tag_ids,
//                                               const int *corner_indices,
//                                               const real_t *object_points,
//                                               const real_t *keypoints,
//                                               fiducial_t *fiducial,
//                                               pose_t *imu_pose,
//                                               extrinsic_t *imu_ext,
//                                               extrinsic_t *cam_ext,
//                                               camera_params_t *cam_params,
//                                               time_delay_t *time_delay);
// void calib_imucam_view_free(calib_imucam_view_t *view);
//
// calib_imucam_t *calib_imucam_malloc(void); void calib_imucam_free(calib_imucam_t *calib);
// void calib_imucam_print(calib_imucam_t *calib);
//
// void calib_imucam_add_imu(calib_imucam_t *calib,
//                           const real_t imu_rate,
//                           const real_t sigma_aw,
//                           const real_t sigma_gw,
//                           const real_t sigma_a,
//                           const real_t sigma_g,
//                           const real_t g,
//                           const real_t *imu_ext);
// void calib_imucam_add_camera(calib_imucam_t *calib,
//                              const int cam_idx,
//                              const int cam_res[2],
//                              const char *proj_model,
//                              const char *dist_model,
//                              const real_t *cam_params,
//                              const real_t *cam_ext);
//
// void calib_imucam_add_imu_event(calib_imucam_t *calib,
//                                 const timestamp_t ts,
//                                 const real_t acc[3],
//                                 const real_t gyr[3]);
// void calib_imucam_add_fiducial_event(calib_imucam_t *calib,
//                                      const timestamp_t ts,
//                                      const int cam_idx,
//                                      const int num_corners,
//                                      const int *tag_ids,
//                                      const int *corner_indices,
//                                      const real_t *object_points,
//                                      const real_t *keypoints);
// void calib_imucam_marginalize(calib_imucam_t *calib);
// int calib_imucam_update(calib_imucam_t *calib);
// void calib_imucam_errors(calib_imucam_t *calib,
//                          real_t *reproj_rmse,
//                          real_t *reproj_mean,
//                          real_t *reproj_median);
// param_order_t *calib_imucam_param_order(const void *data,
//                                         int *sv_size,
//                                         int *r_size);
// void calib_imucam_cost(const void *data, real_t *r);
// void calib_imucam_linearize_compact(const void *data,
//                                     const int sv_size,
//                                     param_order_t *hash,
//                                     real_t *H,
//                                     real_t *g,
//                                     real_t *r);
// void calib_imucam_save_estimates(calib_imucam_t *calib);
// void calib_imucam_solve(calib_imucam_t *calib);
