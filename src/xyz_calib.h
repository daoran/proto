#pragma once

#include "xyz_ds.h"
#include "xyz_se.h"
#include "xyz_aprilgrid.h"

//////////////
// CAMCHAIN //
//////////////

// typedef struct {
//   timestamp_t key;
//   real_t *value;
// } camchain_pose_hash_t;

// typedef struct {
//   int analyzed;
//   int num_cams;
//
//   int **adj_list;
//   real_t **adj_exts;
//   camchain_pose_hash_t **cam_poses;
// } camchain_t;

// camchain_t *camchain_malloc(const int num_cams);
// void camchain_free(camchain_t *cc);
// void camchain_add_pose(camchain_t *cc,
//                        const int cam_idx,
//                        const timestamp_t ts,
//                        const real_t T_CiF[4 * 4]);
// void camchain_adjacency(camchain_t *cc);
// void camchain_adjacency_print(const camchain_t *cc);
// int camchain_find(camchain_t *cc,
//                   const int idx_i,
//                   const int idx_j,
//                   real_t T_CiCj[4 * 4]);

/////////////////////////
// CALIB-CAMERA FACTOR //
/////////////////////////

struct calib_camera_factor_t {
  pose_t *pose;
  extrinsic_t *cam_ext;
  camera_params_t *cam_params;

  timestamp_t ts;
  int cam_idx;
  int tag_id;
  int corner_idx;
  real_t p_FFi[3];
  real_t z[2];

  real_t covar[2 * 2];
  real_t sqrt_info[2 * 2];

  int r_size;
  int num_params;
  int param_types[3];

  real_t *params[3];
  real_t r[2];
  real_t *jacs[3];
  real_t J_pose[2 * 6];
  real_t J_cam_ext[2 * 6];
  real_t J_cam_params[2 * 8];
};

void calib_camera_factor_setup(struct calib_camera_factor_t *factor,
                               pose_t *pose,
                               extrinsic_t *cam_ext,
                               camera_params_t *cam_params,
                               const int cam_idx,
                               const int tag_id,
                               const int corner_idx,
                               const real_t p_FFi[3],
                               const real_t z[2],
                               const real_t var[2]);
int calib_camera_factor_eval(void *factor_ptr);
int calib_camera_factor_ceres_eval(void *factor_ptr,
                                   real_t **params,
                                   real_t *r_out,
                                   real_t **J_out);

/////////////////////////
// CALIB-IMUCAM FACTOR //
/////////////////////////

typedef struct calib_imucam_factor_t {
  fiducial_t *fiducial;        // fiducial pose: T_WF
  pose_t *imu_pose;            // IMU pose: T_WS
  extrinsic_t *imu_ext;        // IMU extrinsic: T_SC0
  extrinsic_t *cam_ext;        // Camera extrinsic: T_C0Ci
  camera_params_t *cam_params; // Camera parameters
  time_delay_t *time_delay;    // Time delay

  timestamp_t ts;
  int cam_idx;
  int tag_id;
  int corner_idx;
  real_t p_FFi[3];
  real_t z[2];
  real_t v[2];

  real_t covar[2 * 2];
  real_t sqrt_info[2 * 2];

  int r_size;
  int num_params;
  int param_types[6];

  real_t *params[6];
  real_t r[2];
  real_t *jacs[6];
  real_t J_fiducial[2 * 6];
  real_t J_imu_pose[2 * 6];
  real_t J_imu_ext[2 * 6];
  real_t J_cam_ext[2 * 6];
  real_t J_cam_params[2 * 8];
  real_t J_time_delay[2 * 1];
} calib_imucam_factor_t;

void calib_imucam_factor_setup(calib_imucam_factor_t *factor,
                               fiducial_t *fiducial,
                               pose_t *pose,
                               extrinsic_t *imu_ext,
                               extrinsic_t *cam_ext,
                               camera_params_t *cam_params,
                               time_delay_t *time_delay,
                               const int cam_idx,
                               const int tag_id,
                               const int corner_idx,
                               const real_t p_FFi[3],
                               const real_t z[2],
                               const real_t v[2],
                               const real_t var[2]);
int calib_imucam_factor_eval(void *factor_ptr);
int calib_imucam_factor_ceres_eval(void *factor_ptr,
                                   real_t **params,
                                   real_t *r_out,
                                   real_t **J_out);

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
  pose_hash_t *poses;
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

calib_camera_t *calib_camera_malloc(void); void calib_camera_free(calib_camera_t *calib);
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

param_order_t *calib_camera_param_order(const void *data,
                                        int *sv_size,
                                        int *r_size);
void calib_camera_cost(const void *data, real_t *r);
void calib_camera_linearize_compact(const void *data,
                                    const int sv_size,
                                    param_order_t *hash,
                                    real_t *H,
                                    real_t *g,
                                    real_t *r);
void calib_camera_linsolve(const void *data,
                           const int sv_size,
                           param_order_t *hash,
                           real_t *H,
                           real_t *g,
                           real_t *dx);
void calib_camera_solve(calib_camera_t *calib);

// ////////////////////////////
// // CAMERA-IMU CALIBRATION //
// ////////////////////////////
//
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

// ////////////////////////
// // GIMBAL CALIBRATION //
// ////////////////////////
//
// typedef struct calib_gimbal_view_t {
//   timestamp_t ts;
//   int view_idx;
//   int cam_idx;
//   int num_corners;
//
//   int *tag_ids;
//   int *corner_indices;
//   real_t *object_points;
//   real_t *keypoints;
//   calib_gimbal_factor_t *calib_factors;
// } calib_gimbal_view_t;
//
// typedef struct calib_gimbal_t {
//   // Settings
//   int fix_fiducial_ext;
//   int fix_gimbal_ext;
//   int fix_poses;
//   int fix_cam_params;
//   int fix_cam_exts;
//   int fix_links;
//   int fix_joints;
//
//   int num_rows;
//   int num_cols;
//   double tag_size;
//   double tag_spacing;
//
//   // Flags
//   int fiducial_ext_ok;
//   int gimbal_ext_ok;
//   int poses_ok;
//   int cams_ok;
//   int links_ok;
//   int joints_ok;
//
//   // Counters
//   int num_cams;
//   int num_views;
//   int num_poses;
//   int num_links;
//   int num_joints;
//   int num_calib_factors;
//   int num_joint_factors;
//
//   // Variables
//   timestamp_t *timestamps;
//   fiducial_t fiducial_ext;
//   extrinsic_t gimbal_ext;
//   extrinsic_t *cam_exts;
//   camera_params_t *cam_params;
//   extrinsic_t *links;
//   joint_t **joints;
//   pose_t *poses;
//
//   // Factors
//   calib_gimbal_view_t ***views;
//   joint_factor_t **joint_factors;
// } calib_gimbal_t;
//
// void calib_gimbal_view_setup(calib_gimbal_view_t *calib);
// calib_gimbal_view_t *calib_gimbal_view_malloc(const timestamp_t ts,
//                                               const int view_idx,
//                                               const int cam_idx,
//                                               const int *tag_ids,
//                                               const int *corner_indices,
//                                               const real_t *object_points,
//                                               const real_t *keypoints,
//                                               const int N,
//                                               fiducial_t *fiducial_ext,
//                                               extrinsic_t *gimbal_ext,
//                                               pose_t *pose,
//                                               extrinsic_t *link0,
//                                               extrinsic_t *link1,
//                                               joint_t *joint0,
//                                               joint_t *joint1,
//                                               joint_t *joint2,
//                                               extrinsic_t *cam_ext,
//                                               camera_params_t *cam_params);
// void calib_gimbal_view_free(calib_gimbal_view_t *calib);
// int calib_gimbal_view_equals(const calib_gimbal_view_t *v0,
//                              const calib_gimbal_view_t *v1);
//
// void calib_gimbal_setup(calib_gimbal_t *calib);
// calib_gimbal_t *calib_gimbal_malloc(void); void calib_gimbal_free(calib_gimbal_t *calib);
// int calib_gimbal_equals(const calib_gimbal_t *calib0,
//                         const calib_gimbal_t *calib1);
// calib_gimbal_t *calib_gimbal_copy(const calib_gimbal_t *src);
// void calib_gimbal_print(const calib_gimbal_t *calib);
// void calib_gimbal_add_fiducial(calib_gimbal_t *calib,
//                                const real_t fiducial_pose[7]);
// void calib_gimbal_add_pose(calib_gimbal_t *calib,
//                            const timestamp_t ts,
//                            const real_t pose[7]);
// void calib_gimbal_add_gimbal_extrinsic(calib_gimbal_t *calib,
//                                        const real_t gimbal_ext[7]);
// void calib_gimbal_add_gimbal_link(calib_gimbal_t *calib,
//                                   const int link_idx,
//                                   const real_t link[7]);
// void calib_gimbal_add_camera(calib_gimbal_t *calib,
//                              const int cam_idx,
//                              const int cam_res[2],
//                              const char *proj_model,
//                              const char *dist_model,
//                              const real_t *cam_params,
//                              const real_t *cam_ext);
// void calib_gimbal_add_view(calib_gimbal_t *calib,
//                            const int pose_idx,
//                            const int view_idx,
//                            const timestamp_t ts,
//                            const int cam_idx,
//                            const int num_corners,
//                            const int *tag_ids,
//                            const int *corner_indices,
//                            const real_t *object_points,
//                            const real_t *keypoints,
//                            const real_t *joints,
//                            const int num_joints);
// int calib_gimbal_remove_view(calib_gimbal_t *calib, const int view_idx);
// calib_gimbal_t *calib_gimbal_load(const char *data_path);
// void calib_gimbal_save(const calib_gimbal_t *calib, const char *data_path);
// int calib_gimbal_validate(calib_gimbal_t *calib);
// void calib_gimbal_nbv(calib_gimbal_t *calib, real_t nbv_joints[3]);
// param_order_t *calib_gimbal_param_order(const void *data,
//                                         int *sv_size,
//                                         int *r_size);
// void calib_gimbal_reproj_errors(const calib_gimbal_t *calib,
//                                 real_t *reproj_rmse,
//                                 real_t *reproj_mean,
//                                 real_t *reproj_median);
// void calib_gimbal_cost(const void *data, real_t *r);
// void calib_gimbal_linearize(const void *data,
//                             const int J_rows,
//                             const int J_cols,
//                             param_order_t *hash,
//                             real_t *J,
//                             real_t *g,
//                             real_t *r);
// void calib_gimbal_linearize_compact(const void *data,
//                                     const int sv_size,
//                                     param_order_t *hash,
//                                     real_t *H,
//                                     real_t *g,
//                                     real_t *r);
