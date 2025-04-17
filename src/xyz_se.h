#pragma once

#include <stb_ds.h>

#include "xyz.h"
#include "xyz_cv.h"

/*******************************************************************************
 * STATE ESTIMATION
 ******************************************************************************/

#define POSITION_PARAM 1
#define ROTATION_PARAM 2
#define POSE_PARAM 3
#define EXTRINSIC_PARAM 4
#define FIDUCIAL_PARAM 5
#define VELOCITY_PARAM 6
#define IMU_BIASES_PARAM 7
#define FEATURE_PARAM 8
#define IDF_BEARING_PARAM 9
#define IDF_POSITION_PARAM 10
#define JOINT_PARAM 11
#define CAMERA_PARAM 12
#define TIME_DELAY_PARAM 13

///////////
// UTILS //
///////////

int schur_complement(const real_t *H,
                     const real_t *b,
                     const int H_size,
                     const int m,
                     const int r,
                     real_t *H_marg,
                     real_t *b_marg);

int shannon_entropy(const real_t *covar, const int m, real_t *entropy);

//////////////
// POSITION //
//////////////

typedef struct pos_t {
  int marginalize;
  int fix;
  real_t data[3];
} pos_t;

void pos_setup(pos_t *pos, const real_t *data);
void pos_copy(const pos_t *src, pos_t *dst);
void pos_fprint(const char *prefix, const pos_t *pos, FILE *f);
void pos_print(const char *prefix, const pos_t *pos);

//////////////
// ROTATION //
//////////////

typedef struct rot_t {
  int marginalize;
  int fix;
  real_t data[4];
} rot_t;

void rot_setup(rot_t *rot, const real_t *data);
void rot_fprint(const char *prefix, const rot_t *rot, FILE *f);
void rot_print(const char *prefix, const rot_t *rot);

//////////
// POSE //
//////////

typedef struct pose_t {
  int marginalize;
  int fix;
  timestamp_t ts;
  real_t data[7];
} pose_t;

void pose_init(real_t *pose);
void pose_setup(pose_t *pose, const timestamp_t ts, const real_t *param);
void pose_copy(const pose_t *src, pose_t *dst);
void pose_fprint(const char *prefix, const pose_t *pose, FILE *f);
void pose_print(const char *prefix, const pose_t *pose);

///////////////
// EXTRINSIC //
///////////////

typedef struct extrinsic_t {
  int marginalize;
  int fix;
  real_t data[7];
} extrinsic_t;

void extrinsic_setup(extrinsic_t *extrinsic, const real_t *param);
void extrinsic_copy(const extrinsic_t *src, extrinsic_t *dst);
void extrinsic_fprint(const char *prefix, const extrinsic_t *exts, FILE *f);
void extrinsic_print(const char *prefix, const extrinsic_t *exts);

//////////////
// FIDUCIAL //
//////////////

/** Fiducial **/
typedef struct fiducial_t {
  int marginalize;
  int fix;
  real_t data[7];
} fiducial_t;

void fiducial_setup(fiducial_t *fiducial, const real_t *param);
void fiducial_copy(const fiducial_t *src, fiducial_t *dst);
void fiducial_fprint(const char *prefix, const fiducial_t *exts, FILE *f);
void fiducial_print(const char *prefix, const fiducial_t *exts);

/** Fiducial Buffer **/
// typedef struct fiducial_buffer_t {
//   fiducial_event_t **data;
//   int size;
//   int capacity;
// } fiducial_buffer_t;

// fiducial_buffer_t *fiducial_buffer_malloc(void); void fiducial_buffer_clear(fiducial_buffer_t *buf);
// void fiducial_buffer_free(fiducial_buffer_t *buf);
// int fiducial_buffer_total_corners(const fiducial_buffer_t *buf);
// void fiducial_buffer_add(fiducial_buffer_t *buf,
//                          const timestamp_t ts,
//                          const int cam_idx,
//                          const int num_corners,
//                          const int *tag_ids,
//                          const int *corner_indices,
//                          const real_t *object_points,
//                          const real_t *keypoints);

///////////////////////
// CAMERA-PARAMETERS //
///////////////////////

typedef struct camera_params_t {
  int marginalize;
  int fix;

  int cam_idx;
  int resolution[2];
  char proj_model[30];
  char dist_model[30];
  real_t data[8];

  project_func_t proj_func;
  back_project_func_t back_proj_func;
  undistort_func_t undistort_func;
} camera_params_t;

void camera_params_setup(camera_params_t *camera,
                         const int cam_idx,
                         const int cam_res[2],
                         const char *proj_model,
                         const char *dist_model,
                         const real_t *data);
void camera_params_copy(const camera_params_t *src, camera_params_t *dst);
void camera_params_fprint(const camera_params_t *cam, FILE *f);
void camera_params_print(const camera_params_t *camera);
void camera_project(const camera_params_t *camera,
                    const real_t p_C[3],
                    real_t z[2]);
void camera_back_project(const camera_params_t *camera,
                         const real_t z[2],
                         real_t bearing[3]);
void camera_undistort_points(const camera_params_t *camera,
                             const real_t *kps,
                             const int num_points,
                             real_t *kps_und);
int solvepnp_camera(const camera_params_t *cam_params,
                    const real_t *img_pts,
                    const real_t *obj_pts,
                    const int N,
                    real_t T_CO[4 * 4]);
void triangulate_batch(const camera_params_t *cam_i,
                       const camera_params_t *cam_j,
                       const real_t T_CiCj[4 * 4],
                       const real_t *kps_i,
                       const real_t *kps_j,
                       const int n,
                       real_t *points,
                       int *status);
void stereo_triangulate(const camera_params_t *cam_i,
                        const camera_params_t *cam_j,
                        const real_t T_WCi[4 * 4],
                        const real_t T_CiCj[4 * 4],
                        const real_t *kps_i,
                        const real_t *kps_j,
                        const int n,
                        real_t *points,
                        int *status);

//////////////
// VELOCITY //
//////////////

typedef struct velocity_t {
  int marginalize;
  int fix;

  timestamp_t ts;
  real_t data[3];
} velocity_t;

void velocity_setup(velocity_t *vel, const timestamp_t ts, const real_t v[3]);
void velocity_copy(const velocity_t *src, velocity_t *dst);

////////////////
// IMU-BIASES //
////////////////

typedef struct imu_biases_t {
  int marginalize;
  int fix;

  timestamp_t ts;
  real_t data[6];
} imu_biases_t;

void imu_biases_setup(imu_biases_t *sb,
                      const timestamp_t ts,
                      const real_t ba[3],
                      const real_t bg[3]);
void imu_biases_copy(const imu_biases_t *src, imu_biases_t *dst);
void imu_biases_get_accel_bias(const imu_biases_t *biases, real_t ba[3]);
void imu_biases_get_gyro_bias(const imu_biases_t *biases, real_t bg[3]);

/////////////
// FEATURE //
/////////////

#define FEATURE_XYZ 0
#define FEATURE_INVERSE_DEPTH 1
#define FEATURE_MAX_LENGTH 20

#define FEATURES_CAPACITY_INITIAL 10000
#define FEATURES_CAPACITY_GROWTH_FACTOR 2

/** Feature **/
typedef struct feature_t {
  int marginalize;
  int fix;
  int type;

  // Feature data
  size_t feature_id;
  int status;
  real_t data[3];
} feature_t;

typedef struct feature_map_t {
  size_t key;
  feature_t feature;
} feature_map_t;

void feature_setup(feature_t *f, const size_t feature_id);
void feature_init(feature_t *f, const size_t feature_id, const real_t *data);
void feature_print(const feature_t *feature);

// void idf_setup(feature_t *f,
//                const size_t feature_id,
//                const size_t pos_id,
//                const camera_params_t *cam_params,
//                const real_t C_WC[3 * 3],
//                const real_t z[2]);
// void idf_point(const feature_t *f, const real_t r_WC[3], real_t p_W[3]);

// /** Features **/
// typedef struct features_t {
//   feature_t **data;
//   size_t num_features;
//   size_t feature_capacity;

//   pos_t **pos_data;
//   size_t num_positions;
//   size_t position_capacity;
// } features_t;

// features_t *features_malloc(void); void features_free(features_t *features);
// int features_exists(const features_t *features, const size_t feature_id);
// void features_add_xyzs(features_t *features,
//                        const size_t *feature_ids,
//                        const real_t *params,
//                        const size_t num_features);
// void features_add_idfs(features_t *features,
//                        const size_t *feature_ids,
//                        const camera_params_t *cam_params,
//                        const real_t T_WC[4 * 4],
//                        const real_t *keypoints,
//                        const size_t num_keypoints);
// void features_get_xyz(const features_t *features,
//                       const size_t feature_id,
//                       feature_t **feature);
// void features_get_idf(const features_t *features,
//                       const size_t feature_id,
//                       feature_t **feature,
//                       pos_t **pos);
// int features_point(const features_t *features,
//                    const size_t feature_id,
//                    real_t p_W[3]);

////////////////
// TIME-DELAY //
////////////////

typedef struct time_delay_t {
  int marginalize;
  int fix;
  real_t data[1];
} time_delay_t;

void time_delay_setup(time_delay_t *time_delay, const real_t param);
void time_delay_copy(const time_delay_t *src, time_delay_t *dst);
void time_delay_print(const char *prefix, const time_delay_t *exts);

///////////
// POINT //
///////////

typedef struct point_t {
  real_t x;
  real_t y;
  real_t z;
} point_t;

///////////
// JOINT //
///////////

typedef struct joint_t {
  int marginalize;
  int fix;

  timestamp_t ts;
  int joint_idx;
  real_t data[1];
} joint_t;

void joint_setup(joint_t *joint,
                 const timestamp_t ts,
                 const int joint_idx,
                 const real_t theta);
void joint_copy(const joint_t *src, joint_t *dst);
void joint_print(const char *prefix, const joint_t *joint);

////////////////
// PARAMETERS //
////////////////

typedef struct param_hash_t {
  int64_t key;
  int param_type;
  void *param_ptr;
} param_hash_t;

#define PARAM_HASH(HASH_NAME, KEY_TYPE, VALUE_TYPE)                            \
  typedef struct HASH_NAME {                                                   \
    KEY_TYPE key;                                                              \
    VALUE_TYPE *value;                                                         \
  } HASH_NAME;

PARAM_HASH(pos_hash_t, timestamp_t, pos_t)
PARAM_HASH(rot_hash_t, timestamp_t, rot_t)
PARAM_HASH(pose_hash_t, timestamp_t, pose_t)
PARAM_HASH(velocity_hash_t, timestamp_t, velocity_t)
PARAM_HASH(imu_biases_hash_t, timestamp_t, imu_biases_t)
PARAM_HASH(feature_hash_t, size_t, feature_t)
PARAM_HASH(joint_hash_t, size_t, joint_t)
PARAM_HASH(extrinsic_hash_t, size_t, extrinsic_t)
PARAM_HASH(fiducial_hash_t, size_t, fiducial_t)
PARAM_HASH(camera_params_hash_t, size_t, camera_params_t)
PARAM_HASH(time_delay_hash_t, size_t, time_delay_t)

typedef struct param_order_t {
  void *key;
  int idx;
  int type;
  int fix;
} param_order_t;

void param_order_free(param_order_t *hash);
void param_type_string(const int param_type, char *s);
size_t param_global_size(const int param_type);
size_t param_local_size(const int param_type);
void param_order_print(const param_order_t *hash);
int param_order_exists(param_order_t **hash, real_t *data);
void param_order_add(param_order_t **hash,
                     const int param_type,
                     const int fix,
                     real_t *data,
                     int *col_idx);
void param_order_add_position(param_order_t **h, pos_t *p, int *c);
void param_order_add_rotation(param_order_t **h, rot_t *p, int *c);
void param_order_add_pose(param_order_t **h, pose_t *p, int *c);
void param_order_add_extrinsic(param_order_t **h, extrinsic_t *p, int *c);
void param_order_add_fiducial(param_order_t **h, fiducial_t *p, int *c);
void param_order_add_velocity(param_order_t **h, velocity_t *p, int *c);
void param_order_add_imu_biases(param_order_t **h, imu_biases_t *p, int *c);
void param_order_add_feature(param_order_t **h, feature_t *p, int *c);
void param_order_add_joint(param_order_t **h, joint_t *p, int *c);
void param_order_add_camera(param_order_t **h, camera_params_t *p, int *c);
void param_order_add_time_delay(param_order_t **h, time_delay_t *p, int *c);

////////////
// FACTOR //
////////////

typedef struct factor_hash_t {
  int64_t key;
  int factor_type;
  void *factor_ptr;
} factor_hash_t;

#define FACTOR_EVAL_PTR                                                        \
  int (*factor_eval)(const void *factor,                                       \
                     real_t **params,                                          \
                     real_t *residuals,                                        \
                     real_t **jacobians)

#define CERES_FACTOR_EVAL(FACTOR_TYPE,                                         \
                          FACTOR,                                              \
                          FACTOR_EVAL,                                         \
                          PARAMS,                                              \
                          R_OUT,                                               \
                          J_OUT)                                               \
  {                                                                            \
    assert(FACTOR);                                                            \
    assert(PARAMS);                                                            \
    assert(R_OUT);                                                             \
                                                                               \
    /* Copy parameters */                                                      \
    for (int i = 0; i < FACTOR->num_params; i++) {                             \
      const int global_size = param_global_size(FACTOR->param_types[i]);       \
      vec_copy(PARAMS[i], global_size, FACTOR->params[i]);                     \
    }                                                                          \
                                                                               \
    /* Evaluate factor */                                                      \
    FACTOR_EVAL(factor_ptr);                                                   \
                                                                               \
    /* Residuals */                                                            \
    vec_copy(FACTOR->r, FACTOR->r_size, r_out);                                \
                                                                               \
    /* Jacobians */                                                            \
    if (J_OUT == NULL) {                                                       \
      return 1;                                                                \
    }                                                                          \
                                                                               \
    const int r_size = FACTOR->r_size;                                         \
    for (int jac_idx = 0; jac_idx < FACTOR->num_params; jac_idx++) {           \
      if (J_OUT[jac_idx]) {                                                    \
        const int gs = param_global_size(FACTOR->param_types[jac_idx]);        \
        const int ls = param_local_size(FACTOR->param_types[jac_idx]);         \
        const int rs = 0;                                                      \
        const int re = r_size - 1;                                             \
        const int cs = 0;                                                      \
        const int ce = ls - 1;                                                 \
        zeros(J_OUT[jac_idx], r_size, gs);                                     \
        mat_block_set(J_OUT[jac_idx],                                          \
                      gs,                                                      \
                      rs,                                                      \
                      re,                                                      \
                      cs,                                                      \
                      ce,                                                      \
                      FACTOR->jacs[jac_idx]);                                  \
      }                                                                        \
    }                                                                          \
    return 1;                                                                  \
  }

int check_factor_jacobian(const void *factor,
                          FACTOR_EVAL_PTR,
                          real_t **params,
                          real_t **jacobians,
                          const int r_size,
                          const int param_size,
                          const int param_idx,
                          const real_t step_size,
                          const real_t tol,
                          const int verbose);

int check_factor_so3_jacobian(const void *factor,
                              FACTOR_EVAL_PTR,
                              real_t **params,
                              real_t **jacobians,
                              const int r_size,
                              const int param_idx,
                              const real_t step_size,
                              const real_t tol,
                              const int verbose);

/////////////////
// POSE FACTOR //
/////////////////

typedef struct pose_factor_t {
  real_t pos_meas[3];
  real_t quat_meas[4];
  pose_t *pose_est;

  real_t covar[6 * 6];
  real_t sqrt_info[6 * 6];

  real_t r[6];
  int r_size;

  int param_types[1];
  real_t *params[1];
  int num_params;

  real_t *jacs[1];
  real_t J_pose[6 * 6];
} pose_factor_t;

void pose_factor_setup(pose_factor_t *factor,
                       pose_t *pose,
                       const real_t var[6]);
int pose_factor_eval(void *factor);

///////////////
// BA FACTOR //
///////////////

typedef struct ba_factor_t {
  pose_t *pose;
  feature_t *feature;
  camera_params_t *camera;

  real_t covar[2 * 2];
  real_t sqrt_info[2 * 2];
  real_t z[2];

  real_t r[2];
  int r_size;

  int param_types[3];
  real_t *params[3];
  int num_params;

  real_t *jacs[3];
  real_t J_pose[2 * 6];
  real_t J_feature[2 * 3];
  real_t J_camera[2 * 8];
} ba_factor_t;

void ba_factor_setup(ba_factor_t *factor,
                     pose_t *pose,
                     feature_t *feature,
                     camera_params_t *camera,
                     const real_t z[2],
                     const real_t var[2]);
int ba_factor_eval(void *factor_ptr);

///////////////////
// CAMERA FACTOR //
///////////////////

typedef struct camera_factor_t {
  pose_t *pose;
  extrinsic_t *extrinsic;
  camera_params_t *camera;
  feature_t *feature;

  real_t covar[2 * 2];
  real_t sqrt_info[2 * 2];
  real_t z[2];

  real_t r[2];
  int r_size;

  int num_params;
  int param_types[4];
  real_t *params[4];

  real_t *jacs[4];
  real_t J_pose[2 * 6];
  real_t J_extrinsic[2 * 6];
  real_t J_feature[2 * 3];
  real_t J_camera[2 * 8];
} camera_factor_t;

void camera_factor_setup(camera_factor_t *factor,
                         pose_t *pose,
                         extrinsic_t *extrinsic,
                         feature_t *feature,
                         camera_params_t *camera,
                         const real_t z[2],
                         const real_t var[2]);
int camera_factor_eval(void *factor_ptr);

////////////////////////////////////////
// INVERSE-DEPTH FEATURE (IDF) FACTOR //
////////////////////////////////////////

// typedef struct idf_factor_t {
//   timestamp_t ts;
//   int cam_idx;
//   size_t feature_id;

//   pose_t *pose;
//   extrinsic_t *extrinsic;
//   camera_params_t *camera;
//   pos_t *idf_pos;
//   feature_t *idf_param;

//   real_t covar[2 * 2];
//   real_t sqrt_info[2 * 2];
//   real_t z[2];

//   int r_size;
//   int num_params;
//   int param_types[5];

//   real_t *params[5];
//   real_t r[2];
//   real_t *jacs[5];
//   real_t J_pose[2 * 6];
//   real_t J_extrinsic[2 * 6];
//   real_t J_camera[2 * 8];
//   real_t J_idf_pos[2 * 3];
//   real_t J_idf_param[2 * 3];
// } idf_factor_t;

// void idf_factor_setup(idf_factor_t *factor,
//                       pose_t *pose,
//                       extrinsic_t *extrinsic,
//                       camera_params_t *camera,
//                       pos_t *idf_pos,
//                       feature_t *idf_param,
//                       const timestamp_t ts,
//                       const int cam_idx,
//                       const size_t feature_id,
//                       const real_t z[2],
//                       const real_t var[2]);
// int idf_factor_eval(void *factor_ptr);

////////////////
// IMU FACTOR //
////////////////

/** IMU Parameters **/
typedef struct imu_params_t {
  int imu_idx;
  real_t rate;

  real_t sigma_aw;
  real_t sigma_gw;
  real_t sigma_a;
  real_t sigma_g;
  real_t g;
} imu_params_t;

/** IMU Buffer **/
#define IMU_BUFFER_MAX_SIZE 1000

typedef struct imu_buffer_t {
  timestamp_t ts[IMU_BUFFER_MAX_SIZE];
  real_t acc[IMU_BUFFER_MAX_SIZE][3];
  real_t gyr[IMU_BUFFER_MAX_SIZE][3];
  int size;
} imu_buffer_t;

void imu_buffer_setup(imu_buffer_t *imu_buf);
void imu_buffer_add(imu_buffer_t *imu_buf,
                    const timestamp_t ts,
                    const real_t acc[3],
                    const real_t gyr[3]);
timestamp_t imu_buffer_first_ts(const imu_buffer_t *imu_buf);
timestamp_t imu_buffer_last_ts(const imu_buffer_t *imu_buf);
void imu_buffer_clear(imu_buffer_t *imu_buf);
void imu_buffer_copy(const imu_buffer_t *from, imu_buffer_t *to);
void imu_buffer_print(const imu_buffer_t *imu_buf);

/** IMU Factor **/
typedef struct imu_factor_t {
  // IMU parameters and buffer
  const imu_params_t *imu_params;
  imu_buffer_t imu_buf;

  // Parameters
  pose_t *pose_i;
  velocity_t *vel_i;
  imu_biases_t *biases_i;
  pose_t *pose_j;
  velocity_t *vel_j;
  imu_biases_t *biases_j;
  int num_params;
  real_t *params[6];
  int param_types[6];

  // Residuals
  int r_size;
  real_t r[15];

  // Jacobians
  real_t *jacs[6];
  real_t J_pose_i[15 * 6];
  real_t J_vel_i[15 * 3];
  real_t J_biases_i[15 * 6];
  real_t J_pose_j[15 * 6];
  real_t J_vel_j[15 * 3];
  real_t J_biases_j[15 * 6];

  // Preintegration variables
  real_t Dt;         // Time difference between pose_i and pose_j in seconds
  real_t F[15 * 15]; // State jacobian
  real_t P[15 * 15]; // State covariance
  real_t Q[18 * 18]; // Noise matrix
  real_t dr[3];      // Relative position
  real_t dv[3];      // Relative velocity
  real_t dq[4];      // Relative rotation
  real_t ba[3];      // Accel biase
  real_t bg[3];      // Gyro biase
  real_t ba_ref[3];
  real_t bg_ref[3];

  // Preintegration step variables
  real_t r_i[3];
  real_t v_i[3];
  real_t q_i[4];
  real_t ba_i[3];
  real_t bg_i[3];

  real_t r_j[3];
  real_t v_j[3];
  real_t q_j[4];
  real_t ba_j[3];
  real_t bg_j[3];

  // Covariance and square-root info
  real_t covar[15 * 15];
  real_t sqrt_info[15 * 15];
} imu_factor_t;

void imu_state_vector(const real_t r[3],
                      const real_t q[4],
                      const real_t v[3],
                      const real_t ba[3],
                      const real_t bg[3],
                      real_t x[16]);
void imu_propagate(const real_t pose_k[7],
                   const real_t vel_k[3],
                   const imu_buffer_t *imu_buf,
                   real_t pose_kp1[7],
                   real_t vel_kp1[3]);
void imu_initial_attitude(const imu_buffer_t *imu_buf, real_t q_WS[4]);
void imu_factor_propagate_step(imu_factor_t *factor,
                               const real_t a_i[3],
                               const real_t w_i[3],
                               const real_t a_j[3],
                               const real_t w_j[3],
                               const real_t dt);
void imu_factor_F_matrix(const real_t q_i[4],
                         const real_t q_j[4],
                         const real_t ba_i[3],
                         const real_t bg_i[3],
                         const real_t a_i[3],
                         const real_t w_i[3],
                         const real_t a_j[3],
                         const real_t w_j[3],
                         const real_t dt,
                         real_t F_dt[15 * 15]);
void imu_factor_form_G_matrix(const imu_factor_t *factor,
                              const real_t a_i[3],
                              const real_t a_j[3],
                              const real_t dt,
                              real_t G_dt[15 * 18]);
void imu_factor_setup(imu_factor_t *factor,
                      const imu_params_t *imu_params,
                      const imu_buffer_t *imu_buf,
                      pose_t *pose_i,
                      velocity_t *v_i,
                      imu_biases_t *biases_i,
                      pose_t *pose_j,
                      velocity_t *v_j,
                      imu_biases_t *biases_j);
void imu_factor_reset(imu_factor_t *factor);
void imu_factor_preintegrate(imu_factor_t *factor);
int imu_factor_residuals(imu_factor_t *factor, real_t **params, real_t *r_out);
int imu_factor_eval(void *factor_ptr);
int imu_factor_ceres_eval(void *factor_ptr,
                          real_t **params,
                          real_t *r_out,
                          real_t **J_out);

//////////////////
// LIDAR FACTOR //
//////////////////

typedef struct pcd_t {
  timestamp_t ts_start;
  timestamp_t ts_end;
  float *data;
  float *time_diffs;
  size_t num_points;
} pcd_t;

typedef struct lidar_factor_t {
  pcd_t *pcd;
  pose_t *pose;
  extrinsic_t *extrinsic;

  real_t *points_W;
  size_t *indices;
  size_t num_points;

  real_t covar[3 * 3];
  real_t sqrt_info[3 * 3];

  real_t *r;
  int r_size;

  int param_types[2];
  real_t *params[2];
  int num_params;

  real_t *jacs[1];
  real_t *J_pose;
} lidar_factor_t;

pcd_t *pcd_malloc(const timestamp_t ts_start,
                  const timestamp_t ts_end,
                  const float *data,
                  const float *time_diffs,
                  const size_t num_points);
void pcd_free(pcd_t *pcd);
void pcd_deskew(pcd_t *points,
                const real_t T_WL_km1[4 * 4],
                const real_t T_WL_km2[4 * 4]);

void lidar_factor_setup(lidar_factor_t *factor,
                        pcd_t *pcd,
                        pose_t *pose_k,
                        const real_t var[3]);
void lidar_factor_eval(void *factor);

////////////////////////
// JOINT-ANGLE FACTOR //
////////////////////////

typedef struct joint_factor_t {
  joint_t *joint;

  real_t z[1];
  real_t covar[1];
  real_t sqrt_info[1];

  int r_size;
  int num_params;
  int param_types[1];

  real_t *params[1];
  real_t r[1];
  real_t *jacs[1];
  real_t J_joint[1 * 1];
} joint_factor_t;

void joint_factor_setup(joint_factor_t *factor,
                        joint_t *joint0,
                        const real_t z,
                        const real_t var);
void joint_factor_copy(const joint_factor_t *src, joint_factor_t *dst);
int joint_factor_eval(void *factor_ptr);
int joint_factor_equals(const joint_factor_t *j0, const joint_factor_t *j1);

//////////////////
// MARGINALIZER //
//////////////////

#define MARG_FACTOR 1
#define BA_FACTOR 2
#define CAMERA_FACTOR 3
#define IDF_FACTOR 4
#define IMU_FACTOR 5
#define CALIB_CAMERA_FACTOR 6
#define CALIB_IMUCAM_FACTOR 7

#define MARG_TRACK(RHASH, MHASH, PARAM)                                        \
  if (PARAM->marginalize == 0) {                                               \
    hmput(RHASH, PARAM, PARAM);                                                \
  } else {                                                                     \
    hmput(MHASH, PARAM, PARAM);                                                \
  }

#define MARG_TRACK_FACTOR(PARAM, PARAM_TYPE)                                   \
  switch (PARAM_TYPE) {                                                        \
    case POSITION_PARAM:                                                       \
      MARG_TRACK(marg->r_positions, marg->m_positions, ((pos_t *) PARAM));     \
      break;                                                                   \
    case ROTATION_PARAM:                                                       \
      MARG_TRACK(marg->r_rotations, marg->m_rotations, ((rot_t *) PARAM));     \
      break;                                                                   \
    case POSE_PARAM:                                                           \
      MARG_TRACK(marg->r_poses, marg->m_poses, ((pose_t *) PARAM));            \
      break;                                                                   \
    case VELOCITY_PARAM:                                                       \
      MARG_TRACK(marg->r_velocities,                                           \
                 marg->m_velocities,                                           \
                 ((velocity_t *) PARAM));                                      \
      break;                                                                   \
    case IMU_BIASES_PARAM:                                                     \
      MARG_TRACK(marg->r_imu_biases,                                           \
                 marg->m_imu_biases,                                           \
                 ((imu_biases_t *) PARAM));                                    \
      break;                                                                   \
    case FEATURE_PARAM:                                                        \
      MARG_TRACK(marg->r_features, marg->m_features, ((feature_t *) PARAM));   \
      break;                                                                   \
    case FIDUCIAL_PARAM:                                                       \
      MARG_TRACK(marg->r_fiducials,                                            \
                 marg->m_fiducials,                                            \
                 ((fiducial_t *) PARAM));                                      \
      break;                                                                   \
    case EXTRINSIC_PARAM:                                                      \
      MARG_TRACK(marg->r_extrinsics,                                           \
                 marg->m_extrinsics,                                           \
                 ((extrinsic_t *) PARAM));                                     \
      break;                                                                   \
    case JOINT_PARAM:                                                          \
      MARG_TRACK(marg->r_joints, marg->m_joints, ((joint_t *) PARAM));         \
      break;                                                                   \
    case CAMERA_PARAM:                                                         \
      MARG_TRACK(marg->r_cam_params,                                           \
                 marg->m_cam_params,                                           \
                 ((camera_params_t *) PARAM));                                 \
      break;                                                                   \
    case TIME_DELAY_PARAM:                                                     \
      MARG_TRACK(marg->r_time_delays,                                          \
                 marg->m_time_delays,                                          \
                 ((time_delay_t *) PARAM));                                    \
      break;                                                                   \
    default:                                                                   \
      FATAL("Implementation Error!\n");                                        \
      break;                                                                   \
  }

#define MARG_INDEX(HASH, PARAM_TYPE, PARAM_ORDER, COL_IDX, SZ, GZ, N)          \
  for (size_t i = 0; i < hmlen(HASH); i++) {                                   \
    real_t *data = HASH[i].value->data;                                        \
    const int fix = HASH[i].value->fix;                                        \
    if (fix == 0) {                                                            \
      SZ += param_local_size(PARAM_TYPE);                                      \
      GZ += param_global_size(PARAM_TYPE);                                     \
      N += 1;                                                                  \
    }                                                                          \
    param_order_add(&PARAM_ORDER, PARAM_TYPE, fix, data, COL_IDX);             \
  }

#define MARG_PARAMS(MARG, HASH, PARAM_TYPE, PARAM_IDX, X0_IDX)                 \
  for (size_t i = 0; i < hmlen(HASH); i++) {                                   \
    const size_t param_size = param_global_size(PARAM_TYPE);                   \
    if (HASH[i].value->fix) {                                                  \
      continue;                                                                \
    }                                                                          \
    void *param = HASH[i].value;                                               \
    real_t *data = HASH[i].value->data;                                        \
                                                                               \
    MARG->param_types[PARAM_IDX] = PARAM_TYPE;                                 \
    MARG->param_ptrs[PARAM_IDX] = param;                                       \
    MARG->params[PARAM_IDX] = data;                                            \
    PARAM_IDX++;                                                               \
                                                                               \
    vec_copy(data, param_size, MARG->x0 + X0_IDX);                             \
    X0_IDX += param_size;                                                      \
  }

#define MARG_H(MARG, FACTOR_TYPE, FACTORS, H, G, LOCAL_SIZE)                   \
  {                                                                            \
    list_node_t *node = FACTORS->first;                                        \
    while (node != NULL) {                                                     \
      FACTOR_TYPE *factor = (FACTOR_TYPE *) node->value;                       \
      solver_fill_hessian(marg->hash,                                          \
                          factor->num_params,                                  \
                          factor->params,                                      \
                          factor->jacs,                                        \
                          factor->r,                                           \
                          factor->r_size,                                      \
                          LOCAL_SIZE,                                          \
                          H,                                                   \
                          G);                                                  \
      node = node->next;                                                       \
    }                                                                          \
  }

#define MARG_PARAM_HASH(PARAM_TYPE, HASH_NAME)                                 \
  typedef struct HASH_NAME {                                                   \
    void *key;                                                                 \
    PARAM_TYPE *value;                                                         \
  } HASH_NAME;

MARG_PARAM_HASH(pos_t, marg_pos_t)
MARG_PARAM_HASH(rot_t, marg_rot_t)
MARG_PARAM_HASH(pose_t, marg_pose_t)
MARG_PARAM_HASH(velocity_t, marg_velocity_t)
MARG_PARAM_HASH(imu_biases_t, marg_imu_biases_t)
MARG_PARAM_HASH(feature_t, marg_feature_t)
MARG_PARAM_HASH(joint_t, marg_joint_t)
MARG_PARAM_HASH(extrinsic_t, marg_extrinsic_t)
MARG_PARAM_HASH(fiducial_t, marg_fiducial_t)
MARG_PARAM_HASH(camera_params_t, marg_camera_params_t)
MARG_PARAM_HASH(time_delay_t, marg_time_delay_t)

typedef struct marg_factor_t {
  // Settings
  int debug;
  int cond_hessian;

  // Flags
  int marginalized;
  int schur_complement_ok;
  int eigen_decomp_ok;

  // parameters
  // -- Remain parameters
  marg_pos_t *r_positions;
  marg_rot_t *r_rotations;
  marg_pose_t *r_poses;
  marg_velocity_t *r_velocities;
  marg_imu_biases_t *r_imu_biases;
  marg_fiducial_t *r_fiducials;
  marg_joint_t *r_joints;
  marg_extrinsic_t *r_extrinsics;
  marg_feature_t *r_features;
  marg_camera_params_t *r_cam_params;
  marg_time_delay_t *r_time_delays;
  // -- Marginal parameters
  marg_pos_t *m_positions;
  marg_rot_t *m_rotations;
  marg_pose_t *m_poses;
  marg_velocity_t *m_velocities;
  marg_imu_biases_t *m_imu_biases;
  marg_feature_t *m_features;
  marg_fiducial_t *m_fiducials;
  marg_extrinsic_t *m_extrinsics;
  marg_joint_t *m_joints;
  marg_camera_params_t *m_cam_params;
  marg_time_delay_t *m_time_delays;

  // Factors
  list_t *ba_factors;
  list_t *camera_factors;
  list_t *idf_factors;
  list_t *imu_factors;
  list_t *calib_camera_factors;
  list_t *calib_imucam_factors;
  struct marg_factor_t *marg_factor;

  // Hessian, Jacobians and residuals
  param_order_t *hash;
  int m_size;
  int r_size;

  real_t *x0;
  real_t *r0;
  real_t *J0;
  real_t *J0_inv;
  real_t *dchi;
  real_t *J0_dchi;

  real_t *H;
  real_t *b;
  real_t *H_marg;
  real_t *b_marg;

  // Parameters, residuals and Jacobians (needed by the solver)
  int num_params;
  int *param_types;
  void **param_ptrs;
  real_t **params;
  real_t *r;
  real_t **jacs;

  // Profiling
  real_t time_hessian_form;
  real_t time_schur_complement;
  real_t time_hessian_decomp;
  real_t time_fejs;
  real_t time_total;
} marg_factor_t;

marg_factor_t *marg_factor_malloc(void);
void marg_factor_free(marg_factor_t *marg);
void marg_factor_print_stats(const marg_factor_t *marg);
void marg_factor_add(marg_factor_t *marg, int factor_type, void *factor_ptr);
void marg_factor_marginalize(marg_factor_t *marg);
int marg_factor_eval(void *marg_ptr);

////////////////
// DATA UTILS //
////////////////

pose_t *load_poses(const char *fp, int *num_poses);
int **assoc_pose_data(pose_t *gnd_poses,
                      size_t num_gnd_poses,
                      pose_t *est_poses,
                      size_t num_est_poses,
                      double threshold,
                      size_t *num_matches);

////////////
// SOLVER //
////////////

#define SOLVER_USE_SUITESPARSE

#define SOLVER_EVAL_FACTOR_COMPACT(HASH,                                       \
                                   SV_SIZE,                                    \
                                   H,                                          \
                                   G,                                          \
                                   FACTOR_EVAL,                                \
                                   FACTOR_PTR,                                 \
                                   R,                                          \
                                   R_IDX)                                      \
  FACTOR_EVAL(FACTOR_PTR);                                                     \
  vec_copy(FACTOR_PTR->r, FACTOR_PTR->r_size, &R[R_IDX]);                      \
  R_IDX += FACTOR_PTR->r_size;                                                 \
  solver_fill_hessian(HASH,                                                    \
                      FACTOR_PTR->num_params,                                  \
                      FACTOR_PTR->params,                                      \
                      FACTOR_PTR->jacs,                                        \
                      FACTOR_PTR->r,                                           \
                      FACTOR_PTR->r_size,                                      \
                      SV_SIZE,                                                 \
                      H,                                                       \
                      G);

typedef struct solver_t {
  // Settings
  int verbose;
  int max_iter;
  real_t lambda;
  real_t lambda_factor;

  // Data
  param_order_t *hash;
  int linearize;
  int r_size;
  int sv_size;
  real_t *H_damped;
  real_t *H;
  real_t *g;
  real_t *r;
  real_t *dx;

  // SuiteSparse
#ifdef SOLVER_USE_SUITESPARSE
  cholmod_common *common;
#endif

  // Callbacks
  param_order_t *(*param_order_func)(const void *data,
                                     int *sv_size,
                                     int *r_size);
  void (*cost_func)(const void *data, real_t *r);
  void (*linearize_func)(const void *data,
                         const int sv_size,
                         param_order_t *hash,
                         real_t *H,
                         real_t *g,
                         real_t *r);
  void (*linsolve_func)(const void *data,
                        const int sv_size,
                        param_order_t *hash,
                        real_t *H,
                        real_t *g,
                        real_t *dx);
} solver_t;

void solver_setup(solver_t *solver);
void solver_print_param_order(const solver_t *solver);
real_t solver_cost(const solver_t *solver, const void *data);
void solver_fill_jacobian(param_order_t *hash,
                          int num_params,
                          real_t **params,
                          real_t **jacs,
                          real_t *r,
                          int r_size,
                          int sv_size,
                          int J_row_idx,
                          real_t *J,
                          real_t *g);
void solver_fill_hessian(param_order_t *hash,
                         int num_params,
                         real_t **params,
                         real_t **jacs,
                         real_t *r,
                         int r_size,
                         int sv_size,
                         real_t *H,
                         real_t *g);
real_t **solver_params_copy(const solver_t *solver);
void solver_params_restore(solver_t *solver, real_t **x);
void solver_params_free(const solver_t *solver, real_t **x);
void solver_update(solver_t *solver, real_t *dx, int sv_size);
int solver_solve(solver_t *solver, void *data);

///////////////////////
// INERTIAL ODOMETRY //
///////////////////////

typedef struct inertial_odometry_t {
  // IMU Parameters
  imu_params_t imu_params;

  // Factors
  int num_factors;
  imu_factor_t *factors;
  marg_factor_t *marg;

  // Variables
  pose_t *poses;
  velocity_t *vels;
  imu_biases_t *biases;
} inertial_odometry_t;

inertial_odometry_t *inertial_odometry_malloc(void);
void inertial_odometry_free(inertial_odometry_t *odom);
void inertial_odometry_save(const inertial_odometry_t *odom,
                            const char *save_path);
param_order_t *inertial_odometry_param_order(const void *data,
                                             int *sv_size,
                                             int *r_size);
void inertial_odometry_cost(const void *data, real_t *r);
void inertial_odometry_linearize_compact(const void *data,
                                         const int sv_size,
                                         param_order_t *hash,
                                         real_t *H,
                                         real_t *g,
                                         real_t *r);

/////////////////////////////
// RELATIVE POSE ESTIMATOR //
/////////////////////////////

int relpose_estimator(const int num_cams,
                      const camera_params_t **cam_params,
                      const real_t **cam_exts,
                      const size_t **fids,
                      const real_t **kps,
                      const int *num_kps,
                      const feature_map_t *feature_map,
                      const real_t T_WB_km1[4 * 4],
                      real_t T_WB_k[4 * 4]);

////////////////////////////
// TWO-STATE FILTER (TSF) //
////////////////////////////

#define TSF_FRAME_LIMIT 1000
#define TSF_EST_MODE 3

/** TSF Frameset **/
typedef struct tsf_frameset_t {
  timestamp_t ts;

  size_t cam0_fids[TSF_FRAME_LIMIT];
  real_t cam0_kps[TSF_FRAME_LIMIT * 3];
  int cam0_num_kps;

  size_t cam1_fids[TSF_FRAME_LIMIT];
  real_t cam1_kps[TSF_FRAME_LIMIT * 3];
  int cam1_num_kps;
} tsf_frameset_t;

/** Two-State Filter (TSF) **/
typedef struct tsf_t {
  // Flags
  int state;
  int num_imus;
  int num_cams;
  int imu_started;
  ssize_t frame_idx;

  // Settings
  int fix_cam_params;
  int fix_cam_exts;
  int fix_imu_ext;
  int fix_time_delay;

  // IMU
  imu_params_t imu_params;
  imu_buffer_t imu_buf;
  extrinsic_t imu_ext;
  time_delay_t time_delay;

  // Vision
  camera_params_t *cam_params;
  extrinsic_t *cam_exts;
  tsf_frameset_t fs_km1;
  tsf_frameset_t fs_k;
  feature_map_t *feature_map;

  // Factors
  imu_factor_t imu_factor;
  marg_factor_t *marg;

  // State
  real_t pose_init[7];
  real_t vel_init[3];
  real_t ba_init[3];
  real_t bg_init[3];
  timestamp_t ts_i;
  timestamp_t ts_j;
  pose_t pose_i;
  pose_t pose_j;
  velocity_t vel_i;
  velocity_t vel_j;
  imu_biases_t biases_i;
  imu_biases_t biases_j;
} tsf_t;

void tsf_frameset_setup(tsf_frameset_t *fs);
void tsf_frameset_reset(tsf_frameset_t *fs);

tsf_t *tsf_malloc(void);
void tsf_free(tsf_t *tsf);
void tsf_print(const tsf_t *tsf);

void tsf_set_init_pose(tsf_t *tsf, real_t pose[7]);
void tsf_set_init_velocity(tsf_t *tsf, real_t vel[3]);
void tsf_add_camera(tsf_t *tsf,
                    const int cam_idx,
                    const int cam_res[2],
                    const char *proj_model,
                    const char *dist_model,
                    const real_t *cam_params,
                    const real_t *cam_ext);
void tsf_add_imu(tsf_t *tsf,
                 const real_t imu_rate,
                 const real_t sigma_aw,
                 const real_t sigma_gw,
                 const real_t sigma_a,
                 const real_t sigma_g,
                 const real_t g,
                 const real_t *imu_ext);

void tsf_imu_event(tsf_t *tsf,
                   const timestamp_t ts,
                   const real_t acc[3],
                   const real_t gyr[3]);
void tsf_camera_event(tsf_t *tsf,
                      const timestamp_t ts,
                      const size_t *cam0_fids,
                      const real_t *cam0_kps,
                      const int num_cam0_kps,
                      const size_t *cam1_fids,
                      const real_t *cam1_kps,
                      const int num_cam1_kps);

void tsf_cost(const void *data, real_t *r);
void tsf_errors(const tsf_t *tsf,
                real_t *reproj_rmse,
                real_t *reproj_mean,
                real_t *reproj_median);
param_order_t *tsf_param_order(const void *data, int *sv_size, int *r_size);
void tsf_linearize_compact(const void *data,
                           const int sv_size,
                           param_order_t *hash,
                           real_t *H,
                           real_t *g,
                           real_t *r);
void tsf_update(tsf_t *tsf, const timestamp_t ts);
