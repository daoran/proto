#include "xyz_calib.h"

//////////////
// CAMCHAIN //
//////////////

/**
 * Allocate memory for the camchain initialzer.
 */
camchain_t *camchain_malloc(const int num_cams) {
  camchain_t *cc = malloc(sizeof(camchain_t) * 1);

  // Flags
  cc->analyzed = 0;
  cc->num_cams = num_cams;

  // Allocate memory for the adjacency list and extrinsics
  cc->adj_list = calloc(cc->num_cams, sizeof(int *));
  cc->adj_exts = calloc(cc->num_cams, sizeof(real_t *));
  for (int cam_idx = 0; cam_idx < cc->num_cams; cam_idx++) {
    cc->adj_list[cam_idx] = calloc(cc->num_cams, sizeof(int));
    cc->adj_exts[cam_idx] = calloc(cc->num_cams * (4 * 4), sizeof(real_t));
  }

  // Allocate memory for camera poses
  cc->cam_poses = calloc(num_cams, sizeof(camchain_pose_hash_t *));
  for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
    cc->cam_poses[cam_idx] = NULL;
    hmdefault(cc->cam_poses[cam_idx], NULL);
  }

  return cc;
}

/**
 * Free camchain initialzer.
 */
void camchain_free(camchain_t *cc) {
  // Adjacency list and extrinsic
  for (int cam_idx = 0; cam_idx < cc->num_cams; cam_idx++) {
    free(cc->adj_list[cam_idx]);
    free(cc->adj_exts[cam_idx]);
  }
  free(cc->adj_list);
  free(cc->adj_exts);

  // Camera poses
  for (int cam_idx = 0; cam_idx < cc->num_cams; cam_idx++) {
    for (int k = 0; k < hmlen(cc->cam_poses[cam_idx]); k++) {
      free(cc->cam_poses[cam_idx][k].value);
    }
    hmfree(cc->cam_poses[cam_idx]);
  }
  free(cc->cam_poses);

  // Finish
  free(cc);
}

/**
 * Add camera pose to camchain.
 */
void camchain_add_pose(camchain_t *cc,
                       const int cam_idx,
                       const timestamp_t ts,
                       const real_t T_CiF[4 * 4]) {
  real_t *tf = malloc(sizeof(real_t) * 4 * 4);
  mat_copy(T_CiF, 4, 4, tf);
  hmput(cc->cam_poses[cam_idx], ts, tf);
}

/**
 * Form camchain adjacency list.
 */
void camchain_adjacency(camchain_t *cc) {
  // Iterate through camera i data
  for (int cam_i = 0; cam_i < cc->num_cams; cam_i++) {
    for (int k = 0; k < hmlen(cc->cam_poses[cam_i]); k++) {
      const timestamp_t ts_i = cc->cam_poses[cam_i][k].key;
      const real_t *T_CiF = hmgets(cc->cam_poses[cam_i], ts_i).value;

      // Iterate through camera j data
      for (int cam_j = cam_i + 1; cam_j < cc->num_cams; cam_j++) {
        // Check if a link has already been discovered
        if (cc->adj_list[cam_i][cam_j] == 1) {
          continue;
        }

        // Check if a link exists between camera i and j in the data
        const real_t *T_CjF = hmgets(cc->cam_poses[cam_j], ts_i).value;
        if (T_CjF == NULL) {
          continue;
        }

        // TODO: Maybe move this outside this loop and collect
        // mutliple measurements and use the median to form T_CiCj and T_CjCi?
        // Form T_CiCj and T_CjCi
        TF_INV(T_CjF, T_FCj);
        TF_INV(T_CiF, T_FCi);
        TF_CHAIN(T_CiCj, 2, T_CiF, T_FCj);
        TF_CHAIN(T_CjCi, 2, T_CjF, T_FCi);

        // Add link between camera i and j
        cc->adj_list[cam_i][cam_j] = 1;
        cc->adj_list[cam_j][cam_i] = 1;
        mat_copy(T_CiCj, 4, 4, &cc->adj_exts[cam_i][cam_j * (4 * 4)]);
        mat_copy(T_CjCi, 4, 4, &cc->adj_exts[cam_j][cam_i * (4 * 4)]);
      }
    }
  }

  // Mark camchain as analyzed
  cc->analyzed = 1;
}

/**
 * Print camchain adjacency matrix.
 */
void camchain_adjacency_print(const camchain_t *cc) {
  for (int i = 0; i < cc->num_cams; i++) {
    printf("%d: ", i);
    for (int j = 0; j < cc->num_cams; j++) {
      printf("%d ", cc->adj_list[i][j]);
    }
    printf("\n");
  }
}

/**
 * The purpose of camchain initializer is to find the initial camera
 * to camera extrinsic of arbitrary cameras. So lets say you are calibrating a
 * N multi-camera rig observing the same calibration fiducial target (F). The
 * idea is as you add the relative pose between the i-th camera (Ci) and
 * fiducial target (F), the camchain initialzer will build an adjacency matrix
 * and form all possible camera-camera extrinsic combinations. This is useful
 * for multi-camera extrinsics where you need to initialize the
 * camera-extrinsic parameter.
 *
 * Usage:
 *
 *   camchain_t *camchain = camchain_malloc(num_cams);
 *   for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
 *     for (int ts_idx = 0; ts_idx < len(camera_poses); ts_idx++) {
 *       timestamp_t ts = camera_timestamps[ts_idx];
 *       real_t *T_CiF = camera_poses[cam_idx][ts_idx];
 *       camchain_add_pose(camchain, cam_idx, ts, T_CiF);
 *     }
 *   }
 *   camchain_adjacency(camchain);
 *   camchain_adjacency_print(camchain);
 *   camchain_find(camchain, cam_i, cam_j, T_CiCj);
 *
 */
int camchain_find(camchain_t *cc,
                  const int cam_i,
                  const int cam_j,
                  real_t T_CiCj[4 * 4]) {
  // Form adjacency
  if (cc->analyzed == 0) {
    camchain_adjacency(cc);
  }

  // Straight forward case where extrinsic of itself is identity
  if (cam_i == cam_j) {
    if (hmlen(cc->cam_poses[cam_i])) {
      eye(T_CiCj, 4, 4);
      return 0;
    } else {
      return -1;
    }
  }

  // Check if T_CiCj was formed before
  if (cc->adj_list[cam_i][cam_j] == 1) {
    mat_copy(&cc->adj_exts[cam_i][cam_j * (4 * 4)], 4, 4, T_CiCj);
    return 0;
  }

  return -1;
}

/////////////////////////
// CALIB-CAMERA FACTOR //
/////////////////////////

/**
 * Setup camera calibration factor
 */
void calib_camera_factor_setup(calib_camera_factor_t *factor,
                               pose_t *pose,
                               extrinsic_t *cam_ext,
                               camera_params_t *cam_params,
                               const int cam_idx,
                               const int tag_id,
                               const int corner_idx,
                               const real_t p_FFi[3],
                               const real_t z[2],
                               const real_t var[2]) {
  assert(factor != NULL);
  assert(pose != NULL);
  assert(cam_ext != NULL);
  assert(cam_params != NULL);
  assert(z != NULL);
  assert(var != NULL);

  // Parameters
  factor->pose = pose;
  factor->cam_ext = cam_ext;
  factor->cam_params = cam_params;
  factor->num_params = 3;

  // Measurement
  factor->cam_idx = cam_idx;
  factor->tag_id = tag_id;
  factor->corner_idx = corner_idx;
  factor->p_FFi[0] = p_FFi[0];
  factor->p_FFi[1] = p_FFi[1];
  factor->p_FFi[2] = p_FFi[2];
  factor->z[0] = z[0];
  factor->z[1] = z[1];

  // Measurement covariance matrix
  factor->covar[0] = var[0];
  factor->covar[1] = 0.0;
  factor->covar[2] = 0.0;
  factor->covar[3] = var[1];

  // Square-root information matrix
  factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);
  factor->sqrt_info[1] = 0.0;
  factor->sqrt_info[2] = 0.0;
  factor->sqrt_info[3] = sqrt(1.0 / factor->covar[3]);

  // Factor residuals, parameters and Jacobians
  factor->r_size = 2;
  factor->param_types[0] = POSE_PARAM;
  factor->param_types[1] = EXTRINSIC_PARAM;
  factor->param_types[2] = CAMERA_PARAM;

  factor->params[0] = factor->pose->data;
  factor->params[1] = factor->cam_ext->data;
  factor->params[2] = factor->cam_params->data;

  factor->jacs[0] = factor->J_pose;
  factor->jacs[1] = factor->J_cam_ext;
  factor->jacs[2] = factor->J_cam_params;
}

int calib_camera_factor_eval(void *factor_ptr) {
  // Map factor
  calib_camera_factor_t *factor = (calib_camera_factor_t *) factor_ptr;
  assert(factor != NULL);

  // Map params
  const real_t *p_FFi = factor->p_FFi;
  TF(factor->params[0], T_BF);                  // Relative pose T_BF
  TF(factor->params[1], T_BCi);                 // Camera extrinsic T_BCi
  const real_t *cam_params = factor->params[2]; // Camera parameters

  // Form T_CiF
  TF_INV(T_BCi, T_CiB);
  TF_CHAIN(T_CiF, 2, T_CiB, T_BF);

  // Project to image plane
  int status = 0;
  real_t z_hat[2];
  TF_POINT(T_CiF, factor->p_FFi, p_CiFi);
  camera_project(factor->cam_params, p_CiFi, z_hat);
  const int res_x = factor->cam_params->resolution[0];
  const int res_y = factor->cam_params->resolution[1];
  const int x_ok = (z_hat[0] > 0 && z_hat[0] < res_x);
  const int y_ok = (z_hat[1] > 0 && z_hat[1] < res_y);
  const int z_ok = p_CiFi[2] > 0;
  if (x_ok && y_ok && z_ok) {
    status = 1;
  }

  // Calculate residuals
  real_t r[2] = {0, 0};
  r[0] = factor->z[0] - z_hat[0];
  r[1] = factor->z[1] - z_hat[1];
  dot(factor->sqrt_info, 2, 2, r, 2, 1, factor->r);

  // Calculate Jacobians
  // -- Zero out jacobians if reprojection is not valid
  if (status == 0) {
    zeros(factor->J_pose, 2, 6);
    zeros(factor->J_cam_ext, 2, 6);
    zeros(factor->J_cam_params, 2, 8);
    return 0;
  }
  // Form: -1 * sqrt_info
  real_t neg_sqrt_info[2 * 2] = {0};
  mat_copy(factor->sqrt_info, 2, 2, neg_sqrt_info);
  mat_scale(neg_sqrt_info, 2, 2, -1.0);
  // Form: Jh_w = -1 * sqrt_info * Jh
  real_t Jh[2 * 3] = {0};
  real_t Jh_w[2 * 3] = {0};
  pinhole_radtan4_project_jacobian(cam_params, p_CiFi, Jh);
  dot(neg_sqrt_info, 2, 2, Jh, 2, 3, Jh_w);
  // Form: J_cam_params
  real_t J_cam_params[2 * 8] = {0};
  pinhole_radtan4_params_jacobian(cam_params, p_CiFi, J_cam_params);

  // -- Jacobians w.r.t relative camera pose T_BF
  {
    // J_pos = Jh * C_CiB
    real_t J_pos[2 * 3] = {0};
    real_t C_CiB[3 * 3] = {0};
    tf_rot_get(T_CiB, C_CiB);
    dot(Jh_w, 2, 3, C_CiB, 3, 3, J_pos);
    factor->jacs[0][0] = J_pos[0];
    factor->jacs[0][1] = J_pos[1];
    factor->jacs[0][2] = J_pos[2];

    factor->jacs[0][6] = J_pos[3];
    factor->jacs[0][7] = J_pos[4];
    factor->jacs[0][8] = J_pos[5];

    // J_rot = Jh * C_CiB * -C_BF @ hat(p_FFi)
    real_t C_BF[3 * 3] = {0};
    real_t C_CiF[3 * 3] = {0};
    tf_rot_get(T_BF, C_BF);
    dot(C_CiB, 3, 3, C_BF, 3, 3, C_CiF);
    mat_scale(C_CiF, 3, 3, -1);

    real_t J_rot[2 * 3] = {0};
    real_t p_FFi_x[3 * 3] = {0};
    hat(p_FFi, p_FFi_x);
    dot3(Jh_w, 2, 3, C_CiF, 3, 3, p_FFi_x, 3, 3, J_rot);

    factor->jacs[0][3] = J_rot[0];
    factor->jacs[0][4] = J_rot[1];
    factor->jacs[0][5] = J_rot[2];

    factor->jacs[0][9] = J_rot[3];
    factor->jacs[0][10] = J_rot[4];
    factor->jacs[0][11] = J_rot[5];
  }

  // -- Jacobians w.r.t camera extrinsic T_BCi
  {
    // J_pos = Jh * -C_CiB
    real_t J_pos[2 * 3] = {0};
    real_t nC_CiB[3 * 3] = {0};
    tf_rot_get(T_CiB, nC_CiB);
    mat_scale(nC_CiB, 3, 3, -1.0);
    dot(Jh_w, 2, 3, nC_CiB, 3, 3, J_pos);
    factor->jacs[1][0] = J_pos[0];
    factor->jacs[1][1] = J_pos[1];
    factor->jacs[1][2] = J_pos[2];

    factor->jacs[1][6] = J_pos[3];
    factor->jacs[1][7] = J_pos[4];
    factor->jacs[1][8] = J_pos[5];

    // J_rot = Jh * -C_CiB * hat(r_BFi - r_BCi) * -C_BCi
    real_t J_rot[2 * 3] = {0};
    real_t r_BFi[3] = {0};
    real_t r_BCi[3] = {0};
    real_t dr[3] = {0};
    real_t hdr[3 * 3] = {0};
    real_t nC_BCi[3 * 3] = {0};

    tf_point(T_BF, p_FFi, r_BFi);
    tf_trans_get(T_BCi, r_BCi);
    dr[0] = r_BFi[0] - r_BCi[0];
    dr[1] = r_BFi[1] - r_BCi[1];
    dr[2] = r_BFi[2] - r_BCi[2];
    hat(dr, hdr);
    tf_rot_get(T_BCi, nC_BCi);
    mat_scale(nC_BCi, 3, 3, -1.0);

    real_t B[3 * 3] = {0};
    dot(hdr, 3, 3, nC_BCi, 3, 3, B);
    dot(J_pos, 2, 3, B, 3, 3, J_rot);

    factor->jacs[1][3] = J_rot[0];
    factor->jacs[1][4] = J_rot[1];
    factor->jacs[1][5] = J_rot[2];

    factor->jacs[1][9] = J_rot[3];
    factor->jacs[1][10] = J_rot[4];
    factor->jacs[1][11] = J_rot[5];
  }

  // -- Jacobians w.r.t. camera parameters
  dot(neg_sqrt_info, 2, 2, J_cam_params, 2, 8, factor->jacs[2]);

  return 0;
}

int calib_camera_factor_ceres_eval(void *factor_ptr,
                                   real_t **params,
                                   real_t *r_out,
                                   real_t **J_out) {
  CERES_FACTOR_EVAL(calib_camera_factor,
                    ((calib_camera_factor_t *) factor_ptr),
                    calib_camera_factor_eval,
                    params,
                    r_out,
                    J_out);
}

/////////////////////////
// CALIB-IMUCAM FACTOR //
/////////////////////////

/**
 * Setup imu-camera time-delay calibration factor
 */
void calib_imucam_factor_setup(calib_imucam_factor_t *factor,
                               fiducial_t *fiducial,
                               pose_t *imu_pose,
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
                               const real_t var[2]) {
  assert(factor != NULL);
  assert(fiducial != NULL);
  assert(imu_pose != NULL);
  assert(imu_ext != NULL);
  assert(cam_ext != NULL);
  assert(cam_params != NULL);
  assert(time_delay != NULL);
  assert(p_FFi != NULL);
  assert(z != NULL);
  assert(v != NULL);
  assert(var != NULL);

  // Parameters
  factor->fiducial = fiducial;
  factor->imu_pose = imu_pose;
  factor->imu_ext = imu_ext;
  factor->cam_ext = cam_ext;
  factor->cam_params = cam_params;
  factor->time_delay = time_delay;
  factor->num_params = 6;

  // Measurement
  factor->cam_idx = cam_idx;
  factor->tag_id = tag_id;
  factor->corner_idx = corner_idx;
  factor->p_FFi[0] = p_FFi[0];
  factor->p_FFi[1] = p_FFi[1];
  factor->p_FFi[2] = p_FFi[2];
  factor->z[0] = z[0];
  factor->z[1] = z[1];
  factor->v[0] = v[0];
  factor->v[1] = v[1];

  // Measurement covariance matrix
  factor->covar[0] = var[0];
  factor->covar[1] = 0.0;
  factor->covar[2] = 0.0;
  factor->covar[3] = var[1];

  // Square-root information matrix
  factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);
  factor->sqrt_info[1] = 0.0;
  factor->sqrt_info[2] = 0.0;
  factor->sqrt_info[3] = sqrt(1.0 / factor->covar[3]);

  // Factor residuals, parameters and Jacobians
  factor->r_size = 2;
  factor->param_types[0] = POSE_PARAM;
  factor->param_types[1] = POSE_PARAM;
  factor->param_types[2] = EXTRINSIC_PARAM;
  factor->param_types[3] = EXTRINSIC_PARAM;
  factor->param_types[4] = CAMERA_PARAM;
  factor->param_types[5] = TIME_DELAY_PARAM;

  factor->params[0] = factor->fiducial->data;
  factor->params[1] = factor->imu_pose->data;
  factor->params[2] = factor->imu_ext->data;
  factor->params[3] = factor->cam_ext->data;
  factor->params[4] = factor->cam_params->data;
  factor->params[5] = factor->time_delay->data;

  factor->jacs[0] = factor->J_fiducial;
  factor->jacs[1] = factor->J_imu_pose;
  factor->jacs[2] = factor->J_imu_ext;
  factor->jacs[3] = factor->J_cam_ext;
  factor->jacs[4] = factor->J_cam_params;
  factor->jacs[5] = factor->J_time_delay;
}

int calib_imucam_factor_eval(void *factor_ptr) {
  // Map factor
  calib_imucam_factor_t *factor = (calib_imucam_factor_t *) factor_ptr;
  assert(factor != NULL);

  // Map params
  TF(factor->params[0], T_WF);                  // Fiducial T_WF
  TF(factor->params[1], T_WS);                  // IMU Pose T_WS
  TF(factor->params[2], T_SC0);                 // IMU extrinsic T_SC0
  TF(factor->params[3], T_C0Ci);                // Camera extrinsic T_C0Ci
  const real_t *cam_params = factor->params[4]; // Camera parameters
  const real_t td = factor->params[5][0];       // Time delay

  // Form T_CiW and T_CiF
  // T_CiW = inv(T_C0Ci) * inv(T_SC0) * inv(T_WS) * T_WF
  TF_INV(T_WS, T_SW);
  TF_INV(T_SC0, T_C0S);
  TF_INV(T_C0Ci, T_CiC0);
  TF_CHAIN(T_CiS, 2, T_CiC0, T_C0S);
  TF_CHAIN(T_CiW, 2, T_CiS, T_SW);
  TF_CHAIN(T_CiF, 2, T_CiW, T_WF);

  // Project to image plane
  real_t z_hat[2];
  TF_POINT(T_CiF, factor->p_FFi, p_CiFi);
  camera_project(factor->cam_params, p_CiFi, z_hat);

  // Calculate residuals
  real_t r[2] = {0, 0};
  r[0] = (factor->z[0] + td * factor->v[0]) - z_hat[0];
  r[1] = (factor->z[1] + td * factor->v[1]) - z_hat[1];
  dot(factor->sqrt_info, 2, 2, r, 2, 1, factor->r);

  // Calculate Jacobians
  // Form: -1 * sqrt_info
  real_t sqrt_info[2 * 2] = {0};
  real_t neg_sqrt_info[2 * 2] = {0};
  mat_copy(factor->sqrt_info, 2, 2, sqrt_info);
  mat_copy(factor->sqrt_info, 2, 2, neg_sqrt_info);
  mat_scale(neg_sqrt_info, 2, 2, -1.0);
  // Form: Jh_w = -1 * sqrt_info * Jh
  real_t Jh[2 * 3] = {0};
  real_t Jh_w[2 * 3] = {0};
  pinhole_radtan4_project_jacobian(cam_params, p_CiFi, Jh);
  dot(neg_sqrt_info, 2, 2, Jh, 2, 3, Jh_w);
  // Form: J_cam_params
  real_t J_cam_params[2 * 8] = {0};
  pinhole_radtan4_params_jacobian(cam_params, p_CiFi, J_cam_params);

  // -- Jacobians w.r.t fiducial pose T_WF
  {
    // J_pos = Jh * C_CiW
    real_t J_pos[2 * 3] = {0};
    real_t C_CiW[3 * 3] = {0};
    tf_rot_get(T_CiW, C_CiW);
    dot(Jh_w, 2, 3, C_CiW, 3, 3, J_pos);
    factor->jacs[0][0] = J_pos[0];
    factor->jacs[0][1] = J_pos[1];
    factor->jacs[0][2] = J_pos[2];

    factor->jacs[0][6] = J_pos[3];
    factor->jacs[0][7] = J_pos[4];
    factor->jacs[0][8] = J_pos[5];

    // J_rot = Jh * C_CiW * -C_WF @ hat(p_FFi)
    real_t C_WF[3 * 3] = {0};
    real_t C_CiF[3 * 3] = {0};
    tf_rot_get(T_WF, C_WF);
    dot(C_CiW, 3, 3, C_WF, 3, 3, C_CiF);
    mat_scale(C_CiF, 3, 3, -1);

    real_t J_rot[2 * 3] = {0};
    real_t p_FFi_x[3 * 3] = {0};
    hat(factor->p_FFi, p_FFi_x);
    dot3(Jh_w, 2, 3, C_CiF, 3, 3, p_FFi_x, 3, 3, J_rot);

    factor->jacs[0][3] = J_rot[0];
    factor->jacs[0][4] = J_rot[1];
    factor->jacs[0][5] = J_rot[2];

    factor->jacs[0][9] = J_rot[3];
    factor->jacs[0][10] = J_rot[4];
    factor->jacs[0][11] = J_rot[5];
  }

  // -- Jacobians w.r.t IMU pose T_WS
  {
    // J_pos = Jh * -C_CiW
    real_t J_pos[2 * 3] = {0};
    real_t nC_CiW[3 * 3] = {0};
    tf_rot_get(T_CiW, nC_CiW);
    mat_scale(nC_CiW, 3, 3, -1.0);
    dot(Jh_w, 2, 3, nC_CiW, 3, 3, J_pos);
    factor->jacs[1][0] = J_pos[0];
    factor->jacs[1][1] = J_pos[1];
    factor->jacs[1][2] = J_pos[2];

    factor->jacs[1][6] = J_pos[3];
    factor->jacs[1][7] = J_pos[4];
    factor->jacs[1][8] = J_pos[5];

    // J_rot = Jh * -C_CiW * hat(p_WFi - r_WS) * -C_WS
    real_t r_WS[3] = {0};
    real_t dp[3] = {0};
    real_t dp_x[3 * 3] = {0};
    real_t p_WFi[3] = {0};
    tf_trans_get(T_WS, r_WS);
    tf_point(T_WF, factor->p_FFi, p_WFi);
    dp[0] = p_WFi[0] - r_WS[0];
    dp[1] = p_WFi[1] - r_WS[1];
    dp[2] = p_WFi[2] - r_WS[2];
    hat(dp, dp_x);

    real_t nC_WS[3 * 3] = {0};
    tf_rot_get(T_WS, nC_WS);
    mat_scale(nC_WS, 3, 3, -1.0);

    real_t J_rot[2 * 3] = {0};
    dot3(J_pos, 2, 3, dp_x, 3, 3, nC_WS, 3, 3, J_rot);

    factor->jacs[1][3] = J_rot[0];
    factor->jacs[1][4] = J_rot[1];
    factor->jacs[1][5] = J_rot[2];

    factor->jacs[1][9] = J_rot[3];
    factor->jacs[1][10] = J_rot[4];
    factor->jacs[1][11] = J_rot[5];
  }

  // -- Jacobians w.r.t IMU extrinsic T_SC0
  {
    // J_pos = Jh * -C_CiS
    real_t J_pos[2 * 3] = {0};
    real_t nC_CiS[3 * 3] = {0};
    tf_rot_get(T_CiS, nC_CiS);
    mat_scale(nC_CiS, 3, 3, -1.0);
    dot(Jh_w, 2, 3, nC_CiS, 3, 3, J_pos);
    factor->jacs[2][0] = J_pos[0];
    factor->jacs[2][1] = J_pos[1];
    factor->jacs[2][2] = J_pos[2];

    factor->jacs[2][6] = J_pos[3];
    factor->jacs[2][7] = J_pos[4];
    factor->jacs[2][8] = J_pos[5];

    // J_rot = Jh * -C_CiS * hat(p_SFi - r_SC0) * -C_SC0
    real_t r_SC0[3] = {0};
    real_t dp[3] = {0};
    real_t dp_x[3 * 3] = {0};
    real_t p_SFi[3] = {0};
    tf_trans_get(T_SC0, r_SC0);
    TF_CHAIN(T_SF, 2, T_SW, T_WF);
    tf_point(T_SF, factor->p_FFi, p_SFi);
    dp[0] = p_SFi[0] - r_SC0[0];
    dp[1] = p_SFi[1] - r_SC0[1];
    dp[2] = p_SFi[2] - r_SC0[2];
    hat(dp, dp_x);

    real_t nC_SC0[3 * 3] = {0};
    tf_rot_get(T_SC0, nC_SC0);
    mat_scale(nC_SC0, 3, 3, -1.0);

    real_t J_rot[2 * 3] = {0};
    dot3(J_pos, 2, 3, dp_x, 3, 3, nC_SC0, 3, 3, J_rot);

    factor->jacs[2][3] = J_rot[0];
    factor->jacs[2][4] = J_rot[1];
    factor->jacs[2][5] = J_rot[2];

    factor->jacs[2][9] = J_rot[3];
    factor->jacs[2][10] = J_rot[4];
    factor->jacs[2][11] = J_rot[5];
  }

  // -- Jacobians w.r.t camera extrinsic T_C0Ci
  {
    // J_pos = Jh * -C_CiC0
    real_t J_pos[2 * 3] = {0};
    real_t nC_CiC0[3 * 3] = {0};
    tf_rot_get(T_CiC0, nC_CiC0);
    mat_scale(nC_CiC0, 3, 3, -1.0);
    dot(Jh_w, 2, 3, nC_CiC0, 3, 3, J_pos);
    factor->jacs[3][0] = J_pos[0];
    factor->jacs[3][1] = J_pos[1];
    factor->jacs[3][2] = J_pos[2];

    factor->jacs[3][6] = J_pos[3];
    factor->jacs[3][7] = J_pos[4];
    factor->jacs[3][8] = J_pos[5];

    // J_rot = Jh * -C_CiC0 * hat(p_C0Fi - r_C0Ci) * -C_C0Ci
    real_t J_rot[2 * 3] = {0};
    real_t r_C0Fi[3] = {0};
    real_t r_C0Ci[3] = {0};
    real_t dr[3] = {0};
    real_t hdr[3 * 3] = {0};
    real_t nC_C0Ci[3 * 3] = {0};

    TF_CHAIN(T_C0F, 2, T_C0Ci, T_CiF);
    tf_point(T_C0F, factor->p_FFi, r_C0Fi);
    tf_trans_get(T_C0Ci, r_C0Ci);
    dr[0] = r_C0Fi[0] - r_C0Ci[0];
    dr[1] = r_C0Fi[1] - r_C0Ci[1];
    dr[2] = r_C0Fi[2] - r_C0Ci[2];
    hat(dr, hdr);
    tf_rot_get(T_C0Ci, nC_C0Ci);
    mat_scale(nC_C0Ci, 3, 3, -1.0);

    real_t B[3 * 3] = {0};
    dot(hdr, 3, 3, nC_C0Ci, 3, 3, B);
    dot(J_pos, 2, 3, B, 3, 3, J_rot);

    factor->jacs[3][3] = J_rot[0];
    factor->jacs[3][4] = J_rot[1];
    factor->jacs[3][5] = J_rot[2];

    factor->jacs[3][9] = J_rot[3];
    factor->jacs[3][10] = J_rot[4];
    factor->jacs[3][11] = J_rot[5];
  }

  // -- Jacobians w.r.t. camera parameters
  dot(neg_sqrt_info, 2, 2, J_cam_params, 2, 8, factor->jacs[4]);

  // -- Jacobians w.r.t. time delay
  real_t J_time_delay[2 * 1] = {factor->v[0], factor->v[1]};
  dot(sqrt_info, 2, 2, J_time_delay, 2, 1, factor->jacs[5]);

  return 0;
}

int calib_imucam_factor_ceres_eval(void *factor_ptr,
                                   real_t **params,
                                   real_t *r_out,
                                   real_t **J_out) {
  CERES_FACTOR_EVAL(calib_imucam_factor,
                    ((calib_imucam_factor_t *) factor_ptr),
                    calib_imucam_factor_eval,
                    params,
                    r_out,
                    J_out);
}

////////////////////////
// CAMERA CALIBRATION //
////////////////////////

/**
 * Malloc camera calibration view.
 */
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
                                              camera_params_t *cam_params) {
  calib_camera_view_t *view = malloc(sizeof(calib_camera_view_t) * 1);

  // Properties
  view->ts = ts;
  view->view_idx = view_idx;
  view->cam_idx = cam_idx;
  view->num_corners = num_corners;

  // Measurements
  if (num_corners) {
    view->tag_ids = malloc(sizeof(int) * num_corners);
    view->corner_indices = malloc(sizeof(int) * num_corners);
    view->object_points = malloc(sizeof(real_t) * num_corners * 3);
    view->keypoints = malloc(sizeof(real_t) * num_corners * 2);
    assert(view->tag_ids != NULL);
    assert(view->corner_indices != NULL);
    assert(view->object_points != NULL);
    assert(view->keypoints != NULL);
  }

  // Factors
  view->factors = malloc(sizeof(struct calib_camera_factor_t) * num_corners);
  assert(view->factors != NULL);

  for (int i = 0; i < num_corners; i++) {
    view->tag_ids[i] = tag_ids[i];
    view->corner_indices[i] = corner_indices[i];
    view->object_points[i * 3] = object_points[i * 3];
    view->object_points[i * 3 + 1] = object_points[i * 3 + 1];
    view->object_points[i * 3 + 2] = object_points[i * 3 + 2];
    view->keypoints[i * 2] = keypoints[i * 2];
    view->keypoints[i * 2 + 1] = keypoints[i * 2 + 1];
  }

  const real_t var[2] = {1.0, 1.0};
  for (int i = 0; i < view->num_corners; i++) {
    const int tag_id = tag_ids[i];
    const int corner_idx = corner_indices[i];
    const real_t *p_FFi = &object_points[i * 3];
    const real_t *z = &keypoints[i * 2];

    view->tag_ids[i] = tag_id;
    view->corner_indices[i] = corner_idx;
    view->object_points[i * 3] = p_FFi[0];
    view->object_points[i * 3 + 1] = p_FFi[1];
    view->object_points[i * 3 + 2] = p_FFi[2];
    view->keypoints[i * 2] = z[0];
    view->keypoints[i * 2 + 1] = z[1];

    calib_camera_factor_setup(&view->factors[i],
                              pose,
                              cam_ext,
                              cam_params,
                              cam_idx,
                              tag_id,
                              corner_idx,
                              p_FFi,
                              z,
                              var);
  }

  return view;
}

/**
 * Free camera calibration view.
 */
void calib_camera_view_free(calib_camera_view_t *view) {
  if (view) {
    free(view->tag_ids);
    free(view->corner_indices);
    free(view->object_points);
    free(view->keypoints);
    free(view->factors);
    free(view);
  }
}

/**
 * Malloc camera calibration problem
 */
calib_camera_t *calib_camera_malloc(void) {
  calib_camera_t *calib = malloc(sizeof(calib_camera_t) * 1);

  // Settings
  calib->fix_cam_exts = 0;
  calib->fix_cam_params = 0;
  calib->verbose = 1;
  calib->max_iter = 20;

  // Flags
  calib->cams_ok = 0;

  // Counters
  calib->num_cams = 0;
  calib->num_views = 0;
  calib->num_factors = 0;

  // Variables
  calib->timestamps = NULL;
  calib->poses = NULL;
  calib->cam_exts = NULL;
  calib->cam_params = NULL;
  hmdefault(calib->poses, NULL);

  // Factors
  calib->view_sets = NULL;
  hmdefault(calib->view_sets, NULL);
  calib->marg = NULL;

  return calib;
}

/**
 * Free camera calibration problem
 */
void calib_camera_free(calib_camera_t *calib) {
  free(calib->cam_exts);
  free(calib->cam_params);

  if (calib->num_views) {
    // View sets
    for (int i = 0; i < arrlen(calib->timestamps); i++) {
      const timestamp_t ts = calib->timestamps[i];
      calib_camera_view_t **cam_views = hmgets(calib->view_sets, ts).value;
      for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
        calib_camera_view_free(cam_views[cam_idx]);
      }
      free(cam_views);
    }

    // Timestamps
    arrfree(calib->timestamps);

    // Poses
    for (int i = 0; i < hmlen(calib->poses); i++) {
      free(calib->poses[i].value);
    }
  }
  hmfree(calib->poses);
  hmfree(calib->view_sets);

  // Free previous marg_factor_t
  marg_factor_free(calib->marg);

  free(calib);
}

/**
 * Print camera calibration.
 */
void calib_camera_print(calib_camera_t *calib) {
  real_t reproj_rmse = 0.0;
  real_t reproj_mean = 0.0;
  real_t reproj_median = 0.0;
  calib_camera_errors(calib, &reproj_rmse, &reproj_mean, &reproj_median);

  printf("settings:\n");
  printf("  fix_cam_exts: %d\n", calib->fix_cam_exts);
  printf("  fix_cam_params: %d\n", calib->fix_cam_params);
  printf("\n");

  printf("statistics:\n");
  printf("  num_cams: %d\n", calib->num_cams);
  printf("  num_views: %d\n", calib->num_views);
  printf("  num_factors: %d\n", calib->num_factors);
  printf("\n");

  printf("reproj_errors:\n");
  printf("  rmse:   %f  # [px]\n", reproj_rmse);
  printf("  mean:   %f  # [px]\n", reproj_mean);
  printf("  median: %f  # [px]\n", reproj_median);
  printf("\n");

  for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
    camera_params_t *cam = &calib->cam_params[cam_idx];
    char param_str[100] = {0};
    vec2str(cam->data, 8, param_str);

    printf("cam%d:\n", cam_idx);
    printf("  resolution: [%d, %d]\n", cam->resolution[0], cam->resolution[1]);
    printf("  proj_model: %s\n", cam->proj_model);
    printf("  dist_model: %s\n", cam->dist_model);
    printf("  param: %s\n", param_str);
    printf("\n");

    if (cam_idx > 0) {
      char tf_str[20] = {0};
      sprintf(tf_str, "T_cam0_cam%d", cam_idx);

      POSE2TF(calib->cam_exts[cam_idx].data, T);
      printf("%s:\n", tf_str);
      printf("  rows: 4\n");
      printf("  cols: 4\n");
      printf("  data: [\n");
      printf("    %.8f, %.8f, %.8f, %.8f,\n", T[0], T[1], T[2], T[3]);
      printf("    %.8f, %.8f, %.8f, %.8f,\n", T[4], T[5], T[6], T[7]);
      printf("    %.8f, %.8f, %.8f, %.8f,\n", T[8], T[9], T[10], T[11]);
      printf("    %.8f, %.8f, %.8f, %.8f,\n", T[12], T[13], T[14], T[15]);
      printf("  ]\n");
    }
  }
}

/**
 * Add camera to camera calibration problem
 */
void calib_camera_add_camera(calib_camera_t *calib,
                             const int cam_idx,
                             const int cam_res[2],
                             const char *proj_model,
                             const char *dist_model,
                             const real_t *cam_params,
                             const real_t *cam_ext) {
  assert(calib != NULL);
  assert(cam_idx <= calib->num_cams);
  assert(cam_res != NULL);
  assert(proj_model != NULL);
  assert(dist_model != NULL);
  assert(cam_params != NULL);
  assert(cam_ext != NULL);

  if (cam_idx > (calib->num_cams - 1)) {
    const int new_size = calib->num_cams + 1;
    calib->cam_params =
        realloc(calib->cam_params, sizeof(camera_params_t) * new_size);
    calib->cam_exts = realloc(calib->cam_exts, sizeof(extrinsic_t) * new_size);
  }

  camera_params_setup(&calib->cam_params[cam_idx],
                      cam_idx,
                      cam_res,
                      proj_model,
                      dist_model,
                      cam_params);
  extrinsic_setup(&calib->cam_exts[cam_idx], cam_ext);
  if (cam_idx == 0) {
    calib->cam_exts[0].fix = 1;
  }

  calib->num_cams++;
  calib->cams_ok = 1;
}

/**
 * Add camera calibration view.
 */
void calib_camera_add_view(calib_camera_t *calib,
                           const timestamp_t ts,
                           const int view_idx,
                           const int cam_idx,
                           const int num_corners,
                           const int *tag_ids,
                           const int *corner_indices,
                           const real_t *object_points,
                           const real_t *keypoints) {
  assert(calib != NULL);
  assert(calib->cams_ok);
  if (num_corners == 0) {
    return;
  }

  // Pose T_C0F
  pose_t *pose = hmgets(calib->poses, ts).value;
  if (pose == NULL) {
    // Estimate relative pose T_CiF
    real_t T_CiF[4 * 4] = {0};
    const int status = solvepnp_camera(&calib->cam_params[cam_idx],
                                       keypoints,
                                       object_points,
                                       num_corners,
                                       T_CiF);
    if (status != 0) {
      return;
    }

    // Form T_BF
    POSE2TF(calib->cam_exts[cam_idx].data, T_BCi);
    TF_CHAIN(T_BF, 2, T_BCi, T_CiF);
    TF_VECTOR(T_BF, pose_vector);

    // New pose
    arrput(calib->timestamps, ts);
    pose = malloc(sizeof(pose_t) * 1);
    pose_setup(pose, ts, pose_vector);
    hmput(calib->poses, ts, pose);
  }

  // Form new view
  calib_camera_view_t **cam_views = hmgets(calib->view_sets, ts).value;
  if (cam_views == NULL) {
    cam_views = calloc(calib->num_cams, sizeof(calib_camera_view_t **));
    for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
      cam_views[cam_idx] = NULL;
    }
    hmput(calib->view_sets, ts, cam_views);
    calib->num_views++;
  }

  calib_camera_view_t *view =
      calib_camera_view_malloc(ts,
                               view_idx,
                               cam_idx,
                               num_corners,
                               tag_ids,
                               corner_indices,
                               object_points,
                               keypoints,
                               pose,
                               &calib->cam_exts[cam_idx],
                               &calib->cam_params[cam_idx]);
  cam_views[cam_idx] = view;
  calib->num_factors += num_corners;
}

void calib_camera_marginalize(calib_camera_t *calib) {
  // Setup marginalization factor
  marg_factor_t *marg = marg_factor_malloc();

  // Get first timestamp
  const timestamp_t ts = calib->timestamps[0];

  // Mark the pose at timestamp to be marginalized
  pose_t *pose = hmgets(calib->poses, ts).value;
  pose->marginalize = 1;

  // Add calib camera factors to marginalization factor
  calib_camera_view_t **cam_views = hmgets(calib->view_sets, ts).value;
  for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
    calib_camera_view_t *view = cam_views[cam_idx];
    if (view == NULL) {
      continue;
    }

    for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
      marg_factor_add(marg, CALIB_CAMERA_FACTOR, &view->factors[factor_idx]);
    }
  }

  // Add previous marginalization factor to new marginalization factor
  if (calib->marg) {
    marg_factor_add(marg, MARG_FACTOR, calib->marg);
  }

  // Marginalize
  marg_factor_marginalize(marg);
  if (calib->marg) {
    marg_factor_free(calib->marg);
  }
  calib->marg = marg;

  // Remove viewset
  for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
    calib_camera_view_free(cam_views[cam_idx]);
  }
  free(cam_views);
  (void) hmdel(calib->view_sets, ts);
  // ^ (void) cast required for now: https://github.com/nothings/stb/issues/1574

  // Remove timestamp
  arrdel(calib->timestamps, 0);

  // Update number of views
  calib->num_views--;
}

/**
 * Add camera calibration data.
 */
int calib_camera_add_data(calib_camera_t *calib,
                          const int cam_idx,
                          const char *data_path) {
  // Get camera data
  int num_files = 0;
  char **files = list_files(data_path, &num_files);

  // Exit if no calibration data
  if (num_files == 0) {
    for (int view_idx = 0; view_idx < num_files; view_idx++) {
      free(files[view_idx]);
    }
    free(files);
    return -1;
  }

  for (int view_idx = 0; view_idx < num_files; view_idx++) {
    // Load aprilgrid
    aprilgrid_t *grid = aprilgrid_load(files[view_idx]);

    // Get aprilgrid measurements
    const timestamp_t ts = grid->timestamp;
    const int num_corners = grid->corners_detected;
    int *tag_ids = malloc(sizeof(int) * num_corners);
    int *corner_indices = malloc(sizeof(int) * num_corners);
    real_t *kps = malloc(sizeof(real_t) * num_corners * 2);
    real_t *pts = malloc(sizeof(real_t) * num_corners * 3);
    aprilgrid_measurements(grid, tag_ids, corner_indices, kps, pts);

    // Add view
    calib_camera_add_view(calib,
                          ts,
                          view_idx,
                          cam_idx,
                          num_corners,
                          tag_ids,
                          corner_indices,
                          pts,
                          kps);

    // Clean up
    free(tag_ids);
    free(corner_indices);
    free(kps);
    free(pts);
    free(files[view_idx]);
    aprilgrid_free(grid);
  }
  free(files);

  return 0;
}

/**
 * Camera calibration reprojection errors.
 */
void calib_camera_errors(calib_camera_t *calib,
                         real_t *reproj_rmse,
                         real_t *reproj_mean,
                         real_t *reproj_median) {
  // Setup
  const int N = calib->num_factors;
  const int r_size = N * 2;
  real_t *r = calloc(r_size, sizeof(real_t));

  // Evaluate residuals
  int r_idx = 0;
  for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
    for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
      const timestamp_t ts = calib->timestamps[view_idx];
      calib_camera_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
      if (view == NULL) {
        continue;
      }

      for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
        struct calib_camera_factor_t *factor = &view->factors[factor_idx];
        calib_camera_factor_eval(factor);
        vec_copy(factor->r, factor->r_size, &r[r_idx]);
        r_idx += factor->r_size;
      } // For each calib factor
    }   // For each cameras
  }     // For each views

  // Calculate reprojection errors
  real_t *errors = calloc(N, sizeof(real_t));
  for (int i = 0; i < N; i++) {
    const real_t x = r[i * 2 + 0];
    const real_t y = r[i * 2 + 1];
    errors[i] = sqrt(x * x + y * y);
  }

  // Calculate RMSE
  real_t sum = 0.0;
  real_t sse = 0.0;
  for (int i = 0; i < N; i++) {
    sum += errors[i];
    sse += errors[i] * errors[i];
  }
  *reproj_rmse = sqrt(sse / N);
  *reproj_mean = sum / N;
  *reproj_median = median(errors, N);

  // Clean up
  free(errors);
  free(r);
}

int calib_camera_shannon_entropy(calib_camera_t *calib, real_t *entropy) {
  // Determine parameter order
  int sv_size = 0;
  int r_size = 0;
  param_order_t *hash = calib_camera_param_order(calib, &sv_size, &r_size);

  // Form Hessian H
  real_t *H = calloc(sv_size * sv_size, sizeof(real_t));
  real_t *g = calloc(sv_size, sizeof(real_t));
  real_t *r = calloc(r_size, sizeof(real_t));
  calib_camera_linearize_compact(calib, sv_size, hash, H, g, r);

  // Estimate covariance
  real_t *covar = calloc(sv_size * sv_size, sizeof(real_t));
  pinv(H, sv_size, sv_size, covar);

  // Grab the rows and columns corresponding to calib parameters
  // In the following we assume the state vector x is ordered:
  //
  //   x = [ poses [1..k], N camera extrinsics, N camera parameters]
  //
  // We are only interested in the Shannon-Entropy, or the uncertainty of the
  // calibration parameters. In this case the N camera extrinsics and
  // parameters, so once we have formed the full Hessian H matrix, inverted it
  // to form the covariance matrix, we can extract the lower right block matrix
  // that corresponds to the uncertainty of the calibration parameters, then
  // use it to calculate the shannon entropy.
  const timestamp_t last_ts = calib->timestamps[calib->num_views - 1];
  void *data = hmgets(calib->poses, last_ts).value->data;
  const int idx_s = hmgets(hash, data).idx + 6;
  const int idx_e = sv_size - 1;
  const int m = idx_e - idx_s + 1;
  real_t *covar_params = calloc(m * m, sizeof(real_t));
  mat_block_get(covar, sv_size, idx_s, idx_e, idx_s, idx_e, covar_params);

  // Calculate shannon-entropy
  int status = 0;
  if (shannon_entropy(covar_params, m, entropy) != 0) {
    status = -1;
  }

  // Clean up
  hmfree(hash);
  free(covar_params);
  free(covar);
  free(H);
  free(g);
  free(r);

  return status;
}

/**
 * Camera calibration parameter order.
 */
param_order_t *calib_camera_param_order(const void *data,
                                        int *sv_size,
                                        int *r_size) {
  // Setup parameter order
  calib_camera_t *calib = (calib_camera_t *) data;
  param_order_t *hash = NULL;
  int col_idx = 0;

  // -- Add body poses
  for (int i = 0; i < hmlen(calib->poses); i++) {
    param_order_add_pose(&hash, calib->poses[i].value, &col_idx);
  }

  // -- Add camera extrinsic
  for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
    param_order_add_extrinsic(&hash, &calib->cam_exts[cam_idx], &col_idx);
  }

  // -- Add camera parameters
  for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
    param_order_add_camera(&hash, &calib->cam_params[cam_idx], &col_idx);
  }

  // Set state-vector and residual size
  *sv_size = col_idx;
  *r_size = (calib->num_factors * 2);
  if (calib->marg) {
    *r_size += calib->marg->r_size;
  }

  return hash;
}

/**
 * Calculate camera calibration problem cost.
 */
void calib_camera_cost(const void *data, real_t *r) {
  // Evaluate factors
  calib_camera_t *calib = (calib_camera_t *) data;

  // -- Evaluate calib camera factors
  int r_idx = 0;
  for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
    for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
      const timestamp_t ts = calib->timestamps[view_idx];
      calib_camera_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
      if (view == NULL) {
        continue;
      }

      for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
        struct calib_camera_factor_t *factor = &view->factors[factor_idx];
        calib_camera_factor_eval(factor);
        vec_copy(factor->r, factor->r_size, &r[r_idx]);
        r_idx += factor->r_size;
      } // For each calib factor
    }   // For each cameras
  }     // For each views

  // -- Evaluate marginalization factor
  if (calib->marg) {
    marg_factor_eval(calib->marg);
    vec_copy(calib->marg->r, calib->marg->r_size, &r[r_idx]);
  }
}

/**
 * Linearize camera calibration problem.
 */
void calib_camera_linearize_compact(const void *data,
                                    const int sv_size,
                                    param_order_t *hash,
                                    real_t *H,
                                    real_t *g,
                                    real_t *r) {
  // Evaluate factors
  calib_camera_t *calib = (calib_camera_t *) data;
  int r_idx = 0;

  // -- Evaluate calib camera factors
  for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
    for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
      const timestamp_t ts = calib->timestamps[view_idx];
      calib_camera_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
      if (view == NULL) {
        continue;
      }

      for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
        struct calib_camera_factor_t *factor = &view->factors[factor_idx];
        calib_camera_factor_eval(factor);
        vec_copy(factor->r, factor->r_size, &r[r_idx]);

        solver_fill_hessian(hash,
                            factor->num_params,
                            factor->params,
                            factor->jacs,
                            factor->r,
                            factor->r_size,
                            sv_size,
                            H,
                            g);
        r_idx += factor->r_size;
      } // For each calib factor
    }   // For each cameras
  }     // For each views

  // -- Evaluate marginalization factor
  if (calib->marg) {
    marg_factor_eval(calib->marg);
    vec_copy(calib->marg->r, calib->marg->r_size, &r[r_idx]);

    solver_fill_hessian(hash,
                        calib->marg->num_params,
                        calib->marg->params,
                        calib->marg->jacs,
                        calib->marg->r,
                        calib->marg->r_size,
                        sv_size,
                        H,
                        g);
  }
}

/**
 * Reduce camera calibration problem via Schur-Complement.
 *
 * The Gauss newton system we are trying to solve has the form:
 *
 *   H dx = b (1)
 *
 * Where the H is the Hessian, dx is the update vector and b is a vector. In the
 * camera calibration problem the Hessian has a arrow head pattern (see (25) in
 * [Triggs2000]). This means to avoid inverting the full H matrix we can
 * decompose (1) as,
 *
 *   [A B * [dx0    [b0
 *    C D]   dx1] =  b1]  (2)
 *
 * and take the Shur-complement of A, we get a reduced system of:
 *
 *   D_bar = D − C * A^-1 * B
 *   b1_bar = b1 − C * A^-1 * b0  (3)
 *
 * Since A is a block diagonal, inverting it is much cheaper than inverting the
 * full H or A matrix. With (3) we can solve for dx1.
 *
 *   D_bar * dx1 = b1_bar
 *
 * And finally back-substitute the newly estimated dx1 to find dx0,
 *
 *   A * dx0 = b0 - B * dx1
 *   dx0 = A^-1 * b0 - B * dx1
 *
 * where in the previous steps we have already computed A^-1.
 *
 * [Triggs2000]:
 *
 *   Triggs, Bill, et al. "Bundle adjustment—a modern synthesis." Vision
 *   Algorithms: Theory and Practice: International Workshop on Vision
 *   Algorithms Corfu, Greece, September 21–22, 1999 Proceedings. Springer
 *   Berlin Heidelberg, 2000.
 *
 */
void calib_camera_linsolve(const void *data,
                           const int sv_size,
                           param_order_t *hash,
                           real_t *H,
                           real_t *g,
                           real_t *dx) {
  calib_camera_t *calib = (calib_camera_t *) data;
  const int m = calib->num_views * 6;
  const int r = sv_size - m;
  const int H_size = sv_size;
  const int bs = 6; // Diagonal block size

  // Extract sub-blocks of matrix H
  // H = [A, B,
  //      C, D]
  real_t *B = malloc(sizeof(real_t) * m * r);
  real_t *C = malloc(sizeof(real_t) * r * m);
  real_t *D = malloc(sizeof(real_t) * r * r);
  real_t *A_inv = malloc(sizeof(real_t) * m * m);
  mat_block_get(H, H_size, 0, m - 1, m, H_size - 1, B);
  mat_block_get(H, H_size, m, H_size - 1, 0, m - 1, C);
  mat_block_get(H, H_size, m, H_size - 1, m, H_size - 1, D);

  // Extract sub-blocks of vector b
  // b = [b0, b1]
  real_t *b0 = malloc(sizeof(real_t) * m);
  real_t *b1 = malloc(sizeof(real_t) * r);
  vec_copy(g, m, b0);
  vec_copy(g + m, r, b1);

  // Invert A
  bdiag_inv_sub(H, sv_size, m, bs, A_inv);

  // Reduce H * dx = b with Shur-Complement
  // D_bar = D - C * A_inv * B
  // b1_bar = b1 - C * A_inv * b0
  real_t *D_bar = malloc(sizeof(real_t) * r * r);
  real_t *b1_bar = malloc(sizeof(real_t) * r * 1);
  dot3(C, r, m, A_inv, m, m, B, m, r, D_bar);
  dot3(C, r, m, A_inv, m, m, b0, m, 1, b1_bar);
  for (int i = 0; i < (r * r); i++) {
    D_bar[i] = D[i] - D_bar[i];
  }
  for (int i = 0; i < r; i++) {
    b1_bar[i] = b1[i] - b1_bar[i];
  }

  // Solve reduced system: D_bar * dx_r = b1_bar
  real_t *dx_r = malloc(sizeof(real_t) * r * 1);
  // Hack: precondition D_bar so linear-solver doesn't complain
  for (int i = 0; i < r; i++) {
    D_bar[i * r + i] += 1e-4;
  }
  chol_solve(D_bar, b1_bar, dx_r, r);

  // Back-subsitute
  real_t *B_dx_r = calloc(m * 1, sizeof(real_t));
  real_t *dx_m = calloc(m * 1, sizeof(real_t));
  dot(B, m, r, dx_r, r, 1, B_dx_r);
  for (int i = 0; i < m; i++) {
    b0[i] = b0[i] - B_dx_r[i];
  }
  bdiag_dot(A_inv, m, m, bs, b0, dx_m);

  // Form full dx vector
  for (int i = 0; i < m; i++) {
    dx[i] = dx_m[i];
  }
  for (int i = 0; i < r; i++) {
    dx[i + m] = dx_r[i];
  }

  // Clean-up
  free(B);
  free(C);
  free(D);
  free(A_inv);

  free(b0);
  free(b1);

  free(D_bar);
  free(b1_bar);

  free(B_dx_r);
  free(dx_m);
  free(dx_r);
}

/**
 * Solve camera calibration problem.
 */
void calib_camera_solve(calib_camera_t *calib) {
  assert(calib != NULL);

  if (calib->num_views == 0) {
    return;
  }

  solver_t solver;
  solver_setup(&solver);
  solver.verbose = calib->verbose;
  solver.max_iter = calib->max_iter;
  solver.cost_func = &calib_camera_cost;
  solver.param_order_func = &calib_camera_param_order;
  solver.linearize_func = &calib_camera_linearize_compact;
  // solver.linsolve_func = &calib_camera_linsolve;
  solver_solve(&solver, calib);

  if (calib->verbose) {
    calib_camera_print(calib);
  }
}

///////////////////////////////
// CALIB IMU-CAM CALIBRATION //
///////////////////////////////

// /**
//  * Malloc imucam calibration view.
//  */
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
//                                               time_delay_t *time_delay) {
//   calib_imucam_view_t *view = MALLOC(calib_imucam_view_t, 1);
//
//   // Properties
//   view->ts = ts;
//   view->view_idx = view_idx;
//   view->cam_idx = cam_idx;
//   view->num_corners = num_corners;
//
//   // Measurements
//   if (num_corners) {
//     view->tag_ids = malloc(sizeof(int) * num_corners);
//     view->corner_indices = malloc(sizeof(int) * num_corners);
//     view->object_points = malloc(sizeof(real_t) * num_corners * 3);
//     view->keypoints = malloc(sizeof(real_t) * num_corners * 2);
//   }
//
//   // Factors
//   view->cam_factors = malloc(sizeof(calib_imucam_factor_t) * num_corners);
//   assert(view->tag_ids != NULL);
//   assert(view->corner_indices != NULL);
//   assert(view->object_points != NULL);
//   assert(view->keypoints != NULL);
//   assert(view->cam_factors != NULL);
//
//   for (int i = 0; i < num_corners; i++) {
//     view->tag_ids[i] = tag_ids[i];
//     view->corner_indices[i] = corner_indices[i];
//     view->object_points[i * 3] = object_points[i * 3];
//     view->object_points[i * 3 + 1] = object_points[i * 3 + 1];
//     view->object_points[i * 3 + 2] = object_points[i * 3 + 2];
//     view->keypoints[i * 2] = keypoints[i * 2];
//     view->keypoints[i * 2 + 1] = keypoints[i * 2 + 1];
//   }
//
//   const real_t var[2] = {1.0, 1.0};
//   for (int i = 0; i < view->num_corners; i++) {
//     const int tag_id = tag_ids[i];
//     const int corner_idx = corner_indices[i];
//     const real_t *p_FFi = &object_points[i * 3];
//     const real_t *z = &keypoints[i * 2];
//     const real_t v[2] = {0.0, 0.0};
//
//     view->tag_ids[i] = tag_id;
//     view->corner_indices[i] = corner_idx;
//     view->object_points[i * 3] = p_FFi[0];
//     view->object_points[i * 3 + 1] = p_FFi[1];
//     view->object_points[i * 3 + 2] = p_FFi[2];
//     view->keypoints[i * 2] = z[0];
//     view->keypoints[i * 2 + 1] = z[1];
//
//     calib_imucam_factor_setup(&view->cam_factors[i],
//                               fiducial,
//                               imu_pose,
//                               imu_ext,
//                               cam_ext,
//                               cam_params,
//                               time_delay,
//                               cam_idx,
//                               tag_id,
//                               corner_idx,
//                               p_FFi,
//                               z,
//                               v,
//                               var);
//   }
//
//   return view;
// }
//
// /**
//  * Free imucam calibration view.
//  */
// void calib_imucam_view_free(calib_imucam_view_t *view) {
//   if (view) {
//     free(view->tag_ids);
//     free(view->corner_indices);
//     free(view->object_points);
//     free(view->keypoints);
//     free(view->cam_factors);
//     free(view);
//   }
// }
//
// /**
//  * Malloc imu-cam calibration problem.
//  */
// calib_imucam_t *calib_imucam_malloc(void) {
//   calib_imucam_t *calib = malloc(sizeof(calib_imucam_t) * 1);
//
//   // Settings
//   calib->fix_fiducial = 0;
//   calib->fix_poses = 0;
//   calib->fix_velocities = 0;
//   calib->fix_biases = 0;
//   calib->fix_cam_params = 0;
//   calib->fix_cam_exts = 0;
//   calib->fix_time_delay = 1;
//   calib->verbose = 1;
//   calib->max_iter = 30;
//
//   // Flags
//   calib->imu_ok = 0;
//   calib->cams_ok = 0;
//   calib->state_initialized = 0;
//
//   // Counters
//   calib->num_imus = 0;
//   calib->num_cams = 0;
//   calib->num_views = 0;
//   calib->num_cam_factors = 0;
//   calib->num_imu_factors = 0;
//
//   // Variables
//   calib->timestamps = NULL;
//   calib->poses = NULL;
//   calib->velocities = NULL;
//   calib->imu_biases = NULL;
//   calib->fiducial = NULL;
//   calib->cam_exts = NULL;
//   calib->cam_params = NULL;
//   calib->imu_ext = NULL;
//   calib->time_delay = NULL;
//
//   // Buffers
//   calib->fiducial_buffer = fiducial_buffer_malloc();
//   imu_buffer_setup(&calib->imu_buf);
//
//   // Factors
//   calib->view_sets = NULL;
//   calib->imu_factors = NULL;
//   hmdefault(calib->view_sets, NULL);
//   hmdefault(calib->imu_factors, NULL);
//
//   return calib;
// }
//
// /**
//  * Free camera calibration problem
//  */
// void calib_imucam_free(calib_imucam_t *calib) {
//   // Fiducial buffer
//   fiducial_buffer_free(calib->fiducial_buffer);
//
//   // View sets
//   if (calib->num_views) {
//     for (int i = 0; i < arrlen(calib->timestamps); i++) {
//       const timestamp_t ts = calib->timestamps[i];
//       calib_imucam_view_t **cam_views = hmgets(calib->view_sets, ts).value;
//       for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//         calib_imucam_view_free(cam_views[cam_idx]);
//       }
//       free(cam_views);
//     }
//   }
//   hmfree(calib->view_sets);
//
//   // IMU factors
//   for (int i = 0; i < hmlen(calib->imu_factors); i++) {
//     free(calib->imu_factors[i].value);
//   }
//   hmfree(calib->imu_factors);
//
//   // Timestamps
//   arrfree(calib->timestamps);
//   // -- Poses
//   for (int k = 0; k < hmlen(calib->poses); k++) {
//     free(calib->poses[k].value);
//   }
//   hmfree(calib->poses);
//   // -- Velocities
//   for (int k = 0; k < hmlen(calib->velocities); k++) {
//     free(calib->velocities[k].value);
//   }
//   hmfree(calib->velocities);
//   // -- IMU biases
//   for (int k = 0; k < hmlen(calib->imu_biases); k++) {
//     free(calib->imu_biases[k].value);
//   }
//   hmfree(calib->imu_biases);
//   // -- Others
//   free(calib->fiducial);
//   free(calib->cam_exts);
//   free(calib->cam_params);
//   free(calib->imu_ext);
//   free(calib->time_delay);
//
//   free(calib);
// }
//
// /**
//  * Print imu-cam calibration problem
//  */
// void calib_imucam_print(calib_imucam_t *calib) {
//   real_t reproj_rmse = 0.0;
//   real_t reproj_mean = 0.0;
//   real_t reproj_median = 0.0;
//   if (calib->num_views) {
//     calib_imucam_errors(calib, &reproj_rmse, &reproj_mean, &reproj_median);
//   }
//
//   printf("settings:\n");
//   printf("  fix_fiducial: %d\n", calib->fix_fiducial);
//   printf("  fix_poses: %d\n", calib->fix_poses);
//   printf("  fix_cam_exts: %d\n", calib->fix_cam_exts);
//   printf("  fix_cam_params: %d\n", calib->fix_cam_params);
//   printf("  fix_time_delay: %d\n", calib->fix_time_delay);
//   printf("\n");
//
//   printf("statistics:\n");
//   printf("  num_cams: %d\n", calib->num_cams);
//   printf("  num_views: %d\n", calib->num_views);
//   printf("\n");
//
//   printf("reproj_errors:\n");
//   printf("  rmse: %f\n", reproj_rmse);
//   printf("  mean: %f\n", reproj_mean);
//   printf("  median: %f\n", reproj_median);
//   printf("\n");
//
//   if (calib->time_delay) {
//     printf("time_delay: %.4e  # [s] (cam_ts = imu_ts + time_delay)\n",
//            calib->time_delay->data[0]);
//     printf("\n");
//   }
//
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     camera_params_t *cam = &calib->cam_params[cam_idx];
//     char param_str[100] = {0};
//     vec2str(cam->data, 8, param_str);
//
//     printf("cam%d:\n", cam_idx);
//     printf("  resolution: [%d, %d]\n", cam->resolution[0], cam->resolution[1]);
//     printf("  proj_model: %s\n", cam->proj_model);
//     printf("  dist_model: %s\n", cam->dist_model);
//     printf("  param: %s\n", param_str);
//     printf("\n");
//
//     if (cam_idx > 0) {
//       char tf_str[20] = {0};
//       sprintf(tf_str, "T_cam0_cam%d", cam_idx);
//
//       extrinsic_t *cam_ext = &calib->cam_exts[cam_idx];
//       POSE2TF(cam_ext->data, T);
//       printf("%s:\n", tf_str);
//       printf("  rows: 4\n");
//       printf("  cols: 4\n");
//       printf("  data: [\n");
//       printf("    %.8f, %.8f, %.8f, %.8f,\n", T[0], T[1], T[2], T[3]);
//       printf("    %.8f, %.8f, %.8f, %.8f,\n", T[4], T[5], T[6], T[7]);
//       printf("    %.8f, %.8f, %.8f, %.8f,\n", T[8], T[9], T[10], T[11]);
//       printf("    %.8f, %.8f, %.8f, %.8f,\n", T[12], T[13], T[14], T[15]);
//       printf("  ]\n");
//     }
//   }
//   printf("\n");
//
//   if (calib->imu_ext) {
//     extrinsic_t *imu_ext = calib->imu_ext;
//     POSE2TF(imu_ext->data, T);
//     printf("T_imu0_cam0:\n");
//     printf("  rows: 4\n");
//     printf("  cols: 4\n");
//     printf("  data: [\n");
//     printf("    %.8f, %.8f, %.8f, %.8f,\n", T[0], T[1], T[2], T[3]);
//     printf("    %.8f, %.8f, %.8f, %.8f,\n", T[4], T[5], T[6], T[7]);
//     printf("    %.8f, %.8f, %.8f, %.8f,\n", T[8], T[9], T[10], T[11]);
//     printf("    %.8f, %.8f, %.8f, %.8f,\n", T[12], T[13], T[14], T[15]);
//     printf("  ]\n");
//   }
// }
//
// /**
//  * Add imu to imu-cam calibration problem.
//  */
// void calib_imucam_add_imu(calib_imucam_t *calib,
//                           const real_t imu_rate,
//                           const real_t sigma_aw,
//                           const real_t sigma_gw,
//                           const real_t sigma_a,
//                           const real_t sigma_g,
//                           const real_t g,
//                           const real_t *imu_ext) {
//   assert(calib != NULL);
//   assert(imu_rate > 0);
//   assert(sigma_aw > 0);
//   assert(sigma_gw > 0);
//   assert(sigma_a > 0);
//   assert(sigma_g > 0);
//   assert(g > 9.0);
//   assert(imu_ext);
//
//   if (calib->num_imus == 1) {
//     LOG_ERROR("Currently only supports 1 IMU!\n");
//     return;
//   }
//
//   // IMU parameters
//   calib->imu_params.imu_idx = 0;
//   calib->imu_params.rate = imu_rate;
//   calib->imu_params.sigma_aw = sigma_aw;
//   calib->imu_params.sigma_gw = sigma_gw;
//   calib->imu_params.sigma_a = sigma_a;
//   calib->imu_params.sigma_g = sigma_g;
//   calib->imu_params.g = g;
//
//   // IMU extrinsic
//   calib->imu_ext = malloc(sizeof(extrinsic_t) * 1);
//   extrinsic_setup(calib->imu_ext, imu_ext);
//
//   // Time delay
//   calib->time_delay = malloc(sizeof(time_delay_t) * 1);
//   time_delay_setup(calib->time_delay, 0.0);
//
//   // Update
//   calib->num_imus++;
// }
//
// /**
//  * Add camera to imu-cam calibration problem.
//  */
// void calib_imucam_add_camera(calib_imucam_t *calib,
//                              const int cam_idx,
//                              const int cam_res[2],
//                              const char *proj_model,
//                              const char *dist_model,
//                              const real_t *cam_params,
//                              const real_t *cam_ext) {
//   assert(calib != NULL);
//   assert(cam_idx <= calib->num_cams);
//   assert(cam_res != NULL);
//   assert(proj_model != NULL);
//   assert(dist_model != NULL);
//   assert(cam_params != NULL);
//   assert(cam_ext != NULL);
//
//   if (cam_idx > (calib->num_cams - 1)) {
//     const int new_size = calib->num_cams + 1;
//     calib->cam_params = REALLOC(calib->cam_params, camera_params_t, new_size);
//     calib->cam_exts = REALLOC(calib->cam_exts, extrinsic_t, new_size);
//   }
//
//   camera_params_setup(&calib->cam_params[cam_idx],
//                       cam_idx,
//                       cam_res,
//                       proj_model,
//                       dist_model,
//                       cam_params);
//   extrinsic_setup(&calib->cam_exts[cam_idx], cam_ext);
//
//   // Fix both camera intrinsics and extrinsics
//   calib->cam_params[cam_idx].fix = 1;
//   calib->cam_exts[cam_idx].fix = 1;
//
//   // Update book keeping
//   calib->num_cams++;
//   calib->cams_ok = 1;
// }
//
// /** Estimate relative pose between camera and fiducial target T_CiF **/
// static int calib_imucam_estimate_relative_pose(calib_imucam_t *calib,
//                                                int *cam_idx,
//                                                real_t T_CiF[4 * 4]) {
//   for (int i = 0; i < calib->fiducial_buffer->size; i++) {
//     const fiducial_event_t *data = calib->fiducial_buffer->data[i];
//     const camera_params_t *cam = &calib->cam_params[data->cam_idx];
//     const int status = solvepnp_camera(cam,
//                                        data->keypoints,
//                                        data->object_points,
//                                        data->num_corners,
//                                        T_CiF);
//     if (status != 0) {
//       return status;
//     }
//
//     *cam_idx = data->cam_idx;
//     break;
//   }
//
//   return 0;
// }
//
// /** Initialize fiducial pose T_WF **/
// static void calib_imucam_initialize_fiducial(calib_imucam_t *calib,
//                                              const timestamp_t ts) {
//   // Estimate relative pose T_CiF
//   int cam_idx = 0;
//   real_t T_CiF[4 * 4] = {0};
//   int status = calib_imucam_estimate_relative_pose(calib, &cam_idx, T_CiF);
//   if (status != 0) {
//     FATAL("FAILED!\n");
//     return;
//   }
//
//   // Form fiducial pose: T_WF
//   const pose_t *pose = hmgets(calib->poses, ts).value;
//   const extrinsic_t *cam_ext = &calib->cam_exts[cam_idx];
//   const extrinsic_t *imu_ext = calib->imu_ext;
//   TF(pose->data, T_WS);
//   TF(cam_ext->data, T_C0Ci);
//   TF(imu_ext->data, T_SC0);
//   TF_CHAIN(T_SCi, 2, T_SC0, T_C0Ci);
//   TF_CHAIN(T_WF, 3, T_WS, T_SCi, T_CiF);
//   TF_VECTOR(T_WF, fiducial_pose);
//
//   // Form fiducial
//   calib->fiducial = malloc(sizeof(fiducial_t) * 1);
//   fiducial_setup(calib->fiducial, fiducial_pose);
// }
//
// /** Add state. **/
// static void calib_imucam_add_state(calib_imucam_t *calib,
//                                    const timestamp_t ts) {
//   // Check timestamp does not already exists
//   if (hmgets(calib->poses, ts).value != NULL) {
//     return;
//   }
//
//   // Setup state-variables
//   real_t pose_k[7] = {0};
//   real_t vel_k[3] = {0};
//   real_t ba_k[3] = {0};
//   real_t bg_k[3] = {0};
//
//   if (calib->state_initialized == 0) {
//     // Initialize first pose
//     real_t r_WS[3] = {0};
//     real_t q_WS[4] = {0};
//     real_t T_WS[4 * 4] = {0};
//     imu_initial_attitude(&calib->imu_buf, q_WS);
//     tf_qr(q_WS, r_WS, T_WS);
//     tf_vector(T_WS, pose_k);
//     calib->state_initialized = 1;
//
//   } else {
//     // Estimate relative pose T_CiF
//     int cam_idx = 0;
//     real_t T_CiF[4 * 4] = {0};
//     int status = calib_imucam_estimate_relative_pose(calib, &cam_idx, T_CiF);
//     if (status != 0) {
//       printf("Failed to estimate relative pose!\n");
//       return;
//     }
//
//     // Form T_WS
//     const extrinsic_t *cam_ext = &calib->cam_exts[cam_idx];
//     const extrinsic_t *imu_ext = calib->imu_ext;
//     const fiducial_t *fiducial = calib->fiducial;
//     TF(fiducial->data, T_WF);
//     TF(cam_ext->data, T_C0Ci);
//     TF(imu_ext->data, T_SC0);
//     TF_INV(T_SC0, T_C0S);
//     TF_INV(T_CiF, T_FCi);
//     TF_INV(T_C0Ci, T_CiC0);
//     TF_CHAIN(T_WS, 4, T_WF, T_FCi, T_CiC0, T_C0S);
//     tf_vector(T_WS, pose_k);
//
//     // Estimate v_WS
//     const int last_idx = arrlen(calib->timestamps) - 1;
//     const timestamp_t last_ts = calib->timestamps[last_idx];
//     const real_t *pose_km1 = hmgets(calib->poses, last_ts).value->data;
//     vel_k[0] = pose_k[0] - pose_km1[0];
//     vel_k[1] = pose_k[1] - pose_km1[1];
//     vel_k[2] = pose_k[2] - pose_km1[2];
//   }
//
//   // Add timestamp
//   arrput(calib->timestamps, ts);
//
//   // Add state
//   // -- Pose
//   pose_t *imu_pose = malloc(sizeof(pose_t) * 1);
//   pose_setup(imu_pose, ts, pose_k);
//   hmput(calib->poses, ts, imu_pose);
//   // -- Velocity
//   velocity_t *vel = malloc(sizeof(velocity_t) * 1);
//   velocity_setup(vel, ts, vel_k);
//   hmput(calib->velocities, ts, vel);
//   // -- IMU biases
//   imu_biases_t *imu_biases = malloc(sizeof(imu_biases_t) * 1);
//   imu_biases_setup(imu_biases, ts, ba_k, bg_k);
//   hmput(calib->imu_biases, ts, imu_biases);
//
//   // Initialize fiducial
//   if (calib->fiducial == NULL) {
//     calib_imucam_initialize_fiducial(calib, ts);
//   }
// }
//
// /**
//  * Add IMU event.
//  */
// void calib_imucam_add_imu_event(calib_imucam_t *calib,
//                                 const timestamp_t ts,
//                                 const real_t acc[3],
//                                 const real_t gyr[3]) {
//   assert(calib != NULL);
//   assert(ts > 0);
//   assert(acc != NULL);
//   assert(gyr != NULL);
//   assert(calib->num_imus > 0);
//
//   // printf("add imu event:      %ld, ", ts);
//   // printf("acc: (%f, %f, %f), ", acc[0], acc[1], acc[2]);
//   // printf("gyr: (%f, %f, %f)\n", gyr[0], gyr[1], gyr[2]);
//
//   imu_buffer_add(&calib->imu_buf, ts, acc, gyr);
//   calib->imu_ok = 1;
// }
//
// /**
//  * Add camera event.
//  */
// void calib_imucam_add_fiducial_event(calib_imucam_t *calib,
//                                      const timestamp_t ts,
//                                      const int cam_idx,
//                                      const int num_corners,
//                                      const int *tag_ids,
//                                      const int *corner_indices,
//                                      const real_t *object_points,
//                                      const real_t *keypoints) {
//   assert(calib != NULL);
//   assert(calib->cams_ok);
//   assert(ts > 0);
//   assert(cam_idx >= 0);
//
//   // Pre-check
//   if (num_corners == 0 || calib->imu_ok == 0) {
//     return;
//   }
//
//   // printf("add fiducial event: %ld\n", ts);
//
//   // Add to buffer
//   fiducial_buffer_add(calib->fiducial_buffer,
//                       ts,
//                       cam_idx,
//                       num_corners,
//                       tag_ids,
//                       corner_indices,
//                       object_points,
//                       keypoints);
// }

// // /**
// //  * Marginalize oldest state variables in IMU-camera calibration.
// //  */
// // void calib_imucam_marginalize(calib_imucam_t *calib) {
// //   // // Setup marginalization factor
// //   // marg_factor_t *marg = marg_factor_malloc();
//
// //   // // Get first timestamp
// //   // const timestamp_t ts = calib->timestamps[0];
//
// //   // // Mark the pose at timestamp to be marginalized
// //   // pose_t *pose = hmgets(calib->poses, ts).value;
// //   // velocity_t *vel = hmgets(calib->velocities, ts).value;
// //   // imu_biases_t *biases = hmgets(calib->biases, ts).value;
// //   // assert(pose != NULL);
// //   // assert(vel != NULL);
// //   // assert(biases != NULL);
// //   // pose->marginalize = 1;
// //   // vel->marginalize = 1;
// //   // biases->marginalize = 1;
//
// //   // // Add calib camera factors to marginalization factor
// //   // calib_imucam_view_t **cam_views = hmgets(calib->view_sets, ts).value;
// //   // for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
// //   //   calib_imucam_view_t *view = cam_views[cam_idx];
// //   //   if (view == NULL) {
// //   //     continue;
// //   //   }
//
// //   //   for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
// //   //     marg_factor_add(marg, CALIB_IMUCAM_FACTOR, &view->factors[factor_idx]);
// //   //     calib->num_cam_factors--;
// //   //   }
// //   // }
//
// //   // // Add imu factor to marginalization factor
// //   // imu_factor_t *imu_factor = hmgets(calib->imu_factors, ts).value;
// //   // marg_factor_add(marg, IMU_FACTOR, imu_factor);
// //   // calib->num_imu_factors--;
//
// //   // // Add previous marginalization factor to new marginalization factor
// //   // if (calib->marg) {
// //   //   marg_factor_add(marg, MARG_FACTOR, calib->marg);
// //   // }
//
// //   // // Marginalize
// //   // marg_factor_marginalize(marg);
// //   // if (calib->marg) {
// //   //   marg_factor_free(calib->marg);
// //   // }
// //   // calib->marg = marg;
//
// //   // // Remove viewset
// //   // for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
// //   //   calib_imucam_view_free(cam_views[cam_idx]);
// //   // }
// //   // free(cam_views);
// //   // hmdel(calib->view_sets, ts);
//
// //   // // Remove IMU factor
// //   // free(imu_factor);
// //   // hmdel(calib->imu_factors, ts);
//
// //   // // Remove timestamp
// //   // arrdel(calib->timestamps, 0);
//
// //   // // Update number of views
// //   // calib->num_views--;
// // }
//
// /** Check update conditions. **/
// static int calib_imucam_update_precheck(calib_imucam_t *calib) {
//   // Check fiducial buffers empty?
//   if (calib->fiducial_buffer->size == 0) {
//     return -1;
//   }
//
//   // Check imu buffer empty?
//   if (calib->imu_buf.size == 0) {
//     return -1;
//   }
//
//   // Check timestamps are same
//   timestamp_t ts = 0;
//   for (int i = 0; i < calib->fiducial_buffer->size; i++) {
//     if (i == 0) {
//       ts = calib->fiducial_buffer->data[i]->ts;
//     }
//
//     if (ts != calib->fiducial_buffer->data[i]->ts) {
//       return -2;
//     }
//   }
//
//   // Check IMU timestamp is after fiducial data
//   if (ts > imu_buffer_last_ts(&calib->imu_buf)) {
//     return -3;
//   }
//
//   return 0;
// }
//
// /*
// static real_t *calib_imucam_optflow(calib_imucam_t *calib,
//                                     const fiducial_event_t *fiducial) {
//   real_t *optflows = calloc(fiducial->num_corners * 2, sizeof(real_t));
//   return optflows;
//   // if (arrlen(calib->timestamps) < 2) {
//   //   return optflows;
//   // }
//
//   // const timestamp_t ts_km1 = calib->timestamps[arrlen(calib->timestamps) - 2];
//   // const timestamp_t ts_k = calib->timestamps[arrlen(calib->timestamps) - 1];
//   // const real_t dt = ts2sec(ts_k) - ts2sec(ts_km1);
//   // calib_imucam_view_t **cam_views = hmgets(calib->view_sets, ts_km1).value;
//   // if (cam_views == NULL || cam_views[fiducial->cam_idx] == NULL) {
//   //   return optflows;
//   // }
//
//   // const calib_imucam_view_t *prev_view = cam_views[fiducial->cam_idx];
//   // for (int i = 0; i < fiducial->num_corners; i++) {
//   //   // Get corner tag id, corner index and keypoint measurement
//   //   const int t_tag_id = fiducial->tag_ids[i];
//   //   const int t_corner_idx = fiducial->corner_indices[i];
//   //   const real_t *kp_k = &fiducial->keypoints[i * 2];
//
//   //   // Find same corner in previous view
//   //   int found_corner = 0;
//   //   real_t *kp_km1 = NULL;
//   //   for (int j = 0; j < prev_view->num_corners; j++) {
//   //     const int q_tag_id = prev_view->tag_ids[j];
//   //     const int q_corner_idx = prev_view->corner_indices[j];
//
//   //     const int tag_id_ok = (t_tag_id == q_tag_id);
//   //     const int corner_idx_ok = (t_corner_idx == q_corner_idx);
//
//   //     if (tag_id_ok && corner_idx_ok) {
//   //       found_corner = 1;
//   //       kp_km1 = &prev_view->keypoints[j * 2];
//   //       break;
//   //     }
//   //   }
//
//   //   // Calculate optical flow
//   //   if (found_corner) {
//   //     optflows[2 * i + 0] = (kp_k[0] - kp_km1[0]) * dt;
//   //     optflows[2 * i + 1] = (kp_k[1] - kp_km1[1]) * dt;
//   //   } else {
//   //     optflows[2 * i + 0] = 0;
//   //     optflows[2 * i + 1] = 0;
//   //   }
//   // }
//
//   // return optflows;
// }
// */
//
// /**
//  * Update IMU-Camera calibration problem.
//  */
// int calib_imucam_update(calib_imucam_t *calib) {
//   assert(calib != NULL);
//
//   // Pre-check
//   if (calib_imucam_update_precheck(calib) != 0) {
//     return -1;
//   }
//
//   // Add state
//   const timestamp_t ts = imu_buffer_last_ts(&calib->imu_buf);
//   calib_imucam_add_state(calib, ts);
//
//   // Form new view
//   calib_imucam_view_t **cam_views = hmgets(calib->view_sets, ts).value;
//   if (cam_views == NULL) {
//     cam_views = calloc(calib->num_cams, sizeof(calib_camera_view_t **));
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       cam_views[cam_idx] = NULL;
//     }
//     hmput(calib->view_sets, ts, cam_views);
//     calib->num_views++;
//   }
//
//   for (int i = 0; i < calib->fiducial_buffer->size; i++) {
//     // Fiducial data
//     const fiducial_event_t *data = calib->fiducial_buffer->data[i];
//     const int cam_idx = data->cam_idx;
//     const int view_idx = calib->num_views;
//     pose_t *imu_pose = hmgets(calib->poses, ts).value;
//
//     calib_imucam_view_t *view =
//         calib_imucam_view_malloc(ts,
//                                  view_idx,
//                                  data->cam_idx,
//                                  data->num_corners,
//                                  data->tag_ids,
//                                  data->corner_indices,
//                                  data->object_points,
//                                  data->keypoints,
//                                  calib->fiducial,
//                                  imu_pose,
//                                  calib->imu_ext,
//                                  &calib->cam_exts[data->cam_idx],
//                                  &calib->cam_params[data->cam_idx],
//                                  calib->time_delay);
//
//     cam_views[cam_idx] = view;
//     calib->num_cam_factors += data->num_corners;
//   }
//
//   // Add imu factor
//   if (calib->num_views >= 2) {
//     // Pose, velocity and biases at km1
//     const size_t idx_km1 = arrlen(calib->timestamps) - 2;
//     const timestamp_t ts_km1 = calib->timestamps[idx_km1];
//     pose_t *pose_km1 = hmgets(calib->poses, ts_km1).value;
//     velocity_t *vel_km1 = hmgets(calib->velocities, ts_km1).value;
//     imu_biases_t *imu_biases_km1 = hmgets(calib->imu_biases, ts_km1).value;
//
//     // Pose, velocity and biases at k
//     const size_t idx_k = arrlen(calib->timestamps) - 1;
//     const timestamp_t ts_k = calib->timestamps[idx_k];
//     pose_t *pose_k = hmgets(calib->poses, ts_k).value;
//     velocity_t *vel_k = hmgets(calib->velocities, ts_k).value;
//     imu_biases_t *imu_biases_k = hmgets(calib->imu_biases, ts_k).value;
//
//     // printf("ts_km1: %ld, ts_k: %ld\n", ts_km1, ts_k);
//
//     // Form IMU factor
//     imu_factor_t *imu_factor = malloc(sizeof(imu_factor_t) * 1);
//     imu_factor_setup(imu_factor,
//                      &calib->imu_params,
//                      &calib->imu_buf,
//                      pose_km1,
//                      vel_km1,
//                      imu_biases_km1,
//                      pose_k,
//                      vel_k,
//                      imu_biases_k);
//     hmput(calib->imu_factors, ts, imu_factor);
//     calib->num_imu_factors++;
//
//     // Clear IMU buffer
//     imu_buffer_clear(&calib->imu_buf);
//   }
//
//   // Clear buffers
//   fiducial_buffer_clear(calib->fiducial_buffer);
//
//   return 0;
// }
//
// /**
//  * IMU-camera calibration reprojection errors.
//  */
// void calib_imucam_errors(calib_imucam_t *calib,
//                          real_t *reproj_rmse,
//                          real_t *reproj_mean,
//                          real_t *reproj_median) {
//   // Setup
//   const int N = calib->num_cam_factors;
//   const int r_size = N * 2;
//   real_t *r = calloc(r_size, sizeof(real_t));
//
//   // Evaluate residuals
//   int r_idx = 0;
//   for (int k = 0; k < arrlen(calib->timestamps); k++) {
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       const timestamp_t ts = calib->timestamps[k];
//       calib_imucam_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
//       if (view == NULL) {
//         continue;
//       }
//
//       for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
//         calib_imucam_factor_t *factor = &view->cam_factors[factor_idx];
//         calib_imucam_factor_eval(factor);
//         vec_copy(factor->r, factor->r_size, &r[r_idx]);
//         r_idx += factor->r_size;
//       } // For each calib factor
//     }   // For each cameras
//   }     // For each timestamp
//
//   // Calculate reprojection errors
//   real_t *errors = calloc(N, sizeof(real_t));
//   for (int i = 0; i < N; i++) {
//     const real_t x = r[i * 2 + 0];
//     const real_t y = r[i * 2 + 1];
//     errors[i] = sqrt(x * x + y * y);
//   }
//
//   // Calculate RMSE
//   real_t sum = 0.0;
//   real_t sse = 0.0;
//   for (int i = 0; i < N; i++) {
//     sum += errors[i];
//     sse += errors[i] * errors[i];
//   }
//   *reproj_rmse = sqrt(sse / N);
//   *reproj_mean = sum / N;
//   *reproj_median = median(errors, N);
//
//   // Clean up
//   free(errors);
//   free(r);
// }
//
// /**
//  * IMU-camera calibration parameter order.
//  */
// param_order_t *calib_imucam_param_order(const void *data,
//                                         int *sv_size,
//                                         int *r_size) {
//   // Setup parameter order
//   calib_imucam_t *calib = (calib_imucam_t *) data;
//   param_order_t *hash = NULL;
//   int col_idx = 0;
//
//   // -- Add poses
//   for (int i = 0; i < hmlen(calib->poses); i++) {
//     param_order_add_pose(&hash, calib->poses[i].value, &col_idx);
//   }
//
//   // -- Add velocities
//   for (int i = 0; i < hmlen(calib->velocities); i++) {
//     param_order_add_velocity(&hash, calib->velocities[i].value, &col_idx);
//   }
//
//   // -- Add biases
//   for (int i = 0; i < hmlen(calib->imu_biases); i++) {
//     param_order_add_imu_biases(&hash, calib->imu_biases[i].value, &col_idx);
//   }
//
//   // -- Add fiducial
//   param_order_add_fiducial(&hash, calib->fiducial, &col_idx);
//
//   // -- Add camera extrinsic
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     param_order_add_extrinsic(&hash, &calib->cam_exts[cam_idx], &col_idx);
//   }
//
//   // -- Add IMU-camera extrinsic
//   param_order_add_extrinsic(&hash, calib->imu_ext, &col_idx);
//
//   // -- Add camera parameters
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     param_order_add_camera(&hash, &calib->cam_params[cam_idx], &col_idx);
//   }
//
//   // -- Add time delay
//   param_order_add_time_delay(&hash, calib->time_delay, &col_idx);
//
//   // Set state-vector and residual size
//   *sv_size = col_idx;
//   *r_size = (calib->num_cam_factors * 2) + (calib->num_imu_factors * 15);
//   // if (calib->marg) {
//   //   *r_size += calib->marg->r_size;
//   // }
//
//   return hash;
// }
//
// /**
//  * Calculate IMU-camera calibration problem cost.
//  */
// void calib_imucam_cost(const void *data, real_t *r) {
//   // Evaluate factors
//   calib_imucam_t *calib = (calib_imucam_t *) data;
//
//   // -- Evaluate vision factors
//   int r_idx = 0;
//   for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       const timestamp_t ts = calib->timestamps[view_idx];
//       calib_imucam_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
//       if (view == NULL) {
//         continue;
//       }
//
//       for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
//         calib_imucam_factor_t *factor = &view->cam_factors[factor_idx];
//         calib_imucam_factor_eval(factor);
//         vec_copy(factor->r, factor->r_size, &r[r_idx]);
//         r_idx += factor->r_size;
//       } // For each calib factor
//     }   // For each cameras
//   }     // For each views
//
//   // -- Evaluate imu factors
//   for (int k = 0; k < hmlen(calib->imu_factors); k++) {
//     imu_factor_t *factor = calib->imu_factors[k].value;
//     imu_factor_eval(factor);
//     vec_copy(factor->r, factor->r_size, &r[r_idx]);
//     r_idx += factor->r_size;
//   }
//
//   // -- Evaluate marginalization factor
//   // if (calib->marg) {
//   //   marg_factor_eval(calib->marg);
//   //   vec_copy(calib->marg->r, calib->marg->r_size, &r[r_idx]);
//   // }
// }
//
// /**
//  * Linearize IMU-camera calibration problem.
//  */
// void calib_imucam_linearize_compact(const void *data,
//                                     const int sv_size,
//                                     param_order_t *hash,
//                                     real_t *H,
//                                     real_t *g,
//                                     real_t *r) {
//   // Evaluate factors
//   calib_imucam_t *calib = (calib_imucam_t *) data;
//   int r_idx = 0;
//
//   // -- Evaluate calib camera factors
//   for (int view_idx = 0; view_idx < calib->num_views; view_idx++) {
//     for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//       const timestamp_t ts = calib->timestamps[view_idx];
//       calib_imucam_view_t *view = hmgets(calib->view_sets, ts).value[cam_idx];
//       if (view == NULL) {
//         continue;
//       }
//
//       for (int factor_idx = 0; factor_idx < view->num_corners; factor_idx++) {
//         calib_imucam_factor_t *factor = &view->cam_factors[factor_idx];
//         calib_imucam_factor_eval(factor);
//         vec_copy(factor->r, factor->r_size, &r[r_idx]);
//
//         solver_fill_hessian(hash,
//                             factor->num_params,
//                             factor->params,
//                             factor->jacs,
//                             factor->r,
//                             factor->r_size,
//                             sv_size,
//                             H,
//                             g);
//         r_idx += factor->r_size;
//       } // For each calib factor
//     }   // For each cameras
//   }     // For each views
//
//   // -- Evaluate imu factors
//   for (int k = 0; k < hmlen(calib->imu_factors); k++) {
//     imu_factor_t *factor = calib->imu_factors[k].value;
//     imu_factor_eval(factor);
//     vec_copy(factor->r, factor->r_size, &r[r_idx]);
//
//     solver_fill_hessian(hash,
//                         factor->num_params,
//                         factor->params,
//                         factor->jacs,
//                         factor->r,
//                         factor->r_size,
//                         sv_size,
//                         H,
//                         g);
//     r_idx += factor->r_size;
//   }
//
//   // -- Evaluate marginalization factor
//   // if (calib->marg) {
//   //   marg_factor_eval(calib->marg);
//   //   vec_copy(calib->marg->r, calib->marg->r_size, &r[r_idx]);
//
//   //   solver_fill_hessian(hash,
//   //                       calib->marg->num_params,
//   //                       calib->marg->params,
//   //                       calib->marg->jacs,
//   //                       calib->marg->r,
//   //                       calib->marg->r_size,
//   //                       sv_size,
//   //                       H,
//   //                       g);
//   // }
// }
//
// void calib_imucam_save_estimates(calib_imucam_t *calib) {
//   FILE *data = fopen("/tmp/calib_imucam.dat", "w");
//
//   // Cameras
//   fprintf(data, "# Camera Parameters\n");
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     char params_str[1024] = {0};
//     vec2csv(calib->cam_params[cam_idx].data, 8, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   // Camera extrinsics
//   fprintf(data, "# Camera Extrinsics\n");
//   for (int cam_idx = 0; cam_idx < calib->num_cams; cam_idx++) {
//     char params_str[100] = {0};
//     vec2csv(calib->cam_exts[cam_idx].data, 7, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   // Camera extrinsics
//   fprintf(data, "# Camera-IMU Extrinsic\n");
//   {
//     char params_str[100] = {0};
//     vec2csv(calib->imu_ext->data, 7, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   // Fiducial
//   fprintf(data, "# Fiducial\n");
//   {
//     char params_str[100] = {0};
//     vec2csv(calib->fiducial->data, 7, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   // Poses
//   fprintf(data, "# Poses\n");
//   for (int k = 0; k < arrlen(calib->timestamps); k++) {
//     const timestamp_t ts = calib->timestamps[k];
//     const pose_t *pose = hmgets(calib->poses, ts).value;
//
//     char params_str[100] = {0};
//     vec2csv(pose->data, 7, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   // Velocities
//   fprintf(data, "# Velocities\n");
//   for (int k = 0; k < arrlen(calib->timestamps); k++) {
//     const timestamp_t ts = calib->timestamps[k];
//     const velocity_t *vel = hmgets(calib->velocities, ts).value;
//
//     char params_str[100] = {0};
//     vec2csv(vel->data, 3, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   // Biases
//   fprintf(data, "# Biases\n");
//   for (int k = 0; k < arrlen(calib->timestamps); k++) {
//     const timestamp_t ts = calib->timestamps[k];
//     const imu_biases_t *vel = hmgets(calib->imu_biases, ts).value;
//
//     char params_str[100] = {0};
//     vec2csv(vel->data, 3, params_str);
//     fprintf(data, "%s\n", params_str);
//   }
//   fprintf(data, "\n");
//
//   fclose(data);
// }
//
// /**
//  * Solve IMU-camera calibration problem.
//  */
// void calib_imucam_solve(calib_imucam_t *calib) {
//   assert(calib != NULL);
//
//   if (calib->num_views == 0) {
//     return;
//   }
//
//   solver_t solver;
//   solver_setup(&solver);
//   solver.verbose = calib->verbose;
//   solver.max_iter = calib->max_iter;
//   solver.cost_func = &calib_imucam_cost;
//   solver.param_order_func = &calib_imucam_param_order;
//   solver.linearize_func = &calib_imucam_linearize_compact;
//   // solver.linsolve_func = &calib_imucam_linsolve;
//   solver_solve(&solver, calib);
//
//   if (calib->verbose) {
//     calib_imucam_print(calib);
//   }
// }
