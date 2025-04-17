#include "xyz_se.h"

#define STB_DS_IMPLEMENTATION
#include <stb_ds.h>

/******************************************************************************
 * STATE ESTIMATION
 *****************************************************************************/

///////////
// UTILS //
///////////

int schur_complement(const real_t *H,
                     const real_t *b,
                     const int H_size,
                     const int m,
                     const int r,
                     real_t *H_marg,
                     real_t *b_marg) {
  assert(H != NULL);
  assert(b);
  assert(H_size > 0);
  assert((m + r) == H_size);
  assert(H_marg != NULL && b_marg != NULL);

  // Extract sub-blocks of matrix H
  // H = [Hmm, Hmr,
  //      Hrm, Hrr]
  real_t *Hmm = malloc(sizeof(real_t) * m * m);
  real_t *Hmr = malloc(sizeof(real_t) * m * r);
  real_t *Hrm = malloc(sizeof(real_t) * m * r);
  real_t *Hrr = malloc(sizeof(real_t) * r * r);
  real_t *Hmm_inv = malloc(sizeof(real_t) * m * m);

  mat_block_get(H, H_size, 0, m - 1, 0, m - 1, Hmm);
  mat_block_get(H, H_size, 0, m - 1, m, H_size - 1, Hmr);
  mat_block_get(H, H_size, m, H_size - 1, 0, m - 1, Hrm);
  mat_block_get(H, H_size, m, H_size - 1, m, H_size - 1, Hrr);

  // Extract sub-blocks of vector b
  // b = [b_mm, b_rr]
  real_t *bmm = malloc(sizeof(real_t) * m);
  real_t *brr = malloc(sizeof(real_t) * r);
  vec_copy(b, m, bmm);
  vec_copy(b + m, r, brr);

  // Invert Hmm
  int status = 0;
  if (eig_inv(Hmm, m, m, 1, Hmm_inv) != 0) {
    status = -1;
  }
  // pinv(Hmm, m, m, Hmm_inv);
  if (check_inv(Hmm, Hmm_inv, m) == -1) {
    status = -1;
    printf("Inverse Hmm failed!\n");
  }

  // Shur-Complement
  // H_marg = H_rr - H_rm * H_mm_inv * H_mr
  // b_marg = b_rr - H_rm * H_mm_inv * b_mm
  if (status == 0) {
    dot3(Hrm, r, m, Hmm_inv, m, m, Hmr, m, r, H_marg);
    dot3(Hrm, r, m, Hmm_inv, m, m, bmm, m, 1, b_marg);
    for (int i = 0; i < (r * r); i++) {
      H_marg[i] = Hrr[i] - H_marg[i];
    }
    for (int i = 0; i < r; i++) {
      b_marg[i] = brr[i] - b_marg[i];
    }
  }

  // Clean-up
  free(Hmm);
  free(Hmr);
  free(Hrm);
  free(Hrr);
  free(Hmm_inv);

  free(bmm);
  free(brr);

  return status;
}

/**
 * Calculate the Shannon Entropy
 */
int shannon_entropy(const real_t *covar, const int m, real_t *entropy) {
  assert(covar != NULL);
  assert(m > 0);
  assert(entropy != NULL);

  real_t covar_det = 0.0f;
  if (svd_det(covar, m, m, &covar_det) != 0) {
    return -1;
  }

  const real_t k = pow(2 * M_PI * exp(1), m);
  *entropy = 0.5 * log(k * covar_det);

  return 0;
}

//////////////
// POSITION //
//////////////

/**
 * Setup position.
 */
void pos_setup(pos_t *pos, const real_t *data) {
  assert(pos != NULL);
  assert(data != NULL);
  pos->marginalize = 0;
  pos->fix = 0;
  pos->data[0] = data[0];
  pos->data[1] = data[1];
  pos->data[2] = data[2];
}

/**
 * Copy position.
 */
void pos_copy(const pos_t *src, pos_t *dst) {
  assert(src != NULL);
  assert(dst != NULL);

  dst->marginalize = src->marginalize;
  dst->fix = src->fix;
  dst->data[0] = src->data[0];
  dst->data[1] = src->data[1];
  dst->data[2] = src->data[2];
}

/**
 * Print position.
 */
void pos_fprint(const char *prefix, const pos_t *pos, FILE *f) {
  assert(prefix != NULL);
  assert(pos != NULL);

  const real_t x = pos->data[0];
  const real_t y = pos->data[1];
  const real_t z = pos->data[2];

  fprintf(f, "%s: [%f, %f, %f]\n", prefix, x, y, z);
}

/**
 * Print position.
 */
void pos_print(const char *prefix, const pos_t *pos) {
  pos_fprint(prefix, pos, stdout);
}

//////////////
// ROTATION //
//////////////

/**
 * Setup rotation.
 */
void rot_setup(rot_t *rot, const real_t *data) {
  assert(rot != NULL);
  assert(data != NULL);
  rot->marginalize = 0;
  rot->fix = 0;
  rot->data[0] = data[0];
  rot->data[1] = data[1];
  rot->data[2] = data[2];
  rot->data[3] = data[3];
}

/**
 * Copy rotation.
 */
void rot_copy(const rot_t *src, rot_t *dst) {
  assert(src != NULL);
  assert(dst != NULL);

  dst->marginalize = src->marginalize;
  dst->fix = src->fix;
  dst->data[0] = src->data[0];
  dst->data[1] = src->data[1];
  dst->data[2] = src->data[2];
  dst->data[3] = src->data[3];
}

/**
 * Print rotation.
 */
void rot_fprint(const char *prefix, const rot_t *rot, FILE *f) {
  assert(prefix != NULL);
  assert(rot != NULL);

  const real_t qw = rot->data[0];
  const real_t qx = rot->data[1];
  const real_t qy = rot->data[2];
  const real_t qz = rot->data[3];

  fprintf(f, "%s: [%f, %f, %f, %f]\n", prefix, qw, qx, qy, qz);
}

/**
 * Print rotation.
 */
void rot_print(const char *prefix, const rot_t *rot) {
  rot_fprint(prefix, rot, stdout);
}

//////////
// POSE //
//////////

/**
 * Initialize pose vector.
 */
void pose_init(real_t *pose) {
  // Translation
  pose[0] = 0.0; // rx
  pose[1] = 0.0; // ry
  pose[2] = 0.0; // rz

  // Rotation (Quaternion)
  pose[3] = 1.0; // qw
  pose[4] = 0.0; // qx
  pose[5] = 0.0; // qy
  pose[6] = 0.0; // qz
}

/**
 * Setup pose.
 */
void pose_setup(pose_t *pose, const timestamp_t ts, const real_t *data) {
  assert(pose != NULL);
  assert(data != NULL);

  // Flags
  pose->marginalize = 0;
  pose->fix = 0;

  // Timestamp
  pose->ts = ts;

  // Translation
  pose->data[0] = data[0]; // rx
  pose->data[1] = data[1]; // ry
  pose->data[2] = data[2]; // rz

  // Rotation (Quaternion)
  pose->data[3] = data[3]; // qw
  pose->data[4] = data[4]; // qx
  pose->data[5] = data[5]; // qy
  pose->data[6] = data[6]; // qz
}

/**
 * Copy pose.
 */
void pose_copy(const pose_t *src, pose_t *dst) {
  assert(src != NULL);
  assert(dst != NULL);

  dst->marginalize = src->marginalize;
  dst->fix = src->fix;
  dst->ts = src->ts;
  dst->data[0] = src->data[0];
  dst->data[1] = src->data[1];
  dst->data[2] = src->data[2];
  dst->data[3] = src->data[3];
  dst->data[4] = src->data[4];
  dst->data[5] = src->data[5];
  dst->data[6] = src->data[6];
}

/**
 * Print pose
 */
void pose_fprint(const char *prefix, const pose_t *pose, FILE *f) {
  const timestamp_t ts = pose->ts;

  const real_t x = pose->data[0];
  const real_t y = pose->data[1];
  const real_t z = pose->data[2];

  const real_t qw = pose->data[3];
  const real_t qx = pose->data[4];
  const real_t qy = pose->data[5];
  const real_t qz = pose->data[6];
  const real_t q[4] = {qw, qx, qy, qz};
  real_t C[3 * 3] = {0};
  quat2rot(q, C);

  fprintf(f, "%s:\n", prefix);
  fprintf(f, "  timestamp: %ld\n", ts);
  fprintf(f, "  num_rows: 4\n");
  fprintf(f, "  num_cols: 4\n");
  fprintf(f, "  data: [\n");
  fprintf(f, "    %f, %f, %f, %f,\n", C[0], C[1], C[2], x);
  fprintf(f, "    %f, %f, %f, %f,\n", C[3], C[4], C[5], y);
  fprintf(f, "    %f, %f, %f, %f,\n", C[6], C[7], C[8], z);
  fprintf(f, "    %f, %f, %f, %f\n", 0.0, 0.0, 0.0, 1.0);
  fprintf(f, "  ]\n");
}

/**
 * Print pose
 */
void pose_print(const char *prefix, const pose_t *pose) {
  pose_fprint(prefix, pose, stdout);
}

///////////////
// EXTRINSIC //
///////////////

/**
 * Setup extrinsic.
 */
void extrinsic_setup(extrinsic_t *exts, const real_t *data) {
  assert(exts != NULL);
  assert(data != NULL);

  // Flags
  exts->marginalize = 0;
  exts->fix = 0;

  // Translation
  exts->data[0] = data[0]; // rx
  exts->data[1] = data[1]; // ry
  exts->data[2] = data[2]; // rz

  // Rotation (Quaternion)
  exts->data[3] = data[3]; // qw
  exts->data[4] = data[4]; // qx
  exts->data[5] = data[5]; // qy
  exts->data[6] = data[6]; // qz
}

/**
 * Copy extrinsic.
 */
void extrinsic_copy(const extrinsic_t *src, extrinsic_t *dst) {
  assert(src != NULL);
  assert(dst != NULL);

  dst->marginalize = src->marginalize;
  dst->fix = src->fix;
  dst->data[0] = src->data[0];
  dst->data[1] = src->data[1];
  dst->data[2] = src->data[2];
  dst->data[3] = src->data[3];
  dst->data[4] = src->data[4];
  dst->data[5] = src->data[5];
  dst->data[6] = src->data[6];
}

/**
 * Print extrinsic.
 */
void extrinsic_fprint(const char *prefix, const extrinsic_t *exts, FILE *f) {
  const real_t x = exts->data[0];
  const real_t y = exts->data[1];
  const real_t z = exts->data[2];

  const real_t qw = exts->data[3];
  const real_t qx = exts->data[4];
  const real_t qy = exts->data[5];
  const real_t qz = exts->data[6];
  const real_t q[4] = {qw, qx, qy, qz};
  real_t C[3 * 3] = {0};
  quat2rot(q, C);

  fprintf(f, "%s:\n", prefix);
  fprintf(f, "  num_rows: 4\n");
  fprintf(f, "  num_cols: 4\n");
  fprintf(f, "  data: [\n");
  fprintf(f, "    %f, %f, %f, %f,\n", C[0], C[1], C[2], x);
  fprintf(f, "    %f, %f, %f, %f,\n", C[3], C[4], C[5], y);
  fprintf(f, "    %f, %f, %f, %f,\n", C[6], C[7], C[8], z);
  fprintf(f, "    %f, %f, %f, %f\n", 0.0, 0.0, 0.0, 1.0);
  fprintf(f, "  ]\n");
}

/**
 * Print extrinsic.
 */
void extrinsic_print(const char *prefix, const extrinsic_t *exts) {
  extrinsic_fprint(prefix, exts, stdout);
}

//////////////
// FIDUCIAL //
//////////////

/**
 * Setup fiducial.
 */
void fiducial_setup(fiducial_t *exts, const real_t *data) {
  assert(exts != NULL);
  assert(data != NULL);

  // Flags
  exts->marginalize = 0;
  exts->fix = 0;

  // Translation
  exts->data[0] = data[0]; // rx
  exts->data[1] = data[1]; // ry
  exts->data[2] = data[2]; // rz

  // Rotation (Quaternion)
  exts->data[3] = data[3]; // qw
  exts->data[4] = data[4]; // qx
  exts->data[5] = data[5]; // qy
  exts->data[6] = data[6]; // qz
}

/**
 * Copy fiducial.
 */
void fiducial_copy(const fiducial_t *src, fiducial_t *dst) {
  assert(src != NULL);
  assert(dst != NULL);

  dst->marginalize = src->marginalize;
  dst->fix = src->fix;
  dst->data[0] = src->data[0];
  dst->data[1] = src->data[1];
  dst->data[2] = src->data[2];
  dst->data[3] = src->data[3];
  dst->data[4] = src->data[4];
  dst->data[5] = src->data[5];
  dst->data[6] = src->data[6];
}

/**
 * Print fiducial.
 */
void fiducial_fprint(const char *prefix, const fiducial_t *fiducial, FILE *f) {
  const real_t x = fiducial->data[0];
  const real_t y = fiducial->data[1];
  const real_t z = fiducial->data[2];

  const real_t qw = fiducial->data[3];
  const real_t qx = fiducial->data[4];
  const real_t qy = fiducial->data[5];
  const real_t qz = fiducial->data[6];
  const real_t q[4] = {qw, qx, qy, qz};
  real_t C[3 * 3] = {0};
  quat2rot(q, C);

  fprintf(f, "%s:\n", prefix);
  fprintf(f, "  num_rows: 4\n");
  fprintf(f, "  num_cols: 4\n");
  fprintf(f, "  data: [\n");
  fprintf(f, "    %f, %f, %f, %f,\n", C[0], C[1], C[2], x);
  fprintf(f, "    %f, %f, %f, %f,\n", C[3], C[4], C[5], y);
  fprintf(f, "    %f, %f, %f, %f,\n", C[6], C[7], C[8], z);
  fprintf(f, "    %f, %f, %f, %f\n", 0.0, 0.0, 0.0, 1.0);
  fprintf(f, "  ]\n");
}

/**
 * Print fiducial.
 */
void fiducial_print(const char *prefix, const fiducial_t *fiducial) {
  fiducial_fprint(prefix, fiducial, stdout);
}

// /**
//  * Malloc fiducial buffer.
//  */
// fiducial_buffer_t *fiducial_buffer_malloc(void) {
//   fiducial_buffer_t *buf = malloc(sizeof(fiducial_buffer_t) * 1);
//   buf->data = calloc(10, sizeof(fiducial_event_t *));
//   buf->size = 0;
//   buf->capacity = 10;
//   return buf;
// }
//
// /**
//  * Clear fiducial buffer.
//  */
// void fiducial_buffer_clear(fiducial_buffer_t *buf) {
//   for (int i = 0; i < buf->size; i++) {
//     free(buf->data[i]->tag_ids);
//     free(buf->data[i]->corner_indices);
//     free(buf->data[i]->object_points);
//     free(buf->data[i]->keypoints);
//     free(buf->data[i]);
//     buf->data[i] = NULL;
//   }
//   buf->size = 0;
// }
//
// /**
//  * Free fiducial buffer.
//  */
// void fiducial_buffer_free(fiducial_buffer_t *buf) {
//   fiducial_buffer_clear(buf);
//   free(buf->data);
//   free(buf);
// }
//
// /**
//  * Obtain total number of corners in fiducial buffer.
//  */
// int fiducial_buffer_total_corners(const fiducial_buffer_t *buf) {
//   int total_corners = 0;
//   for (int i = 0; i < buf->size; i++) {
//     total_corners += buf->data[i]->num_corners;
//   }
//   return total_corners;
// }
//
// /**
//  * Add fiducial data to buffer.
//  */
// void fiducial_buffer_add(fiducial_buffer_t *buf,
//                          const timestamp_t ts,
//                          const int cam_idx,
//                          const int num_corners,
//                          const int *tag_ids,
//                          const int *corner_indices,
//                          const real_t *object_points,
//                          const real_t *keypoints) {
//   // Pre-check
//   if (buf->size == 10) {
//     FATAL("Fiducial buffer is full!\n");
//   }
//
//   // Add to buffer
//   int idx = buf->size;
//
//   buf->data[idx] = malloc(sizeof(fiducial_event_t) * 1);
//   buf->data[idx]->ts = ts;
//   buf->data[idx]->cam_idx = cam_idx;
//   buf->data[idx]->num_corners = num_corners;
//
//   buf->data[idx]->tag_ids = calloc(num_corners, sizeof(int));
//   buf->data[idx]->corner_indices = calloc(num_corners, sizeof(int));
//   buf->data[idx]->object_points = calloc(num_corners * 3, sizeof(real_t));
//   buf->data[idx]->keypoints = calloc(num_corners * 2, sizeof(real_t));
//   for (int i = 0; i < num_corners; i++) {
//     buf->data[idx]->tag_ids[i] = tag_ids[i];
//     buf->data[idx]->corner_indices[i] = corner_indices[i];
//     buf->data[idx]->object_points[i * 3 + 0] = object_points[i * 3 + 0];
//     buf->data[idx]->object_points[i * 3 + 1] = object_points[i * 3 + 1];
//     buf->data[idx]->object_points[i * 3 + 2] = object_points[i * 3 + 2];
//     buf->data[idx]->keypoints[i * 2 + 0] = keypoints[i * 2 + 0];
//     buf->data[idx]->keypoints[i * 2 + 1] = keypoints[i * 2 + 1];
//   }
//
//   buf->size++;
// }

///////////////////////
// CAMERA-PARAMETERS //
///////////////////////

/**
 * Setup camera parameters
 */
void camera_params_setup(camera_params_t *camera,
                         const int cam_idx,
                         const int cam_res[2],
                         const char *proj_model,
                         const char *dist_model,
                         const real_t *data) {
  assert(camera != NULL);
  assert(cam_res != NULL);
  assert(proj_model != NULL);
  assert(dist_model != NULL);
  assert(data != NULL);

  camera->marginalize = 0;
  camera->fix = 0;

  camera->cam_idx = cam_idx;
  camera->resolution[0] = cam_res[0];
  camera->resolution[1] = cam_res[1];

  string_copy(camera->proj_model, proj_model);
  string_copy(camera->dist_model, dist_model);

  camera->data[0] = data[0];
  camera->data[1] = data[1];
  camera->data[2] = data[2];
  camera->data[3] = data[3];
  camera->data[4] = data[4];
  camera->data[5] = data[5];
  camera->data[6] = data[6];
  camera->data[7] = data[7];

  if (streqs(proj_model, "pinhole") && streqs(dist_model, "radtan4")) {
    camera->proj_func = pinhole_radtan4_project;
    camera->back_proj_func = pinhole_radtan4_back_project;
    camera->undistort_func = pinhole_radtan4_undistort;
  } else if (streqs(proj_model, "pinhole") && streqs(dist_model, "equi4")) {
    camera->proj_func = pinhole_equi4_project;
    camera->back_proj_func = pinhole_equi4_back_project;
    camera->undistort_func = pinhole_equi4_undistort;
  } else {
    FATAL("Unknown [%s-%s] camera model!\n", proj_model, dist_model);
  }
}

/**
 * Copy camera parameters.
 */
void camera_params_copy(const camera_params_t *src, camera_params_t *dst) {
  dst->marginalize = src->marginalize;
  dst->fix = src->fix;

  dst->cam_idx = src->cam_idx;
  dst->resolution[0] = src->resolution[0];
  dst->resolution[1] = src->resolution[1];
  strcpy(dst->proj_model, src->proj_model);
  strcpy(dst->dist_model, src->dist_model);
  dst->data[0] = src->data[0];
  dst->data[1] = src->data[1];
  dst->data[2] = src->data[2];
  dst->data[3] = src->data[3];
  dst->data[4] = src->data[4];
  dst->data[5] = src->data[5];
  dst->data[6] = src->data[6];
  dst->data[7] = src->data[7];

  dst->proj_func = src->proj_func;
  dst->back_proj_func = src->back_proj_func;
  dst->undistort_func = src->undistort_func;
}

/**
 * Print camera parameters
 */
void camera_params_fprint(const camera_params_t *cam, FILE *f) {
  assert(cam != NULL);

  fprintf(f, "cam%d:\n", cam->cam_idx);
  fprintf(f,
          "  resolution: [%d, %d]\n",
          cam->resolution[0],
          cam->resolution[1]);
  fprintf(f, "  proj_model: %s\n", cam->proj_model);
  fprintf(f, "  dist_model: %s\n", cam->dist_model);
  fprintf(f, "  data: [");
  for (int i = 0; i < 8; i++) {
    if ((i + 1) < 8) {
      fprintf(f, "%.4f, ", cam->data[i]);
    } else {
      fprintf(f, "%.4f", cam->data[i]);
    }
  }
  fprintf(f, "]\n");
}

/**
 * Print camera parameters
 */
void camera_params_print(const camera_params_t *cam) {
  assert(cam != NULL);
  camera_params_fprint(cam, stdout);
}

/**
 * Project 3D point to image point.
 */
void camera_project(const camera_params_t *camera,
                    const real_t p_C[3],
                    real_t z[2]) {
  assert(camera != NULL);
  assert(camera->proj_func != NULL);
  assert(p_C != NULL);
  assert(z != NULL);
  camera->proj_func(camera->data, p_C, z);
}

/**
 * Back project image point to bearing vector.
 */
void camera_back_project(const camera_params_t *camera,
                         const real_t z[2],
                         real_t bearing[3]) {
  assert(camera != NULL);
  assert(z != NULL);
  assert(bearing != NULL);
  camera->back_proj_func(camera->data, z, bearing);
}

/**
 * Undistort image points.
 */
void camera_undistort_points(const camera_params_t *camera,
                             const real_t *kps,
                             const int num_points,
                             real_t *kps_und) {
  assert(camera != NULL);
  assert(kps != NULL);
  assert(kps_und != NULL);

  for (int i = 0; i < num_points; i++) {
    const real_t *z_in = &kps[2 * i];
    real_t *z_out = &kps_und[i * 2];
    camera->undistort_func(camera->data, z_in, z_out);
  }
}

/**
 * Solve the Perspective-N-Points problem.
 *
 * **IMPORTANT**: This function assumes that object points lie on the plane
 * because the initialization step uses DLT to estimate the homography between
 * camera and planar object, then the relative pose between them is recovered.
 */
int solvepnp_camera(const camera_params_t *cam_params,
                    const real_t *img_pts,
                    const real_t *obj_pts,
                    const int N,
                    real_t T_CO[4 * 4]) {
  assert(cam_params != NULL);
  assert(img_pts != NULL);
  assert(obj_pts != NULL);
  assert(N > 0);
  assert(T_CO != NULL);

  // Undistort keypoints
  real_t *img_pts_ud = malloc(sizeof(real_t) * N * 2);
  camera_undistort_points(cam_params, img_pts, N, img_pts_ud);

  // Estimate relative pose T_CO
  const int status = solvepnp(cam_params->data, img_pts_ud, obj_pts, N, T_CO);
  free(img_pts_ud);

  return status;
}

/**
 * Triangulate features in batch.
 */
void triangulate_batch(const camera_params_t *cam_i,
                       const camera_params_t *cam_j,
                       const real_t T_CiCj[4 * 4],
                       const real_t *kps_i,
                       const real_t *kps_j,
                       const int n,
                       real_t *points,
                       int *status) {
  assert(cam_i != NULL);
  assert(cam_j != NULL);
  assert(T_CiCj != NULL);
  assert(kps_i != NULL);
  assert(kps_j != NULL);
  assert(n > 0);
  assert(points != NULL);
  assert(status != NULL);

  // Setup projection matrices
  real_t P_i[3 * 4] = {0};
  real_t P_j[3 * 4] = {0};
  TF_IDENTITY(T_eye);
  pinhole_projection_matrix(cam_i->data, T_eye, P_i);
  pinhole_projection_matrix(cam_j->data, T_CiCj, P_j);

  // Triangulate features
  for (int i = 0; i < n; i++) {
    // Undistort keypoints
    real_t z_i[2] = {0};
    real_t z_j[2] = {0};
    cam_i->undistort_func(cam_i->data, &kps_i[i * 2], z_i);
    cam_j->undistort_func(cam_j->data, &kps_j[i * 2], z_j);

    // Triangulate
    real_t p[3] = {0};
    linear_triangulation(P_i, P_j, z_i, z_j, p);
    points[i * 3 + 0] = p[0];
    points[i * 3 + 1] = p[1];
    points[i * 3 + 2] = p[2];
    status[i] = 0;
  }
}

//////////////
// VELOCITY //
//////////////

/**
 * Setup velocity
 */
void velocity_setup(velocity_t *vel, const timestamp_t ts, const real_t v[3]) {
  assert(vel != NULL);
  assert(v != NULL);

  // Flags
  vel->marginalize = 0;
  vel->fix = 0;

  // Timestamp
  vel->ts = ts;

  // Accel biases
  vel->data[0] = v[0];
  vel->data[1] = v[1];
  vel->data[2] = v[2];
}

/**
 * Copy velocity.
 */
void velocity_copy(const velocity_t *src, velocity_t *dst) {
  assert(src != NULL);
  assert(dst != NULL);

  dst->marginalize = src->marginalize;
  dst->fix = src->fix;
  dst->ts = src->ts;
  dst->data[0] = src->data[0];
  dst->data[1] = src->data[1];
  dst->data[2] = src->data[2];
}

////////////////
// IMU-BIASES //
////////////////

/**
 * Setup speed and biases
 */
void imu_biases_setup(imu_biases_t *biases,
                      const timestamp_t ts,
                      const real_t ba[3],
                      const real_t bg[3]) {
  assert(biases != NULL);
  assert(ba != NULL);
  assert(bg != NULL);

  // Flags
  biases->marginalize = 0;
  biases->fix = 0;

  // Timestamp
  biases->ts = ts;

  // Accel biases
  biases->data[0] = ba[0];
  biases->data[1] = ba[1];
  biases->data[2] = ba[2];

  // Gyro biases
  biases->data[3] = bg[0];
  biases->data[4] = bg[1];
  biases->data[5] = bg[2];
}

/**
 * Copy imu_biases.
 */
void imu_biases_copy(const imu_biases_t *src, imu_biases_t *dst) {
  assert(src != NULL);
  assert(dst != NULL);

  dst->marginalize = src->marginalize;
  dst->fix = src->fix;
  dst->ts = src->ts;
  dst->data[0] = src->data[0];
  dst->data[1] = src->data[1];
  dst->data[2] = src->data[2];
  dst->data[3] = src->data[3];
  dst->data[4] = src->data[4];
  dst->data[5] = src->data[5];
}

/**
 * Get IMU accelerometer biases
 */
void imu_biases_get_accel_bias(const imu_biases_t *biases, real_t ba[3]) {
  ba[0] = biases->data[0];
  ba[1] = biases->data[1];
  ba[2] = biases->data[2];
}

/**
 * Get IMU gyroscope biases
 */
void imu_biases_get_gyro_bias(const imu_biases_t *biases, real_t bg[3]) {
  bg[0] = biases->data[3];
  bg[1] = biases->data[4];
  bg[2] = biases->data[5];
}

/////////////
// FEATURE //
/////////////

/**
 * Setup feature.
 */
void feature_setup(feature_t *f, const size_t feature_id) {
  assert(f != NULL);

  f->marginalize = 0;
  f->fix = 0;

  f->type = FEATURE_XYZ;
  f->feature_id = feature_id;
  f->status = 0;
  f->data[0] = 0.0;
  f->data[1] = 0.0;
  f->data[2] = 0.0;
}

/**
 * Initialize feature.
 */
void feature_init(feature_t *f, const size_t feature_id, const real_t *data) {
  assert(f != NULL);
  assert(data != NULL);

  f->marginalize = 0;
  f->fix = 0;

  f->type = FEATURE_XYZ;
  f->feature_id = feature_id;
  f->status = 1;
  f->data[0] = data[0];
  f->data[1] = data[1];
  f->data[2] = data[2];
}

/**
 * Print feature.
 */
void feature_print(const feature_t *f) {
  printf("feature_id: %ld\n", f->feature_id);
  printf("status: %d\n", f->status);
  printf("data: (%.2f, %.2f, %.2f)\n", f->data[0], f->data[1], f->data[2]);
  printf("\n");
}

// /**
//  * Setup inverse-depth feature.
//  */
// void idf_setup(feature_t *f,
//                const size_t feature_id,
//                const size_t pos_id,
//                const camera_params_t *cam_params,
//                const real_t C_WC[3 * 3],
//                const real_t z[2]) {
//   // Keypoint to bearing (u, v, 1)
//   real_t bearing[3] = {0};
//   camera_back_project(cam_params, z, bearing);

//   // Convert bearing to theta, phi and rho
//   DOT(C_WC, 3, 3, bearing, 3, 1, h_W);
//   const real_t theta = atan2(h_W[0], h_W[2]);
//   const real_t phi = atan2(-h_W[1], sqrt(h_W[0] * h_W[0] + h_W[2] * h_W[2]));
//   const real_t rho = 0.1;

//   // Set data
//   f->marginalize = 0;
//   f->type = FEATURE_INVERSE_DEPTH;
//   f->feature_id = feature_id;
//   f->status = 1;
//   f->data[0] = theta;
//   f->data[1] = phi;
//   f->data[2] = rho;

//   f->cam_params = cam_params;
//   f->pos_id = pos_id;
// }

// /**
//  * Convert inverse-depth feature to 3D point.
//  */
// void idf_point(const feature_t *f, const real_t r_WC[3], real_t p_W[3]) {
//   const real_t x = r_WC[0];
//   const real_t y = r_WC[1];
//   const real_t z = r_WC[2];
//   const real_t theta = f->data[0];
//   const real_t phi = f->data[1];
//   const real_t depth = 1.0 / f->data[2];

//   const real_t cphi = cos(phi);
//   const real_t sphi = sin(phi);
//   const real_t ctheta = cos(theta);
//   const real_t stheta = sin(theta);
//   const real_t m[3] = {cphi * stheta, -sphi, cphi * ctheta};

//   p_W[0] = x + depth * m[0];
//   p_W[1] = y + depth * m[1];
//   p_W[2] = z + depth * m[2];
// }

// /**
//  * Malloc features.
//  */
// features_t *features_malloc(void) {
//   features_t *features = malloc(sizeof(features_t) * 1);

//   features->data = calloc(FEATURES_CAPACITY_INITIAL, sizeof(feature_t *));
//   features->num_features = 0;
//   features->feature_capacity = FEATURES_CAPACITY_INITIAL;

//   features->pos_data = calloc(FEATURES_CAPACITY_INITIAL, sizeof(feature_t *));
//   features->num_positions = 0;
//   features->position_capacity = FEATURES_CAPACITY_INITIAL;

//   return features;
// }

// /**
//  * Free features.
//  */
// void features_free(features_t *features) {
//   assert(features != NULL);

//   for (size_t i = 0; i < features->feature_capacity; i++) {
//     free(features->data[i]);
//   }
//   free(features->data);

//   for (size_t i = 0; i < features->position_capacity; i++) {
//     free(features->pos_data[i]);
//   }
//   free(features->pos_data);

//   free(features);
// }

// /**
//  * Check whether feature with `feature_id` exists
//  * @returns 1 for yes, 0 for no
//  */
// int features_exists(const features_t *features, const size_t feature_id) {
//   return features->data[feature_id] != NULL;
// }

// /**
//  * Add XYZ feature.
//  */
// void features_add_xyzs(features_t *features,
//                        const size_t *feature_ids,
//                        const real_t *params,
//                        const size_t num_features) {
//   assert(features != NULL);
//   assert(feature_ids != NULL);
//   assert(params != NULL);

//   // Expand features dynamic array if needed
//   if (feature_ids[num_features - 1] >= features->feature_capacity) {
//     size_t old_size = features->feature_capacity;
//     size_t new_size = old_size * FEATURES_CAPACITY_GROWTH_FACTOR;
//     features->data = realloc(features->data, sizeof(feature_t *) * new_size);
//     features->feature_capacity = new_size;
//     for (size_t i = old_size; i < new_size; i++) {
//       features->data[i] = NULL;
//     }
//     // The above step is quite important because by default realloc will not
//     // initialize pointers to NULL, and there will be no way of knowing
//     // whether a feature exists.
//   }

//   // Add features
//   for (size_t i = 0; i < num_features; i++) {
//     feature_t *f = MALLOC(feature_t, 1);
//     feature_init(f, feature_ids[i], params + i * 3);
//     features->data[feature_ids[i]] = f;
//     features->num_features++;
//   }
// }

// /**
//  * Add inverse-depth feature.
//  */
// void features_add_idfs(features_t *features,
//                        const size_t *feature_ids,
//                        const camera_params_t *cam_params,
//                        const real_t T_WC[4 * 4],
//                        const real_t *keypoints,
//                        const size_t num_keypoints) {
//   assert(features != NULL);
//   assert(feature_ids != NULL);
//   assert(cam_params != NULL);
//   assert(T_WC != NULL);
//   assert(keypoints != NULL);

//   // Pre-check
//   if (num_keypoints == 0) {
//     return;
//   }

//   // Expand features dynamic array if needed
//   if (feature_ids[num_keypoints - 1] >= features->feature_capacity) {
//     size_t old_size = features->feature_capacity;
//     size_t new_size = old_size * FEATURES_CAPACITY_GROWTH_FACTOR;
//     features->data = realloc(features->data, sizeof(feature_t *) * new_size);
//     features->feature_capacity = new_size;
//     for (size_t i = old_size; i < new_size; i++) {
//       features->data[i] = NULL;
//     }
//     // The above step is quite important because by default realloc will not
//     // initialize pointers to NULL, and there will be no way of knowing
//     // whether a feature exists.
//   }

//   // Expand positions dynamic array if needed
//   const size_t pos_id = features->num_positions;
//   if (pos_id >= features->position_capacity) {
//     size_t old_size = features->position_capacity;
//     size_t new_size = old_size * FEATURES_CAPACITY_GROWTH_FACTOR;
//     features->data = realloc(features->pos_data, sizeof(pos_t *) * new_size);
//     features->position_capacity = new_size;
//     for (size_t i = old_size; i < new_size; i++) {
//       features->pos_data[i] = NULL;
//     }
//     // The above step is quite important because by default realloc will not
//     // initialize pointers to NULL, and there will be no way of knowing
//     // whether a feature exists.
//   }

//   // Setup
//   TF_ROT(T_WC, C_WC);
//   TF_TRANS(T_WC, r_WC);

//   // Add feature
//   for (size_t i = 0; i < num_keypoints; i++) {
//     const size_t feature_id = feature_ids[i];
//     feature_t *f = malloc(sizeof(feature_t) * 1);
//     idf_setup(f, feature_id, pos_id, cam_params, C_WC, keypoints + i * 2);
//     features->data[feature_id] = f;
//     features->num_features++;
//   }

//   // Add inverse-depth "first-seen" position
//   pos_t *pos = malloc(sizeof(pos_t) * 1);
//   pos_setup(pos, r_WC);
//   features->pos_data[pos_id] = pos;
//   features->num_positions++;
// }

// /**
//  * Returns pointer to feature with `feature_id`.
//  */
// void features_get_xyz(const features_t *features,
//                       const size_t feature_id,
//                       feature_t **feature) {
//   *feature = features->data[feature_id];
// }

// /**
//  * Returns pointer to feature with `feature_id`.
//  */
// void features_get_idf(const features_t *features,
//                       const size_t feature_id,
//                       feature_t **feature,
//                       pos_t **pos) {
//   *feature = features->data[feature_id];
//   *pos = features->pos_data[(*feature)->pos_id];
// }

// /**
//  * Returns 3D point corresponding to feature.
//  */
// int features_point(const features_t *features,
//                    const size_t feature_id,
//                    real_t p_W[3]) {
//   if (features_exists(features, feature_id) != 0) {
//     return -1;
//   }

//   const feature_t *f = features->data[feature_id];
//   if (f->type == FEATURE_XYZ) {
//     p_W[0] = f->data[0];
//     p_W[1] = f->data[1];
//     p_W[2] = f->data[2];
//   } else if (f->type == FEATURE_INVERSE_DEPTH) {
//     const pos_t *pos = features->pos_data[f->pos_id];
//     idf_point(f, pos->data, p_W);
//   } else {
//     FATAL("Invalid feature type [%d]!\n", f->type);
//   }

//   return 0;
// }

// /**
//  * Return IDFB feature ids, keypoints and points.
//  */
// void idfb_points(idfb_t *idfb,
//                  size_t **feature_ids,
//                  real_t **points,
//                  size_t *num_points) {
//   *num_points = hmlen(idfb->params);
//   *feature_ids = malloc(sizeof(size_t) * *num_points);
//   *points = malloc(sizeof(real_t) * *num_points * 3);

//   for (size_t i = 0; i < hmlen(idfb->params); i++) {
//     (*feature_ids)[i] = idfb->params[i].key;
//     idf_point(&idfb->params[i].param, &idfb->pos, &(*points)[i * 3]);
//   }
// }

////////////////
// TIME-DELAY //
////////////////

/**
 * Setup time-delay.
 */
void time_delay_setup(time_delay_t *time_delay, const real_t td) {
  assert(time_delay != NULL);
  time_delay->marginalize = 0;
  time_delay->fix = 0;
  time_delay->data[0] = td;
}

/**
 * Copy time_delay.
 */
void time_delay_copy(const time_delay_t *src, time_delay_t *dst) {
  assert(src != NULL);
  assert(dst != NULL);

  dst->marginalize = src->marginalize;
  dst->fix = src->fix;
  dst->data[0] = src->data[0];
}

/**
 * Print time-delay.
 */
void time_delay_print(const char *prefix, const time_delay_t *td) {
  printf("[%s] time_delay: %f\n", prefix, td->data[0]);
}

///////////
// JOINT //
///////////

/**
 * Joint Angle Setup.
 */
void joint_setup(joint_t *joint,
                 const timestamp_t ts,
                 const int joint_idx,
                 const real_t theta) {
  assert(joint != NULL);
  joint->marginalize = 0;
  joint->fix = 0;

  joint->ts = ts;
  joint->joint_idx = joint_idx;
  joint->data[0] = theta;
}

/**
 * Copy joint.
 */
void joint_copy(const joint_t *src, joint_t *dst) {
  assert(src != NULL);
  assert(dst != NULL);

  dst->marginalize = src->marginalize;
  dst->fix = src->fix;

  dst->ts = src->ts;
  dst->joint_idx = src->joint_idx;
  dst->data[0] = src->data[0];
}

/**
 * Print Joint Angle.
 */
void joint_print(const char *prefix, const joint_t *joint) {
  printf("[%s] ", prefix);
  printf("ts: %ld ", joint->ts);
  printf("data: %f\n", joint->data[0]);
}

////////////////
// PARAMETERS //
////////////////

/**
 * Free parameter order.
 */
void param_order_free(param_order_t *hash) { hmfree(hash); }

/**
 * Return parameter type as a string
 */
void param_type_string(const int param_type, char *s) {
  switch (param_type) {
    case POSITION_PARAM:
      strcpy(s, "POSITION_PARAM");
      break;
    case ROTATION_PARAM:
      strcpy(s, "ROTATION_PARAM");
      break;
    case POSE_PARAM:
      strcpy(s, "POSE_PARAM");
      break;
    case EXTRINSIC_PARAM:
      strcpy(s, "EXTRINSIC_PARAM");
      break;
    case FIDUCIAL_PARAM:
      strcpy(s, "FIDUCIAL_PARAM");
      break;
    case VELOCITY_PARAM:
      strcpy(s, "VELOCITY_PARAM");
      break;
    case IMU_BIASES_PARAM:
      strcpy(s, "IMU_BIASES_PARAM");
      break;
    case FEATURE_PARAM:
      strcpy(s, "FEATURE_PARAM");
      break;
    case IDF_BEARING_PARAM:
      strcpy(s, "IDF_BEARING_PARAM");
      break;
    case IDF_POSITION_PARAM:
      strcpy(s, "IDF_POSITION_PARAM");
      break;
    case JOINT_PARAM:
      strcpy(s, "JOINT_PARAM");
      break;
    case CAMERA_PARAM:
      strcpy(s, "CAMERA_PARAM");
      break;
    case TIME_DELAY_PARAM:
      strcpy(s, "TIME_DELAY_PARAM");
      break;
    default:
      FATAL("Invalid param type [%d]!\n", param_type);
      break;
  }
}

/**
 * Return parameter global size depending on parameter type
 */
size_t param_global_size(const int param_type) {
  size_t param_size = 0;

  switch (param_type) {
    case POSITION_PARAM:
      param_size = 3;
      break;
    case ROTATION_PARAM:
      param_size = 4;
      break;
    case POSE_PARAM:
    case EXTRINSIC_PARAM:
    case FIDUCIAL_PARAM:
      param_size = 7;
      break;
    case VELOCITY_PARAM:
      param_size = 3;
      break;
    case IMU_BIASES_PARAM:
      param_size = 6;
      break;
    case FEATURE_PARAM:
      param_size = 3;
      break;
    case IDF_BEARING_PARAM:
      param_size = 3;
      break;
    case JOINT_PARAM:
      param_size = 1;
      break;
    case CAMERA_PARAM:
      param_size = 8;
      break;
    case TIME_DELAY_PARAM:
      param_size = 1;
      break;
    default:
      FATAL("Invalid param type [%d]!\n", param_type);
      break;
  }

  return param_size;
}

/**
 * Return parameter local size depending on parameter type
 */
size_t param_local_size(const int param_type) {
  size_t param_size = 0;

  switch (param_type) {
    case POSITION_PARAM:
      param_size = 3;
      break;
    case ROTATION_PARAM:
      param_size = 3;
      break;
    case POSE_PARAM:
    case EXTRINSIC_PARAM:
    case FIDUCIAL_PARAM:
      param_size = 6;
      break;
    case VELOCITY_PARAM:
      param_size = 3;
      break;
    case IMU_BIASES_PARAM:
      param_size = 6;
      break;
    case FEATURE_PARAM:
      param_size = 3;
      break;
    case IDF_BEARING_PARAM:
      param_size = 3;
      break;
    case JOINT_PARAM:
      param_size = 1;
      break;
    case CAMERA_PARAM:
      param_size = 8;
      break;
    case TIME_DELAY_PARAM:
      param_size = 1;
      break;
    default:
      FATAL("Invalid param type [%d]!\n", param_type);
      break;
  }

  return param_size;
}

/**
 * Print parameter order.
 */
void param_order_print(const param_order_t *hash) {
  for (int idx = 0; idx < hmlen(hash); idx++) {
    const int param_type = hash[idx].type;
    const int col_idx = hash[idx].idx;
    if (col_idx != -1) {
      char s[100] = {0};
      param_type_string(param_type, s);
      printf("param[%d]: %s, idx: %d\n", idx, s, col_idx);
    }
  }
}

/**
 * Check if param has already been added.
 */
int param_order_exists(param_order_t **hash, real_t *data) {
  return hmgetp_null(*hash, data) != NULL;
}

/**
 * Add parameter to hash
 */
void param_order_add(param_order_t **hash,
                     const int param_type,
                     const int fix,
                     real_t *data,
                     int *col_idx) {
  if (fix == 0) {
    param_order_t kv = {data, *col_idx, param_type, fix};
    hmputs(*hash, kv);
    *col_idx += param_local_size(param_type);
  } else {
    param_order_t kv = {data, -1, param_type, fix};
    hmputs(*hash, kv);
  }
}

/** Add position parameter **/
void param_order_add_position(param_order_t **h, pos_t *p, int *c) {
  void *data = p->data;
  int fix = p->fix || p->marginalize;
  param_order_add(h, POSITION_PARAM, fix, data, c);
}

/** Add rotation parameter **/
void param_order_add_rotation(param_order_t **h, rot_t *p, int *c) {
  void *data = p->data;
  int fix = p->fix || p->marginalize;
  param_order_add(h, ROTATION_PARAM, fix, data, c);
}

/** Add pose parameter **/
void param_order_add_pose(param_order_t **h, pose_t *p, int *c) {
  void *data = p->data;
  int fix = p->fix || p->marginalize;
  param_order_add(h, POSE_PARAM, fix, data, c);
}

/** Add extrinsic parameter **/
void param_order_add_extrinsic(param_order_t **h, extrinsic_t *p, int *c) {
  void *data = p->data;
  int fix = p->fix || p->marginalize;
  param_order_add(h, EXTRINSIC_PARAM, fix, data, c);
}

/** Add fiducial parameter **/
void param_order_add_fiducial(param_order_t **h, fiducial_t *p, int *c) {
  void *data = p->data;
  int fix = p->fix || p->marginalize;
  param_order_add(h, FIDUCIAL_PARAM, fix, data, c);
}

/** Add velocity parameter **/
void param_order_add_velocity(param_order_t **h, velocity_t *p, int *c) {
  void *data = p->data;
  int fix = p->fix || p->marginalize;
  param_order_add(h, VELOCITY_PARAM, fix, data, c);
}

/** Add IMU biases parameter **/
void param_order_add_imu_biases(param_order_t **h, imu_biases_t *p, int *c) {
  void *data = p->data;
  int fix = p->fix || p->marginalize;
  param_order_add(h, IMU_BIASES_PARAM, fix, data, c);
}

/** Add feature parameter **/
void param_order_add_feature(param_order_t **h, feature_t *p, int *c) {
  void *data = p->data;
  int fix = p->fix || p->marginalize;
  param_order_add(h, FEATURE_PARAM, fix, data, c);
}

/** Add joint parameter **/
void param_order_add_joint(param_order_t **h, joint_t *p, int *c) {
  void *data = p->data;
  int fix = p->fix || p->marginalize;
  param_order_add(h, JOINT_PARAM, fix, data, c);
}

/** Add camera parameter **/
void param_order_add_camera(param_order_t **h, camera_params_t *p, int *c) {
  void *data = p->data;
  int fix = p->fix || p->marginalize;
  param_order_add(h, CAMERA_PARAM, fix, data, c);
}

/** Add time delay parameter **/
void param_order_add_time_delay(param_order_t **h, time_delay_t *p, int *c) {
  void *data = p->data;
  int fix = p->fix || p->marginalize;
  param_order_add(h, TIME_DELAY_PARAM, fix, data, c);
}

////////////
// FACTOR //
////////////

int check_factor_jacobian(const void *factor,
                          FACTOR_EVAL_PTR,
                          real_t **params,
                          real_t **jacobians,
                          const int r_size,
                          const int param_size,
                          const int param_idx,
                          const real_t step_size,
                          const real_t tol,
                          const int verbose) {
  // Form jacobian name
  char J_name[10] = {0};
  if (snprintf(J_name, 10, "J%d", param_idx) <= 0) {
    return -1;
  }

  // Setup
  real_t *r = calloc(r_size, sizeof(real_t));
  real_t *J_numdiff = calloc(r_size * param_size, sizeof(real_t));

  // Evaluate factor
  if (factor_eval(factor, params, r, NULL) != 0) {
    free(r);
    free(J_numdiff);
    return -2;
  }

  // Numerical diff - forward finite difference
  for (int i = 0; i < param_size; i++) {
    real_t *r_fwd = calloc(r_size, sizeof(real_t));
    real_t *r_diff = calloc(r_size, sizeof(real_t));

    params[param_idx][i] += step_size;
    factor_eval(factor, params, r_fwd, NULL);
    params[param_idx][i] -= step_size;

    vec_sub(r_fwd, r, r_diff, r_size);
    vec_scale(r_diff, r_size, 1.0 / step_size);
    mat_col_set(J_numdiff, param_size, r_size, i, r_diff);

    free(r_fwd);
    free(r_diff);
  }

  // Check jacobian
  const int retval = check_jacobian(J_name,
                                    J_numdiff,
                                    jacobians[param_idx],
                                    r_size,
                                    param_size,
                                    tol,
                                    verbose);
  free(r);
  free(J_numdiff);

  return retval;
}

int check_factor_so3_jacobian(const void *factor,
                              FACTOR_EVAL_PTR,
                              real_t **params,
                              real_t **jacobians,
                              const int r_size,
                              const int param_idx,
                              const real_t step_size,
                              const real_t tol,
                              const int verbose) {
  // Form jacobian name
  char J_name[10] = {0};
  if (snprintf(J_name, 10, "J%d", param_idx) <= 0) {
    return -1;
  }

  // Setup
  const int param_size = 3;
  real_t *r = calloc(r_size, sizeof(real_t));
  real_t *J_numdiff = calloc(r_size * param_size, sizeof(real_t));

  // Evaluate factor
  if (factor_eval(factor, params, r, NULL) != 0) {
    free(r);
    free(J_numdiff);
    return -2;
  }

  for (int i = 0; i < param_size; i++) {
    real_t *r_fwd = calloc(r_size, sizeof(real_t));
    real_t *r_diff = calloc(r_size, sizeof(real_t));

    quat_perturb(params[param_idx], i, step_size);
    factor_eval(factor, params, r_fwd, NULL);
    quat_perturb(params[param_idx], i, -step_size);

    vec_sub(r_fwd, r, r_diff, r_size);
    vec_scale(r_diff, r_size, 1.0 / step_size);
    mat_col_set(J_numdiff, param_size, r_size, i, r_diff);

    free(r_fwd);
    free(r_diff);
  }

  // Check Jacobian
  const int retval = check_jacobian(J_name,
                                    J_numdiff,
                                    jacobians[param_idx],
                                    r_size,
                                    param_size,
                                    tol,
                                    verbose);
  free(r);
  free(J_numdiff);

  return retval;
}

/////////////////
// POSE FACTOR //
/////////////////

/**
 * Setup pose factor
 */
void pose_factor_setup(pose_factor_t *factor,
                       pose_t *pose,
                       const real_t var[6]) {
  assert(factor != NULL);
  assert(pose != NULL);
  assert(var != NULL);

  // Parameters
  factor->pose_est = pose;

  // Measurement
  factor->pos_meas[0] = pose->data[0];
  factor->pos_meas[1] = pose->data[1];
  factor->pos_meas[2] = pose->data[2];
  factor->quat_meas[0] = pose->data[3];
  factor->quat_meas[1] = pose->data[4];
  factor->quat_meas[2] = pose->data[5];
  factor->quat_meas[3] = pose->data[6];

  // Measurement covariance matrix
  zeros(factor->covar, 6, 6);
  factor->covar[0] = var[0];
  factor->covar[7] = var[1];
  factor->covar[14] = var[2];
  factor->covar[21] = var[3];
  factor->covar[28] = var[4];
  factor->covar[35] = var[5];

  // Square root information matrix
  zeros(factor->sqrt_info, 6, 6);
  factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);
  factor->sqrt_info[7] = sqrt(1.0 / factor->covar[7]);
  factor->sqrt_info[14] = sqrt(1.0 / factor->covar[14]);
  factor->sqrt_info[21] = sqrt(1.0 / factor->covar[21]);
  factor->sqrt_info[28] = sqrt(1.0 / factor->covar[28]);
  factor->sqrt_info[35] = sqrt(1.0 / factor->covar[35]);

  // Factor residuals, parameters and Jacobians
  factor->r_size = 6;
  factor->num_params = 1;
  factor->param_types[0] = POSE_PARAM;
  factor->params[0] = factor->pose_est->data;
  factor->jacs[0] = factor->J_pose;
}

/**
 * Evaluate pose factor
 * @returns `0` for success, `-1` for failure
 */
int pose_factor_eval(void *factor_ptr) {
  assert(factor_ptr != NULL);
  pose_factor_t *factor = (pose_factor_t *) factor_ptr;

  // Map params
  const real_t r_est[3] = {factor->params[0][0],
                           factor->params[0][1],
                           factor->params[0][2]};
  const real_t q_est[4] = {factor->params[0][3],
                           factor->params[0][4],
                           factor->params[0][5],
                           factor->params[0][6]};
  const real_t *r_meas = factor->pos_meas;
  const real_t *q_meas = factor->quat_meas;

  // Calculate pose error
  // -- Translation error
  // dr = r_meas - r_est;
  real_t dr[3] = {0};
  dr[0] = r_meas[0] - r_est[0];
  dr[1] = r_meas[1] - r_est[1];
  dr[2] = r_meas[2] - r_est[2];

  // -- Rotation error
  // dq = quat_mul(quat_inv(q_meas), q_est);
  real_t dq[4] = {0};
  real_t q_meas_inv[4] = {0};
  quat_inv(q_meas, q_meas_inv);
  quat_mul(q_meas_inv, q_est, dq);

  // dtheta = 2 * dq;
  real_t dtheta[3] = {0};
  dtheta[0] = 2 * dq[1];
  dtheta[1] = 2 * dq[2];
  dtheta[2] = 2 * dq[3];

  // -- Set residuals
  // r = factor.sqrt_info * [dr; dtheta];
  real_t r[6] = {0};
  r[0] = dr[0];
  r[1] = dr[1];
  r[2] = dr[2];
  r[3] = dtheta[0];
  r[4] = dtheta[1];
  r[5] = dtheta[2];
  dot(factor->sqrt_info, 6, 6, r, 6, 1, factor->r);

  // Calculate Jacobians
  const real_t dqw = dq[0];
  const real_t dqx = dq[1];
  const real_t dqy = dq[2];
  const real_t dqz = dq[3];

  real_t J[6 * 6] = {0};

  J[0] = -1.0;
  J[1] = 0.0;
  J[2] = 0.0;
  J[6] = 0.0;
  J[7] = -1.0;
  J[8] = 0.0;
  J[12] = 0.0;
  J[13] = 0.0;
  J[14] = -1.0;

  J[21] = dqw;
  J[22] = -dqz;
  J[23] = dqy;
  J[27] = dqz;
  J[28] = dqw;
  J[29] = -dqx;
  J[33] = -dqy;
  J[34] = dqx;
  J[35] = dqw;

  dot(factor->sqrt_info, 6, 6, J, 6, 6, factor->jacs[0]);

  return 0;
}

///////////////
// BA FACTOR //
///////////////

/**
 * Setup bundle adjustment factor
 */
void ba_factor_setup(ba_factor_t *factor,
                     pose_t *pose,
                     feature_t *feature,
                     camera_params_t *camera,
                     const real_t z[2],
                     const real_t var[2]) {
  assert(factor != NULL);
  assert(pose != NULL);
  assert(feature != NULL);
  assert(camera != NULL);
  assert(var != NULL);

  // Parameters
  factor->pose = pose;
  factor->feature = feature;
  factor->camera = camera;
  factor->num_params = 3;

  // Measurement covariance
  factor->covar[0] = var[0];
  factor->covar[1] = 0.0;
  factor->covar[2] = 0.0;
  factor->covar[3] = var[1];

  // Square-root information matrix
  factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);
  factor->sqrt_info[1] = 0.0;
  factor->sqrt_info[2] = 0.0;
  factor->sqrt_info[3] = sqrt(1.0 / factor->covar[3]);

  // Measurement
  factor->z[0] = z[0];
  factor->z[1] = z[1];

  // Factor parameters, residuals and Jacobians
  factor->r_size = 2;
  factor->num_params = 3;

  factor->param_types[0] = POSE_PARAM;
  factor->param_types[1] = FEATURE_PARAM;
  factor->param_types[2] = CAMERA_PARAM;

  factor->params[0] = factor->pose->data;
  factor->params[1] = factor->feature->data;
  factor->params[2] = factor->camera->data;

  factor->jacs[0] = factor->J_pose;
  factor->jacs[1] = factor->J_feature;
  factor->jacs[2] = factor->J_camera;
}

/**
 * Camera pose jacobian
 */
static void ba_factor_pose_jacobian(const real_t Jh_weighted[2 * 3],
                                    const real_t T_WC[4 * 4],
                                    const real_t p_W[3],
                                    real_t *J) {
  // Pre-check
  if (J == NULL) {
    return;
  }

  // Jh_weighted = -1 * sqrt_info * Jh;
  // J_pos = Jh_weighted * -C_CW;
  // J_rot = Jh_weighted * -C_CW * hat(p_W - r_WC) * -C_WC;
  // J = [J_pos, J_rot]

  // Setup
  real_t C_WC[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};
  real_t r_WC[3] = {0};
  tf_rot_get(T_WC, C_WC);
  tf_trans_get(T_WC, r_WC);
  mat_transpose(C_WC, 3, 3, C_CW);

  // J_pos = -1 * sqrt_info * Jh * -C_CW;
  real_t J_pos[2 * 3] = {0};
  real_t neg_C_CW[3 * 3] = {0};
  mat_copy(C_CW, 3, 3, neg_C_CW);
  mat_scale(neg_C_CW, 3, 3, -1.0);
  dot(Jh_weighted, 2, 3, neg_C_CW, 3, 3, J_pos);

  J[0] = J_pos[0];
  J[1] = J_pos[1];
  J[2] = J_pos[2];

  J[6] = J_pos[3];
  J[7] = J_pos[4];
  J[8] = J_pos[5];

  /**
   * Jh_weighted = -1 * sqrt_info * Jh;
   * J_rot = Jh_weighted * -C_CW * hat(p_W - r_WC) * -C_WC;
   * where:
   *
   *   A = -C_CW;
   *   B = hat(p_W - r_WC);
   *   C = -C_WC;
   */
  real_t J_rot[2 * 3] = {0};
  real_t A[3 * 3] = {0};
  mat_copy(neg_C_CW, 3, 3, A);

  real_t B[3 * 3] = {0};
  real_t dp[3] = {0};
  dp[0] = p_W[0] - r_WC[0];
  dp[1] = p_W[1] - r_WC[1];
  dp[2] = p_W[2] - r_WC[2];
  hat(dp, B);

  real_t C[3 * 3] = {0};
  mat_copy(C_WC, 3, 3, C);
  mat_scale(C, 3, 3, -1.0);

  real_t AB[3 * 3] = {0};
  real_t ABC[3 * 3] = {0};
  dot(A, 3, 3, B, 3, 3, AB);
  dot(AB, 3, 3, C, 3, 3, ABC);
  dot(Jh_weighted, 2, 3, ABC, 3, 3, J_rot);

  J[3] = J_rot[0];
  J[4] = J_rot[1];
  J[5] = J_rot[2];

  J[9] = J_rot[3];
  J[10] = J_rot[4];
  J[11] = J_rot[5];
}

/**
 * Feature jacobian
 */
static void ba_factor_feature_jacobian(const real_t Jh_weighted[2 * 3],
                                       const real_t T_WC[4 * 4],
                                       real_t *J) {
  // Pre-check
  if (J == NULL) {
    return;
  }

  // Jh_weighted = -1 * sqrt_info * Jh;
  // J = Jh_weighted * C_CW;
  real_t C_WC[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};
  tf_rot_get(T_WC, C_WC);
  mat_transpose(C_WC, 3, 3, C_CW);
  dot(Jh_weighted, 2, 3, C_CW, 3, 3, J);
}

/**
 * Camera parameters jacobian
 */
static void ba_factor_camera_jacobian(const real_t neg_sqrt_info[2 * 2],
                                      const real_t J_cam_params[2 * 8],
                                      real_t *J) {
  // Pre-check
  if (J == NULL) {
    return;
  }

  // J = -1 * sqrt_info * J_cam_params;
  dot(neg_sqrt_info, 2, 2, J_cam_params, 2, 8, J);
}

/**
 * Evaluate bundle adjustment factor
 * @returns `0` for success, `-1` for failure
 */
int ba_factor_eval(void *factor_ptr) {
  assert(factor_ptr != NULL);
  ba_factor_t *factor = (ba_factor_t *) factor_ptr;

  // Map params
  real_t T_WCi[4 * 4] = {0};
  tf(factor->params[0], T_WCi);
  const real_t *p_W = factor->params[1];
  const real_t *cam_params = factor->params[2];

  // Calculate residuals
  // -- Project point from world to image plane
  real_t T_CiW[4 * 4] = {0};
  real_t p_Ci[3] = {0};
  real_t z_hat[2];
  tf_inv(T_WCi, T_CiW);
  tf_point(T_CiW, p_W, p_Ci);
  camera_project(factor->camera, p_Ci, z_hat);
  // -- Residual
  real_t r[2] = {0};
  r[0] = factor->z[0] - z_hat[0];
  r[1] = factor->z[1] - z_hat[1];
  // -- Weighted residual
  dot(factor->sqrt_info, 2, 2, r, 2, 1, factor->r);

  // Calculate jacobians
  // -- Form: -1 * sqrt_info
  real_t neg_sqrt_info[2 * 2] = {0};
  mat_copy(factor->sqrt_info, 2, 2, neg_sqrt_info);
  mat_scale(neg_sqrt_info, 2, 2, -1.0);
  // -- Form: Jh_weighted = -1 * sqrt_info * Jh
  real_t Jh[2 * 3] = {0};
  real_t Jh_w[2 * 3] = {0};
  pinhole_radtan4_project_jacobian(cam_params, p_Ci, Jh);
  dot(neg_sqrt_info, 2, 2, Jh, 2, 3, Jh_w);
  // -- Form: J_cam_params
  real_t J_cam_params[2 * 8] = {0};
  pinhole_radtan4_params_jacobian(cam_params, p_Ci, J_cam_params);
  // -- Fill jacobians
  ba_factor_pose_jacobian(Jh_w, T_WCi, p_W, factor->jacs[0]);
  ba_factor_feature_jacobian(Jh_w, T_WCi, factor->jacs[1]);
  ba_factor_camera_jacobian(neg_sqrt_info, J_cam_params, factor->jacs[2]);

  return 0;
}

///////////////////
// CAMERA FACTOR //
///////////////////

/**
 * Setup camera factor
 */
void camera_factor_setup(camera_factor_t *factor,
                         pose_t *pose,
                         extrinsic_t *extrinsic,
                         feature_t *feature,
                         camera_params_t *camera,
                         const real_t z[2],
                         const real_t var[2]) {
  assert(factor != NULL);
  assert(pose != NULL);
  assert(extrinsic != NULL);
  assert(feature != NULL);
  assert(camera != NULL);
  assert(z != NULL);
  assert(var != NULL);

  // Parameters
  factor->pose = pose;
  factor->extrinsic = extrinsic;
  factor->feature = feature;
  factor->camera = camera;

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

  // Measurement
  factor->z[0] = z[0];
  factor->z[1] = z[1];

  // Parameters, residuals, jacobians
  factor->r_size = 2;
  factor->num_params = 4;

  factor->param_types[0] = POSE_PARAM;
  factor->param_types[1] = EXTRINSIC_PARAM;
  factor->param_types[2] = FEATURE_PARAM;
  factor->param_types[3] = CAMERA_PARAM;

  factor->params[0] = factor->pose->data;
  factor->params[1] = factor->extrinsic->data;
  factor->params[2] = factor->feature->data;
  factor->params[3] = factor->camera->data;

  factor->jacs[0] = factor->J_pose;
  factor->jacs[1] = factor->J_extrinsic;
  factor->jacs[2] = factor->J_feature;
  factor->jacs[3] = factor->J_camera;
}

/**
 * Pose jacobian
 */
static void camera_factor_pose_jacobian(const real_t Jh_w[2 * 3],
                                        const real_t T_WB[4 * 4],
                                        const real_t T_BC[4 * 4],
                                        const real_t p_W[3],
                                        real_t J[2 * 6]) {
  assert(Jh_w != NULL);
  assert(T_BC != NULL);
  assert(T_WB != NULL);
  assert(p_W != NULL);
  assert(J != NULL);

  // Jh_w = -1 * sqrt_info * Jh;
  // J_pos = Jh_w * C_CB * -C_BW;
  // J_rot = Jh_w * C_CB * C_BW * hat(p_W - r_WB) * -C_WB;
  // J = [J_pos, J_rot];

  // Setup
  real_t C_BW[3 * 3] = {0};
  real_t C_CB[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};

  TF_ROT(T_WB, C_WB);
  TF_ROT(T_BC, C_BC);
  mat_transpose(C_WB, 3, 3, C_BW);
  mat_transpose(C_BC, 3, 3, C_CB);
  dot(C_CB, 3, 3, C_BW, 3, 3, C_CW);

  // Form: -C_BW
  real_t neg_C_BW[3 * 3] = {0};
  mat_copy(C_BW, 3, 3, neg_C_BW);
  mat_scale(neg_C_BW, 3, 3, -1.0);

  // Form: -C_CW
  real_t neg_C_CW[3 * 3] = {0};
  dot(C_CB, 3, 3, neg_C_BW, 3, 3, neg_C_CW);

  // Form: -C_WB
  real_t neg_C_WB[3 * 3] = {0};
  mat_copy(C_WB, 3, 3, neg_C_WB);
  mat_scale(neg_C_WB, 3, 3, -1.0);

  // Form: C_CB * -C_BW * hat(p_W - r_WB) * -C_WB
  real_t p[3] = {0};
  real_t S[3 * 3] = {0};
  TF_TRANS(T_WB, r_WB);
  vec_sub(p_W, r_WB, p, 3);
  hat(p, S);

  real_t A[3 * 3] = {0};
  real_t B[3 * 3] = {0};
  dot(neg_C_CW, 3, 3, S, 3, 3, A);
  dot(A, 3, 3, neg_C_WB, 3, 3, B);

  // Form: J_pos = Jh_w * C_CB * -C_BW;
  real_t J_pos[2 * 3] = {0};
  dot(Jh_w, 2, 3, neg_C_CW, 3, 3, J_pos);

  J[0] = J_pos[0];
  J[1] = J_pos[1];
  J[2] = J_pos[2];

  J[6] = J_pos[3];
  J[7] = J_pos[4];
  J[8] = J_pos[5];

  // Form: J_rot = Jh_w * C_CB * -C_BW * hat(p_W - r_WB) * -C_WB;
  real_t J_rot[2 * 3] = {0};
  dot(Jh_w, 2, 3, B, 3, 3, J_rot);

  J[3] = J_rot[0];
  J[4] = J_rot[1];
  J[5] = J_rot[2];

  J[9] = J_rot[3];
  J[10] = J_rot[4];
  J[11] = J_rot[5];
}

/**
 * Body-camera extrinsic jacobian
 */
static void camera_factor_extrinsic_jacobian(const real_t Jh_w[2 * 3],
                                             const real_t T_BC[4 * 4],
                                             const real_t p_C[3],
                                             real_t J[2 * 6]) {
  assert(Jh_w != NULL);
  assert(T_BC != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  // Jh_w = -1 * sqrt_info * Jh;
  // J_pos = Jh_w * -C_CB;
  // J_rot = Jh_w * C_CB * hat(C_BC * p_C);

  // Setup
  real_t C_BC[3 * 3] = {0};
  real_t C_CB[3 * 3] = {0};
  real_t C_BW[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};

  tf_rot_get(T_BC, C_BC);
  mat_transpose(C_BC, 3, 3, C_CB);
  dot(C_CB, 3, 3, C_BW, 3, 3, C_CW);

  // Form: -C_CB
  real_t neg_C_CB[3 * 3] = {0};
  mat_copy(C_CB, 3, 3, neg_C_CB);
  mat_scale(neg_C_CB, 3, 3, -1.0);

  // Form: -C_BC
  real_t neg_C_BC[3 * 3] = {0};
  mat_copy(C_BC, 3, 3, neg_C_BC);
  mat_scale(neg_C_BC, 3, 3, -1.0);

  // Form: -C_CB * hat(C_BC * p_C) * -C_BC
  real_t p[3] = {0};
  real_t S[3 * 3] = {0};
  dot(C_BC, 3, 3, p_C, 3, 1, p);
  hat(p, S);

  real_t A[3 * 3] = {0};
  real_t B[3 * 3] = {0};
  dot(neg_C_CB, 3, 3, S, 3, 3, A);
  dot(A, 3, 3, neg_C_BC, 3, 3, B);

  // Form: J_rot = Jh_w * -C_CB;
  real_t J_pos[2 * 3] = {0};
  dot(Jh_w, 2, 3, neg_C_CB, 3, 3, J_pos);

  J[0] = J_pos[0];
  J[1] = J_pos[1];
  J[2] = J_pos[2];

  J[6] = J_pos[3];
  J[7] = J_pos[4];
  J[8] = J_pos[5];

  // Form: J_rot = Jh_w * -C_CB * hat(C_BC * p_C) * -C_BC;
  real_t J_rot[2 * 3] = {0};
  dot(Jh_w, 2, 3, B, 3, 3, J_rot);

  J[3] = J_rot[0];
  J[4] = J_rot[1];
  J[5] = J_rot[2];

  J[9] = J_rot[3];
  J[10] = J_rot[4];
  J[11] = J_rot[5];
}

/**
 * Camera parameters jacobian
 */
static void camera_factor_camera_jacobian(const real_t neg_sqrt_info[2 * 2],
                                          const real_t J_cam_params[2 * 8],
                                          real_t J[2 * 8]) {
  assert(neg_sqrt_info != NULL);
  assert(J_cam_params != NULL);
  assert(J != NULL);

  // J = -1 * sqrt_info * J_cam_params;
  dot(neg_sqrt_info, 2, 2, J_cam_params, 2, 8, J);
}

/**
 * Feature jacobian
 */
static void camera_factor_feature_jacobian(const real_t Jh_w[2 * 3],
                                           const real_t T_WB[4 * 4],
                                           const real_t T_BC[4 * 4],
                                           real_t J[2 * 3]) {
  if (J == NULL) {
    return;
  }
  assert(Jh_w != NULL);
  assert(T_WB != NULL);
  assert(T_BC != NULL);
  assert(J != NULL);

  // Jh_w = -1 * sqrt_info * Jh;
  // J = Jh_w * C_CW;

  // Setup
  real_t T_WC[4 * 4] = {0};
  real_t C_WC[3 * 3] = {0};
  real_t C_CW[3 * 3] = {0};
  dot(T_WB, 4, 4, T_BC, 4, 4, T_WC);
  tf_rot_get(T_WC, C_WC);
  mat_transpose(C_WC, 3, 3, C_CW);

  // Form: J = -1 * sqrt_info * Jh * C_CW;
  dot(Jh_w, 2, 3, C_CW, 3, 3, J);
}

/**
 * Evaluate vision factor
 * @returns `0` for success, `-1` for failure
 */
int camera_factor_eval(void *factor_ptr) {
  camera_factor_t *factor = (camera_factor_t *) factor_ptr;
  assert(factor != NULL);
  assert(factor->pose);
  assert(factor->extrinsic);
  assert(factor->feature);
  assert(factor->camera);

  // Map params
  real_t T_WB[4 * 4] = {0};
  real_t T_BCi[4 * 4] = {0};
  tf(factor->params[0], T_WB);
  tf(factor->params[1], T_BCi);
  const real_t *p_W = factor->params[2];
  const real_t *cam_params = factor->params[3];

  // Form camera pose
  TF_CHAIN(T_WCi, 2, T_WB, T_BCi);
  TF_INV(T_WCi, T_CiW);

  // Transform feature from world to camera frame
  TF_POINT(T_CiW, p_W, p_Ci);

  // Calculate residuals
  // -- Project point from world to image plane
  real_t z_hat[2];
  camera_project(factor->camera, p_Ci, z_hat);
  // -- Residual
  real_t r[2] = {0};
  r[0] = factor->z[0] - z_hat[0];
  r[1] = factor->z[1] - z_hat[1];
  // -- Weighted residual
  dot(factor->sqrt_info, 2, 2, r, 2, 1, factor->r);

  // Calculate jacobians
  // -- Form: -1 * sqrt_info
  real_t neg_sqrt_info[2 * 2] = {0};
  mat_copy(factor->sqrt_info, 2, 2, neg_sqrt_info);
  mat_scale(neg_sqrt_info, 2, 2, -1.0);
  // -- Form: Jh_ = -1 * sqrt_info * Jh
  real_t Jh[2 * 3] = {0};
  real_t Jh_[2 * 3] = {0};
  pinhole_radtan4_project_jacobian(cam_params, p_Ci, Jh);
  dot(neg_sqrt_info, 2, 2, Jh, 2, 3, Jh_);
  // -- Form: J_cam_params
  real_t J_cam_params[2 * 8] = {0};
  pinhole_radtan4_params_jacobian(cam_params, p_Ci, J_cam_params);
  // -- Fill Jacobians
  camera_factor_pose_jacobian(Jh_, T_WB, T_BCi, p_W, factor->jacs[0]);
  camera_factor_extrinsic_jacobian(Jh_, T_BCi, p_Ci, factor->jacs[1]);
  camera_factor_feature_jacobian(Jh_, T_WB, T_BCi, factor->jacs[2]);
  camera_factor_camera_jacobian(neg_sqrt_info, J_cam_params, factor->jacs[3]);

  return 0;
}

/////////////////////////////////////////
// INVERSE-DEPTH FEATURES (IDF) FACTOR //
/////////////////////////////////////////

// /**
//  * Pose jacobian
//  */
// static void idf_factor_pose_jacobian(const real_t Jh_w[2 * 3],
//                                      const real_t T_WB[3 * 3],
//                                      const real_t T_BC[3 * 3],
//                                      const real_t p_W[3],
//                                      real_t J[2 * 6]) {
//   assert(Jh_w != NULL);
//   assert(T_BC != NULL);
//   assert(T_WB != NULL);
//   assert(p_W != NULL);
//   assert(J != NULL);

//   // Jh_w = -1 * sqrt_info * Jh;
//   // J_pos = Jh_w * C_CB * -C_BW;
//   // J_rot = Jh_w * C_CB * C_BW * hat(p_W - r_WB) * -C_WB;
//   // J = [J_pos, J_rot];

//   // Setup
//   real_t C_BW[3 * 3] = {0};
//   real_t C_CB[3 * 3] = {0};

//   TF_ROT(T_WB, C_WB);
//   TF_ROT(T_BC, C_BC);
//   mat_transpose(C_WB, 3, 3, C_BW);
//   mat_transpose(C_BC, 3, 3, C_CB);
//   DOT(C_CB, 3, 3, C_BW, 3, 3, C_CW);

//   // Form: -C_BW
//   real_t neg_C_BW[3 * 3] = {0};
//   mat_copy(C_BW, 3, 3, neg_C_BW);
//   mat_scale(neg_C_BW, 3, 3, -1.0);

//   // Form: -C_CW
//   real_t neg_C_CW[3 * 3] = {0};
//   dot(C_CB, 3, 3, neg_C_BW, 3, 3, neg_C_CW);

//   // Form: -C_WB
//   real_t neg_C_WB[3 * 3] = {0};
//   mat_copy(C_WB, 3, 3, neg_C_WB);
//   mat_scale(neg_C_WB, 3, 3, -1.0);

//   // Form: C_CB * -C_BW * hat(p_W - r_WB) * -C_WB
//   real_t p[3] = {0};
//   real_t S[3 * 3] = {0};
//   TF_TRANS(T_WB, r_WB);
//   vec_sub(p_W, r_WB, p, 3);
//   hat(p, S);

//   DOT(neg_C_CW, 3, 3, S, 3, 3, A);
//   DOT(A, 3, 3, neg_C_WB, 3, 3, B);

//   // Form: J_pos = Jh_w * C_CB * -C_BW;
//   DOT(Jh_w, 2, 3, neg_C_CW, 3, 3, J_pos);
//   J[0] = J_pos[0];
//   J[1] = J_pos[1];
//   J[2] = J_pos[2];

//   J[6] = J_pos[3];
//   J[7] = J_pos[4];
//   J[8] = J_pos[5];

//   // Form: J_rot = Jh_w * C_CB * -C_BW * hat(p_W - r_WB) * -C_WB;
//   DOT(Jh_w, 2, 3, B, 3, 3, J_rot);

//   J[3] = J_rot[0];
//   J[4] = J_rot[1];
//   J[5] = J_rot[2];

//   J[9] = J_rot[3];
//   J[10] = J_rot[4];
//   J[11] = J_rot[5];
// }

// /**
//  * Body-camera extrinsic jacobian
//  */
// static void idf_factor_extrinsic_jacobian(const real_t Jh_w[2 * 3],
//                                           const real_t T_BC[3 * 3],
//                                           const real_t p_C[3],
//                                           real_t J[2 * 6]) {
//   assert(Jh_w != NULL);
//   assert(T_BC != NULL);
//   assert(p_C != NULL);
//   assert(J != NULL);

//   // Jh_w = -1 * sqrt_info * Jh;
//   // J_pos = Jh_w * -C_CB;
//   // J_rot = Jh_w * C_CB * hat(C_BC * p_C);

//   // Setup
//   real_t C_CB[3 * 3] = {0};
//   real_t C_BW[3 * 3] = {0};
//   real_t C_CW[3 * 3] = {0};

//   TF_ROT(T_BC, C_BC);
//   mat_transpose(C_BC, 3, 3, C_CB);
//   dot(C_CB, 3, 3, C_BW, 3, 3, C_CW);

//   // Form: -C_CB
//   real_t neg_C_CB[3 * 3] = {0};
//   mat_copy(C_CB, 3, 3, neg_C_CB);
//   mat_scale(neg_C_CB, 3, 3, -1.0);

//   // Form: -C_BC
//   real_t neg_C_BC[3 * 3] = {0};
//   mat_copy(C_BC, 3, 3, neg_C_BC);
//   mat_scale(neg_C_BC, 3, 3, -1.0);

//   // Form: -C_CB * hat(C_BC * p_C) * -C_BC
//   real_t p[3] = {0};
//   real_t S[3 * 3] = {0};
//   dot(C_BC, 3, 3, p_C, 3, 1, p);
//   hat(p, S);

//   real_t A[3 * 3] = {0};
//   real_t B[3 * 3] = {0};
//   dot(neg_C_CB, 3, 3, S, 3, 3, A);
//   dot(A, 3, 3, neg_C_BC, 3, 3, B);

//   // Form: J_rot = Jh_w * -C_CB;
//   real_t J_pos[2 * 3] = {0};
//   dot(Jh_w, 2, 3, neg_C_CB, 3, 3, J_pos);

//   J[0] = J_pos[0];
//   J[1] = J_pos[1];
//   J[2] = J_pos[2];

//   J[6] = J_pos[3];
//   J[7] = J_pos[4];
//   J[8] = J_pos[5];

//   // Form: J_rot = Jh_w * -C_CB * hat(C_BC * p_C) * -C_BC;
//   real_t J_rot[2 * 3] = {0};
//   dot(Jh_w, 2, 3, B, 3, 3, J_rot);

//   J[3] = J_rot[0];
//   J[4] = J_rot[1];
//   J[5] = J_rot[2];

//   J[9] = J_rot[3];
//   J[10] = J_rot[4];
//   J[11] = J_rot[5];
// }

// /**
//  * Camera parameters jacobian
//  */
// static void idf_factor_camera_jacobian(const real_t neg_sqrt_info[2 * 2],
//                                        const real_t J_cam_params[2 * 8],
//                                        real_t J[2 * 8]) {
//   assert(neg_sqrt_info != NULL);
//   assert(J_cam_params != NULL);
//   assert(J != NULL);

//   // J = -1 * sqrt_info * J_cam_params;
//   dot(neg_sqrt_info, 2, 2, J_cam_params, 2, 8, J);
// }

// /**
//  * Feature jacobian
//  */
// static void idf_factor_feature_jacobian(const real_t Jh_w[2 * 3],
//                                         const real_t T_WB[4 * 4],
//                                         const real_t T_BC[4 * 4],
//                                         const real_t p_W[3],
//                                         const feature_t *idf_param,
//                                         real_t J_idf_pos[2 * 3],
//                                         real_t J_idf_param[2 * 3]) {
//   assert(Jh_w != NULL);
//   assert(T_WB != NULL);
//   assert(T_BC != NULL);
//   assert(idf_param != NULL);
//   assert(J_idf_pos != NULL);
//   assert(J_idf_param != NULL);

//   const real_t theta = idf_param->data[0];
//   const real_t phi = idf_param->data[1];
//   const real_t rho = idf_param->data[2];
//   const real_t d = 1.0 / rho;
//   const real_t k = -1.0 / (rho * rho);

//   const real_t cphi = cos(phi);
//   const real_t sphi = sin(phi);
//   const real_t ctheta = cos(theta);
//   const real_t stheta = sin(theta);
//   const real_t m[3] = {cphi * stheta, -sphi, cphi * ctheta};
//   const real_t J_theta[3] = {d * cphi * ctheta, 0.0, d * cphi * -stheta};
//   const real_t J_phi[3] = {d * -sphi * stheta, d * -cphi, d * -sphi * ctheta};
//   const real_t J_rho[3] = {k * m[0], k * m[1], k * m[2]};

//   real_t J_idf[3 * 6] = {0};
//   J_idf[0] = 1.0;
//   J_idf[6] = 0.0;
//   J_idf[12] = 0.0;

//   J_idf[1] = 0.0;
//   J_idf[7] = 1.0;
//   J_idf[13] = 0.0;

//   J_idf[2] = 0.0;
//   J_idf[8] = 0.0;
//   J_idf[14] = 1.0;

//   J_idf[3] = J_theta[0];
//   J_idf[9] = J_theta[1];
//   J_idf[15] = J_theta[2];

//   J_idf[4] = J_phi[0];
//   J_idf[10] = J_phi[1];
//   J_idf[16] = J_phi[2];

//   J_idf[5] = J_rho[0];
//   J_idf[11] = J_rho[1];
//   J_idf[17] = J_rho[2];

//   // Jh_w = -1 * sqrt_info * Jh;
//   // J = Jh_w * C_CW * J_idf;
//   real_t J[2 * 6] = {0};
//   TF_CHAIN(T_WC, 2, T_WB, T_BC);
//   TF_ROT(T_WC, C_WC);
//   MAT_TRANSPOSE(C_WC, 3, 3, C_CW);
//   dot3(Jh_w, 2, 3, C_CW, 3, 3, J_idf, 3, 6, J);

//   J_idf_pos[0] = J[0];
//   J_idf_pos[1] = J[1];
//   J_idf_pos[2] = J[2];
//   J_idf_pos[3] = J[6];
//   J_idf_pos[4] = J[7];
//   J_idf_pos[5] = J[8];

//   J_idf_param[0] = J[3];
//   J_idf_param[1] = J[4];
//   J_idf_param[2] = J[5];
//   J_idf_param[3] = J[9];
//   J_idf_param[4] = J[10];
//   J_idf_param[5] = J[11];
// }

// /**
//  * Setup IDF factor
//  */
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
//                       const real_t var[2]) {
//   assert(factor != NULL);
//   assert(pose != NULL);
//   assert(extrinsic != NULL);
//   assert(camera != NULL);
//   assert(idf_pos != NULL);
//   assert(idf_param != NULL);

//   // Property
//   factor->ts = ts;
//   factor->cam_idx = cam_idx;
//   factor->feature_id = feature_id;

//   // Parameters
//   factor->pose = pose;
//   factor->extrinsic = extrinsic;
//   factor->camera = camera;
//   factor->idf_pos = idf_pos;
//   factor->idf_param = idf_param;

//   // Measurement covariance matrix
//   factor->covar[0] = var[0];
//   factor->covar[1] = 0.0;
//   factor->covar[2] = 0.0;
//   factor->covar[3] = var[1];

//   // Square-root information matrix
//   factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);
//   factor->sqrt_info[1] = 0.0;
//   factor->sqrt_info[2] = 0.0;
//   factor->sqrt_info[3] = sqrt(1.0 / factor->covar[3]);

//   // Measurement
//   factor->z[0] = z[0];
//   factor->z[1] = z[1];

//   // Parameters, residuals, jacobians
//   factor->r_size = 2;
//   factor->num_params = 5;

//   factor->param_types[0] = POSE_PARAM;
//   factor->param_types[1] = EXTRINSIC_PARAM;
//   factor->param_types[2] = CAMERA_PARAM;
//   factor->param_types[3] = POSITION_PARAM;
//   factor->param_types[4] = IDF_BEARING_PARAM;

//   factor->params[0] = factor->pose->data;
//   factor->params[1] = factor->extrinsic->data;
//   factor->params[2] = factor->camera->data;
//   factor->params[3] = factor->idf_pos->data;
//   factor->params[4] = factor->idf_param->data;

//   factor->jacs[0] = factor->J_pose;
//   factor->jacs[1] = factor->J_extrinsic;
//   factor->jacs[2] = factor->J_camera;
//   factor->jacs[3] = factor->J_idf_pos;
//   factor->jacs[4] = factor->J_idf_param;
// }

// /**
//  * Evaluate IDF factor
//  */
// int idf_factor_eval(void *factor_ptr) {
//   idf_factor_t *factor = (idf_factor_t *) factor_ptr;

//   // Form T_CiW
//   TF(factor->pose->data, T_WB);
//   TF(factor->extrinsic->data, T_BCi);
//   TF_CHAIN(T_WCi, 2, T_WB, T_BCi);
//   // TF_TRANS(T_WCi, r_WCi);
//   TF_INV(T_WCi, T_CiW);

//   // Calculate residuals and jacobians
//   // -- Form: -1 * sqrt_info
//   real_t nsqrt_info[2 * 2] = {0};
//   mat_copy(factor->sqrt_info, 2, 2, nsqrt_info);
//   mat_scale(nsqrt_info, 2, 2, -1.0);

//   // Form 3D point in world frame
//   real_t p_W[3] = {0};
//   idf_point(factor->idf_param, factor->idf_pos->data, p_W);

//   // Project to image frame
//   real_t z_hat[2];
//   TF_POINT(T_CiW, p_W, p_Ci);
//   camera_project(factor->camera, p_Ci, z_hat);

//   // Residual z - z_hat
//   real_t r[2] = {0};
//   r[0] = factor->z[0] - z_hat[0];
//   r[1] = factor->z[1] - z_hat[1];
//   // -- Weighted residual
//   dot(factor->sqrt_info, 2, 2, r, 2, 1, factor->r);

//   // -- Form: Jh_ = -1 * sqrt_info * Jh
//   real_t Jh[2 * 3] = {0};
//   real_t Jh_w[2 * 3] = {0};
//   pinhole_radtan4_project_jacobian(factor->camera->data, p_Ci, Jh);
//   dot(nsqrt_info, 2, 2, Jh, 2, 3, Jh_w);

//   // -- Form: J_camera
//   real_t J_camera[2 * 8] = {0};
//   pinhole_radtan4_params_jacobian(factor->camera->data, p_Ci, J_camera);

//   // -- Fill Jacobians
//   idf_factor_pose_jacobian(Jh_w, T_WB, T_BCi, p_W, factor->jacs[0]);
//   idf_factor_extrinsic_jacobian(Jh_w, T_BCi, p_Ci, factor->jacs[1]);
//   idf_factor_camera_jacobian(nsqrt_info, J_camera, factor->jacs[2]);
//   idf_factor_feature_jacobian(Jh_w,
//                               T_WB,
//                               T_BCi,
//                               p_W,
//                               factor->idf_param,
//                               factor->jacs[3],
//                               factor->jacs[4]);

//   return 0;
// }

////////////////
// IMU FACTOR //
////////////////

/**
 * Setup IMU buffer
 */
void imu_buffer_setup(imu_buffer_t *imu_buf) {
  for (int k = 0; k < IMU_BUFFER_MAX_SIZE; k++) {
    imu_buf->ts[k] = 0.0;

    imu_buf->acc[k][0] = 0.0;
    imu_buf->acc[k][1] = 0.0;
    imu_buf->acc[k][2] = 0.0;

    imu_buf->gyr[k][0] = 0.0;
    imu_buf->gyr[k][1] = 0.0;
    imu_buf->gyr[k][2] = 0.0;
  }

  imu_buf->size = 0;
}

/**
 * Print IMU buffer
 */
void imu_buffer_print(const imu_buffer_t *imu_buf) {
  for (int k = 0; k < imu_buf->size; k++) {
    const real_t *acc = imu_buf->acc[k];
    const real_t *gyr = imu_buf->gyr[k];

    printf("ts: %ld ", imu_buf->ts[k]);
    printf("acc: [%.2f, %.2f, %.2f] ", acc[0], acc[1], acc[2]);
    printf("gyr: [%.2f, %.2f, %.2f] ", gyr[0], gyr[1], gyr[2]);
    printf("\n");
  }
}

/**
 * Add measurement to IMU buffer
 */
void imu_buffer_add(imu_buffer_t *imu_buf,
                    const timestamp_t ts,
                    const real_t acc[3],
                    const real_t gyr[3]) {
  assert(imu_buf->size < IMU_BUFFER_MAX_SIZE);
  const int k = imu_buf->size;
  imu_buf->ts[k] = ts;
  imu_buf->acc[k][0] = acc[0];
  imu_buf->acc[k][1] = acc[1];
  imu_buf->acc[k][2] = acc[2];
  imu_buf->gyr[k][0] = gyr[0];
  imu_buf->gyr[k][1] = gyr[1];
  imu_buf->gyr[k][2] = gyr[2];
  imu_buf->size++;
}

/**
 * Return first timestamp in IMU buffer
 */
timestamp_t imu_buffer_first_ts(const imu_buffer_t *imu_buf) {
  assert(imu_buf != NULL);
  return imu_buf->ts[0];
}

/**
 * Return last timestamp in IMU buffer
 */
timestamp_t imu_buffer_last_ts(const imu_buffer_t *imu_buf) {
  assert(imu_buf != NULL);
  return imu_buf->ts[imu_buf->size - 1];
}

/**
 * Clear IMU buffer
 */
void imu_buffer_clear(imu_buffer_t *imu_buf) {
  for (int k = 0; k < imu_buf->size; k++) {
    timestamp_t *ts = &imu_buf->ts[k];
    real_t *acc = imu_buf->acc[k];
    real_t *gyr = imu_buf->gyr[k];

    *ts = 0;

    acc[0] = 0.0;
    acc[1] = 0.0;
    acc[2] = 0.0;

    gyr[0] = 0.0;
    gyr[1] = 0.0;
    gyr[2] = 0.0;
  }
  imu_buf->size = 0;
}

/**
 * Copy IMU buffer
 */
void imu_buffer_copy(const imu_buffer_t *src, imu_buffer_t *dst) {
  dst->size = 0;
  for (int k = 0; k < src->size; k++) {
    dst->ts[k] = src->ts[k];

    dst->acc[k][0] = src->acc[k][0];
    dst->acc[k][1] = src->acc[k][1];
    dst->acc[k][2] = src->acc[k][2];

    dst->gyr[k][0] = src->gyr[k][0];
    dst->gyr[k][1] = src->gyr[k][1];
    dst->gyr[k][2] = src->gyr[k][2];
  }
  dst->size = src->size;
}

/**
 * Form IMU state vector
 */
void imu_state_vector(const real_t r[3],
                      const real_t q[4],
                      const real_t v[3],
                      const real_t ba[3],
                      const real_t bg[3],
                      real_t x[16]) {
  assert(r != NULL);
  assert(q != NULL);
  assert(v != NULL);
  assert(ba != NULL);
  assert(bg != NULL);
  assert(x != NULL);

  x[0] = r[0];
  x[1] = r[1];
  x[2] = r[2];

  x[3] = q[0];
  x[4] = q[1];
  x[5] = q[2];
  x[6] = q[3];

  x[7] = v[0];
  x[8] = v[1];
  x[9] = v[2];

  x[10] = ba[0];
  x[11] = ba[1];
  x[12] = ba[2];

  x[13] = bg[0];
  x[14] = bg[1];
  x[15] = bg[2];
}

/**
 * Propagate IMU measurement
 */
void imu_propagate(const real_t pose_k[7],
                   const real_t vel_k[3],
                   const imu_buffer_t *imu_buf,
                   real_t pose_kp1[7],
                   real_t vel_kp1[3]) {
  // Initialize state
  real_t r[3] = {pose_k[0], pose_k[1], pose_k[2]};
  real_t v[3] = {vel_k[0], vel_k[1], vel_k[2]};
  real_t q[4] = {pose_k[3], pose_k[4], pose_k[5], pose_k[6]};
  const real_t g[3] = {0.0, 0.0, -9.81};

  real_t dt = 0.0;
  for (int k = 0; k < imu_buf->size; k++) {
    // Calculate dt
    if ((k + 1) < imu_buf->size) {
      timestamp_t ts_k = imu_buf->ts[k];
      timestamp_t ts_kp1 = imu_buf->ts[k + 1];
      dt = ts2sec(ts_kp1) - ts2sec(ts_k);
    }

    // Map out accelerometer and gyroscope measurements
    const real_t *a = imu_buf->acc[k];
    const real_t *w = imu_buf->gyr[k];

    // Precompute:
    // acc = (q * a * q_conj) + g
    //     = (C * a) + g
    real_t acc[3] = {0};
    quat_transform(q, a, acc);
    acc[0] += g[0];
    acc[1] += g[1];
    acc[2] += g[2];

    // Update position:
    // r = r + (v * dt) + (0.5 * ((C * a) + g) * dt_sq);
    r[0] += (v[0] * dt) + (0.5 * acc[0] * dt * dt);
    r[1] += (v[1] * dt) + (0.5 * acc[1] * dt * dt);
    r[2] += (v[2] * dt) + (0.5 * acc[2] * dt * dt);

    // Update velocity
    // v = v + (C * a + g) * dt;
    v[0] += acc[0] * dt;
    v[1] += acc[1] * dt;
    v[2] += acc[2] * dt;

    // Update rotation
    quat_update_dt(q, w, dt);
    quat_normalize(q);
  }

  // Map results
  pose_kp1[0] = r[0];
  pose_kp1[1] = r[1];
  pose_kp1[2] = r[2];
  pose_kp1[3] = q[0];
  pose_kp1[4] = q[1];
  pose_kp1[5] = q[2];
  pose_kp1[6] = q[3];

  vel_kp1[0] = v[0];
  vel_kp1[1] = v[1];
  vel_kp1[2] = v[2];
}

/**
 * Initialize roll and pitch with accelerometer measurements.
 */
void imu_initial_attitude(const imu_buffer_t *imu_buf, real_t q_WS[4]) {
  // Get mean accelerometer measurements
  real_t ax = 0.0;
  real_t ay = 0.0;
  real_t az = 0.0;
  for (size_t k = 0; k < imu_buf->size; k++) {
    ax += imu_buf->acc[k][0];
    ay += imu_buf->acc[k][1];
    az += imu_buf->acc[k][2];
  }
  ax /= imu_buf->size;
  ay /= imu_buf->size;
  az /= imu_buf->size;

  // Initialize orientation
  const real_t ypr[3] = {0.0,
                         atan2(-ax, sqrt(ay * ay + az * az)),
                         atan2(ay, az)};
  euler2quat(ypr, q_WS);

  // const real_t a[3] = {ax, ay, az};
  // const real_t g[3] = {0.0, 0.0, 9.81};
  // real_t C[3 * 3] = {0};
  // real_t q[4] = {0};
  // real_t euler[3] = {0};
  // vecs2rot(a, g, C);
  // rot2quat(C, q);
  // quat2euler(q, euler);
  // print_vector("euler", euler, 3);
  // print_vector("ypr", ypr, 3);
  // exit(0);
}

/**
 * Propagate IMU measurement
 */
void imu_factor_propagate_step(imu_factor_t *factor,
                               const real_t a_i[3],
                               const real_t w_i[3],
                               const real_t a_j[3],
                               const real_t w_j[3],
                               const real_t dt) {
  assert(factor != NULL);
  assert(a_i != NULL);
  assert(w_i != NULL);
  assert(a_j != NULL);
  assert(w_j != NULL);
  assert(dt > 0.0);

  // Setup
  const real_t dt_sq = dt * dt;
  const real_t *r_i = factor->dr;
  const real_t *v_i = factor->dv;
  const real_t *q_i = factor->dq;
  const real_t *ba_i = factor->ba;
  const real_t *bg_i = factor->bg;

  // Gyroscope measurement
  const real_t wx = 0.5 * (w_i[0] + w_j[0]) - bg_i[0];
  const real_t wy = 0.5 * (w_i[1] + w_j[1]) - bg_i[1];
  const real_t wz = 0.5 * (w_i[2] + w_j[2]) - bg_i[2];
  const real_t dq[4] = {1.0, 0.5 * wx * dt, 0.5 * wy * dt, 0.5 * wz * dt};

  // Update orientation
  real_t q_j[4] = {0};
  quat_mul(q_i, dq, q_j);
  quat_normalize(q_j);

  // Accelerometer measurement
  const real_t a_ii[3] = {a_i[0] - ba_i[0], a_i[1] - ba_i[1], a_i[2] - ba_i[2]};
  const real_t a_jj[3] = {a_j[0] - ba_i[0], a_j[1] - ba_i[1], a_j[2] - ba_i[2]};
  real_t acc_i[3] = {0};
  real_t acc_j[3] = {0};
  quat_transform(q_i, a_ii, acc_i);
  quat_transform(q_j, a_jj, acc_j);
  real_t a[3] = {0};
  a[0] = 0.5 * (acc_i[0] + acc_j[0]);
  a[1] = 0.5 * (acc_i[1] + acc_j[1]);
  a[2] = 0.5 * (acc_i[2] + acc_j[2]);

  // Update position:
  // r_j = r_i + (v_i * dt) + (0.5 * a * dt_sq)
  real_t r_j[3] = {0};
  r_j[0] = r_i[0] + (v_i[0] * dt) + (0.5 * a[0] * dt_sq);
  r_j[1] = r_i[1] + (v_i[1] * dt) + (0.5 * a[1] * dt_sq);
  r_j[2] = r_i[2] + (v_i[2] * dt) + (0.5 * a[2] * dt_sq);

  // Update velocity:
  // v_j = v_i + a * dt
  real_t v_j[3] = {0};
  v_j[0] = v_i[0] + a[0] * dt;
  v_j[1] = v_i[1] + a[1] * dt;
  v_j[2] = v_i[2] + a[2] * dt;

  // Update biases
  // ba_j = ba_i;
  // bg_j = bg_i;
  real_t ba_j[3] = {0};
  real_t bg_j[3] = {0};
  vec_copy(ba_i, 3, ba_j);
  vec_copy(bg_i, 3, bg_j);

  // Write outputs
  vec_copy(r_j, 3, factor->r_j);
  vec_copy(v_j, 3, factor->v_j);
  vec_copy(q_j, 4, factor->q_j);
  vec_copy(ba_j, 3, factor->ba_j);
  vec_copy(bg_j, 3, factor->bg_j);

  vec_copy(r_j, 3, factor->dr);
  vec_copy(v_j, 3, factor->dv);
  vec_copy(q_j, 4, factor->dq);
  vec_copy(ba_j, 3, factor->ba);
  vec_copy(bg_j, 3, factor->bg);
}

/**
 * Form IMU Noise Matrix Q
 */
static void imu_factor_form_Q_matrix(const imu_params_t *imu_params,
                                     real_t Q[18 * 18]) {
  assert(imu_params != NULL);
  assert(Q != NULL);

  const real_t sigma_a_sq = imu_params->sigma_a * imu_params->sigma_a;
  const real_t sigma_g_sq = imu_params->sigma_g * imu_params->sigma_g;
  const real_t sigma_ba_sq = imu_params->sigma_aw * imu_params->sigma_aw;
  const real_t sigma_bg_sq = imu_params->sigma_gw * imu_params->sigma_gw;

  real_t q[18] = {0};
  q[0] = sigma_a_sq;
  q[1] = sigma_a_sq;
  q[2] = sigma_a_sq;

  q[3] = sigma_g_sq;
  q[4] = sigma_g_sq;
  q[5] = sigma_g_sq;

  q[6] = sigma_a_sq;
  q[7] = sigma_a_sq;
  q[8] = sigma_a_sq;

  q[9] = sigma_g_sq;
  q[10] = sigma_g_sq;
  q[11] = sigma_g_sq;

  q[12] = sigma_ba_sq;
  q[13] = sigma_ba_sq;
  q[14] = sigma_ba_sq;

  q[15] = sigma_bg_sq;
  q[16] = sigma_bg_sq;
  q[17] = sigma_bg_sq;

  zeros(Q, 18, 18);
  mat_diag_set(Q, 18, 18, q);
}

// F11 = eye(3)
#define IMU_FACTOR_F11(void)                                                   \
  real_t F11[3 * 3] = {0};                                                     \
  eye(F11, 3, 3);

// F12 = -0.25 * dC_i @ acc_i_x * dt_sq
// F12 += -0.25 * dC_j @ acc_j_x @ (eye(3) - gyr_x * dt) * dt_sq
#define IMU_FACTOR_F12(dCi_acc_i_x, dCj_acc_j_x, I_m_gyr_x_dt, dt_sq)          \
  real_t F12_A[3 * 3] = {0};                                                   \
  mat_copy(dCi_acc_i_x, 3, 3, F12_A);                                          \
  mat_scale(F12_A, 3, 3, -0.25);                                               \
  mat_scale(F12_A, 3, 3, dt_sq);                                               \
                                                                               \
  real_t F12_B[3 * 3] = {0};                                                   \
  mat_copy(dCj_acc_j_x, 3, 3, F12_B);                                          \
  mat_scale(F12_B, 3, 3, -0.25);                                               \
                                                                               \
  real_t F12_C[3 * 3] = {0};                                                   \
  mat_copy(I_m_gyr_x_dt, 3, 3, F12_C);                                         \
  mat_scale(F12_C, 3, 3, dt_sq);                                               \
                                                                               \
  real_t F12_D[3 * 3] = {0};                                                   \
  dot(F12_B, 3, 3, F12_C, 3, 3, F12_D);                                        \
                                                                               \
  real_t F12[3 * 3] = {0};                                                     \
  mat_add(F12_A, F12_D, F12, 3, 3);

// F13 = eye(3) * dt
#define IMU_FACTOR_F13(dt)                                                     \
  real_t F13[3 * 3] = {0};                                                     \
  eye(F13, 3, 3);                                                              \
  mat_scale(F13, 3, 3, dt);

// F14 = -0.25 * (dC_i + dC_j) * dt_sq
#define IMU_FACTOR_F14(dCi_dCj, dt_sq)                                         \
  real_t F14[3 * 3] = {0};                                                     \
  mat_copy(dCi_dCj, 3, 3, F14);                                                \
  mat_scale(F14, 3, 3, -0.25);                                                 \
  mat_scale(F14, 3, 3, dt_sq);

// F15 = 0.25 * -dC_j @ acc_j_x * dt_sq * -dt
#define IMU_FACTOR_F15(dCj_acc_j_x, dt, dt_sq)                                 \
  real_t F15[3 * 3] = {0};                                                     \
  mat_copy(dCj_acc_j_x, 3, 3, F15);                                            \
  mat_scale(F15, 3, 3, -1.0 * 0.25 * dt_sq * -dt);

// F22 = eye(3) - gyr_x * dt
#define IMU_FACTOR_F22(I_m_gyr_x_dt)                                           \
  real_t F22[3 * 3] = {0};                                                     \
  mat_copy(I_m_gyr_x_dt, 3, 3, F22);

// F25 = -eye(3) * dt
#define IMU_FACTOR_F25(dt)                                                     \
  real_t F25[3 * 3] = {0};                                                     \
  F25[0] = -dt;                                                                \
  F25[4] = -dt;                                                                \
  F25[8] = -dt;

// F32 = -0.5 * dC_i @ acc_i_x * dt
// F32 += -0.5 * dC_j @ acc_j_x @ (eye(3) - gyr_x * dt)* dt
#define IMU_FACTOR_F32(dCi_acc_i_x, dCj_acc_j_x, I_m_gyr_x_dt, dt)             \
  real_t F32_A[3 * 3] = {0};                                                   \
  mat_copy(dCi_acc_i_x, 3, 3, F32_A);                                          \
  for (int i = 0; i < 9; i++) {                                                \
    F32_A[i] = -0.5 * F32_A[i] * dt;                                           \
  }                                                                            \
                                                                               \
  real_t F32_B[3 * 3] = {0};                                                   \
  dot(dCj_acc_j_x, 3, 3, I_m_gyr_x_dt, 3, 3, F32_B);                           \
  for (int i = 0; i < 9; i++) {                                                \
    F32_B[i] = -0.5 * F32_B[i] * dt;                                           \
  }                                                                            \
                                                                               \
  real_t F32[3 * 3] = {0};                                                     \
  mat_add(F32_A, F32_B, F32, 3, 3);

// F33 = eye(3)
#define IMU_FACTOR_F33(void)                                                   \
  real_t F33[3 * 3] = {0};                                                     \
  F33[0] = 1.0;                                                                \
  F33[4] = 1.0;                                                                \
  F33[8] = 1.0;

// F34 = -0.5 * (dC_i + dC_j) * dt
#define IMU_FACTOR_F34(dC_i, dC_j, dt)                                         \
  real_t F34[3 * 3] = {0};                                                     \
  for (int i = 0; i < 9; i++) {                                                \
    F34[i] = -0.5 * dCi_dCj[i] * dt;                                           \
  }

// F35 = 0.5 * -dC_j @ acc_j_x * dt * -dt
#define IMU_FACTOR_F35(dCj_acc_j_x, dt)                                        \
  real_t F35[3 * 3] = {0};                                                     \
  for (int i = 0; i < 9; i++) {                                                \
    F35[i] = 0.5 * -1.0 * dCj_acc_j_x[i] * dt * -dt;                           \
  }

// F44 = eye(3)
#define IMU_FACTOR_F44(void)                                                   \
  real_t F44[3 * 3] = {0};                                                     \
  F44[0] = 1.0;                                                                \
  F44[4] = 1.0;                                                                \
  F44[8] = 1.0;

// F55 = eye(3)
#define IMU_FACTOR_F55(void)                                                   \
  real_t F55[3 * 3] = {0};                                                     \
  F55[0] = 1.0;                                                                \
  F55[4] = 1.0;                                                                \
  F55[8] = 1.0;

/**
 * Form IMU Transition Matrix F
 */
void imu_factor_F_matrix(const real_t q_i[4],
                         const real_t q_j[4],
                         const real_t ba_i[3],
                         const real_t bg_i[3],
                         const real_t a_i[3],
                         const real_t w_i[3],
                         const real_t a_j[3],
                         const real_t w_j[3],
                         const real_t dt,
                         real_t F_dt[15 * 15]) {
  // Setup
  const real_t dt_sq = dt * dt;

  // gyr_x = hat(0.5 * (imu_buf.gyr[k] + imu_buf.gyr[k + 1]) - bg_i)
  real_t gyr[3] = {0};
  real_t gyr_x[3 * 3] = {0};
  gyr[0] = 0.5 * (w_i[0] + w_j[0]) - bg_i[0];
  gyr[1] = 0.5 * (w_i[1] + w_j[1]) - bg_i[1];
  gyr[2] = 0.5 * (w_i[2] + w_j[2]) - bg_i[2];
  hat(gyr, gyr_x);

  // acc_i_x = hat(imu_buf.acc[k] - ba_i)
  // acc_j_x = hat(imu_buf.acc[k + 1] - ba_i)
  real_t acc_i[3] = {a_i[0] - ba_i[0], a_i[1] - ba_i[1], a_i[2] - ba_i[2]};
  real_t acc_j[3] = {a_j[0] - ba_i[0], a_j[1] - ba_i[1], a_j[2] - ba_i[2]};
  real_t acc_i_x[3 * 3] = {0};
  real_t acc_j_x[3 * 3] = {0};
  hat(acc_i, acc_i_x);
  hat(acc_j, acc_j_x);

  // dC_i = quat2rot(q_i)
  // dC_j = quat2rot(q_j)
  real_t dC_i[3 * 3] = {0};
  real_t dC_j[3 * 3] = {0};
  quat2rot(q_i, dC_i);
  quat2rot(q_j, dC_j);

  // (dC_i + dC_j)
  real_t dCi_dCj[3 * 3] = {0};
  mat_add(dC_i, dC_j, dCi_dCj, 3, 3);

  // dC_i @ acc_i_x
  real_t dCi_acc_i_x[3 * 3] = {0};
  dot(dC_i, 3, 3, acc_i_x, 3, 3, dCi_acc_i_x);

  // dC_j @ acc_j_x
  real_t dCj_acc_j_x[3 * 3] = {0};
  dot(dC_j, 3, 3, acc_j_x, 3, 3, dCj_acc_j_x);

  // (eye(3) - gyr_x * dt)
  real_t I_m_gyr_x_dt[3 * 3] = {0};
  I_m_gyr_x_dt[0] = 1.0 - gyr_x[0] * dt;
  I_m_gyr_x_dt[1] = 0.0 - gyr_x[1] * dt;
  I_m_gyr_x_dt[2] = 0.0 - gyr_x[2] * dt;

  I_m_gyr_x_dt[3] = 0.0 - gyr_x[3] * dt;
  I_m_gyr_x_dt[4] = 1.0 - gyr_x[4] * dt;
  I_m_gyr_x_dt[5] = 0.0 - gyr_x[5] * dt;

  I_m_gyr_x_dt[6] = 0.0 - gyr_x[6] * dt;
  I_m_gyr_x_dt[7] = 0.0 - gyr_x[7] * dt;
  I_m_gyr_x_dt[8] = 1.0 - gyr_x[8] * dt;

  IMU_FACTOR_F11();
  IMU_FACTOR_F12(dCi_acc_i_x, dCj_acc_j_x, I_m_gyr_x_dt, dt_sq);
  IMU_FACTOR_F13(dt);
  IMU_FACTOR_F14(dCi_dCj, dt_sq);
  IMU_FACTOR_F15(dCj_acc_j_x, dt, dt_sq);
  IMU_FACTOR_F22(I_m_gyr_x_dt);
  IMU_FACTOR_F25(dt);
  IMU_FACTOR_F32(dCi_acc_i_x, dCj_acc_j_x, I_m_gyr_x_dt, dt);
  IMU_FACTOR_F33();
  IMU_FACTOR_F34(dC_i, dC_j, dt);
  IMU_FACTOR_F35(dCj_acc_j_x, dt);
  IMU_FACTOR_F44();
  IMU_FACTOR_F55();

  // Fill matrix F
  zeros(F_dt, 15, 15);

  // -- Row block 1
  mat_block_set(F_dt, 15, 0, 2, 0, 2, F11);
  mat_block_set(F_dt, 15, 0, 2, 3, 5, F12);
  mat_block_set(F_dt, 15, 0, 2, 6, 8, F13);
  mat_block_set(F_dt, 15, 0, 2, 9, 11, F14);
  mat_block_set(F_dt, 15, 0, 2, 12, 14, F15);

  // -- Row block 2
  mat_block_set(F_dt, 15, 3, 5, 3, 5, F22);
  mat_block_set(F_dt, 15, 3, 5, 12, 14, F25);

  // -- Row block 3
  mat_block_set(F_dt, 15, 6, 8, 3, 5, F32);
  mat_block_set(F_dt, 15, 6, 8, 6, 8, F33);
  mat_block_set(F_dt, 15, 6, 8, 9, 11, F34);
  mat_block_set(F_dt, 15, 6, 8, 12, 14, F35);

  // -- Row block 4
  mat_block_set(F_dt, 15, 9, 11, 9, 11, F44);

  // -- Row block 5
  mat_block_set(F_dt, 15, 12, 14, 12, 14, F55);
}

/**
 * Form IMU Input Matrix G
 */
void imu_factor_form_G_matrix(const imu_factor_t *factor,
                              const real_t a_i[3],
                              const real_t a_j[3],
                              const real_t dt,
                              real_t G_dt[15 * 18]) {

  // dt_sq = dt * dt
  const real_t dt_sq = dt * dt;

  // dC_i = quat2rot(q_i)
  // dC_j = quat2rot(q_j)
  real_t dC_i[3 * 3] = {0};
  real_t dC_j[3 * 3] = {0};
  quat2rot(factor->q_i, dC_i);
  quat2rot(factor->q_j, dC_j);

  // acc_i_x = hat(imu_buf.acc[k] - ba_i)
  // acc_j_x = hat(imu_buf.acc[k + 1] - ba_i)
  const real_t *ba_i = factor->ba_i;
  real_t acc_i[3] = {a_i[0] - ba_i[0], a_i[1] - ba_i[1], a_i[2] - ba_i[2]};
  real_t acc_j[3] = {a_j[0] - ba_i[0], a_j[1] - ba_i[1], a_j[2] - ba_i[2]};
  real_t acc_i_x[3 * 3] = {0};
  real_t acc_j_x[3 * 3] = {0};
  hat(acc_i, acc_i_x);
  hat(acc_j, acc_j_x);

  // dC_j @ acc_j_x
  real_t dC_j_acc_j_x[3 * 3] = {0};
  dot(dC_j, 3, 3, acc_j_x, 3, 3, dC_j_acc_j_x);

  // G11 = 0.25 * dC_i * dt_sq
  real_t G11[3 * 3] = {0};
  for (int i = 0; i < 9; i++) {
    G11[i] = 0.25 * dC_i[i] * dt_sq;
  }

  // G12 = 0.25 * -dC_j @ acc_j_x * dt_sq * 0.5 * dt
  real_t G12[3 * 3] = {0};
  for (int i = 0; i < 9; i++) {
    G12[i] = 0.25 * -dC_j_acc_j_x[i] * dt_sq * 0.5 * dt;
  }

  // G13 = 0.25 * dC_j @ acc_j_x * dt_sq
  real_t G13[3 * 3] = {0};
  for (int i = 0; i < 9; i++) {
    G13[i] = 0.25 * dC_j_acc_j_x[i] * dt_sq;
  }

  // G14 = 0.25 * -dC_j @ acc_j_x * dt_sq * 0.5 * dt
  real_t G14[3 * 3] = {0};
  for (int i = 0; i < 9; i++) {
    G14[i] = 0.25 * -dC_j_acc_j_x[i] * dt_sq * 0.5 * dt;
  }

  // G22 = eye(3) * dt
  real_t G22[3 * 3] = {0};
  G22[0] = dt;
  G22[4] = dt;
  G22[8] = dt;

  // G24 = eye(3) * dt
  real_t G24[3 * 3] = {0};
  G24[0] = dt;
  G24[4] = dt;
  G24[8] = dt;

  // G31 = 0.5 * dC_i * dt
  real_t G31[3 * 3] = {0};
  for (int i = 0; i < 9; i++) {
    G31[i] = 0.5 * dC_i[i] * dt;
  }

  // G32 = 0.5 * -dC_j @ acc_j_x * dt * 0.5 * dt
  real_t G32[3 * 3] = {0};
  for (int i = 0; i < 9; i++) {
    G32[i] = 0.5 * -dC_j_acc_j_x[i] * dt * 0.5 * dt;
  }

  // G33 = 0.5 * dC_j * dt
  real_t G33[3 * 3] = {0};
  for (int i = 0; i < 9; i++) {
    G33[i] = 0.5 * dC_j[i] * dt;
  }

  // G34 = 0.5 * -dC_j @ acc_j_x * dt * 0.5 * dt
  real_t G34[3 * 3] = {0};
  for (int i = 0; i < 9; i++) {
    G34[i] = 0.5 * -dC_j_acc_j_x[i] * dt * 0.5 * dt;
  }

  // G45 = eye(3) * dt
  real_t G45[3 * 3] = {0};
  G45[0] = dt;
  G45[4] = dt;
  G45[8] = dt;

  // G56 = eye(3) * dt
  real_t G56[3 * 3] = {0};
  G56[0] = dt;
  G56[4] = dt;
  G56[8] = dt;

  // Fill matrix G
  zeros(G_dt, 15, 18);
  mat_block_set(G_dt, 18, 0, 2, 0, 2, G11);
  mat_block_set(G_dt, 18, 0, 2, 3, 5, G12);
  mat_block_set(G_dt, 18, 0, 2, 6, 8, G13);
  mat_block_set(G_dt, 18, 0, 2, 9, 11, G14);
  mat_block_set(G_dt, 18, 3, 5, 3, 5, G22);
  mat_block_set(G_dt, 18, 3, 5, 9, 11, G24);
  mat_block_set(G_dt, 18, 6, 8, 0, 2, G31);
  mat_block_set(G_dt, 18, 6, 8, 3, 5, G32);
  mat_block_set(G_dt, 18, 6, 8, 6, 8, G33);
  mat_block_set(G_dt, 18, 6, 8, 9, 11, G34);
  mat_block_set(G_dt, 18, 9, 11, 12, 14, G45);
  mat_block_set(G_dt, 18, 12, 14, 15, 17, G56);
}

/**
 * IMU Factor setup
 */
void imu_factor_setup(imu_factor_t *factor,
                      const imu_params_t *imu_params,
                      const imu_buffer_t *imu_buf,
                      pose_t *pose_i,
                      velocity_t *vel_i,
                      imu_biases_t *biases_i,
                      pose_t *pose_j,
                      velocity_t *vel_j,
                      imu_biases_t *biases_j) {
  // IMU buffer and parameters
  factor->imu_params = imu_params;
  imu_buffer_copy(imu_buf, &factor->imu_buf);

  // Parameters
  factor->pose_i = pose_i;
  factor->vel_i = vel_i;
  factor->biases_i = biases_i;
  factor->pose_j = pose_j;
  factor->vel_j = vel_j;
  factor->biases_j = biases_j;

  factor->num_params = 6;
  factor->params[0] = factor->pose_i->data;
  factor->params[1] = factor->vel_i->data;
  factor->params[2] = factor->biases_i->data;
  factor->params[3] = factor->pose_j->data;
  factor->params[4] = factor->vel_j->data;
  factor->params[5] = factor->biases_j->data;
  factor->param_types[0] = POSE_PARAM;
  factor->param_types[1] = VELOCITY_PARAM;
  factor->param_types[2] = IMU_BIASES_PARAM;
  factor->param_types[3] = POSE_PARAM;
  factor->param_types[4] = VELOCITY_PARAM;
  factor->param_types[5] = IMU_BIASES_PARAM;

  // Residuals
  factor->r_size = 15;

  // Jacobians
  factor->jacs[0] = factor->J_pose_i;
  factor->jacs[1] = factor->J_vel_i;
  factor->jacs[2] = factor->J_biases_i;
  factor->jacs[3] = factor->J_pose_j;
  factor->jacs[4] = factor->J_vel_j;
  factor->jacs[5] = factor->J_biases_j;

  // Preintegrate
  imu_factor_preintegrate(factor);
}

/**
 * Reset IMU Factor
 */
void imu_factor_reset(imu_factor_t *factor) {
  // Residuals
  zeros(factor->r, 15, 1);

  // Jacobians
  zeros(factor->J_pose_i, 15, 6);
  zeros(factor->J_vel_i, 15, 3);
  zeros(factor->J_biases_i, 15, 6);
  zeros(factor->J_pose_j, 15, 6);
  zeros(factor->J_vel_j, 15, 3);
  zeros(factor->J_biases_j, 15, 6);

  // Pre-integration variables
  factor->Dt = 0.0;
  eye(factor->F, 15, 15);                                  // State jacobian
  zeros(factor->P, 15, 15);                                // State covariance
  imu_factor_form_Q_matrix(factor->imu_params, factor->Q); // Noise matrix
  zeros(factor->dr, 3, 1);                                 // Rel position
  zeros(factor->dv, 3, 1);                                 // Rel velocity
  quat_setup(factor->dq);                                  // Rel rotation
  imu_biases_get_accel_bias(factor->biases_i, factor->ba); // Accel bias
  imu_biases_get_gyro_bias(factor->biases_i, factor->bg);  // Gyro bias
  zeros(factor->ba_ref, 3, 1);
  zeros(factor->bg_ref, 3, 1);

  // Preintegration step variables
  zeros(factor->r_i, 3, 1);
  zeros(factor->v_i, 3, 1);
  quat_setup(factor->q_i);
  zeros(factor->ba_i, 3, 1);
  zeros(factor->bg_i, 3, 1);

  zeros(factor->r_j, 3, 1);
  zeros(factor->v_j, 3, 1);
  quat_setup(factor->q_j);
  zeros(factor->ba_j, 3, 1);
  zeros(factor->bg_j, 3, 1);
}

void imu_factor_preintegrate(imu_factor_t *factor) {
  // Reset variables
  imu_factor_reset(factor);

  // Pre-integrate imu measuremenets
  // -------------------------------
  // This step is essentially like a Kalman Filter whereby you propagate the
  // system inputs (in this case the system is an IMU model with
  // acceleration and angular velocity as inputs. In this step we are
  // interested in the:
  //
  // - Relative position between pose i and pose j
  // - Relative rotation between pose i and pose j
  // - Relative velocity between pose i and pose j
  // - Relative accelerometer bias between pose i and pose j
  // - Relative gyroscope bias between pose i and pose j
  // - Covariance
  //
  // The covariance can be square-rooted to form the square-root information
  // matrix used by the non-linear least squares algorithm to weigh the
  // parameters
  for (int k = 1; k < factor->imu_buf.size; k++) {
    const timestamp_t ts_i = factor->imu_buf.ts[k - 1];
    const timestamp_t ts_j = factor->imu_buf.ts[k];
    const real_t dt = ts2sec(ts_j) - ts2sec(ts_i);
    const real_t *a_i = factor->imu_buf.acc[k - 1];
    const real_t *w_i = factor->imu_buf.gyr[k - 1];
    const real_t *a_j = factor->imu_buf.acc[k];
    const real_t *w_j = factor->imu_buf.gyr[k];

    if (ts_i < factor->pose_i->ts) {
      continue;
    } else if (ts_j > factor->pose_j->ts) {
      break;
    }

    // Propagate
    imu_factor_propagate_step(factor, a_i, w_i, a_j, w_j, dt);

    // Form transition Matrix F
    const real_t *q_i = factor->q_i;
    const real_t *q_j = factor->q_j;
    const real_t *ba_i = factor->ba_i;
    const real_t *bg_i = factor->bg_i;
    real_t F_dt[15 * 15] = {0};
    imu_factor_F_matrix(q_i, q_j, ba_i, bg_i, a_i, w_i, a_j, w_j, dt, F_dt);

    // Input Jacobian G
    real_t G_dt[15 * 18] = {0};
    imu_factor_form_G_matrix(factor, a_i, a_j, dt, G_dt);

    // Update state matrix F
    // F = F_dt * F;
    real_t state_F[15 * 15] = {0};
    mat_copy(factor->F, 15, 15, state_F);
    dot(F_dt, 15, 15, state_F, 15, 15, factor->F);

    // Update covariance matrix P
    // P = F * P * F' + G * Q * G';
    real_t A[15 * 15] = {0};
    real_t B[15 * 15] = {0};
    dot_XAXt(F_dt, 15, 15, factor->P, 15, 15, A);
    dot_XAXt(G_dt, 15, 18, factor->Q, 18, 18, B);
    mat_add(A, B, factor->P, 15, 15);

    // Update overall dt
    factor->Dt += dt;
  }

  // Keep track of linearized accel / gyro biases
  vec3_copy(factor->biases_i->data + 0, factor->ba_ref);
  vec3_copy(factor->biases_i->data + 3, factor->bg_ref);

  // Covariance
  enforce_spd(factor->P, 15, 15);
  mat_copy(factor->P, 15, 15, factor->covar);

  // Square root information
  real_t info[15 * 15] = {0};
  real_t sqrt_info[15 * 15] = {0};

  pinv(factor->covar, 15, 15, info);
  assert(check_inv(info, factor->covar, 15) == 0);
  zeros(factor->sqrt_info, 15, 15);
  chol(info, 15, sqrt_info);
  mat_transpose(sqrt_info, 15, 15, factor->sqrt_info);
}

static void imu_factor_pose_i_jac(imu_factor_t *factor,
                                  const real_t dr_est[3],
                                  const real_t dv_est[3],
                                  const real_t dq[4]) {
  // Setup
  real_t C_i[3 * 3] = {0};
  real_t C_j[3 * 3] = {0};
  real_t C_it[3 * 3] = {0};
  real_t C_jt[3 * 3] = {0};
  pose_get_rot(factor->pose_i->data, C_i);
  pose_get_rot(factor->pose_j->data, C_j);
  mat_transpose(C_i, 3, 3, C_it);
  mat_transpose(C_j, 3, 3, C_jt);

  // Jacobian w.r.t pose_i
  real_t J_pose_i[15 * 6] = {0};

  // -- Jacobian w.r.t. r_i
  real_t drij_dri[3 * 3] = {0};

  for (int idx = 0; idx < 9; idx++) {
    drij_dri[idx] = -1.0 * C_it[idx];
  }
  mat_block_set(J_pose_i, 6, 0, 2, 0, 2, drij_dri);

  // -- Jacobian w.r.t. q_i
  HAT(dr_est, drij_dCi);
  HAT(dv_est, dvij_dCi);

  // -(quat_left(rot2quat(C_j.T @ C_i)) @ quat_right(dq))[1:4, 1:4]
  real_t dtheta_dCi[3 * 3] = {0};
  {
    DOT(C_jt, 3, 3, C_i, 3, 3, C_ji);
    ROT2QUAT(C_ji, q_ji);

    real_t Left[4 * 4] = {0};
    real_t Right[4 * 4] = {0};
    quat_left(q_ji, Left);
    quat_right(dq, Right);
    DOT(Left, 4, 4, Right, 4, 4, LR);

    mat_block_get(LR, 4, 1, 3, 1, 3, dtheta_dCi);
    mat_scale(dtheta_dCi, 3, 3, -1.0);
  }

  mat_block_set(J_pose_i, 6, 0, 2, 3, 5, drij_dCi);
  mat_block_set(J_pose_i, 6, 3, 5, 3, 5, dvij_dCi);
  mat_block_set(J_pose_i, 6, 6, 8, 3, 5, dtheta_dCi);

  // -- Multiply with sqrt_info
  dot(factor->sqrt_info, 15, 15, J_pose_i, 15, 6, factor->jacs[0]);
}

void imu_factor_velocity_i_jac(imu_factor_t *factor) {
  real_t q_i[4] = {0};
  real_t C_i[3 * 3] = {0};
  real_t C_it[3 * 3] = {0};
  real_t drij_dvi[3 * 3] = {0};
  real_t dvij_dvi[3 * 3] = {0};

  pose_get_quat(factor->pose_i->data, q_i);
  quat2rot(q_i, C_i);
  mat_transpose(C_i, 3, 3, C_it);

  for (int idx = 0; idx < 9; idx++) {
    drij_dvi[idx] = -1.0 * C_it[idx] * factor->Dt;
    dvij_dvi[idx] = -1.0 * C_it[idx];
  }

  real_t J_vel_i[15 * 3] = {0};
  mat_block_set(J_vel_i, 3, 0, 2, 0, 2, drij_dvi);
  mat_block_set(J_vel_i, 3, 3, 5, 0, 2, dvij_dvi);
  dot(factor->sqrt_info, 15, 15, J_vel_i, 15, 3, factor->jacs[1]);
}

void imu_factor_biases_i_jac(imu_factor_t *factor,
                             const real_t dq_dbg[3],
                             const real_t dr_dba[3],
                             const real_t dv_dba[3],
                             const real_t dr_dbg[3],
                             const real_t dv_dbg[3]) {
  real_t q_i[4] = {0};
  real_t q_j[4] = {0};
  pose_get_quat(factor->pose_i->data, q_i);
  pose_get_quat(factor->pose_j->data, q_j);

  QUAT2ROT(factor->dq, dC);
  QUAT2ROT(q_i, C_i);
  QUAT2ROT(q_j, C_j);

  MAT_TRANSPOSE(dC, 3, 3, dCt);
  MAT_TRANSPOSE(C_j, 3, 3, C_jt);
  DOT(C_jt, 3, 3, C_i, 3, 3, C_ji);
  DOT(C_ji, 3, 3, dC, 3, 3, C_ji_dC);
  ROT2QUAT(C_ji_dC, qji_dC);

  real_t left_xyz[3 * 3] = {0};
  quat_left_xyz(qji_dC, left_xyz);

  // Jacobian w.r.t IMU biases
  real_t J_biases_i[15 * 6] = {0};
  real_t mI3[3 * 3] = {0};
  mI3[0] = -1.0;
  mI3[4] = -1.0;
  mI3[8] = -1.0;

  // -- Jacobian w.r.t ba_i
  real_t drij_dbai[3 * 3] = {0};
  real_t dvij_dbai[3 * 3] = {0};
  for (int idx = 0; idx < 9; idx++) {
    drij_dbai[idx] = -1.0 * dr_dba[idx];
    dvij_dbai[idx] = -1.0 * dv_dba[idx];
  }
  mat_block_set(J_biases_i, 6, 0, 2, 0, 2, drij_dbai);
  mat_block_set(J_biases_i, 6, 3, 5, 0, 2, dvij_dbai);

  // -- Jacobian w.r.t bg_i
  real_t drij_dbgi[3 * 3] = {0};
  real_t dvij_dbgi[3 * 3] = {0};
  for (int idx = 0; idx < 9; idx++) {
    drij_dbgi[idx] = -1.0 * dr_dbg[idx];
    dvij_dbgi[idx] = -1.0 * dv_dbg[idx];
  }

  real_t dtheta_dbgi[3 * 3] = {0};
  dot(left_xyz, 3, 3, dq_dbg, 3, 3, dtheta_dbgi);
  for (int i = 0; i < 9; i++) {
    dtheta_dbgi[i] *= -1.0;
  }

  mat_block_set(J_biases_i, 6, 0, 2, 3, 5, drij_dbgi);
  mat_block_set(J_biases_i, 6, 3, 5, 3, 5, dvij_dbgi);
  mat_block_set(J_biases_i, 6, 6, 8, 3, 5, dtheta_dbgi);
  mat_block_set(J_biases_i, 6, 9, 11, 0, 2, mI3);
  mat_block_set(J_biases_i, 6, 12, 14, 3, 5, mI3);

  // -- Multiply with sqrt info
  dot(factor->sqrt_info, 15, 15, J_biases_i, 15, 6, factor->jacs[2]);
}

void imu_factor_pose_j_jac(imu_factor_t *factor, const real_t dq[4]) {
  // Setup
  real_t q_i[4] = {0};
  real_t q_j[4] = {0};
  real_t C_i[3 * 3] = {0};
  real_t C_j[3 * 3] = {0};
  real_t C_it[3 * 3] = {0};

  pose_get_quat(factor->pose_i->data, q_i);
  pose_get_quat(factor->pose_j->data, q_j);
  quat2rot(q_i, C_i);
  quat2rot(q_j, C_j);
  mat_transpose(C_i, 3, 3, C_it);

  // Jacobian w.r.t. pose_j
  real_t J_pose_j[15 * 6] = {0};

  // -- Jacobian w.r.t. r_j
  real_t drij_drj[3 * 3] = {0};
  mat_copy(C_it, 3, 3, drij_drj);
  mat_block_set(J_pose_j, 6, 0, 2, 0, 2, drij_drj);

  // -- Jacobian w.r.t. q_j
  // quat_left_xyz(rot2quat(dC.T @ C_i.T @ C_j))
  QUAT2ROT(dq, dC);
  MAT_TRANSPOSE(dC, 3, 3, dCt);
  DOT3(dCt, 3, 3, C_it, 3, 3, C_j, 3, 3, dCt_C_it_C_j);
  ROT2QUAT(dCt_C_it_C_j, dqij_dqj);

  real_t dtheta_dqj[3 * 3] = {0};
  quat_left_xyz(dqij_dqj, dtheta_dqj);

  mat_block_set(J_pose_j, 6, 6, 8, 3, 5, dtheta_dqj);
  dot(factor->sqrt_info, 15, 15, J_pose_j, 15, 6, factor->jacs[3]);
}

void imu_factor_velocity_j_jac(imu_factor_t *factor) {
  real_t q_i[4] = {0};
  real_t C_i[3 * 3] = {0};
  real_t C_it[3 * 3] = {0};

  pose_get_quat(factor->pose_i->data, q_i);
  quat2rot(q_i, C_i);
  mat_transpose(C_i, 3, 3, C_it);

  real_t dv_dvj[3 * 3] = {0};
  mat_copy(C_it, 3, 3, dv_dvj);
  real_t J_vel_j[15 * 3] = {0};
  mat_block_set(J_vel_j, 3, 3, 5, 0, 2, dv_dvj);
  dot(factor->sqrt_info, 15, 15, J_vel_j, 15, 3, factor->jacs[4]);
}

void imu_factor_biases_j_jac(imu_factor_t *factor) {
  real_t J_biases_j[15 * 6] = {0};
  real_t I3[3 * 3] = {0};
  eye(I3, 3, 3);
  mat_block_set(J_biases_j, 6, 9, 11, 0, 2, I3);
  mat_block_set(J_biases_j, 6, 12, 14, 3, 5, I3);
  dot(factor->sqrt_info, 15, 15, J_biases_j, 15, 6, factor->jacs[5]);
}

/**
 * Evaluate IMU factor
 * @returns `0` for success, `-1` for failure
 */
int imu_factor_eval(void *factor_ptr) {
  imu_factor_t *factor = (imu_factor_t *) factor_ptr;
  assert(factor != NULL);
  assert(factor->pose_i);
  assert(factor->pose_j);
  assert(factor->vel_i);
  assert(factor->vel_j);
  assert(factor->biases_i);
  assert(factor->biases_j);

  // Map params
  real_t r_i[3] = {0};
  real_t q_i[4] = {0};
  real_t v_i[3] = {0};
  real_t ba_i[3] = {0};
  real_t bg_i[3] = {0};

  real_t r_j[3] = {0};
  real_t q_j[4] = {0};
  real_t v_j[3] = {0};
  real_t ba_j[3] = {0};
  real_t bg_j[3] = {0};

  pose_get_trans(factor->pose_i->data, r_i);
  pose_get_quat(factor->pose_i->data, q_i);
  vec_copy(factor->vel_i->data, 3, v_i);
  imu_biases_get_accel_bias(factor->biases_i, ba_i);
  imu_biases_get_gyro_bias(factor->biases_i, bg_i);

  pose_get_trans(factor->pose_j->data, r_j);
  pose_get_quat(factor->pose_j->data, q_j);
  vec_copy(factor->vel_j->data, 3, v_j);
  imu_biases_get_accel_bias(factor->biases_j, ba_j);
  imu_biases_get_gyro_bias(factor->biases_j, bg_j);

  // Correct the relative position, velocity and rotation
  // -- Extract Jacobians from error-state jacobian
  real_t dr_dba[3 * 3] = {0};
  real_t dr_dbg[3 * 3] = {0};
  real_t dv_dba[3 * 3] = {0};
  real_t dv_dbg[3 * 3] = {0};
  real_t dq_dbg[3 * 3] = {0};
  mat_block_get(factor->F, 15, 0, 2, 9, 11, dr_dba);
  mat_block_get(factor->F, 15, 0, 2, 12, 14, dr_dbg);
  mat_block_get(factor->F, 15, 3, 5, 9, 11, dv_dba);
  mat_block_get(factor->F, 15, 3, 5, 12, 14, dv_dbg);
  mat_block_get(factor->F, 15, 6, 8, 12, 14, dq_dbg);

  real_t dba[3] = {0};
  dba[0] = ba_i[0] - factor->ba[0];
  dba[1] = ba_i[1] - factor->ba[1];
  dba[2] = ba_i[2] - factor->ba[2];

  real_t dbg[3] = {0};
  dbg[0] = bg_i[0] - factor->bg[0];
  dbg[1] = bg_i[1] - factor->bg[1];
  dbg[2] = bg_i[2] - factor->bg[2];

  // -- Correct relative position
  // dr = dr + dr_dba * dba + dr_dbg * dbg
  real_t dr[3] = {0};
  {
    DOT(dr_dba, 3, 3, dba, 3, 1, ba_correction);
    DOT(dr_dbg, 3, 3, dbg, 3, 1, bg_correction);
    dr[0] = factor->dr[0] + ba_correction[0] + bg_correction[0];
    dr[1] = factor->dr[1] + ba_correction[1] + bg_correction[1];
    dr[2] = factor->dr[2] + ba_correction[2] + bg_correction[2];
  }
  // -- Correct relative velocity
  // dv = dv + dv_dba * dba + dv_dbg * dbg
  real_t dv[3] = {0};
  {
    DOT(dv_dba, 3, 3, dba, 3, 1, ba_correction);
    DOT(dv_dbg, 3, 3, dbg, 3, 1, bg_correction);
    dv[0] = factor->dv[0] + ba_correction[0] + bg_correction[0];
    dv[1] = factor->dv[1] + ba_correction[1] + bg_correction[1];
    dv[2] = factor->dv[2] + ba_correction[2] + bg_correction[2];
  }
  // -- Correct relative rotation
  // dq = quat_mul(dq, [1.0, 0.5 * dq_dbg * dbg])
  real_t dq[4] = {0};
  {
    real_t theta[3] = {0};
    dot(dq_dbg, 3, 3, dbg, 3, 1, theta);

    real_t q_correction[4] = {0};
    q_correction[0] = 1.0;
    q_correction[1] = 0.5 * theta[0];
    q_correction[2] = 0.5 * theta[1];
    q_correction[3] = 0.5 * theta[2];

    quat_mul(factor->dq, q_correction, dq);
    quat_normalize(dq);
  }

  // Form residuals
  const real_t g_W[3] = {0.0, 0.0, 9.81};
  const real_t Dt = factor->Dt;
  const real_t Dt_sq = Dt * Dt;
  QUAT2ROT(q_i, C_i);
  MAT_TRANSPOSE(C_i, 3, 3, C_it);

  // dr_est = C_i.T @ ((r_j - r_i) - (v_i * Dt) + (0.5 * g_W * Dt_sq))
  real_t dr_est[3] = {0};
  {
    real_t dr_tmp[3] = {0};
    dr_tmp[0] = (r_j[0] - r_i[0]) - (v_i[0] * Dt) + (0.5 * g_W[0] * Dt_sq);
    dr_tmp[1] = (r_j[1] - r_i[1]) - (v_i[1] * Dt) + (0.5 * g_W[1] * Dt_sq);
    dr_tmp[2] = (r_j[2] - r_i[2]) - (v_i[2] * Dt) + (0.5 * g_W[2] * Dt_sq);

    dot(C_it, 3, 3, dr_tmp, 3, 1, dr_est);
  }

  // dv_est = C_i.T @ ((v_j - v_i) + (g_W * Dt))
  real_t dv_est[3] = {0};
  {
    real_t dv_tmp[3] = {0};
    dv_tmp[0] = (v_j[0] - v_i[0]) + (g_W[0] * Dt);
    dv_tmp[1] = (v_j[1] - v_i[1]) + (g_W[1] * Dt);
    dv_tmp[2] = (v_j[2] - v_i[2]) + (g_W[2] * Dt);

    dot(C_it, 3, 3, dv_tmp, 3, 1, dv_est);
  }

  // err_pos = dr_est - dr
  real_t err_pos[3] = {0.0, 0.0, 0.0};
  err_pos[0] = dr_est[0] - dr[0];
  err_pos[1] = dr_est[1] - dr[1];
  err_pos[2] = dr_est[2] - dr[2];

  // err_vel = dv_est - dv
  real_t err_vel[3] = {0.0, 0.0, 0.0};
  err_vel[0] = dv_est[0] - dv[0];
  err_vel[1] = dv_est[1] - dv[1];
  err_vel[2] = dv_est[2] - dv[2];

  // err_rot = (2.0 * quat_mul(quat_inv(dq), quat_mul(quat_inv(q_i), q_j)))[1:4]
  real_t err_rot[3] = {0.0, 0.0, 0.0};
  {
    real_t dq_inv[4] = {0};
    real_t q_i_inv[4] = {0};
    real_t q_i_inv_j[4] = {0};
    real_t err_quat[4] = {0};

    quat_inv(dq, dq_inv);
    quat_inv(q_i, q_i_inv);
    quat_mul(q_i_inv, q_j, q_i_inv_j);
    quat_mul(dq_inv, q_i_inv_j, err_quat);

    err_rot[0] = 2.0 * err_quat[1];
    err_rot[1] = 2.0 * err_quat[2];
    err_rot[2] = 2.0 * err_quat[3];
  }

  // err_ba = ba_j - ba_i
  real_t err_ba[3] = {ba_j[0] - ba_i[0], ba_j[1] - ba_i[1], ba_j[2] - ba_i[2]};

  // err_bg = bg_j - bg_i
  real_t err_bg[3] = {bg_j[0] - bg_i[0], bg_j[1] - bg_i[1], bg_j[2] - bg_i[2]};

  // Residual vector
  // r = sqrt_info * [err_pos; err_vel; err_rot; err_ba; err_bg]
  {
    real_t r_raw[15] = {0};
    r_raw[0] = err_pos[0];
    r_raw[1] = err_pos[1];
    r_raw[2] = err_pos[2];

    r_raw[3] = err_vel[0];
    r_raw[4] = err_vel[1];
    r_raw[5] = err_vel[2];

    r_raw[6] = err_rot[0];
    r_raw[7] = err_rot[1];
    r_raw[8] = err_rot[2];

    r_raw[9] = err_ba[0];
    r_raw[10] = err_ba[1];
    r_raw[11] = err_ba[2];

    r_raw[12] = err_bg[0];
    r_raw[13] = err_bg[1];
    r_raw[14] = err_bg[2];

    dot(factor->sqrt_info, 15, 15, r_raw, 15, 1, factor->r);
  }

  // Form Jacobians
  imu_factor_pose_i_jac(factor, dr_est, dv_est, dq);
  imu_factor_velocity_i_jac(factor);
  imu_factor_biases_i_jac(factor, dq_dbg, dr_dba, dv_dba, dr_dbg, dv_dbg);
  imu_factor_pose_j_jac(factor, dq);
  imu_factor_velocity_j_jac(factor);
  imu_factor_biases_j_jac(factor);

  return 0;
}

int imu_factor_ceres_eval(void *factor_ptr,
                          real_t **params,
                          real_t *r_out,
                          real_t **J_out) {
  CERES_FACTOR_EVAL(imu_jactor,
                    ((imu_factor_t *) factor_ptr),
                    imu_factor_eval,
                    params,
                    r_out,
                    J_out);
}

//////////////////
// LIDAR FACTOR //
//////////////////

pcd_t *pcd_malloc(const timestamp_t ts_start,
                  const timestamp_t ts_end,
                  const float *data,
                  const float *time_diffs,
                  const size_t num_points) {
  pcd_t *pcd = malloc(sizeof(pcd_t));

  pcd->ts_start = ts_start;
  pcd->ts_end = ts_end;

  pcd->data = malloc(sizeof(float) * 3 * num_points);
  for (size_t i = 0; i < num_points; ++i) {
    pcd->data[i * 3 + 0] = data[i * 3 + 0];
    pcd->data[i * 3 + 1] = data[i * 3 + 1];
    pcd->data[i * 3 + 2] = data[i * 3 + 2];
  }

  pcd->time_diffs = malloc(sizeof(float) * num_points);
  for (size_t i = 0; i < num_points; ++i) {
    pcd->time_diffs[i] = time_diffs[i];
  }
  pcd->num_points = num_points;

  return pcd;
}

void pcd_free(pcd_t *pcd) {
  if (pcd == NULL) {
    return;
  }

  free(pcd->data);
  free(pcd->time_diffs);
  free(pcd);
}

void pcd_deskew(pcd_t *pcd,
                const real_t T_WL_km1[4 * 4],
                const real_t T_WL_km2[4 * 4]) {
  assert(pcd);
  assert(T_WL_km1);
  assert(T_WL_km2);

  // Setup
  const real_t ts_start = ts2sec(pcd->ts_start);
  const real_t ts_end = ts2sec(pcd->ts_end);
  const real_t dt = ts_end - ts_start;
  TF_ROT(T_WL_km2, C_WL_km2);
  TF_ROT(T_WL_km1, C_WL_km1);
  TF_TRANS(T_WL_km2, r_WL_km2);
  TF_TRANS(T_WL_km1, r_WL_km1);

  // v_WL = (C_WL_km2' * (r_WL_km1 - r_WL_km2)) / dt
  VEC_SUB(r_WL_km1, r_WL_km2, dr, 3);
  MAT_TRANSPOSE(C_WL_km2, 3, 3, C_WL_t_km2);
  DOT(C_WL_t_km2, 3, 3, dr, 3, 1, v_WL);
  vec_scale(v_WL, 3, 1.0 / dt);

  // w_WL = (Log(C_WL_km2' * C_WL_km1)) / dt
  real_t w_WL[3] = {0};
  DOT(C_WL_t_km2, 3, 3, C_WL_km1, 3, 3, dC);
  lie_Log(dC, w_WL);
  vec_scale(w_WL, 3, 1.0 / dt);

  // Deskew point cloud
  // p = Exp(s_i * w_WL) * p + s_i * v_WL
  for (size_t i = 0; i < pcd->num_points; ++i) {
    real_t p[3] = {
        pcd->data[i * 3 + 0],
        pcd->data[i * 3 + 1],
        pcd->data[i * 3 + 2],
    };

    const real_t s_i = pcd->time_diffs[i];
    const real_t v_WL_i[3] = {s_i * v_WL[0], s_i * v_WL[1], s_i * v_WL[2]};
    const real_t w_WL_i[3] = {s_i * w_WL[0], s_i * w_WL[1], s_i * w_WL[2]};

    real_t dC[3 * 3] = {0};
    lie_Exp(w_WL_i, dC);
    DOT(dC, 3, 3, p, 3, 1, p_new);
    vec_add(p_new, v_WL_i, p, 3);
  }
}

/**
 * Setup lidar factor
 */
void lidar_factor_setup(lidar_factor_t *factor,
                        pcd_t *pcd,
                        pose_t *pose,
                        const real_t var[3]) {
  assert(factor != NULL);
  assert(pcd != NULL);
  assert(pose != NULL);
  assert(var != NULL);

  // Parameters
  factor->pcd = pcd;
  factor->pose = pose;

  // Measurement covariance
  zeros(factor->covar, 3, 3);
  factor->covar[0] = var[0];
  factor->covar[4] = var[1];
  factor->covar[8] = var[2];

  // Square-root information matrix
  zeros(factor->sqrt_info, 3, 3);
  factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);
  factor->sqrt_info[4] = sqrt(1.0 / factor->covar[1]);
  factor->sqrt_info[8] = sqrt(1.0 / factor->covar[2]);

  // Factor parameters, residuals and Jacobians
  factor->r_size = pcd->num_points * 3;
  factor->num_params = 1;
  factor->param_types[0] = POSE_PARAM;
  factor->params[0] = factor->pose->data;
  factor->jacs[0] = factor->J_pose;
}

/**
 * Evaluate lidar factor
 */
void lidar_factor_eval(void *factor_ptr) {
  lidar_factor_t *factor = (lidar_factor_t *) factor_ptr;
  assert(factor != NULL);
  assert(factor->pose);
  assert(factor->pcd);

  // Map params
  // TF(factor->params[0], T_WB);
  // TF(factor->params[1], T_BL);
  // TF_CHAIN(T_WL, 2, T_WB, T_BL);
  // TF_INV(T_WL, T_LW);
  // TF_ROT(T_LW, C_LW);
  // TF_TRANS(T_LW, r_LW);

  // Calculate residuals
  // points_W = points_L * C_LW + r_LW
  // real_t *points_W_hat = malloc(sizeof(real_t) * 3 * factor->pcd->num_points);
  // dot(factor->pcd->data, factor->pcd->num_points, 3, C_LW, 3, 3, points_W_hat);
  // for (size_t i = 0; i < factor->pcd->num_points; ++i) {
  //   points_W_hat[i * 3 + 0] += r_LW[0];
  //   points_W_hat[i * 3 + 1] += r_LW[1];
  //   points_W_hat[i * 3 + 2] += r_LW[2];
  // }
  // r = points_W - points_W_hat
  // factor->r = malloc(sizeof(real_t) * 3 * factor->pcd->num_points);
  // for (size_t i = 0; i < factor->num_points; ++i) {
  //   factor->r[i * 3 + 0] = points_W[i * 3 + 0] - points_W_hat[i * 3 + 0];
  //   factor->r[i * 3 + 1] = points_W[i * 3 + 1] - points_W_hat[i * 3 + 1];
  //   factor->r[i * 3 + 2] = points_W[i * 3 + 2] - points_W_hat[i * 3 + 2];
  // }

  // Calculate jacobians
  // -- Form: -1 * sqrt_info
  // real_t neg_sqrt_info[3 * 3] = {0};
  // mat_copy(factor->sqrt_info, 3, 3, neg_sqrt_info);
  // mat_scale(neg_sqrt_info, 3, 3, -1.0);
  // -- Fill jacobians
  // const size_t num_rows = 3 * factor->num_points;
  // const size_t num_cols = 6;
  // factor->J_pose = malloc(sizeof(real_t) * num_rows * num_cols);
  // const int stride = 6;
  // for (size_t i = 0; i < factor->num_points; ++i) {
  //   const int rs = i * 3 + 0;
  //   const int re = i * 3 + 2;
  //   const int cs = 0;
  //   const int ce = 5;
  //   mat_block_set(factor->J_pose,
  //                 stride,
  //                 rs,
  //                 re,
  //                 cs,
  //                 ce,
  //                 J_pose_i);
  // }
}

////////////////////////
// JOINT-ANGLE FACTOR //
////////////////////////

/**
 * Setup joint-angle factor
 */
void joint_factor_setup(joint_factor_t *factor,
                        joint_t *joint,
                        const real_t z,
                        const real_t var) {
  assert(factor != NULL);
  assert(joint != NULL);

  // Parameters
  factor->joint = joint;
  factor->num_params = 1;

  // Measurement
  factor->z[0] = z;

  // Measurement covariance matrix
  factor->covar[0] = var;

  // Square-root information matrix
  factor->sqrt_info[0] = sqrt(1.0 / factor->covar[0]);

  // Factor residuals, parameters and Jacobians
  factor->r_size = 1;
  factor->num_params = 1;
  factor->param_types[0] = JOINT_PARAM;
  factor->params[0] = factor->joint->data;
  factor->jacs[0] = factor->J_joint;
}

/**
 * Copy joint factor.
 */
void joint_factor_copy(const joint_factor_t *src, joint_factor_t *dst) {
  assert(src != NULL);
  assert(dst != NULL);

  dst->joint = src->joint;
  dst->z[0] = src->z[0];
  dst->covar[0] = src->covar[0];
  dst->sqrt_info[0] = src->sqrt_info[0];

  dst->r_size = src->r_size;
  dst->num_params = src->num_params;
  dst->param_types[0] = src->param_types[0];

  dst->params[0] = src->params[0];
  dst->r[0] = src->r[0];
  dst->J_joint[0] = src->J_joint[0];
}

/**
 * Evaluate joint-angle factor
 * @returns `0` for success, `-1` for failure
 */
int joint_factor_eval(void *factor_ptr) {
  assert(factor_ptr != NULL);

  // Map factor
  joint_factor_t *factor = (joint_factor_t *) factor_ptr;

  // Calculate residuals
  factor->r[0] = factor->sqrt_info[0] * (factor->z[0] - factor->joint->data[0]);

  // Calculate Jacobians
  factor->jacs[0][0] = -1 * factor->sqrt_info[0];

  return 0;
}

/**
 * Check if two joint factors are equal in value.
 */
int joint_factor_equals(const joint_factor_t *j0, const joint_factor_t *j1) {
  CHECK(vec_equals(j0->z, j1->z, 1));
  CHECK(mat_equals(j0->covar, j1->covar, 1, 1, 1e-8));
  CHECK(mat_equals(j0->sqrt_info, j1->sqrt_info, 1, 1, 1e-8));

  CHECK(j0->r_size == j1->r_size);
  CHECK(j0->num_params == j1->num_params);
  CHECK(j0->param_types[0] == j1->param_types[0]);

  CHECK(vec_equals(j0->params[0], j1->params[0], 1));
  CHECK(vec_equals(j0->r, j1->r, 1));
  CHECK(vec_equals(j0->jacs[0], j1->jacs[0], 1));
  CHECK(mat_equals(j0->J_joint, j1->J_joint, 1, 1, 1e-8));

  return 1;
error:
  return 0;
}

//////////////////
// MARGINALIZER //
//////////////////

/**
 * Malloc marginalization factor.
 */
marg_factor_t *marg_factor_malloc(void) {
  marg_factor_t *marg = malloc(sizeof(marg_factor_t) * 1);

  // Settings
  marg->debug = 1;
  marg->cond_hessian = 1;

  // Flags
  marg->marginalized = 0;
  marg->schur_complement_ok = 0;
  marg->eigen_decomp_ok = 0;

  // Parameters
  // -- Remain parameters
  marg->r_positions = NULL;
  marg->r_rotations = NULL;
  marg->r_poses = NULL;
  marg->r_velocities = NULL;
  marg->r_imu_biases = NULL;
  marg->r_fiducials = NULL;
  marg->r_joints = NULL;
  marg->r_extrinsics = NULL;
  marg->r_features = NULL;
  marg->r_cam_params = NULL;
  marg->r_time_delays = NULL;
  // -- Marginal parameters
  marg->m_positions = NULL;
  marg->m_rotations = NULL;
  marg->m_poses = NULL;
  marg->m_velocities = NULL;
  marg->m_imu_biases = NULL;
  marg->m_fiducials = NULL;
  marg->m_joints = NULL;
  marg->m_extrinsics = NULL;
  marg->m_features = NULL;
  marg->m_cam_params = NULL;
  marg->m_time_delays = NULL;

  // Factors
  marg->ba_factors = list_malloc();
  marg->camera_factors = list_malloc();
  marg->idf_factors = list_malloc();
  marg->imu_factors = list_malloc();
  marg->calib_camera_factors = list_malloc();
  marg->calib_imucam_factors = list_malloc();
  marg->marg_factor = NULL;

  // Hessian and residuals
  marg->hash = NULL;
  marg->m_size = 0;
  marg->r_size = 0;

  marg->x0 = NULL;
  marg->r0 = NULL;
  marg->J0 = NULL;
  marg->dchi = NULL;
  marg->J0_dchi = NULL;

  marg->J0 = NULL;
  marg->J0_inv = NULL;
  marg->H = NULL;
  marg->b = NULL;
  marg->H_marg = NULL;
  marg->b_marg = NULL;

  // Parameters, residuals and Jacobians
  marg->num_params = 0;
  marg->param_types = NULL;
  marg->params = NULL;
  marg->r = NULL;
  marg->jacs = NULL;

  // Profiling
  marg->time_hessian_form = 0;
  marg->time_schur_complement = 0;
  marg->time_hessian_decomp = 0;
  marg->time_fejs = 0;
  marg->time_total = 0;

  return marg;
}

/**
 * Free marginalization factor.
 */
void marg_factor_free(marg_factor_t *marg) {
  if (marg == NULL) {
    return;
  }

  // Parameters
  // -- Remain parameters
  hmfree(marg->r_positions);
  hmfree(marg->r_rotations);
  hmfree(marg->r_poses);
  hmfree(marg->r_velocities);
  hmfree(marg->r_imu_biases);
  hmfree(marg->r_features);
  hmfree(marg->r_joints);
  hmfree(marg->r_extrinsics);
  // hmfree(marg->r_fiducials);
  hmfree(marg->r_cam_params);
  hmfree(marg->r_time_delays);
  // -- Marginal parameters
  hmfree(marg->m_positions);
  hmfree(marg->m_rotations);
  hmfree(marg->m_poses);
  hmfree(marg->m_velocities);
  hmfree(marg->m_imu_biases);
  hmfree(marg->m_features);
  hmfree(marg->m_joints);
  hmfree(marg->m_extrinsics);
  // hmfree(marg->m_fiducials);
  hmfree(marg->m_cam_params);
  hmfree(marg->m_time_delays);

  // Factors
  list_free(marg->ba_factors);
  list_free(marg->camera_factors);
  list_free(marg->idf_factors);
  list_free(marg->imu_factors);
  list_free(marg->calib_camera_factors);
  list_free(marg->calib_imucam_factors);

  // Residuals
  hmfree(marg->hash);
  free(marg->x0);
  free(marg->r0);
  free(marg->J0);
  free(marg->J0_inv);
  free(marg->dchi);
  free(marg->J0_dchi);

  free(marg->H);
  free(marg->b);
  free(marg->H_marg);
  free(marg->b_marg);

  // Jacobians
  free(marg->param_types);
  if (marg->param_ptrs) {
    free(marg->param_ptrs);
  }
  free(marg->params);
  free(marg->r);
  for (int i = 0; i < marg->num_params; i++) {
    free(marg->jacs[i]);
  }
  free(marg->jacs);

  free(marg);
}

void marg_factor_print_stats(const marg_factor_t *marg) {
  printf("Parameters to be marginalized:\n");
  printf("------------------------------\n");
  printf("m_positions: %ld\n", hmlen(marg->m_positions));
  printf("m_rotations: %ld\n", hmlen(marg->m_rotations));
  printf("m_poses: %ld\n", hmlen(marg->m_poses));
  printf("m_velocities: %ld\n", hmlen(marg->m_velocities));
  printf("m_imu_biases: %ld\n", hmlen(marg->m_imu_biases));
  printf("m_features: %ld\n", hmlen(marg->m_features));
  printf("m_joints: %ld\n", hmlen(marg->m_joints));
  printf("m_extrinsics: %ld\n", hmlen(marg->m_extrinsics));
  // printf("m_fiducials: %ld\n", hmlen(marg->m_fiducials));
  printf("m_cam_params: %ld\n", hmlen(marg->m_cam_params));
  printf("m_time_delays: %ld\n", hmlen(marg->m_time_delays));
  printf("\n");

  printf("Parameters to remain:\n");
  printf("---------------------\n");
  printf("r_positions: %ld\n", hmlen(marg->r_positions));
  printf("r_rotations: %ld\n", hmlen(marg->r_rotations));
  printf("r_poses: %ld\n", hmlen(marg->r_poses));
  printf("r_velocities: %ld\n", hmlen(marg->r_velocities));
  printf("r_imu_biases: %ld\n", hmlen(marg->r_imu_biases));
  printf("r_features: %ld\n", hmlen(marg->r_features));
  printf("r_joints: %ld\n", hmlen(marg->r_joints));
  printf("r_extrinsics: %ld\n", hmlen(marg->r_extrinsics));
  // printf("r_fiducials: %ld\n", hmlen(marg->r_fiducials));
  printf("r_cam_params: %ld\n", hmlen(marg->r_cam_params));
  printf("r_time_delays: %ld\n", hmlen(marg->r_time_delays));
  printf("\n");
}

/**
 * Add factor to marginalization factor.
 */
void marg_factor_add(marg_factor_t *marg, int factor_type, void *factor_ptr) {
  assert(marg != NULL);
  assert(factor_ptr != NULL);

  switch (factor_type) {
    case MARG_FACTOR:
      if (marg->marg_factor == NULL) {
        marg->marg_factor = factor_ptr;
      } else {
        LOG_ERROR("Marginalization factor already set!");
        FATAL("Implementation Error!\n");
      }
      break;
    case BA_FACTOR:
      list_push(marg->ba_factors, factor_ptr);
      break;
    case CAMERA_FACTOR:
      list_push(marg->camera_factors, factor_ptr);
      break;
    case IDF_FACTOR:
      list_push(marg->idf_factors, factor_ptr);
      break;
    case IMU_FACTOR:
      list_push(marg->imu_factors, factor_ptr);
      break;
    case CALIB_CAMERA_FACTOR:
      list_push(marg->calib_camera_factors, factor_ptr);
      break;
    case CALIB_IMUCAM_FACTOR:
      list_push(marg->calib_imucam_factors, factor_ptr);
      break;
    default:
      FATAL("Implementation Error!\n");
      break;
  };
}

/**
 * Form Hessian matrix using data in marginalization factor.
 */
static void marg_factor_hessian_form(marg_factor_t *marg) {
  // Track Factor Params
  // -- Track marginalization factor params
  if (marg->marg_factor) {
    for (int i = 0; i < marg->marg_factor->num_params; i++) {
      void *param = marg->marg_factor->param_ptrs[i];
      int param_type = marg->marg_factor->param_types[i];
      MARG_TRACK_FACTOR(param, param_type);
    }
  }
  // -- Track BA factor params
  {
    list_node_t *node = marg->ba_factors->first;
    while (node != NULL) {
      ba_factor_t *factor = (ba_factor_t *) node->value;
      MARG_TRACK(marg->r_poses, marg->m_poses, factor->pose);
      MARG_TRACK(marg->r_features, marg->m_features, factor->feature);
      MARG_TRACK(marg->r_cam_params, marg->m_cam_params, factor->camera);
      node = node->next;
    }
  }
  // -- Track camera factor params
  {
    list_node_t *node = marg->camera_factors->first;
    while (node != NULL) {
      camera_factor_t *factor = (camera_factor_t *) node->value;
      MARG_TRACK(marg->r_poses, marg->m_poses, factor->pose);
      MARG_TRACK(marg->r_extrinsics, marg->m_extrinsics, factor->extrinsic);
      MARG_TRACK(marg->r_features, marg->m_features, factor->feature);
      MARG_TRACK(marg->r_cam_params, marg->m_cam_params, factor->camera);
      node = node->next;
    }
  }
  // -- Track IDF factor params
  // {
  //   list_node_t *node = marg->idf_factors->first;
  //   while (node != NULL) {
  //     idf_factor_t *factor = (idf_factor_t *) node->value;
  //     MARG_TRACK(marg->r_poses, marg->m_poses, factor->pose);
  //     MARG_TRACK(marg->r_extrinsics, marg->m_extrinsics, factor->extrinsic);
  //     MARG_TRACK(marg->r_cam_params, marg->m_cam_params, factor->camera);
  //     MARG_TRACK(marg->r_positions, marg->m_positions, factor->idf_pos);
  //     MARG_TRACK(marg->r_features, marg->m_features, factor->idf_param);
  //     node = node->next;
  //   }
  // }
  // -- Track IMU factor params
  {
    list_node_t *node = marg->imu_factors->first;
    while (node != NULL) {
      imu_factor_t *factor = (imu_factor_t *) node->value;
      MARG_TRACK(marg->r_poses, marg->m_poses, factor->pose_i);
      MARG_TRACK(marg->r_velocities, marg->m_velocities, factor->vel_i);
      MARG_TRACK(marg->r_imu_biases, marg->m_imu_biases, factor->biases_i);
      MARG_TRACK(marg->r_poses, marg->m_poses, factor->pose_j);
      MARG_TRACK(marg->r_velocities, marg->m_velocities, factor->vel_j);
      MARG_TRACK(marg->r_imu_biases, marg->m_imu_biases, factor->biases_j);
      node = node->next;
    }
  }
  // // -- Track calib camera factor params
  // {
  //   list_node_t *node = marg->calib_camera_factors->first;
  //   while (node != NULL) {
  //     calib_camera_factor_t *factor = (calib_camera_factor_t *) node->value;
  //     MARG_TRACK(marg->r_poses, marg->m_poses, factor->pose);
  //     MARG_TRACK(marg->r_extrinsics, marg->m_extrinsics, factor->cam_ext);
  //     MARG_TRACK(marg->r_cam_params, marg->m_cam_params, factor->cam_params);
  //     node = node->next;
  //   }
  // }
  // // -- Track calib imucam factor params
  // {
  //   list_node_t *node = marg->calib_imucam_factors->first;
  //   while (node != NULL) {
  //     calib_imucam_factor_t *factor = (calib_imucam_factor_t *) node->value;
  //     MARG_TRACK(marg->r_fiducials, marg->m_fiducials, factor->fiducial);
  //     MARG_TRACK(marg->r_poses, marg->m_poses, factor->imu_pose);
  //     MARG_TRACK(marg->r_extrinsics, marg->m_extrinsics, factor->imu_ext);
  //     MARG_TRACK(marg->r_extrinsics, marg->m_extrinsics, factor->cam_ext);
  //     MARG_TRACK(marg->r_cam_params, marg->m_cam_params, factor->cam_params);
  //     MARG_TRACK(marg->r_time_delays, marg->m_time_delays, factor->time_delay);
  //     node = node->next;
  //   }
  // }

  // Determine parameter block column indicies for Hessian matrix H
  // clang-format off
  int H_idx = 0; // Column / row index of Hessian matrix H
  int m = 0;     // Marginal local parameter length
  int r = 0;     // Remain local parameter length
  int gm = 0;    // Marginal global parameter length
  int gr = 0;    // Remain global parameter length
  int nm = 0;    // Number of marginal parameters
  int nr = 0;    // Number of remain parameters
  // -- Column indices for parameter blocks to be marginalized
  MARG_INDEX(marg->m_positions, POSITION_PARAM, marg->hash, &H_idx, m, gm, nm);
  MARG_INDEX(marg->m_rotations, ROTATION_PARAM, marg->hash, &H_idx, m, gm, nm);
  MARG_INDEX(marg->m_poses, POSE_PARAM, marg->hash, &H_idx, m, gm, nm);
  MARG_INDEX(marg->m_velocities, VELOCITY_PARAM, marg->hash, &H_idx, m, gm, nm);
  MARG_INDEX(marg->m_imu_biases, IMU_BIASES_PARAM, marg->hash, &H_idx, m, gm, nm);
  MARG_INDEX(marg->m_features, FEATURE_PARAM, marg->hash, &H_idx, m, gm, nm);
  MARG_INDEX(marg->m_joints, JOINT_PARAM, marg->hash, &H_idx, m, gm, nm);
  MARG_INDEX(marg->m_extrinsics, EXTRINSIC_PARAM, marg->hash, &H_idx, m, gm, nm);
  MARG_INDEX(marg->m_fiducials, FIDUCIAL_PARAM, marg->hash, &H_idx, m, gm, nm);
  MARG_INDEX(marg->m_cam_params, CAMERA_PARAM, marg->hash, &H_idx, m, gm, nm);
  MARG_INDEX(marg->m_time_delays, TIME_DELAY_PARAM, marg->hash, &H_idx, m, gm, nm);
  // -- Column indices for parameter blocks to remain
  MARG_INDEX(marg->r_positions, POSITION_PARAM, marg->hash, &H_idx, r, gr, nr);
  MARG_INDEX(marg->r_rotations, ROTATION_PARAM, marg->hash, &H_idx, r, gr, nr);
  MARG_INDEX(marg->r_poses, POSE_PARAM, marg->hash, &H_idx, r, gr, nr);
  MARG_INDEX(marg->r_velocities, VELOCITY_PARAM, marg->hash, &H_idx, r, gr, nr);
  MARG_INDEX(marg->r_imu_biases, IMU_BIASES_PARAM, marg->hash, &H_idx, r, gr, nr);
  MARG_INDEX(marg->r_features, FEATURE_PARAM, marg->hash, &H_idx, r, gr, nr);
  MARG_INDEX(marg->r_joints, JOINT_PARAM, marg->hash, &H_idx, r, gr, nr);
  MARG_INDEX(marg->r_extrinsics, EXTRINSIC_PARAM, marg->hash, &H_idx, r, gr, nr);
  MARG_INDEX(marg->r_fiducials, FIDUCIAL_PARAM, marg->hash, &H_idx, r, gr, nr);
  MARG_INDEX(marg->r_cam_params, CAMERA_PARAM, marg->hash, &H_idx, r, gr, nr);
  MARG_INDEX(marg->r_time_delays, TIME_DELAY_PARAM, marg->hash, &H_idx, r, gr, nr);
  // clang-format on

  // Track linearization point x0 and parameter pointers
  assert(gm > 0);
  assert(nm > 0);
  assert(gr > 0);
  assert(nr > 0);

  int param_idx = 0;
  int x0_idx = 0;
  marg->x0 = malloc(sizeof(real_t) * gr);
  marg->num_params = nr;
  marg->param_types = malloc(sizeof(int) * nr);
  marg->param_ptrs = malloc(sizeof(void *) * nr);
  marg->params = malloc(sizeof(real_t *) * nr);
  MARG_PARAMS(marg, marg->r_positions, POSITION_PARAM, param_idx, x0_idx);
  MARG_PARAMS(marg, marg->r_rotations, ROTATION_PARAM, param_idx, x0_idx);
  MARG_PARAMS(marg, marg->r_poses, POSE_PARAM, param_idx, x0_idx);
  MARG_PARAMS(marg, marg->r_velocities, VELOCITY_PARAM, param_idx, x0_idx);
  MARG_PARAMS(marg, marg->r_imu_biases, IMU_BIASES_PARAM, param_idx, x0_idx);
  MARG_PARAMS(marg, marg->r_features, FEATURE_PARAM, param_idx, x0_idx);
  MARG_PARAMS(marg, marg->r_joints, JOINT_PARAM, param_idx, x0_idx);
  MARG_PARAMS(marg, marg->r_extrinsics, EXTRINSIC_PARAM, param_idx, x0_idx);
  MARG_PARAMS(marg, marg->r_fiducials, FIDUCIAL_PARAM, param_idx, x0_idx);
  MARG_PARAMS(marg, marg->r_cam_params, CAMERA_PARAM, param_idx, x0_idx);
  MARG_PARAMS(marg, marg->r_time_delays, TIME_DELAY_PARAM, param_idx, x0_idx);

  // Allocate memory LHS and RHS of Gauss newton
  marg->m_size = m;
  marg->r_size = r;
  const int ls = m + r;
  real_t *H = calloc(ls * ls, sizeof(real_t));
  real_t *b = calloc(ls * 1, sizeof(real_t));

  // Fill Hessian
  if (marg->marg_factor) {
    solver_fill_hessian(marg->hash,
                        marg->marg_factor->num_params,
                        marg->marg_factor->params,
                        marg->marg_factor->jacs,
                        marg->marg_factor->r,
                        marg->marg_factor->r_size,
                        ls,
                        H,
                        b);
  }

  // param_order_print(marg->hash);
  MARG_H(marg, ba_factor_t, marg->ba_factors, H, b, ls);
  MARG_H(marg, camera_factor_t, marg->camera_factors, H, b, ls);
  // MARG_H(marg, idf_factor_t, marg->idf_factors, H, b, ls);
  MARG_H(marg, imu_factor_t, marg->imu_factors, H, b, ls);
  // MARG_H(marg, calib_camera_factor_t, marg->calib_camera_factors, H, b, ls);
  // MARG_H(marg, calib_imucam_factor_t, marg->calib_imucam_factors, H, b, ls);
  marg->H = H;
  marg->b = b;
  // param_order_print(marg->hash);
  // mat_save("/tmp/H.csv", marg->H, ls, ls);
  // mat_save("/tmp/b.csv", marg->b, ls, 1);
}

/**
 * Perform Schur-Complement.
 */
static void marg_factor_schur_complement(marg_factor_t *marg) {
  // Compute Schurs Complement
  const int m = marg->m_size;
  const int r = marg->r_size;
  const int ls = m + r;
  const real_t *H = marg->H;
  const real_t *b = marg->b;
  real_t *H_marg = malloc(sizeof(real_t) * r * r);
  real_t *b_marg = malloc(sizeof(real_t) * r * 1);
  if (schur_complement(H, b, ls, m, r, H_marg, b_marg) == 0) {
    marg->schur_complement_ok = 1;
  }
  marg->H_marg = H_marg;
  marg->b_marg = b_marg;

  // Enforce symmetry: H_marg = 0.5 * (H_marg + H_marg')
  // if (marg->cond_hessian) {
  //   enforce_spd(marg->H_marg, r, r);
  // }

  // printf("m: %d\n", m);
  // printf("r: %d\n", r);
  // mat_save("/tmp/H.csv", marg->H, ls, ls);
  // mat_save("/tmp/b.csv", marg->b, ls, 1);
  // mat_save("/tmp/H_marg.csv", marg->H_marg, r, r);
  // mat_save("/tmp/b_marg.csv", marg->b_marg, r, 1);
  // exit(0);
}

/**
 * Decompose Hessian into two Jacobians.
 */
static void marg_factor_hessian_decomp(marg_factor_t *marg) {
  // Decompose H_marg into Jt and J, and in the process also obtain inv(J).
  // Hessian H_marg can be decomposed via Eigen-decomposition:
  //
  //   H_marg = J' * J = V * diag(w) * V'
  //   J = diag(w^{0.5}) * V'
  //   J_inv = diag(w^-0.5) * V'
  //
  // -- Setup
  const int r = marg->r_size;
  real_t *J = calloc(r * r, sizeof(real_t));
  real_t *J_inv = calloc(r * r, sizeof(real_t));
  real_t *V = calloc(r * r, sizeof(real_t));
  real_t *Vt = calloc(r * r, sizeof(real_t));
  real_t *w = calloc(r, sizeof(real_t));
  real_t *W_sqrt = calloc(r * r, sizeof(real_t));
  real_t *W_inv_sqrt = calloc(r * r, sizeof(real_t));

  // -- Eigen decomposition
  if (eig_sym(marg->H_marg, r, r, V, w) != 0) {
    free(J);
    free(J_inv);
    free(V);
    free(Vt);
    free(w);
    free(W_sqrt);
    free(W_inv_sqrt);
    return;
  }
  mat_transpose(V, r, r, Vt);

  // -- Form J and J_inv:
  //
  //   J = diag(w^0.5) * V'
  //   J_inv = diag(w^-0.5) * V'
  //
  const real_t tol = 1e-18;
  for (int i = 0; i < r; i++) {
    if (w[i] > tol) {
      W_sqrt[(i * r) + i] = sqrt(w[i]);
      W_inv_sqrt[(i * r) + i] = sqrt(1.0 / w[i]);
    } else {
      W_sqrt[(i * r) + i] = 0.0;
      W_inv_sqrt[(i * r) + i] = 0.0;
    }
  }
  dot(W_sqrt, r, r, Vt, r, r, J);
  dot(W_inv_sqrt, r, r, Vt, r, r, J_inv);
  mat_scale(J_inv, r, r, -1.0);
  marg->eigen_decomp_ok = 1;

  // Check J' * J == H_marg
  if (marg->debug) {
    real_t *Jt = calloc(r * r, sizeof(real_t));
    real_t *H_ = calloc(r * r, sizeof(real_t));
    mat_transpose(J, r, r, Jt);
    dot(Jt, r, r, J, r, r, H_);

    real_t diff = 0.0;
    for (int i = 0; i < (r * r); i++) {
      diff += pow(H_[i] - marg->H_marg[i], 2);
    }

    if (diff > 1e-2) {
      marg->eigen_decomp_ok = 0;
      LOG_WARN("J' * J != H_marg. Diff is %.2e\n", diff);
      LOG_WARN("This is bad ... Usually means marginalization is bad!\n");
    }

    free(Jt);
    free(H_);
  }

  // Check J_inv * J == eye
  // if (marg->debug) {
  //   if (check_inv(J, J_inv, r) != 0) {
  //     marg->eigen_decomp_ok = 0;
  //     LOG_WARN("inv(J) * J != eye\n");
  //   }
  // }

  // Update
  marg->J0 = J;
  marg->J0_inv = J_inv;

  // Clean up
  free(V);
  free(Vt);
  free(w);
  free(W_sqrt);
  free(W_inv_sqrt);
}

static void marg_factor_form_fejs(marg_factor_t *marg) {
  // Track Linearized residuals, jacobians
  // -- Linearized residuals: r0 = -J0_inv * b_marg;
  marg->r0 = malloc(sizeof(real_t) * marg->r_size);
  dot(marg->J0_inv,
      marg->r_size,
      marg->r_size,
      marg->b_marg,
      marg->r_size,
      1,
      marg->r0);
  // -- Linearized jacobians: J0 = J;
  marg->dchi = malloc(sizeof(real_t) * marg->r_size);
  marg->J0_dchi = malloc(sizeof(real_t) * marg->r_size);

  // Form First-Estimate Jacobians (FEJ)
  const size_t m = marg->r_size;
  const int col_offset = -marg->m_size;
  const int rs = 0;
  const int re = m - 1;
  marg->r = malloc(sizeof(real_t) * m);
  marg->jacs = malloc(sizeof(real_t *) * marg->num_params);

  char param_type[100] = {0};
  for (size_t i = 0; i < marg->num_params; i++) {
    real_t *param_ptr = marg->params[i];
    const param_order_t *param_info = &hmgets(marg->hash, param_ptr);
    param_type_string(param_info->type, param_type);
    const int n = param_local_size(param_info->type);
    const int cs = param_info->idx + col_offset;
    const int ce = cs + n - 1;

    marg->jacs[i] = malloc(sizeof(real_t) * m * n);
    mat_block_get(marg->J0, m, rs, re, cs, ce, marg->jacs[i]);
  }
}

void marg_factor_marginalize(marg_factor_t *marg) {
  // Form Hessian and RHS of Gauss newton
  TIC(hessian_form);
  marg_factor_hessian_form(marg);
  marg->time_hessian_form = TOC(hessian_form);
  marg->time_total += marg->time_hessian_form;

  // Apply Schur Complement
  TIC(schur);
  marg_factor_schur_complement(marg);
  marg->time_schur_complement = TOC(schur);
  marg->time_total += marg->time_schur_complement;

  // Decompose marginalized Hessian
  TIC(hessian_decomp);
  marg_factor_hessian_decomp(marg);
  marg->time_hessian_decomp = TOC(hessian_decomp);
  marg->time_total += marg->time_hessian_decomp;

  // Form FEJs
  TIC(fejs);
  marg_factor_form_fejs(marg);
  marg->time_fejs = TOC(fejs);
  marg->time_total += marg->time_fejs;

  // Update state
  marg->marginalized = 1;
}

int marg_factor_eval(void *marg_ptr) {
  assert(marg_ptr);

  // Map factor
  marg_factor_t *marg = (marg_factor_t *) marg_ptr;
  assert(marg->marginalized == 1);

  // Compute residuals
  // -- Compute dchi vector
  int param_row_idx = 0;
  int dchi_row_idx = 0;
  for (size_t i = 0; i < marg->num_params; i++) {
    const int param_type = marg->param_types[i];
    const int param_size = param_global_size(param_type);
    const int local_size = param_local_size(param_type);
    const real_t *x0 = marg->x0 + param_row_idx;
    const real_t *x = marg->params[i];

    // Calculate i-th dchi
    switch (param_type) {
      case POSE_PARAM:
      case FIDUCIAL_PARAM:
      case EXTRINSIC_PARAM: {
        // Pose minus
        // dr = r - r0
        const real_t dr[3] = {x[0] - x0[0], x[1] - x0[1], x[2] - x0[2]};

        // dq = q0.inverse() * q
        const real_t q[4] = {x[3], x[4], x[5], x[6]};
        const real_t q0[4] = {x0[3], x0[4], x0[5], x0[6]};
        real_t q0_inv[4] = {0};
        real_t dq[4] = {0};
        quat_inv(q0, q0_inv);
        quat_mul(q0_inv, q, dq);

        marg->dchi[dchi_row_idx + 0] = dr[0];
        marg->dchi[dchi_row_idx + 1] = dr[1];
        marg->dchi[dchi_row_idx + 2] = dr[2];
        marg->dchi[dchi_row_idx + 3] = 2.0 * dq[1];
        marg->dchi[dchi_row_idx + 4] = 2.0 * dq[2];
        marg->dchi[dchi_row_idx + 5] = 2.0 * dq[3];
      } break;
      default:
        // Trivial minus: x - x0
        vec_sub(x, x0, marg->dchi + dchi_row_idx, param_size);
        break;
    }
    param_row_idx += param_size;
    dchi_row_idx += local_size;
  }
  // -- Compute residuals: r = r0 + J0 * dchi;
  dot(marg->J0,
      marg->r_size,
      marg->r_size,
      marg->dchi,
      marg->r_size,
      1,
      marg->J0_dchi);
  for (int i = 0; i < marg->r_size; i++) {
    marg->r[i] = marg->r0[i] + marg->J0_dchi[i];
  }

  return 0;
}

////////////////
// DATA UTILS //
////////////////

static int
parse_pose_data(const int i, const int j, const char *entry, pose_t *poses) {
  switch (j) {
    case 0:
      poses[i].ts = strtol(entry, NULL, 10);
      break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
      poses[i].data[j - 1] = strtod(entry, NULL);
      break;
    default:
      return -1;
  }

  return 0;
}

/**
 * Load poses from file `fp`. The number of poses in file
 * will be outputted to `num_poses`.
 */
pose_t *load_poses(const char *fp, int *num_poses) {
  assert(fp != NULL);
  assert(num_poses != NULL);

  // Obtain number of rows and columns in dsv data
  int num_rows = dsv_rows(fp);
  int num_cols = dsv_cols(fp, ',');
  if (num_rows == -1 || num_cols == -1) {
    return NULL;
  }

  // Initialize memory for pose data
  *num_poses = num_rows;
  pose_t *poses = malloc(sizeof(pose_t) * num_rows);

  // Load file
  FILE *infile = fopen(fp, "r");
  if (infile == NULL) {
    free(poses);
    return NULL;
  }

  // Loop through data
  char line[MAX_LINE_LENGTH] = {0};
  int row_idx = 0;
  int col_idx = 0;

  // Loop through data line by line
  while (fgets(line, MAX_LINE_LENGTH, infile) != NULL) {
    // Ignore if comment line
    if (line[0] == '#') {
      continue;
    }

    // Iterate through values in line separated by commas
    char entry[100] = {0};
    for (size_t i = 0; i < strlen(line); i++) {
      char c = line[i];
      if (c == ' ') {
        continue;
      }

      if (c == ',' || c == '\n') {
        if (parse_pose_data(row_idx, col_idx, entry, poses) != 0) {
          return NULL;
        }
        memset(entry, '\0', sizeof(char) * 100);
        col_idx++;

      } else {
        entry[strlen(entry)] = c;
      }
    }

    col_idx = 0;
    row_idx++;
  }

  // Clean up
  fclose(infile);

  return poses;
}

/**
 * Associate pose data
 */
int **assoc_pose_data(pose_t *gnd_poses,
                      size_t num_gnd_poses,
                      pose_t *est_poses,
                      size_t num_est_poses,
                      double threshold,
                      size_t *num_matches) {
  assert(gnd_poses != NULL);
  assert(est_poses != NULL);
  assert(num_gnd_poses != 0);
  assert(num_est_poses != 0);

  size_t gnd_idx = 0;
  size_t est_idx = 0;
  size_t k_end =
      (num_gnd_poses > num_est_poses) ? num_est_poses : num_gnd_poses;

  size_t match_idx = 0;
  int **matches = malloc(sizeof(int *) * k_end);

  while ((gnd_idx + 1) < num_gnd_poses && (est_idx + 1) < num_est_poses) {
    // Calculate time difference between ground truth and
    // estimate
    double gnd_k_time = ts2sec(gnd_poses[gnd_idx].ts);
    double est_k_time = ts2sec(est_poses[est_idx].ts);
    double t_k_diff = fabs(gnd_k_time - est_k_time);

    // Check to see if next ground truth timestamp forms
    // a smaller time diff
    double t_kp1_diff = threshold;
    if ((gnd_idx + 1) < num_gnd_poses) {
      double gnd_kp1_time = ts2sec(gnd_poses[gnd_idx + 1].ts);
      t_kp1_diff = fabs(gnd_kp1_time - est_k_time);
    }

    // Conditions to call this pair (ground truth and
    // estimate) a match
    int threshold_met = t_k_diff < threshold;
    int smallest_diff = t_k_diff < t_kp1_diff;

    // Mark pairs as a match or increment appropriate
    // indices
    if (threshold_met && smallest_diff) {
      matches[match_idx] = malloc(sizeof(int) * 2);
      matches[match_idx][0] = gnd_idx;
      matches[match_idx][1] = est_idx;
      match_idx++;

      gnd_idx++;
      est_idx++;

    } else if (gnd_k_time > est_k_time) {
      est_idx++;

    } else if (gnd_k_time < est_k_time) {
      gnd_idx++;
    }
  }

  // Clean up
  if (match_idx == 0) {
    free(matches);
    matches = NULL;
  }

  *num_matches = match_idx;
  return matches;
}

////////////
// SOLVER //
////////////

/**
 * Setup Solver
 */
void solver_setup(solver_t *solver) {
  assert(solver);

  // Settings
  solver->verbose = 0;
  solver->max_iter = 10;
  solver->lambda = 1e4;
  solver->lambda_factor = 10.0;

  // Data
  solver->hash = NULL;
  solver->linearize = 0;
  solver->r_size = 0;
  solver->sv_size = 0;
  solver->H_damped = NULL;
  solver->H = NULL;
  solver->g = NULL;
  solver->r = NULL;
  solver->dx = NULL;

  // SuiteSparse
#ifdef SOLVER_USE_SUITESPARSE
  solver->common = NULL;
#endif

  // Callbacks
  solver->param_order_func = NULL;
  solver->cost_func = NULL;
  solver->linearize_func = NULL;
  solver->linsolve_func = NULL;
}

/**
 * Calculate cost with residual vector `r` of length `r_size`.
 */
real_t solver_cost(const solver_t *solver, const void *data) {
  solver->cost_func(data, solver->r);
  real_t r_sq = {0};
  dot(solver->r, 1, solver->r_size, solver->r, solver->r_size, 1, &r_sq);
  return 0.5 * r_sq;
}

/**
 * Fill Jacobian matrix
 */
void solver_fill_jacobian(param_order_t *hash,
                          int num_params,
                          real_t **params,
                          real_t **jacs,
                          real_t *r,
                          int r_size,
                          int sv_size,
                          int J_row_idx,
                          real_t *J,
                          real_t *g) {
  for (int i = 0; i < num_params; i++) {
    // Check if i-th parameter is fixed
    if (hmgets(hash, params[i]).fix) {
      continue;
    }

    // Get i-th parameter and corresponding Jacobian
    int idx_i = hmgets(hash, params[i]).idx;
    int size_i = param_local_size(hmgets(hash, params[i]).type);
    const real_t *J_i = jacs[i];

    // Fill in the Jacobian
    const int rs = J_row_idx;
    const int re = rs + r_size - 1;
    const int cs = idx_i;
    const int ce = idx_i + size_i - 1;
    mat_block_set(J, sv_size, rs, re, cs, ce, J_i);

    // Fill in the R.H.S of H dx = g, where g = -J_i' * r
    real_t *Jt_i = malloc(sizeof(real_t) * r_size * size_i);
    real_t *g_i = malloc(sizeof(real_t) * size_i);
    mat_transpose(J_i, r_size, size_i, Jt_i);
    mat_scale(Jt_i, size_i, r_size, -1);
    dot(Jt_i, size_i, r_size, r, r_size, 1, g_i);
    for (int g_idx = 0; g_idx < size_i; g_idx++) {
      g[idx_i + g_idx] += g_i[g_idx];
    }

    // Clean up
    free(g_i);
    free(Jt_i);
  }
}

/**
 * Fill Hessian matrix
 */
void solver_fill_hessian(param_order_t *hash,
                         int num_params,
                         real_t **params,
                         real_t **jacs,
                         real_t *r,
                         int r_size,
                         int sv_size,
                         real_t *H,
                         real_t *g) {
  if (H == NULL || g == NULL) {
    return;
  }

  for (int i = 0; i < num_params; i++) {
    // Check if i-th parameter is fixed
    if (hmgets(hash, params[i]).fix) {
      continue;
    }

    // Get i-th parameter and corresponding Jacobian
    int idx_i = hmgets(hash, params[i]).idx;
    int size_i = param_local_size(hmgets(hash, params[i]).type);
    const real_t *J_i = jacs[i];
    real_t *Jt_i = malloc(sizeof(real_t) * r_size * size_i);
    mat_transpose(J_i, r_size, size_i, Jt_i);

    for (int j = i; j < num_params; j++) {
      // Check if j-th parameter is fixed
      if (hmgets(hash, params[j]).fix) {
        continue;
      }

      // Get j-th parameter and corresponding Jacobian
      int idx_j = hmgets(hash, params[j]).idx;
      int size_j = param_local_size(hmgets(hash, params[j]).type);
      const real_t *J_j = jacs[j];
      real_t *H_ij = malloc(sizeof(real_t) * size_i * size_j);
      dot(Jt_i, size_i, r_size, J_j, r_size, size_j, H_ij);

      // Fill Hessian H
      int rs = idx_i;
      int re = idx_i + size_i - 1;
      int cs = idx_j;
      int ce = idx_j + size_j - 1;

      if (i == j) {
        // Fill diagonal
        mat_block_add(H, sv_size, rs, re, cs, ce, H_ij);
      } else {
        // Fill off-diagonal
        real_t *H_ji = malloc(sizeof(real_t) * size_j * size_i);
        mat_transpose(H_ij, size_i, size_j, H_ji);
        mat_block_add(H, sv_size, rs, re, cs, ce, H_ij);
        mat_block_add(H, sv_size, cs, ce, rs, re, H_ji);
        free(H_ji);
      }

      // Clean up
      free(H_ij);
    }

    // Fill in the R.H.S of H dx = g, where g = -J_i' * r
    real_t *g_i = malloc(sizeof(real_t) * size_i);
    mat_scale(Jt_i, size_i, r_size, -1);
    dot(Jt_i, size_i, r_size, r, r_size, 1, g_i);
    for (int g_idx = 0; g_idx < size_i; g_idx++) {
      g[idx_i + g_idx] += g_i[g_idx];
    }

    // Clean up
    free(g_i);
    free(Jt_i);
  }
}

/**
 * Create a copy of the parameter vector
 */
real_t **solver_params_copy(const solver_t *solver) {
  real_t **x = malloc(sizeof(real_t *) * hmlen(solver->hash));

  for (int idx = 0; idx < hmlen(solver->hash); idx++) {
    const int global_size = param_global_size(solver->hash[idx].type);
    x[idx] = malloc(sizeof(real_t) * global_size);

    for (int i = 0; i < global_size; i++) {
      x[idx][i] = ((real_t *) solver->hash[idx].key)[i];
    }
  }

  return x;
}

/**
 * Restore parameter values
 */
void solver_params_restore(solver_t *solver, real_t **x) {
  for (int idx = 0; idx < hmlen(solver->hash); idx++) {
    for (int i = 0; i < param_global_size(solver->hash[idx].type); i++) {
      ((real_t *) solver->hash[idx].key)[i] = x[idx][i];
    }
  }
}

/**
 * Free params
 */
void solver_params_free(const solver_t *solver, real_t **x) {
  for (int idx = 0; idx < hmlen(solver->hash); idx++) {
    free(x[idx]);
  }
  free(x);
}

/**
 * Update parameter
 */
void solver_update(solver_t *solver, real_t *dx, int sv_size) {
  for (int i = 0; i < hmlen(solver->hash); i++) {
    if (solver->hash[i].fix) {
      continue;
    }

    real_t *data = solver->hash[i].key;
    int idx = solver->hash[i].idx;
    switch (solver->hash[i].type) {
      case POSITION_PARAM:
        for (int i = 0; i < 3; i++) {
          data[i] += dx[idx + i];
        }
        break;
      case POSE_PARAM:
      case FIDUCIAL_PARAM:
      case EXTRINSIC_PARAM:
        pose_update(data, dx + idx);
        break;
      case VELOCITY_PARAM:
        for (int i = 0; i < 3; i++) {
          data[i] += dx[idx + i];
        }
        break;
      case IMU_BIASES_PARAM:
        for (int i = 0; i < 6; i++) {
          data[i] += dx[idx + i];
        }
        break;
      case FEATURE_PARAM:
        for (int i = 0; i < 3; i++) {
          data[i] += dx[idx + i];
        }
        break;
      case IDF_BEARING_PARAM:
        for (int i = 0; i < 3; i++) {
          data[i] += dx[idx + i];
        }
        break;
      case JOINT_PARAM:
        data[0] += dx[idx];
        break;
      case CAMERA_PARAM:
        for (int i = 0; i < 8; i++) {
          data[i] += dx[idx + i];
        }
        break;
      case TIME_DELAY_PARAM:
        data[0] += dx[idx];
        break;
      default:
        FATAL("Invalid param type [%d]!\n", solver->hash[i].type);
        break;
    }
  }
}

/**
 * Step nonlinear least squares problem.
 */
real_t **solver_step(solver_t *solver, const real_t lambda_k, void *data) {
  // Linearize non-linear system
  if (solver->linearize) {
    // Linearize
    zeros(solver->H, solver->sv_size, solver->sv_size);
    zeros(solver->g, solver->sv_size, 1);
    zeros(solver->r, solver->r_size, 1);

    solver->linearize_func(data,
                           solver->sv_size,
                           solver->hash,
                           solver->H,
                           solver->g,
                           solver->r);

    // param_order_print(solver->hash);
    // gnuplot_matshow(solver->H, solver->sv_size, solver->sv_size);
    // mat_save("/tmp/H_solver.csv", solver->H, solver->sv_size, solver->sv_size);
    // exit(0);
  }

  // Damp Hessian: H = H + lambda * I
  mat_copy(solver->H, solver->sv_size, solver->sv_size, solver->H_damped);
  for (int i = 0; i < solver->sv_size; i++) {
    solver->H_damped[(i * solver->sv_size) + i] += lambda_k;
  }

  // Solve non-linear system
  if (solver->linsolve_func) {
    solver->linsolve_func(data,
                          solver->sv_size,
                          solver->hash,
                          solver->H_damped,
                          solver->g,
                          solver->dx);
  } else {
    // Solve: H * dx = g
#ifdef SOLVER_USE_SUITESPARSE
    suitesparse_chol_solve(solver->common,
                           solver->H_damped,
                           solver->sv_size,
                           solver->sv_size,
                           solver->g,
                           solver->sv_size,
                           solver->dx);
#else
    chol_solve(solver->H_damped, solver->g, solver->dx, solver->sv_size);
#endif
  }

  // Update
  real_t **x_copy = solver_params_copy(solver);
  solver_update(solver, solver->dx, solver->sv_size);

  return x_copy;
}

/**
 * Solve nonlinear least squares problem.
 */
int solver_solve(solver_t *solver, void *data) {
  assert(solver != NULL);
  assert(solver->param_order_func != NULL);
  assert(solver->cost_func != NULL);
  assert(solver->linearize_func != NULL);
  assert(data != NULL);

  // Determine parameter order
  int sv_size = 0;
  int r_size = 0;
  solver->hash = solver->param_order_func(data, &sv_size, &r_size);
  assert(sv_size > 0);
  assert(r_size > 0);

  // Calculate initial cost
  solver->linearize = 1;
  solver->r_size = r_size;
  solver->sv_size = sv_size;
  solver->H_damped = calloc(sv_size * sv_size, sizeof(real_t));
  solver->H = calloc(sv_size * sv_size, sizeof(real_t));
  solver->g = calloc(sv_size, sizeof(real_t));
  solver->r = calloc(r_size, sizeof(real_t));
  solver->dx = calloc(sv_size, sizeof(real_t));
  real_t J_km1 = solver_cost(solver, data);
  if (solver->verbose) {
    printf("iter 0: lambda_k: %.2e, J: %.4e\n", solver->lambda, J_km1);
  }

  // Start cholmod workspace
#ifdef SOLVER_USE_SUITESPARSE
  solver->common = malloc(sizeof(cholmod_common) * 1);
  cholmod_start(solver->common);
#endif

  // Solve
  int max_iter = solver->max_iter;
  real_t lambda_k = solver->lambda;
  real_t J_k = 0.0;

  for (int iter = 0; iter < max_iter; iter++) {
    // Linearize and calculate cost
    real_t **x_copy = solver_step(solver, lambda_k, data);
    J_k = solver_cost(solver, data);

    // Accept or reject update*/
    const real_t dJ = J_k - J_km1;
    const real_t dx_norm = vec_norm(solver->dx, solver->sv_size);
    if (J_k < J_km1) {
      // Accept update
      J_km1 = J_k;
      lambda_k /= solver->lambda_factor;
      solver->linearize = 1;
    } else {
      // Reject update
      lambda_k *= solver->lambda_factor;
      solver_params_restore(solver, x_copy);
      solver->linearize = 0;
    }
    lambda_k = clip_value(lambda_k, 1e-8, 1e8);
    solver_params_free(solver, x_copy);

    // Display
    if (solver->verbose) {
      printf("iter %d: lambda_k: %.2e, J: %.4e, dJ: %.2e, norm(dx): %.2e\n",
             iter + 1,
             lambda_k,
             J_km1,
             dJ,
             dx_norm);
    }

    // Termination criteria
    if (solver->linearize && fabs(dJ) < fabs(-1e-10)) {
      // printf("dJ < -1e-10\n");
      break;
    } else if (solver->linearize && dx_norm < 1e-10) {
      // printf("dx_norm < 1e-10\n");
      break;
    }
  }

  // Clean up
#ifdef SOLVER_USE_SUITESPARSE
  cholmod_finish(solver->common);
  free(solver->common);
  solver->common = NULL;
#endif
  hmfree(solver->hash);
  free(solver->H_damped);
  free(solver->H);
  free(solver->g);
  free(solver->r);
  free(solver->dx);

  return 0;
}

///////////////////////
// INERTIAL ODOMETRY //
///////////////////////

/**
 * Malloc inertial odometry.
 */
inertial_odometry_t *inertial_odometry_malloc(void) {
  inertial_odometry_t *io = malloc(sizeof(inertial_odometry_t) * 1);

  io->num_factors = 0;
  io->factors = NULL;
  io->marg = NULL;

  io->poses = NULL;
  io->vels = NULL;
  io->biases = NULL;

  return io;
}

/**
 * Free inertial odometry.
 */
void inertial_odometry_free(inertial_odometry_t *odom) {
  free(odom->factors);
  free(odom->poses);
  free(odom->vels);
  free(odom->biases);
  free(odom);
}

/**
 * Save inertial odometry.
 */
void inertial_odometry_save(const inertial_odometry_t *odom,
                            const char *save_path) {
  // Load file
  FILE *fp = fopen(save_path, "w");
  if (fp == NULL) {
    FATAL("Failed to open [%s]!\n", save_path);
  }

  // Write header
  fprintf(fp, "#ts,");
  fprintf(fp, "rx,ry,rz,qw,qx,qy,qz,");
  fprintf(fp, "vx,vy,vz,");
  fprintf(fp, "ba_x,ba_y,ba_z,");
  fprintf(fp, "bg_x,bg_y,bg_z\n");

  // Write data
  for (int k = 0; k < (odom->num_factors + 1); k++) {
    const real_t *pos = odom->poses[k].data;
    const real_t *quat = odom->poses[k].data + 3;
    const real_t *vel = odom->vels[k].data;
    const real_t *ba = odom->biases[k].data;
    const real_t *bg = odom->biases[k].data + 3;
    fprintf(fp, "%ld,", odom->poses[k].ts);
    fprintf(fp, "%f,%f,%f,", pos[0], pos[1], pos[2]);
    fprintf(fp, "%f,%f,%f,%f,", quat[0], quat[1], quat[2], quat[3]);
    fprintf(fp, "%f,%f,%f,", vel[0], vel[1], vel[2]);
    fprintf(fp, "%f,%f,%f,", ba[0], ba[1], ba[2]);
    fprintf(fp, "%f,%f,%f", bg[0], bg[1], bg[2]);
    fprintf(fp, "\n");
  }
}

/**
 * Determine inertial odometry parameter order.
 */
param_order_t *inertial_odometry_param_order(const void *data,
                                             int *sv_size,
                                             int *r_size) {
  // Setup parameter order
  inertial_odometry_t *odom = (inertial_odometry_t *) data;
  param_order_t *hash = NULL;
  int col_idx = 0;

  for (int k = 0; k <= odom->num_factors; k++) {
    param_order_add_pose(&hash, &odom->poses[k], &col_idx);
    param_order_add_velocity(&hash, &odom->vels[k], &col_idx);
    param_order_add_imu_biases(&hash, &odom->biases[k], &col_idx);
  }

  *sv_size = col_idx;
  *r_size = odom->num_factors * 15;
  return hash;
}

/**
 * Calculate inertial odometry cost.
 */
void inertial_odometry_cost(const void *data, real_t *r) {
  // Evaluate factors
  inertial_odometry_t *odom = (inertial_odometry_t *) data;
  for (int k = 0; k < odom->num_factors; k++) {
    imu_factor_t *factor = &odom->factors[k];
    imu_factor_eval(factor);
    vec_copy(factor->r, factor->r_size, &r[k * factor->r_size]);
  }
}

/**
 * Linearize inertial odometry problem.
 */
void inertial_odometry_linearize_compact(const void *data,
                                         const int sv_size,
                                         param_order_t *hash,
                                         real_t *H,
                                         real_t *g,
                                         real_t *r) {
  // Evaluate factors
  inertial_odometry_t *odom = (inertial_odometry_t *) data;

  for (int k = 0; k < odom->num_factors; k++) {
    imu_factor_t *factor = &odom->factors[k];
    imu_factor_eval(factor);
    vec_copy(factor->r, factor->r_size, &r[k * factor->r_size]);

    solver_fill_hessian(hash,
                        factor->num_params,
                        factor->params,
                        factor->jacs,
                        factor->r,
                        factor->r_size,
                        sv_size,
                        H,
                        g);
  }
}

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
                      real_t T_WB_k[4 * 4]) {
  return 0;
}

////////////////////////////
// TWO-STATE FILTER (TSF) //
////////////////////////////

/**
 * TSF frameset setup
 */
void tsf_frameset_setup(tsf_frameset_t *fs) {
  fs->ts = 0;

  memset(fs->cam0_fids, 0, sizeof(size_t) * TSF_FRAME_LIMIT);
  memset(fs->cam0_kps, 0, sizeof(real_t) * TSF_FRAME_LIMIT * 2);
  fs->cam0_num_kps = 0;

  memset(fs->cam1_fids, 0, sizeof(size_t) * TSF_FRAME_LIMIT);
  memset(fs->cam1_kps, 0, sizeof(real_t) * TSF_FRAME_LIMIT * 2);
  fs->cam1_num_kps = 0;
}

/**
 * TSF reset
 */
void tsf_frameset_reset(tsf_frameset_t *fs) { tsf_frameset_setup(fs); }

/**
 * TSF Malloc.
 */
tsf_t *tsf_malloc(void) {
  tsf_t *tsf = malloc(sizeof(tsf_t) * 1);

  // Flags
  tsf->state = 0;
  tsf->num_imus = 0;
  tsf->num_cams = 0;
  tsf->imu_started = 0;
  tsf->frame_idx = -1;

  // Settings
  tsf->fix_cam_params = 0;
  tsf->fix_cam_exts = 0;
  tsf->fix_imu_ext = 0;
  tsf->fix_time_delay = 0;

  // IMU
  // tsf->imu_params = NULL;
  imu_buffer_setup(&tsf->imu_buf);
  // tsf->imu_ext = NULL;
  // tsf->time_delay = NULL;

  // Vision
  tsf->cam_params = NULL;
  tsf->cam_exts = NULL;
  tsf->feature_map = NULL;

  // Factors
  // tsf->imu_factor = NULL;
  tsf->marg = NULL;

  // State
  pose_init(tsf->pose_init);
  memset(tsf->vel_init, 0, sizeof(real_t) * 3);
  memset(tsf->ba_init, 0, sizeof(real_t) * 3);
  memset(tsf->bg_init, 0, sizeof(real_t) * 3);
  tsf->ts_i = 0;
  tsf->ts_j = 0;
  // tsf->pose_i = NULL;
  // tsf->pose_j = NULL;
  // tsf->vel_i = NULL;
  // tsf->vel_j = NULL;
  // tsf->biases_i = NULL;
  // tsf->biases_j = NULL;

  return tsf;
}

/**
 * Free TSF.
 */
void tsf_free(tsf_t *tsf) {
  // IMU
  // free(tsf->imu_params);
  // free(tsf->imu_ext);
  // free(tsf->time_delay);

  // VISION
  free(tsf->cam_params);
  free(tsf->cam_exts);
  hmfree(tsf->feature_map);

  // FACTORS
  // free(tsf->imu_factor);
  marg_factor_free(tsf->marg);

  // STATE
  // free(tsf->pose_i);
  // free(tsf->pose_j);
  // free(tsf->vel_i);
  // free(tsf->vel_j);
  // free(tsf->biases_i);
  // free(tsf->biases_j);

  free(tsf);
}

/**
 * Print TSF.
 */
void tsf_print(const tsf_t *tsf) {
  printf("state: %d\n", tsf->state);
  printf("num_imus: %d\n", tsf->num_imus);
  printf("num_cams: %d\n", tsf->num_cams);
  printf("imu_started: %d\n", tsf->imu_started);
  printf("\n");

  printf("fix_cam_params: %d\n", tsf->fix_cam_params);
  printf("fix_cam_exts: %d\n", tsf->fix_cam_exts);
  printf("fix_imu_ext: %d\n", tsf->fix_imu_ext);
  printf("fix_time_delay: %d\n", tsf->fix_time_delay);
  printf("\n");
}

/**
 * Set TSF initial pose.
 */
void tsf_set_init_pose(tsf_t *tsf, real_t pose[7]) {
  for (int i = 0; i < 7; i++) {
    tsf->pose_init[i] = pose[i];
  }
}

/**
 * Set TSF initial velocity.
 */
void tsf_set_init_velocity(tsf_t *tsf, real_t vel[3]) {
  tsf->vel_init[0] = vel[0];
  tsf->vel_init[1] = vel[1];
  tsf->vel_init[2] = vel[2];
}

/**
 * Add camera to TSF.
 */
void tsf_add_camera(tsf_t *tsf,
                    const int cam_idx,
                    const int cam_res[2],
                    const char *proj_model,
                    const char *dist_model,
                    const real_t *intrinsic,
                    const real_t *extrinsic) {
  assert(tsf != NULL);
  assert(cam_idx <= tsf->num_cams);
  assert(cam_res != NULL);
  assert(proj_model != NULL);
  assert(dist_model != NULL);
  assert(intrinsic != NULL);
  assert(extrinsic != NULL);

  if (cam_idx > (tsf->num_cams - 1)) {
    const int new_size = tsf->num_cams + 1;
    tsf->cam_params = realloc(tsf->cam_params, sizeof(camera_params_t) * new_size);
    tsf->cam_exts = realloc(tsf->cam_exts, sizeof(extrinsic_t) * new_size);
  }

  camera_params_setup(&tsf->cam_params[cam_idx],
                      cam_idx,
                      cam_res,
                      proj_model,
                      dist_model,
                      intrinsic);
  extrinsic_setup(&tsf->cam_exts[cam_idx], extrinsic);
  tsf->num_cams++;
}

/**
 * Add IMU to TSF.
 */
void tsf_add_imu(tsf_t *tsf,
                 const real_t imu_rate,
                 const real_t sigma_aw,
                 const real_t sigma_gw,
                 const real_t sigma_a,
                 const real_t sigma_g,
                 const real_t g,
                 const real_t *imu_ext) {
  assert(tsf != NULL);
  assert(imu_rate > 0);
  assert(sigma_aw > 0);
  assert(sigma_gw > 0);
  assert(sigma_a > 0);
  assert(sigma_g > 0);
  assert(g > 9.0);
  assert(imu_ext);

  if (tsf->num_imus == 1) {
    LOG_ERROR("Currently only supports 1 IMU!\n");
    return;
  }

  tsf->imu_params.imu_idx = 0;
  tsf->imu_params.rate = imu_rate;
  tsf->imu_params.sigma_aw = sigma_aw;
  tsf->imu_params.sigma_gw = sigma_gw;
  tsf->imu_params.sigma_a = sigma_a;
  tsf->imu_params.sigma_g = sigma_g;
  tsf->imu_params.g = g;

  extrinsic_setup(&tsf->imu_ext, imu_ext);
  tsf->imu_ext.fix = tsf->fix_imu_ext;

  time_delay_setup(&tsf->time_delay, 0.0);
  tsf->time_delay.fix = tsf->fix_time_delay;

  tsf->num_imus = 1;
}

/**
 * TSF handle IMU event.
 */
void tsf_imu_event(tsf_t *tsf,
                   const timestamp_t ts,
                   const real_t acc[3],
                   const real_t gyr[3]) {
  assert(tsf != NULL);
  assert(ts >= 0);
  assert(acc != NULL);
  assert(gyr != NULL);

  // Add IMU measurement to buffer
  imu_buffer_add(&tsf->imu_buf, ts, acc, gyr);
  tsf->imu_started = 1;
}

static void tsf_extract_stereo_keypoints(const size_t *fids0,
                                         const real_t *kps0,
                                         const int num_kps0,
                                         const size_t *fids1,
                                         const real_t *kps1,
                                         const int num_kps1,
                                         const int limit,
                                         size_t *match_fids,
                                         int *num_match_fids,
                                         real_t *kps0_out,
                                         real_t *kps1_out) {
  // Initialize output to zero
  // memset(match_fids, sizeof(int), limit);
  // *num_match_fids = 0;
  // memset(kps0_out, sizeof(real_t), limit * 2);
  // memset(kps1_out, sizeof(real_t), limit * 2);

  // Extract features with same feature id
  int cam0_idx = 0;
  int cam1_idx = 0;
  for (int i = 0; i < MAX(num_kps0, num_kps1); i++) {
    // Check bounds
    if (i >= num_kps0) {
      break;
    } else if (i >= num_kps1) {
      break;
    }
    const size_t fid0 = fids0[cam0_idx];
    const size_t fid1 = fids1[cam1_idx];
    if (fid0 == fid1) {
      match_fids[*num_match_fids] = fid0;
      (*num_match_fids)++;
      cam0_idx++;
      cam1_idx++;
    } else if (fid0 < fid1) {
      cam0_idx++;
    } else if (fid0 > fid1) {
      cam1_idx++;
    }

    if (*num_match_fids == limit) {
      break;
    }
  }

  // Extact keypoints
  // -- Extrack keypoints from cam0
  int idx0 = 0;
  for (int i = 0; i < *num_match_fids; i++) {
    size_t fid = match_fids[i];
    for (int j = idx0; j < num_kps0; j++) {
      if (fids0[j] == fid) {
        kps0_out[i * 2 + 0] = kps0[j * 2 + 0];
        kps0_out[i * 2 + 1] = kps0[j * 2 + 1];
        idx0 = j + 1;
        break;
      }
    }
  }
  // -- Extrack keypoints from cam1
  int idx1 = 0;
  for (int i = 0; i < *num_match_fids; i++) {
    size_t fid = match_fids[i];
    for (int j = idx1; j < num_kps1; j++) {
      if (fids1[j] == fid) {
        kps1_out[i * 2 + 0] = kps1[j * 2 + 0];
        kps1_out[i * 2 + 1] = kps1[j * 2 + 1];
        idx1 = j + 1;
        break;
      }
    }
  }
}

/**
 * TSF handle camera event.
 */
void tsf_camera_event(tsf_t *tsf,
                      const timestamp_t ts,
                      const size_t *cam0_fids,
                      const real_t *cam0_kps,
                      const int num_cam0_kps,
                      const size_t *cam1_fids,
                      const real_t *cam1_kps,
                      const int num_cam1_kps) {
  assert(tsf != NULL);
  assert(ts >= 0);
  assert(cam0_fids != NULL);
  assert(cam0_kps != NULL);
  assert(cam1_fids != NULL);
  assert(cam1_kps != NULL);
  tsf->frame_idx++;

  // Initialize features
  if (tsf->frame_idx == 0) {
    // Extract common feature ids and keypoints
    const int limit = 1000;
    size_t match_fids[1000] = {0};
    int num_match_fids = 0;
    real_t kps0[1000 * 2] = {0};
    real_t kps1[1000 * 2] = {0};
    tsf_extract_stereo_keypoints(cam0_fids,
                                 cam0_kps,
                                 num_cam0_kps,
                                 cam1_fids,
                                 cam1_kps,
                                 num_cam1_kps,
                                 limit,
                                 match_fids,
                                 &num_match_fids,
                                 kps0,
                                 kps1);

    // Triangulate features
    // -- Setup projection matrices
    real_t P_i[3 * 4] = {0};
    real_t P_j[3 * 4] = {0};
    POSE2TF(tsf->cam_exts[0].data, T_SC0);
    POSE2TF(tsf->cam_exts[1].data, T_SC1);
    TF_INV(T_SC0, T_C0S);
    TF_CHAIN(T_C0C1, 2, T_C0S, T_SC1);
    // pinhole_projection_matrix(tsf->cam0_params.data, T_WC0, P_i);
    // pinhole_projection_matrix(tsf->cam1_params.data, T_WC1, P_j);

    // -- Triangulate features
    const real_t *cam0_params = tsf->cam_params[0].data;
    const real_t *cam1_params = tsf->cam_params[1].data;
    for (int i = 0; i < num_match_fids; i++) {
      // Undistort keypoints
      real_t z_i[2] = {0};
      real_t z_j[2] = {0};
      tsf->cam_params[0].undistort_func(cam0_params, &kps0[i * 2], z_i);
      tsf->cam_params[1].undistort_func(cam1_params, &kps1[i * 2], z_j);

      // Triangulate
      real_t p[3] = {0};
      linear_triangulation(P_i, P_j, z_i, z_j, p);

      // Add new feature
      const int fid = match_fids[i];
      feature_map_t feature;
      feature_setup(&feature.feature, fid);
      hmputs(tsf->feature_map, feature);
    }

    // Form frameset km1
    tsf_frameset_setup(&tsf->fs_km1);
    tsf->fs_km1.ts = ts;
    for (int i = 0; i < num_match_fids; i++) {
      tsf->fs_km1.cam0_fids[i] = match_fids[i];
      tsf->fs_km1.cam0_kps[i * 2 + 0] = kps0[i * 2 + 0];
      tsf->fs_km1.cam0_kps[i * 2 + 1] = kps0[i * 2 + 1];

      tsf->fs_km1.cam1_fids[i] = match_fids[i];
      tsf->fs_km1.cam1_kps[i * 2 + 0] = kps1[i * 2 + 0];
      tsf->fs_km1.cam1_kps[i * 2 + 1] = kps1[i * 2 + 1];
    }
    tsf->fs_km1.cam0_num_kps = num_match_fids;
    tsf->fs_km1.cam1_num_kps = num_match_fids;

    return;
  }
}

// static size_t *tsf_unique_feature_ids(tsf_t *tsf, size_t *n) {
//   size_t *fids_unique = malloc(sizeof(size_t) * tsf->num_factors_i + tsf->num_factors_j);
//   size_t fid_idx = 0;

//   // Load unique feature ids with feature ids from the previous step
//   for (int i = 0; i < tsf->num_factors_i; i++) {
//     fids_unique[fid_idx++] = tsf->idf_factors_i[i].feature_id;
//   }

//   // Loop feature ids in the current step and add only untracked feature ids
//   for (int j = 0; j < tsf->num_factors_j; j++) {
//     size_t fid = tsf->idf_factors_j[j].feature_id;

//     int found = 0;
//     for (int i = 0; i < fid_idx; i++) {
//       if (fids_unique[i] == fid) {
//         found = 1;
//         break;
//       } else if (fids_unique[i] > fid) {
//         found = 0;
//         break;
//       }
//     }

//     if (found == 0) {
//       fids_unique[fid_idx++] = fid;
//     }
//   }

//   *n = fid_idx;
//   return fids_unique;
// }

/**
 * Form parameter order.
 */
param_order_t *tsf_param_order(const void *data, int *sv_size, int *r_size) {
  // Setup parameter order
  tsf_t *tsf = (tsf_t *) data;
  param_order_t *hash = NULL;
  int col_idx = 0;

  // Add state at timestep k - 1
  param_order_add_pose(&hash, &tsf->pose_i, &col_idx);
  if (tsf->num_imus) {
    param_order_add_velocity(&hash, &tsf->vel_i, &col_idx);
    param_order_add_imu_biases(&hash, &tsf->biases_i, &col_idx);
  }

  // Add state at timestep k
  param_order_add_pose(&hash, &tsf->pose_j, &col_idx);
  if (tsf->num_imus) {
    param_order_add_velocity(&hash, &tsf->vel_j, &col_idx);
    param_order_add_imu_biases(&hash, &tsf->biases_j, &col_idx);
  }

  // // Add camera extrinsic
  // for (int cam_idx = 0; cam_idx < tsf->num_cams; cam_idx++) {
  //   param_order_add_extrinsic(&hash, &tsf->cam_exts[cam_idx], &col_idx);
  // }

  // // Add camera parameters
  // for (int cam_idx = 0; cam_idx < tsf->num_cams; cam_idx++) {
  //   param_order_add_camera(&hash, &tsf->cam_params[cam_idx], &col_idx);
  // }

  // // Add features
  // size_t n = 0;
  // size_t *fids = tsf_unique_feature_ids(tsf, &n);
  // for (size_t i = 0; i < n; i++) {
  //   feature_t *idf_param = NULL;
  //   pos_t *idf_pos = NULL;
  //   features_get_idf(tsf->features, fids[i], &idf_param, &idf_pos);

  //   param_order_add(&hash, IDF_BEARING_PARAM, 0, idf_param->data, &col_idx);
  //   if (param_order_exists(&hash, idf_pos->data) == 0) {
  //     param_order_add(&hash, POSITION_PARAM, 0, idf_pos->data, &col_idx);
  //   }
  // }
  // free(fids);

  // Set state-vector and residual size
  *sv_size = col_idx;
  *r_size = 0;
  // *r_size += tsf->num_factors_i * 2;
  // *r_size += tsf->num_factors_j * 2;
  *r_size += tsf->num_imus * 15;
  if (tsf->marg) {
    *r_size += tsf->marg->r_size;
  }

  return hash;
}

/**
 * Calculate problem cost.
 */
void tsf_cost(const void *data, real_t *r) {
  // Evaluate factors
  tsf_t *tsf = (tsf_t *) data;
  int r_idx = 0;

  // -- Evaluate IDF factors
  // for (int i = 0; i < tsf->num_factors_i; i++) {
  //   idf_factor_t *factor = &tsf->idf_factors_i[i];
  //   idf_factor_eval(factor);
  //   vec_copy(factor->r, factor->r_size, &r[r_idx]);
  //   r_idx += factor->r_size;
  // }
  // for (int j = 0; j < tsf->num_factors_j; j++) {
  //   idf_factor_t *factor = &tsf->idf_factors_j[j];
  //   idf_factor_eval(factor);
  //   vec_copy(factor->r, factor->r_size, &r[r_idx]);
  //   r_idx += factor->r_size;
  // }

  // -- Evaluate Imu factor
  {
    imu_factor_t *factor = &tsf->imu_factor;
    imu_factor_eval(factor);
    vec_copy(factor->r, factor->r_size, &r[r_idx]);
    r_idx += factor->r_size;
  }

  // -- Evaluate marginalization factor
  if (tsf->marg) {
    marg_factor_eval(tsf->marg);
    vec_copy(tsf->marg->r, tsf->marg->r_size, &r[r_idx]);
  }
}

// /**
//  * TSF reprojection errors.
//  */
// void tsf_reproj_errors(const tsf_t *tsf,
//                        real_t *reproj_rmse,
//                        real_t *reproj_mean,
//                        real_t *reproj_median) {
//   // Setup
//   const int N = (tsf->num_factors_i + tsf->num_factors_j);
//   const int r_size = N * 2;
//   int r_idx = 0;
//   real_t *r = calloc(r_size, sizeof(real_t));
//   for (int i = 0; i < tsf->num_factors_i; i++) {
//     idf_factor_t *factor = &tsf->idf_factors_i[i];
//     idf_factor_eval(factor);
//     vec_copy(factor->r, factor->r_size, &r[r_idx]);
//     r_idx += factor->r_size;
//   }
//   for (int j = 0; j < tsf->num_factors_j; j++) {
//     idf_factor_t *factor = &tsf->idf_factors_j[j];
//     idf_factor_eval(factor);
//     vec_copy(factor->r, factor->r_size, &r[r_idx]);
//     r_idx += factor->r_size;
//   }

//   // Calculate reprojection errors
//   real_t *errors = calloc(N, sizeof(real_t));
//   for (int i = 0; i < N; i++) {
//     const real_t x = r[i * 2 + 0];
//     const real_t y = r[i * 2 + 1];
//     errors[i] = sqrt(x * x + y * y);
//   }

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

//   // Clean up
//   free(errors);
//   free(r);
// }

/**
 * Linearize SF Non-linear Least Square Problem.
 */
void tsf_linearize_compact(const void *data,
                           const int sv_size,
                           param_order_t *hash,
                           real_t *H,
                           real_t *g,
                           real_t *r) {
  // Evaluate factors
  tsf_t *tsf = (tsf_t *) data;
  size_t r_idx = 0;

  // -- IMU factor
  if (tsf->num_imus) {
    imu_factor_t *factor = &tsf->imu_factor;
    SOLVER_EVAL_FACTOR_COMPACT(hash,
                               sv_size,
                               H,
                               g,
                               imu_factor_eval,
                               factor,
                               r,
                               r_idx);
  }

  // // -- IDF factors
  // for (int i = 0; i < tsf->num_factors_i; i++) {
  //   idf_factor_t *factor = &tsf->idf_factors_i[i];
  //   idf_factor_eval(factor);
  //   vec_copy(factor->r, factor->r_size, &r[r_idx]);

  //   solver_fill_hessian(hash,
  //                       factor->num_params,
  //                       factor->params,
  //                       factor->jacs,
  //                       factor->r,
  //                       factor->r_size,
  //                       sv_size,
  //                       H,
  //                       g);
  //   r_idx += factor->r_size;
  // }
  // for (int j = 0; j < tsf->num_factors_j; j++) {
  //   idf_factor_t *factor = &tsf->idf_factors_j[j];
  //   idf_factor_eval(factor);
  //   vec_copy(factor->r, factor->r_size, &r[r_idx]);

  //   solver_fill_hessian(hash,
  //                       factor->num_params,
  //                       factor->params,
  //                       factor->jacs,
  //                       factor->r,
  //                       factor->r_size,
  //                       sv_size,
  //                       H,
  //                       g);
  //   r_idx += factor->r_size;
  // }

  // -- Marginalization factor
  if (tsf->marg) {
    marg_factor_eval(tsf->marg);
    vec_copy(tsf->marg->r, tsf->marg->r_size, &r[r_idx]);

    solver_fill_hessian(hash,
                        tsf->marg->num_params,
                        tsf->marg->params,
                        tsf->marg->jacs,
                        tsf->marg->r,
                        tsf->marg->r_size,
                        sv_size,
                        H,
                        g);
  }
}

// static int tsf_process_data(tsf_t *tsf) {
//   // Map out frame data
//   const int cam_idx = 0;
//   const int fs_idx = (tsf->frame_idx == 0) ? 0 : 1;
//   const camera_params_t *cam = &tsf->cam_params[cam_idx];
//   const tsf_frameset_t *fs = tsf->frame_sets[fs_idx];
//   const tsf_frame_t *f = fs->cam_frames[cam_idx];
//   const timestamp_t ts = fs->ts;
//   const int n = f->num_measurements;
//   const size_t *fids = f->feature_ids;
//   const real_t *kps = f->keypoints;

//   // Initialize pose at k
//   pose_t *pose_k = NULL;
//   if (tsf->frame_idx == 0) {
//     pose_k = malloc(sizeof(pose_t) * 1);
//     const real_t pose_data[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
//     pose_setup(pose_k, ts, pose_data);
//     tsf->pose_i = pose_k;
//   } else {
//     pose_k = malloc(sizeof(pose_t) * 1);
//     pose_setup(pose_k, ts, tsf->pose_i->data);
//     tsf->pose_j = pose_k;
//   }

//   // Form camera pose T_WCi at k
//   POSE2TF(pose_k->data, T_WB_k);
//   POSE2TF(tsf->cam_exts[cam_idx].data, T_BCi);
//   TF_CHAIN(T_WCi_k, 2, T_WB_k, T_BCi);

//   // Add new features
//   if (fs_idx == 0) {
//     features_add_idfs(tsf->features, fids, cam, T_WCi_k, kps, n);

//   } else {
//     size_t *fids_new = malloc(sizeof(size_t) * n);
//     real_t *kps_new = malloc(sizeof(real_t) * n * 2);
//     int n_new = 0;
//     for (int i = 0; i < n; i++) {
//       if (features_exists(tsf->features, fids[i]) == 0) {
//         fids_new[n_new] = fids[i];
//         kps_new[n_new * 2 + 0] = kps[i * 2 + 0];
//         kps_new[n_new * 2 + 1] = kps[i * 2 + 1];
//         n_new++;
//       }
//     }
//     features_add_idfs(tsf->features, fids_new, cam, T_WCi_k, kps_new, n_new);
//     free(fids_new);
//     free(kps_new);
//   }

//   // Create IDF factors
//   const real_t var[2] = {1.0, 1.0};
//   idf_factor_t *factors = malloc(sizeof(idf_factor_t) * n);

//   for (int i = 0; i < n; i++) {
//     // Get IDF
//     const size_t fid = fids[i];
//     const real_t *z = &kps[i * 2];
//     pos_t *idf_pos = NULL;
//     feature_t *idf_param = NULL;
//     features_get_idf(tsf->features, fid, &idf_param, &idf_pos);

//     // Form IDF factor
//     idf_factor_setup(&factors[i],
//                      pose_k,
//                      &tsf->cam_exts[cam_idx],
//                      &tsf->cam_params[cam_idx],
//                      idf_pos,
//                      idf_param,
//                      pose_k->ts,
//                      cam_idx,
//                      fid,
//                      z,
//                      var);
//   }

//   if (tsf->frame_idx == 0) {
//     tsf->idf_factors_i = factors;
//     tsf->num_factors_i = n;
//   } else {
//     tsf->idf_factors_j = factors;
//     tsf->num_factors_j = n;
//   }

//   return n;
// }

static void tsf_solve(tsf_t *tsf) {
  assert(tsf != NULL);

  // Pre-check
  if (tsf->frame_idx == 0) {
    return;
  }

  // Solve
  solver_t solver;
  solver_setup(&solver);
  solver.verbose = 1;
  solver.max_iter = 5;
  solver.cost_func = &tsf_cost;
  solver.param_order_func = &tsf_param_order;
  solver.linearize_func = &tsf_linearize_compact;
  solver_solve(&solver, tsf);

  // Print reprojection errors
  // real_t reproj_rmse = 0.0;
  // real_t reproj_mean = 0.0;
  // real_t reproj_median = 0.0;
  // tsf_reproj_errors(tsf, &reproj_rmse, &reproj_mean, &reproj_median);
  // printf("reproj_error:\n");
  // printf("  rmse: %.2f\n", reproj_rmse);
  // printf("  mean: %.2f\n", reproj_mean);
  // printf("  median: %.2f\n", reproj_median);
  // printf("\n");
}

static void tsf_marginalize(tsf_t *tsf) {
  assert(tsf != NULL);

  // Pre-check
  if (tsf->frame_idx == 0) {
    return;
  }

  // Setup
  marg_factor_t *marg = marg_factor_malloc();

  // Mark variables to be marginalized
  tsf->pose_i.marginalize = 1;
  tsf->vel_i.marginalize = 1;
  tsf->biases_i.marginalize = 1;

  // Add factors to be marginalized
  // for (int i = 0; i < tsf->num_factors_i; i++) {
  //   marg_factor_add(marg, IDF_FACTOR, &tsf->idf_factors_i[i]);
  // }
  marg_factor_add(marg, IMU_FACTOR, &tsf->imu_factor);
  if (tsf->marg) {
    marg_factor_add(marg, MARG_FACTOR, tsf->marg);
  }

  // Marginalize
  marg_factor_marginalize(marg);

  // Free previous and set new marginalization factor
  marg_factor_free(tsf->marg);
  tsf->marg = marg;
}

/**
 * Update TSF.
 */
void tsf_update(tsf_t *tsf, const timestamp_t ts) {
  const timestamp_t ts_i = imu_buffer_first_ts(&tsf->imu_buf);
  const timestamp_t ts_j = imu_buffer_last_ts(&tsf->imu_buf);
  const real_t dt = ts2sec(ts_j - ts_i);
  if (dt > 0.1) {
    pose_setup(&tsf->pose_i, ts_i, tsf->pose_init);
    pose_setup(&tsf->pose_j, ts_j, tsf->pose_init);
    velocity_setup(&tsf->vel_i, ts_i, tsf->vel_init);
    velocity_setup(&tsf->vel_j, ts_j, tsf->vel_init);
    imu_biases_setup(&tsf->biases_i, ts_i, tsf->ba_init, tsf->bg_init);
    imu_biases_setup(&tsf->biases_j, ts_j, tsf->ba_init, tsf->bg_init);

    imu_factor_setup(&tsf->imu_factor,
                     &tsf->imu_params,
                     &tsf->imu_buf,
                     &tsf->pose_i,
                     &tsf->vel_i,
                     &tsf->biases_i,
                     &tsf->pose_j,
                     &tsf->vel_j,
                     &tsf->biases_j);
    imu_buffer_clear(&tsf->imu_buf);
    imu_factor_eval(&tsf->imu_factor);

    tsf_solve(tsf);
    tsf_marginalize(tsf);
    exit(0);
  }

  // tsf_process_data(tsf);
  // tsf_solve(tsf);
  // exit(0);
  // tsf_marginalize(tsf);

  // Update book-keeping
  // - Move current frame to previous
  // - Move current factors to previous
  // - Move current pose to previous
  // if (tsf->frame_idx > 0) {
  // Frameset
  // tsf_frameset_free(tsf->frame_sets[0]);
  // tsf->frame_sets[0] = tsf->frame_sets[1];
  // tsf->frame_sets[1] = NULL;

  // Factors
  // free(tsf->idf_factors_i);
  // tsf->idf_factors_i = tsf->idf_factors_j;
  // tsf->num_factors_i = tsf->num_factors_j;
  // tsf->idf_factors_j = NULL;
  // tsf->num_factors_j = 0;

  // Poses
  // free(tsf->pose_i);
  // tsf->pose_i = tsf->pose_j;
  // tsf->pose_j = NULL;
  // }

  // Increment frame index
  // tsf->frame_idx++;
}
