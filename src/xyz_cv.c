#include "xyz_cv.h"

/******************************************************************************
 * CV
 *****************************************************************************/

///////////
// IMAGE //
///////////

/**
 * Setup image `img` with `width`, `height` and `data`.
 */
void image_setup(image_t *img,
                 const int width,
                 const int height,
                 uint8_t *data) {
  assert(img != NULL);
  img->width = width;
  img->height = height;
  img->data = data;
}

/**
 * Load image at `file_path`.
 * @returns Heap allocated image
 */
image_t *image_load(const char *file_path) {
  assert(file_path != NULL);

#ifdef USE_STB
  int img_w = 0;
  int img_h = 0;
  int img_c = 0;
  stbi_set_flip_vertically_on_load(1);
  uint8_t *data = stbi_load(file_path, &img_w, &img_h, &img_c, 0);
  if (!data) {
    FATAL("Failed to load image file: [%s]", file_path);
  }

  image_t *img = malloc(sizeof(image_t) * 1);
  img->width = img_w;
  img->height = img_h;
  img->channels = img_c;
  img->data = data;
  return img;
#else
  FATAL("Not Implemented!");
#endif
}

/**
 * Print image properties.
 */
void image_print_properties(const image_t *img) {
  assert(img != NULL);
  printf("img.width: %d\n", img->width);
  printf("img.height: %d\n", img->height);
  printf("img.channels: %d\n", img->channels);
}

/**
 * Free image.
 */
void image_free(image_t *img) {
  assert(img != NULL);
  free(img->data);
  free(img);
}

////////////
// RADTAN //
////////////

/**
 * Distort 2x1 point `p` using Radial-Tangential distortion, where the
 * distortion params are stored in `params` (k1, k2, p1, p2), results are
 * written to 2x1 vector `p_d`.
 */
void radtan4_distort(const real_t params[4],
                     const real_t p_in[2],
                     real_t p_out[2]) {
  assert(params != NULL);
  assert(p_in != NULL);
  assert(p_out != NULL);

  // Distortion parameters
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t p1 = params[2];
  const real_t p2 = params[3];

  // Point
  const real_t x = p_in[0];
  const real_t y = p_in[1];

  // Apply radial distortion
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;
  const real_t radial_factor = 1.0 + (k1 * r2) + (k2 * r4);
  const real_t x_dash = x * radial_factor;
  const real_t y_dash = y * radial_factor;

  // Apply tangential distortion
  const real_t xy = x * y;
  const real_t x_ddash = x_dash + (2.0 * p1 * xy + p2 * (r2 + 2.0 * x2));
  const real_t y_ddash = y_dash + (2.0 * p2 * xy + p1 * (r2 + 2.0 * y2));

  // Distorted point
  p_out[0] = x_ddash;
  p_out[1] = y_ddash;
}

/**
 * Radial-Tangential undistort
 */
void radtan4_undistort(const real_t params[4],
                       const real_t p_in[2],
                       real_t p_out[2]) {
  const int max_iter = 5;
  real_t p[2] = {p_in[0], p_in[1]};

  for (int i = 0; i < max_iter; i++) {
    // Error
    real_t p_d[2] = {0};
    radtan4_distort(params, p, p_d);
    const real_t e[2] = {p_in[0] - p_d[0], p_in[1] - p_d[1]};

    // Jacobian
    real_t J[2 * 2] = {0};
    radtan4_point_jacobian(params, p, J);

    // Calculate update
    // dp = inv(J' * J) * J' * e;
    real_t Jt[2 * 2] = {0};
    real_t JtJ[2 * 2] = {0};
    real_t JtJ_inv[2 * 2] = {0};
    real_t dp[2] = {0};

    mat_transpose(J, 2, 2, Jt);
    dot(Jt, 2, 2, J, 2, 2, JtJ);
    pinv(JtJ, 2, 2, JtJ_inv);
    dot3(JtJ_inv, 2, 2, Jt, 2, 2, e, 2, 1, dp);

    // Update
    p[0] += dp[0];
    p[1] += dp[1];

    // Calculate cost
    // cost = e' * e
    const real_t cost = e[0] * e[0] + e[1] * e[1];
    if (cost < 1.0e-15) {
      break;
    }
  }

  // Return result
  p_out[0] = p[0];
  p_out[1] = p[1];
}

/**
 * Form Radial-Tangential point jacobian, using distortion `params` (k1, k2,
 * p1, p2), 2x1 image point `p`, the jacobian is written to 2x2 `J_point`.
 */
void radtan4_point_jacobian(const real_t params[4],
                            const real_t p[2],
                            real_t J_point[2 * 2]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(J_point != NULL);

  // Distortion parameters
  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t p1 = params[2];
  const real_t p2 = params[3];

  // Point
  const real_t x = p[0];
  const real_t y = p[1];

  // Apply radial distortion
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  // Point Jacobian is 2x2
  J_point[0] = k1 * r2 + k2 * r4 + 2 * p1 * y + 6 * p2 * x;
  J_point[0] += x * (2 * k1 * x + 4 * k2 * x * r2) + 1;
  J_point[1] = 2 * p1 * x + 2 * p2 * y + y * (2 * k1 * x + 4 * k2 * x * r2);
  J_point[2] = J_point[1];
  J_point[3] = k1 * r2 + k2 * r4 + 6 * p1 * y + 2 * p2 * x;
  J_point[3] += y * (2 * k1 * y + 4 * k2 * y * r2) + 1;
}

/**
 * Form Radial-Tangential parameter jacobian, using distortion `params` (k1,
 * k2, p1, p2), 2x1 image point `p`, the jacobian is written to 2x4 `J_param`.
 */
void radtan4_params_jacobian(const real_t params[4],
                             const real_t p[2],
                             real_t J_param[2 * 4]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(J_param != NULL);
  UNUSED(params);

  // Point
  const real_t x = p[0];
  const real_t y = p[1];

  // Setup
  const real_t x2 = x * x;
  const real_t y2 = y * y;
  const real_t xy = x * y;
  const real_t r2 = x2 + y2;
  const real_t r4 = r2 * r2;

  // Param Jacobian is 2x4
  J_param[0] = x * r2;
  J_param[1] = x * r4;
  J_param[2] = 2 * xy;
  J_param[3] = 3 * x2 + y2;

  J_param[4] = y * r2;
  J_param[5] = y * r4;
  J_param[6] = x2 + 3 * y2;
  J_param[7] = 2 * xy;
}

//////////
// EQUI //
//////////

/**
 * Distort 2x1 point `p` using Equi-Distant distortion, where the
 * distortion params are stored in `params` (k1, k2, k3, k4), results are
 * written to 2x1 vector `p_d`.
 */
void equi4_distort(const real_t params[4],
                   const real_t p_in[2],
                   real_t p_out[2]) {
  assert(params != NULL);
  assert(p_in != NULL);
  assert(p_out != NULL);

  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t k3 = params[2];
  const real_t k4 = params[3];

  const real_t x = p_in[0];
  const real_t y = p_in[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const real_t s = thd / r;

  p_out[0] = s * x;
  p_out[1] = s * y;
}

/**
 * Equi-distant un-distort
 */
void equi4_undistort(const real_t dist_params[4],
                     const real_t p_in[2],
                     real_t p_out[2]) {
  const real_t k1 = dist_params[0];
  const real_t k2 = dist_params[1];
  const real_t k3 = dist_params[2];
  const real_t k4 = dist_params[3];

  const real_t thd = sqrt(p_in[0] * p_in[0] + p_in[1] * p_in[1]);
  real_t th = thd; // Initial guess
  for (int i = 20; i > 0; i--) {
    const real_t th2 = th * th;
    const real_t th4 = th2 * th2;
    const real_t th6 = th4 * th2;
    const real_t th8 = th4 * th4;
    th = thd / (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  }

  const real_t scaling = tan(th) / thd;
  p_out[0] = p_in[0] * scaling;
  p_out[1] = p_in[1] * scaling;
}

/**
 * Form Equi-Distant point jacobian, using distortion `params` (k1, k2, k3,
 * k4), 2x1 image point `p`, the jacobian is written to 2x2 `J_point`.
 */
void equi4_point_jacobian(const real_t params[4],
                          const real_t p[2],
                          real_t J_point[2 * 2]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(J_point != NULL);

  const real_t k1 = params[0];
  const real_t k2 = params[1];
  const real_t k3 = params[2];
  const real_t k4 = params[3];

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th4 = th2 * th2;
  const real_t th6 = th4 * th2;
  const real_t th8 = th4 * th4;
  const real_t thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);

  const real_t th_r = 1.0 / (r * r + 1.0);
  const real_t thd_th =
      1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
  const real_t s = thd / r;
  const real_t s_r = thd_th * th_r / r - thd / (r * r);
  const real_t r_x = 1.0 / r * x;
  const real_t r_y = 1.0 / r * y;

  // Point Jacobian is 2x2
  J_point[0] = s + x * s_r * r_x;
  J_point[1] = x * s_r * r_y;
  J_point[2] = y * s_r * r_x;
  J_point[3] = s + y * s_r * r_y;
}

/**
 * Form Equi-Distant parameter jacobian, using distortion `params` (k1, k2,
 * k3, k4), 2x1 image point `p`, the jacobian is written to 2x4 `J_param`.
 */
void equi4_params_jacobian(const real_t params[4],
                           const real_t p[2],
                           real_t J_param[2 * 4]) {
  assert(params != NULL);
  assert(p != NULL);
  assert(J_param != NULL);
  UNUSED(params);

  const real_t x = p[0];
  const real_t y = p[1];
  const real_t r = sqrt(x * x + y * y);

  const real_t th = atan(r);
  const real_t th2 = th * th;
  const real_t th3 = th2 * th;
  const real_t th5 = th3 * th2;
  const real_t th7 = th5 * th2;
  const real_t th9 = th7 * th2;

  // Param Jacobian is 2x4
  J_param[0] = x * th3 / r;
  J_param[1] = x * th5 / r;
  J_param[2] = x * th7 / r;
  J_param[3] = x * th9 / r;

  J_param[4] = y * th3 / r;
  J_param[5] = y * th5 / r;
  J_param[6] = y * th7 / r;
  J_param[7] = y * th9 / r;
}

/////////////
// PINHOLE //
/////////////

/**
 * Estimate pinhole focal length. The focal length is estimated using
 * `image_width` [pixels], and `fov` (Field of view of the camera) [rad].
 */
real_t pinhole_focal(const int image_width, const real_t fov) {
  return ((image_width / 2.0) / tan(deg2rad(fov) / 2.0));
}

/**
 * From 3x3 camera matrix K using pinhole camera parameters.
 *
 *   K = [fx,  0,  cx,
 *         0  fy,  cy,
 *         0   0,   1];
 *
 * where `params` is assumed to contain the fx, fy, cx, cy in that order.
 */
void pinhole_K(const real_t params[4], real_t K[3 * 3]) {
  K[0] = params[0];
  K[1] = 0.0;
  K[2] = params[2];

  K[3] = 0.0;
  K[4] = params[1];
  K[5] = params[3];

  K[6] = 0.0;
  K[7] = 0.0;
  K[8] = 1.0;
}

/**
 * Form 3x4 pinhole projection matrix `P`:
 *
 *   P = K * [-C | -C * r];
 *
 * Where K is the pinhole camera matrix formed using the camera parameters
 * `params` (fx, fy, cx, cy), C and r is the rotation and translation
 * component of the camera pose represented as a 4x4 homogenous transform `T`.
 */
void pinhole_projection_matrix(const real_t params[4],
                               const real_t T[4 * 4],
                               real_t P[3 * 4]) {
  assert(params != NULL);
  assert(T != NULL);
  assert(P != NULL);

  // Form K matrix
  real_t K[3 * 3] = {0};
  pinhole_K(params, K);

  // Invert camera pose
  real_t T_inv[4 * 4] = {0};
  tf_inv(T, T_inv);

  // Extract rotation and translation component
  real_t C[3 * 3] = {0};
  real_t r[3] = {0};
  tf_rot_get(T_inv, C);
  tf_trans_get(T_inv, r);

  // Form [C | r] matrix
  real_t Cr[3 * 4] = {0};
  Cr[0] = C[0];
  Cr[1] = C[1];
  Cr[2] = C[2];
  Cr[3] = r[0];

  Cr[4] = C[3];
  Cr[5] = C[4];
  Cr[6] = C[5];
  Cr[7] = r[1];

  Cr[8] = C[6];
  Cr[9] = C[7];
  Cr[10] = C[8];
  Cr[11] = r[2];

  // Form projection matrix P = K * [C | r]
  dot(K, 3, 3, Cr, 3, 4, P);
}

/**
 * Project 3D point `p_C` observed from the camera to the image plane `z`
 * using pinhole parameters `params` (fx, fy, cx, cy).
 */
void pinhole_project(const real_t params[4], const real_t p_C[3], real_t z[2]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(z != NULL);

  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  const real_t px = p_C[0] / p_C[2];
  const real_t py = p_C[1] / p_C[2];

  z[0] = px * fx + cx;
  z[1] = py * fy + cy;
}

/**
 * Form Pinhole point jacobian `J` using pinhole parameters `params`.
 */
void pinhole_point_jacobian(const real_t params[4], real_t J[2 * 2]) {
  assert(params != NULL);
  assert(J != NULL);

  J[0] = params[0];
  J[1] = 0.0;
  J[2] = 0.0;
  J[3] = params[1];
}

/**
 * Form Pinhole parameter jacobian `J` using pinhole parameters `params` and
 * 2x1 image point `x`.
 */
void pinhole_params_jacobian(const real_t params[4],
                             const real_t x[2],
                             real_t J[2 * 4]) {
  assert(params != NULL);
  assert(x != NULL);
  assert(J != NULL);
  UNUSED(params);

  J[0] = x[0];
  J[1] = 0.0;
  J[2] = 1.0;
  J[3] = 0.0;

  J[4] = 0.0;
  J[5] = x[1];
  J[6] = 0.0;
  J[7] = 1.0;
}

/////////////////////
// PINHOLE-RADTAN4 //
/////////////////////

/**
 * Projection of 3D point to image plane using Pinhole + Radial-Tangential.
 */
void pinhole_radtan4_project(const real_t params[8],
                             const real_t p_C[3],
                             real_t z[2]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(z != NULL);

  // Project
  const real_t p[2] = {p_C[0] / p_C[2], p_C[1] / p_C[2]};

  // Distort
  const real_t d[4] = {params[4], params[5], params[6], params[7]};
  real_t p_d[2] = {0};
  radtan4_distort(d, p, p_d);

  // Scale and center
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  z[0] = p_d[0] * fx + cx;
  z[1] = p_d[1] * fy + cy;
}

/**
 * Pinhole Radial-Tangential Undistort
 */
void pinhole_radtan4_undistort(const real_t params[8],
                               const real_t z_in[2],
                               real_t z_out[2]) {
  assert(params != NULL);
  assert(z_in != NULL);
  assert(z_out != NULL);

  // Back project and undistort
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];
  const real_t p[2] = {(z_in[0] - cx) / fx, (z_in[1] - cy) / fy};
  real_t p_undist[2] = {0};
  radtan4_undistort(params + 4, p, p_undist);

  // Project undistorted point to image plane
  z_out[0] = p_undist[0] * fx + cx;
  z_out[1] = p_undist[1] * fy + cy;
}

/**
 * Pinhole Radial-Tangential back project
 */
void pinhole_radtan4_back_project(const real_t params[8],
                                  const real_t z[2],
                                  real_t ray[3]) {
  assert(params != NULL);
  assert(z != NULL);
  assert(ray != NULL);

  // Back project and undistort
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];
  const real_t p[2] = {(z[0] - cx) / fx, (z[1] - cy) / fy};
  real_t p_undist[2] = {0};
  radtan4_undistort(params + 4, p, p_undist);

  ray[0] = p_undist[0];
  ray[1] = p_undist[1];
  ray[2] = 1.0;
}

/**
 * Projection Jacobian of Pinhole + Radial-Tangential.
 */
void pinhole_radtan4_project_jacobian(const real_t params[8],
                                      const real_t p_C[3],
                                      real_t J[2 * 3]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  // Project
  const real_t x = p_C[0];
  const real_t y = p_C[1];
  const real_t z = p_C[2];
  const real_t p[2] = {x / z, y / z};

  // Projection Jacobian
  real_t J_p[2 * 3] = {0};
  J_p[0] = 1.0 / z;
  J_p[1] = 0.0;
  J_p[2] = -x / (z * z);
  J_p[3] = 0.0;
  J_p[4] = 1.0 / z;
  J_p[5] = -y / (z * z);

  // Distortion Point Jacobian
  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};
  real_t J_d[2 * 2] = {0};
  radtan4_point_jacobian(d, p, J_d);

  // Project Point Jacobian
  real_t J_k[2 * 3] = {0};
  pinhole_point_jacobian(params, J_k);

  // J = J_k * J_d * J_p;
  real_t J_dp[2 * 3] = {0};
  dot(J_d, 2, 2, J_p, 2, 3, J_dp);
  dot(J_k, 2, 2, J_dp, 2, 3, J);
}

/**
 * Parameter Jacobian of Pinhole + Radial-Tangential.
 */
void pinhole_radtan4_params_jacobian(const real_t params[8],
                                     const real_t p_C[3],
                                     real_t J[2 * 8]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];
  const real_t k[4] = {fx, fy, cx, cy};

  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};

  // Project
  const real_t x = p_C[0];
  const real_t y = p_C[1];
  const real_t z = p_C[2];
  const real_t p[2] = {x / z, y / z};

  // Distort
  real_t p_d[2] = {0};
  radtan4_distort(d, p, p_d);

  // Project params Jacobian: J_proj_params
  real_t J_proj_params[2 * 4] = {0};
  pinhole_params_jacobian(k, p_d, J_proj_params);

  // Project point Jacobian: J_proj_point
  real_t J_proj_point[2 * 2] = {0};
  pinhole_point_jacobian(k, J_proj_point);

  // Distortion point Jacobian: J_dist_params
  real_t J_dist_params[2 * 4] = {0};
  radtan4_params_jacobian(d, p, J_dist_params);

  // Radtan4 params Jacobian: J_radtan4
  real_t J_radtan4[2 * 4] = {0};
  dot(J_proj_point, 2, 2, J_dist_params, 2, 4, J_radtan4);

  // J = [J_proj_params, J_proj_point * J_dist_params]
  J[0] = J_proj_params[0];
  J[1] = J_proj_params[1];
  J[2] = J_proj_params[2];
  J[3] = J_proj_params[3];

  J[8] = J_proj_params[4];
  J[9] = J_proj_params[5];
  J[10] = J_proj_params[6];
  J[11] = J_proj_params[7];

  J[4] = J_radtan4[0];
  J[5] = J_radtan4[1];
  J[6] = J_radtan4[2];
  J[7] = J_radtan4[3];

  J[12] = J_radtan4[4];
  J[13] = J_radtan4[5];
  J[14] = J_radtan4[6];
  J[15] = J_radtan4[7];
}

///////////////////
// PINHOLE-EQUI4 //
///////////////////

/**
 * Projection of 3D point to image plane using Pinhole + Equi-Distant.
 */
void pinhole_equi4_project(const real_t params[8],
                           const real_t p_C[3],
                           real_t z[2]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(z != NULL);

  // Project
  const real_t p[2] = {p_C[0] / p_C[2], p_C[1] / p_C[2]};

  // Distort
  const real_t d[4] = {params[4], params[5], params[6], params[7]};
  real_t p_d[2] = {0};
  equi4_distort(d, p, p_d);

  // Scale and center
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];

  z[0] = p_d[0] * fx + cx;
  z[1] = p_d[1] * fy + cy;
}

/**
 * Pinhole Equi-distant Undistort
 */
void pinhole_equi4_undistort(const real_t params[8],
                             const real_t z_in[2],
                             real_t z_out[2]) {
  assert(params != NULL);
  assert(z_in != NULL);
  assert(z_out != NULL);

  // Back project and undistort
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];
  const real_t p[2] = {(z_in[0] - cx) / fx, (z_in[1] - cy) / fy};
  real_t p_undist[2] = {0};
  equi4_undistort(params + 4, p, p_undist);

  // Project undistorted point to image plane
  z_out[0] = p_undist[0] * fx + cx;
  z_out[1] = p_undist[1] * fy + cy;
}

/**
 * Pinhole Equi-distant back project
 */
void pinhole_equi4_back_project(const real_t params[8],
                                const real_t z[2],
                                real_t ray[3]) {
  assert(params != NULL);
  assert(z != NULL);
  assert(ray != NULL);

  // Back project and undistort
  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];
  const real_t p[2] = {(z[0] - cx) / fx, (z[1] - cy) / fy};
  real_t p_undist[2] = {0};
  equi4_undistort(params + 4, p, p_undist);

  ray[0] = p_undist[0];
  ray[1] = p_undist[1];
  ray[2] = 1.0;
}

/**
 * Projection Jacobian of Pinhole + Equi-Distant.
 */
void pinhole_equi4_project_jacobian(const real_t params[8],
                                    const real_t p_C[3],
                                    real_t J[2 * 3]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  // Project
  const real_t x = p_C[0];
  const real_t y = p_C[1];
  const real_t z = p_C[2];
  const real_t p[2] = {x / z, y / z};

  // Projection Jacobian
  real_t J_p[2 * 3] = {0};
  J_p[0] = 1.0 / z;
  J_p[1] = 0.0;
  J_p[2] = -x / (z * z);
  J_p[3] = 0.0;
  J_p[4] = 1.0 / z;
  J_p[5] = -y / (z * z);

  // Distortion Point Jacobian
  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t k3 = params[6];
  const real_t k4 = params[7];
  const real_t d[4] = {k1, k2, k3, k4};
  real_t J_d[2 * 2] = {0};
  equi4_point_jacobian(d, p, J_d);

  // Project Point Jacobian
  real_t J_k[2 * 3] = {0};
  pinhole_point_jacobian(params, J_k);

  // J = J_k * J_d * J_p;
  real_t J_dp[2 * 3] = {0};
  dot(J_d, 2, 2, J_p, 2, 3, J_dp);
  dot(J_k, 2, 2, J_dp, 2, 3, J);
}

void pinhole_equi4_params_jacobian(const real_t params[8],
                                   const real_t p_C[3],
                                   real_t J[2 * 8]) {
  assert(params != NULL);
  assert(p_C != NULL);
  assert(J != NULL);

  const real_t fx = params[0];
  const real_t fy = params[1];
  const real_t cx = params[2];
  const real_t cy = params[3];
  const real_t k[4] = {fx, fy, cx, cy};

  const real_t k1 = params[4];
  const real_t k2 = params[5];
  const real_t p1 = params[6];
  const real_t p2 = params[7];
  const real_t d[4] = {k1, k2, p1, p2};

  // Project
  const real_t x = p_C[0];
  const real_t y = p_C[1];
  const real_t z = p_C[2];
  const real_t p[2] = {x / z, y / z};

  // Distort
  real_t p_d[2] = {0};
  equi4_distort(d, p, p_d);

  // Project params Jacobian: J_proj_params
  real_t J_proj_params[2 * 4] = {0};
  pinhole_params_jacobian(k, p_d, J_proj_params);

  // Project point Jacobian: J_proj_point
  real_t J_proj_point[2 * 2] = {0};
  pinhole_point_jacobian(k, J_proj_point);

  // Distortion point Jacobian: J_dist_params
  real_t J_dist_params[2 * 4] = {0};
  equi4_params_jacobian(d, p, J_dist_params);

  // Radtan4 params Jacobian: J_equi4
  real_t J_equi4[2 * 4] = {0};
  dot(J_proj_point, 2, 2, J_dist_params, 2, 4, J_equi4);

  // J = [J_proj_params, J_proj_point * J_dist_params]
  J[0] = J_proj_params[0];
  J[1] = J_proj_params[1];
  J[2] = J_proj_params[2];
  J[3] = J_proj_params[3];

  J[8] = J_proj_params[4];
  J[9] = J_proj_params[5];
  J[10] = J_proj_params[6];
  J[11] = J_proj_params[7];

  J[4] = J_equi4[0];
  J[5] = J_equi4[1];
  J[6] = J_equi4[2];
  J[7] = J_equi4[3];

  J[12] = J_equi4[4];
  J[13] = J_equi4[5];
  J[14] = J_equi4[6];
  J[15] = J_equi4[7];
}

//////////////
// GEOMETRY //
//////////////

/**
 * Triangulate a single 3D point `p` observed by two different camera frames
 * represented by two 3x4 camera projection matrices `P_i` and `P_j`, and the
 * 2D image point correspondance `z_i` and `z_j`.
 */
void linear_triangulation(const real_t P_i[3 * 4],
                          const real_t P_j[3 * 4],
                          const real_t z_i[2],
                          const real_t z_j[2],
                          real_t p[3]) {
  assert(P_i != NULL);
  assert(P_j != NULL);
  assert(z_i != NULL);
  assert(z_j != NULL);
  assert(p != NULL);

  // Form A matrix
  real_t A[4 * 4] = {0};
  // -- ROW 1
  A[0] = -P_i[4] + P_i[8] * z_i[1];
  A[1] = -P_i[5] + P_i[9] * z_i[1];
  A[2] = P_i[10] * z_i[1] - P_i[6];
  A[3] = P_i[11] * z_i[1] - P_i[7];
  // -- ROW 2
  A[4] = -P_i[0] + P_i[8] * z_i[0];
  A[5] = -P_i[1] + P_i[9] * z_i[0];
  A[6] = P_i[10] * z_i[0] - P_i[2];
  A[7] = P_i[11] * z_i[0] - P_i[3];
  // -- ROW 3
  A[8] = -P_j[4] + P_j[8] * z_j[1];
  A[9] = -P_j[5] + P_j[9] * z_j[1];
  A[10] = P_j[10] * z_j[1] - P_j[6];
  A[11] = P_j[11] * z_j[1] - P_j[7];
  // -- ROW 4
  A[12] = -P_j[0] + P_j[8] * z_j[0];
  A[13] = -P_j[1] + P_j[9] * z_j[0];
  A[14] = P_j[10] * z_j[0] - P_j[2];
  A[15] = P_j[11] * z_j[0] - P_j[3];

  // Form A_t
  real_t A_t[4 * 4] = {0};
  mat_transpose(A, 4, 4, A_t);

  // SVD
  real_t A2[4 * 4] = {0};
  real_t s[4] = {0};
  real_t U[4 * 4] = {0};
  real_t V[4 * 4] = {0};
  dot(A_t, 4, 4, A, 4, 4, A2);
  svd(A2, 4, 4, U, s, V);

  // Get best row of V_t
  real_t min_s = s[0];
  real_t x = V[0];
  real_t y = V[4];
  real_t z = V[8];
  real_t w = V[12];
  for (int i = 1; i < 4; i++) {
    if (s[i] < min_s) {
      min_s = s[i];
      x = V[i + 0];
      y = V[i + 4];
      z = V[i + 8];
      w = V[i + 12];
    }
  }

  // Normalize the scale to obtain the 3D point
  p[0] = x / w;
  p[1] = y / w;
  p[2] = z / w;
}

/**
 * Find Homography.
 *
 * A Homography is a transformation (a 3x3 matrix) that maps the normalized
 * image points from one image to the corresponding normalized image points in
 * the other image. Specifically, let x and y be the n-th homogeneous points
 * of pts_i and pts_j:
 *
 *   x = [u_i, v_i, 1.0]
 *   y = [u_j, v_j, 1.0]
 *
 * The Homography is a 3x3 matrix that transforms x to y:
 *
 *   y = H * x
 *
 * **IMPORTANT**: The normalized image points `pts_i` and `pts_j` must
 * correspond to points in 3D that on a plane.
 */
int homography_find(const real_t *pts_i,
                    const real_t *pts_j,
                    const int num_points,
                    real_t H[3 * 3]) {

  const int Am = 2 * num_points;
  const int An = 9;
  real_t *A = malloc(sizeof(real_t) * Am * An);

  for (int n = 0; n < num_points; n++) {
    const real_t x_i = pts_i[n * 2 + 0];
    const real_t y_i = pts_i[n * 2 + 1];
    const real_t x_j = pts_j[n * 2 + 0];
    const real_t y_j = pts_j[n * 2 + 1];

    const int rs = n * 18;
    const int re = n * 18 + 9;
    A[rs + 0] = -x_i;
    A[rs + 1] = -y_i;
    A[rs + 2] = -1.0;
    A[rs + 3] = 0.0;
    A[rs + 4] = 0.0;
    A[rs + 5] = 0.0;
    A[rs + 6] = x_i * x_j;
    A[rs + 7] = y_i * x_j;
    A[rs + 8] = x_j;

    A[re + 0] = 0.0;
    A[re + 1] = 0.0;
    A[re + 2] = 0.0;
    A[re + 3] = -x_i;
    A[re + 4] = -y_i;
    A[re + 5] = -1.0;
    A[re + 6] = x_i * y_j;
    A[re + 7] = y_i * y_j;
    A[re + 8] = y_j;
  }

  real_t *U = malloc(sizeof(real_t) * Am * Am);
  real_t *s = malloc(sizeof(real_t) * Am);
  real_t *V = malloc(sizeof(real_t) * An * An);
  if (svd(A, Am, An, U, s, V) != 0) {
    return -1;
  }

  // Form the Homography matrix using the last column of V and normalize
  H[0] = V[8] / V[80];
  H[1] = V[17] / V[80];
  H[2] = V[26] / V[80];

  H[3] = V[35] / V[80];
  H[4] = V[44] / V[80];
  H[5] = V[53] / V[80];

  H[6] = V[62] / V[80];
  H[7] = V[71] / V[80];
  H[8] = V[80] / V[80];

  // Clean up
  free(A);
  free(U);
  free(s);
  free(V);

  return 0;
}

/**
 * Compute relative pose between camera and planar object `T_CF` using `N` 3D
 * object points `obj_pts`, 2D image points in pixels, as well as the pinhole
 * focal lengths `fx`, `fy` and principal centers `cx` and `cy`.
 *
 * Source:
 *
 *   Section 4.1.3: From homography to pose computation
 *
 *   Marchand, Eric, Hideaki Uchiyama, and Fabien Spindler. "Pose estimation
 *   for augmented reality: a hands-on survey." IEEE transactions on
 *   visualization and computer graphics 22.12 (2015): 2633-2651.
 *
 *   https://github.com/lagadic/camera_localization
 *
 * Returns:
 *
 *   `0` for success and `-1` for failure.
 *
 */
int homography_pose(const real_t *proj_params,
                    const real_t *img_pts,
                    const real_t *obj_pts,
                    const int N,
                    real_t T_CF[4 * 4]) {
  // Form A to compute ||Ah|| = 0 using SVD, where A is an (N * 2) x 9 matrix
  // and h is the vectorized Homography matrix h, N is the number of points.
  // if N == 4, the matrix has more columns than rows. The solution is to add
  // an extra line with zeros.
  const int num_rows = 2 * N + ((N == 4) ? 1 : 0);
  const int num_cols = 9;
  const real_t fx = proj_params[0];
  const real_t fy = proj_params[1];
  const real_t cx = proj_params[2];
  const real_t cy = proj_params[3];
  real_t *A = malloc(sizeof(real_t) * num_rows * num_cols);

  for (int i = 0; i < N; i++) {
    const real_t kp[2] = {img_pts[i * 2 + 0], img_pts[i * 2 + 1]};
    const real_t x0[2] = {obj_pts[i * 3 + 0], obj_pts[i * 3 + 1]};
    const real_t x1[2] = {(kp[0] - cx) / fx, (kp[1] - cy) / fy};

    const int rs = i * 18;
    const int re = i * 18 + 9;
    A[rs + 0] = 0.0;
    A[rs + 1] = 0.0;
    A[rs + 2] = 0.0;
    A[rs + 3] = -x0[0];
    A[rs + 4] = -x0[1];
    A[rs + 5] = -1.0;
    A[rs + 6] = x1[1] * x0[0];
    A[rs + 7] = x1[1] * x0[1];
    A[rs + 8] = x1[1];

    A[re + 0] = x0[0];
    A[re + 1] = x0[1];
    A[re + 2] = 1.0;
    A[re + 3] = 0.0;
    A[re + 4] = 0.0;
    A[re + 5] = 0.0;
    A[re + 6] = -x1[0] * x0[0];
    A[re + 7] = -x1[0] * x0[1];
    A[re + 8] = -x1[0];
  }

  const int Am = num_rows;
  const int An = num_cols;
  real_t *U = malloc(sizeof(real_t) * Am * Am);
  real_t *s = malloc(sizeof(real_t) * Am);
  real_t *V = malloc(sizeof(real_t) * An * An);
  if (svd(A, Am, An, U, s, V) != 0) {
    free(A);
    free(U);
    free(s);
    free(V);
    return -1;
  }

  // Form the Homography matrix using the last column of V
  real_t H[3 * 3] = {0};
  H[0] = V[8];
  H[1] = V[17];
  H[2] = V[26];

  H[3] = V[35];
  H[4] = V[44];
  H[5] = V[53];

  H[6] = V[62];
  H[7] = V[71];
  H[8] = V[80];

  if (H[8] < 0) {
    for (int i = 0; i < 9; i++) {
      H[i] *= -1.0;
    }
  }

  // Normalize H to ensure that || c1 || = 1
  const real_t H_norm = sqrt(H[0] * H[0] + H[3] * H[3] + H[6] * H[6]);
  for (int i = 0; i < 9; i++) {
    H[i] /= H_norm;
  }

  // Form translation vector
  const real_t r[3] = {H[2], H[5], H[8]};

  // Form Rotation matrix
  const real_t c1[3] = {H[0], H[3], H[6]};
  const real_t c2[3] = {H[1], H[4], H[7]};
  real_t c3[3] = {0};
  vec3_cross(c1, c2, c3);

  real_t C[3 * 3] = {0};
  for (int i = 0; i < 3; i++) {
    C[(i * 3) + 0] = c1[i];
    C[(i * 3) + 1] = c2[i];
    C[(i * 3) + 2] = c3[i];
  }

  // Set T_CF
  T_CF[0] = C[0];
  T_CF[1] = C[1];
  T_CF[2] = C[2];
  T_CF[3] = r[0];

  T_CF[4] = C[3];
  T_CF[5] = C[4];
  T_CF[6] = C[5];
  T_CF[7] = r[1];

  T_CF[8] = C[6];
  T_CF[9] = C[7];
  T_CF[10] = C[8];
  T_CF[11] = r[2];

  T_CF[12] = 0.0;
  T_CF[13] = 0.0;
  T_CF[14] = 0.0;
  T_CF[15] = 1.0;

  // Clean up
  free(A);
  free(U);
  free(s);
  free(V);

  return 0;
}

// static int kneip_solve_quadratic(const real_t factors[5],
//                                  real_t real_roots[4]) {
//   const real_t A = factors[0];
//   const real_t B = factors[1];
//   const real_t C = factors[2];
//   const real_t D = factors[3];
//   const real_t E = factors[4];

//   const real_t A_pw2 = A * A;
//   const real_t B_pw2 = B * B;
//   const real_t A_pw3 = A_pw2 * A;
//   const real_t B_pw3 = B_pw2 * B;
//   const real_t A_pw4 = A_pw3 * A;
//   const real_t B_pw4 = B_pw3 * B;

//   const real_t alpha = -3 * B_pw2 / (8 * A_pw2) + C / A;
//   const real_t beta = B_pw3 / (8 * A_pw3) - B * C / (2 * A_pw2) + D / A;
//   const real_t gamma = -3 * B_pw4 / (256 * A_pw4) + B_pw2 * C / (16 * A_pw3) -
//                        B * D / (4 * A_pw2) + E / A;

//   const real_t alpha_pw2 = alpha * alpha;
//   const real_t alpha_pw3 = alpha_pw2 * alpha;

//   const real_complex_t P = (-alpha_pw2 / 12 - gamma);
//   const real_complex_t Q =
//       -alpha_pw3 / 108 + alpha * gamma / 3 - pow(beta, 2) / 8;
//   const real_complex_t R =
//       -Q / 2.0 + sqrt(pow(Q, 2.0) / 4.0 + pow(P, 3.0) / 27.0);

//   const real_complex_t U = pow(R, (1.0 / 3.0));
//   real_complex_t y;
//   if (fabs(creal(U)) < 1e-10) {
//     y = -5.0 * alpha / 6.0 - pow(Q, (1.0 / 3.0));
//   } else {
//     y = -5.0 * alpha / 6.0 - P / (3.0 * U) + U;
//   }

//   const real_complex_t w = sqrt(alpha + 2.0 * y);
//   const real_t m = -B / (4.0 * A);
//   const real_t a = sqrt(-(3.0 * alpha + 2.0 * y + 2.0 * beta / w));
//   const real_t b = sqrt(-(3.0 * alpha + 2.0 * y - 2.0 * beta / w));
//   real_roots[0] = creal(m + 0.5 * (w + a));
//   real_roots[1] = creal(m + 0.5 * (w - a));
//   real_roots[2] = creal(m + 0.5 * (-w + b));
//   real_roots[3] = creal(m + 0.5 * (-w - b));

//   return 0;
// }

// /**
//  * Kneip's Perspective-3-Point solver.
//  *
//  * This function uses 3 2D point correspondants to 3D features to determine
//  * the camera pose.
//  *
//  * Source: Kneip, Laurent, Davide Scaramuzza, and Roland Siegwart. "A novel
//  * parametrization of the perspective-three-point problem for a direct
//  * computation of absolute camera position and orientation." CVPR 2011. IEEE,
//  * 2011.
//  */
// int p3p_kneip(const real_t features[3][3],
//               const real_t points[3][3],
//               real_t solutions[4][4 * 4]) {
//   assert(features != NULL);
//   assert(points != NULL);
//   assert(solutions != NULL);

//   // Extract points
//   real_t P1[3] = {points[0][0], points[0][1], points[0][2]};
//   real_t P2[3] = {points[1][0], points[1][1], points[1][2]};
//   real_t P3[3] = {points[2][0], points[2][1], points[2][2]};

//   // Verify points are not colinear
//   real_t temp1[3] = {P2[0] - P1[0], P2[1] - P1[1], P2[2] - P2[2]};
//   real_t temp2[3] = {P3[0] - P1[0], P3[1] - P1[1], P3[2] - P2[2]};
//   real_t temp3[3] = {0};
//   vec3_cross(temp1, temp2, temp3);
//   if (fabs(vec3_norm(temp3)) > 1e-10) {
//     return -1;
//   }

//   // Extract feature vectors
//   real_t f1[3] = {features[0][0], features[0][1], features[0][2]};
//   real_t f2[3] = {features[1][0], features[1][1], features[1][2]};
//   real_t f3[3] = {features[2][0], features[2][1], features[2][2]};

//   // Creation of intermediate camera frame
//   real_t e1[3] = {f1[0], f1[1], f1[2]};
//   real_t e3[3] = {0};
//   vec3_cross(f1, f2, e3);
//   vec3_normalize(e3);
//   real_t e2[3] = {0};
//   vec3_cross(e3, e1, e2);

//   // clang-format off
//   real_t T[3 * 3] = {
//     e1[0], e1[1], e1[2],
//     e2[0], e2[1], e2[2],
//     e3[0], e3[1], e3[2]
//   };
//   // clang-format on

//   // f3 = T * f3;
//   {
//     real_t x[3] = {0};
//     x[0] = T[0] * f3[0] + T[1] * f3[1] + T[2] * f3[2];
//     x[1] = T[3] * f3[0] + T[4] * f3[1] + T[5] * f3[2];
//     x[2] = T[6] * f3[0] + T[7] * f3[1] + T[8] * f3[2];
//     f3[0] = x[0];
//     f3[1] = x[1];
//     f3[2] = x[2];
//   }

//   // Reinforce that f3(2,0) > 0 for having theta in [0;pi]
//   if (f3[2] > 0) {
//     // f1 = features.col(1);
//     f1[0] = features[0][0];
//     f1[1] = features[0][1];
//     f1[2] = features[0][2];

//     // f2 = features.col(0);
//     f2[0] = features[1][0];
//     f2[1] = features[1][1];
//     f2[2] = features[1][2];

//     // f3 = features.col(2);
//     f3[0] = features[2][0];
//     f3[1] = features[2][1];
//     f3[2] = features[2][2];

//     // e1 = f1;
//     e1[0] = f1[0];
//     e1[1] = f1[1];
//     e1[2] = f1[2];

//     // e3 = f1.cross(f2);
//     // e3 = e3 / e3.norm();
//     vec3_cross(f1, f2, e3);
//     vec3_normalize(e3);

//     // e2 = e3.cross(e1);
//     vec3_cross(e3, e1, e2);

//     // T.row(0) = e1.transpose();
//     T[0] = e1[0];
//     T[1] = e1[1];
//     T[2] = e1[2];

//     // T.row(1) = e2.transpose();
//     T[3] = e2[0];
//     T[4] = e2[1];
//     T[5] = e2[2];

//     // T.row(2) = e3.transpose();
//     T[6] = e3[0];
//     T[7] = e3[1];
//     T[8] = e3[2];

//     // f3 = T * f3;
//     {
//       real_t x[3] = {0};
//       x[0] = T[0] * f3[0] + T[1] * f3[1] + T[2] * f3[2];
//       x[1] = T[3] * f3[0] + T[4] * f3[1] + T[5] * f3[2];
//       x[2] = T[6] * f3[0] + T[7] * f3[1] + T[8] * f3[2];
//       f3[0] = x[0];
//       f3[1] = x[1];
//       f3[2] = x[2];
//     }

//     // P1 = points.col(1);
//     P1[0] = points[0][0];
//     P1[1] = points[0][1];
//     P1[2] = points[0][2];

//     // P2 = points.col(0);
//     P2[0] = points[1][0];
//     P2[1] = points[1][1];
//     P2[2] = points[1][2];

//     // P3 = points.col(2);
//     P3[0] = points[2][0];
//     P3[1] = points[2][1];
//     P3[2] = points[2][2];
//   }

//   // Creation of intermediate world frame
//   // n1 = P2 - P1;
//   // n1 = n1 / n1.norm();
//   real_t n1[3] = {0};
//   vec3_sub(P2, P1, n1);
//   vec3_normalize(n1);

//   // n3 = n1.cross(P3 - P1);
//   // n3 = n3 / n3.norm();
//   real_t n3[3] = {0};
//   vec3_sub(P3, P1, n3);
//   vec3_normalize(n3);

//   // n2 = n3.cross(n1);
//   real_t n2[3] = {0};
//   vec3_cross(n3, n1, n2);

//   // N.row(0) = n1.transpose();
//   // N.row(1) = n2.transpose();
//   // N.row(2) = n3.transpose();
//   // clang-format off
//   real_t N[3 * 3] = {
//     n1[0], n1[1], n1[2],
//     n2[0], n2[1], n2[2],
//     n3[0], n3[1], n3[2]
//   };
//   // clang-format on

//   // Extraction of known parameters
//   // P3 = N * (P3 - P1);
//   {
//     real_t d[3] = {0};
//     vec3_sub(P3, P1, d);
//     P3[0] = N[0] * d[0] + N[1] * d[1] + N[2] * d[2];
//     P3[1] = N[3] * d[0] + N[4] * d[1] + N[5] * d[2];
//     P3[2] = N[6] * d[0] + N[7] * d[1] + N[8] * d[2];
//   }

//   real_t dP21[3] = {0};
//   vec3_sub(P2, P1, dP21);
//   real_t d_12 = vec3_norm(dP21);
//   real_t f_1 = f3[0] / f3[2];
//   real_t f_2 = f3[1] / f3[2];
//   real_t p_1 = P3[0];
//   real_t p_2 = P3[1];

//   // cos_beta = f1.dot(f2);
//   // b = 1 / (1 - pow(cos_beta, 2)) - 1;
//   const real_t cos_beta = f1[0] * f2[0] + f1[1] * f2[1] + f1[1] * f2[1];
//   real_t b = 1 / (1 - pow(cos_beta, 2)) - 1;
//   if (cos_beta < 0) {
//     b = -sqrt(b);
//   } else {
//     b = sqrt(b);
//   }

//   // Definition of temporary variables for avoiding multiple computation
//   const real_t f_1_pw2 = pow(f_1, 2);
//   const real_t f_2_pw2 = pow(f_2, 2);
//   const real_t p_1_pw2 = pow(p_1, 2);
//   const real_t p_1_pw3 = p_1_pw2 * p_1;
//   const real_t p_1_pw4 = p_1_pw3 * p_1;
//   const real_t p_2_pw2 = pow(p_2, 2);
//   const real_t p_2_pw3 = p_2_pw2 * p_2;
//   const real_t p_2_pw4 = p_2_pw3 * p_2;
//   const real_t d_12_pw2 = pow(d_12, 2);
//   const real_t b_pw2 = pow(b, 2);

//   // Computation of factors of 4th degree polynomial
//   real_t factors[5] = {0};
//   factors[0] = -f_2_pw2 * p_2_pw4 - p_2_pw4 * f_1_pw2 - p_2_pw4;
//   factors[1] = 2 * p_2_pw3 * d_12 * b + 2 * f_2_pw2 * p_2_pw3 * d_12 * b -
//                2 * f_2 * p_2_pw3 * f_1 * d_12;
//   factors[2] =
//       -f_2_pw2 * p_2_pw2 * p_1_pw2 - f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2 -
//       f_2_pw2 * p_2_pw2 * d_12_pw2 + f_2_pw2 * p_2_pw4 + p_2_pw4 * f_1_pw2 +
//       2 * p_1 * p_2_pw2 * d_12 + 2 * f_1 * f_2 * p_1 * p_2_pw2 * d_12 * b -
//       p_2_pw2 * p_1_pw2 * f_1_pw2 + 2 * p_1 * p_2_pw2 * f_2_pw2 * d_12 -
//       p_2_pw2 * d_12_pw2 * b_pw2 - 2 * p_1_pw2 * p_2_pw2;
//   factors[3] = 2 * p_1_pw2 * p_2 * d_12 * b + 2 * f_2 * p_2_pw3 * f_1 * d_12 -
//                2 * f_2_pw2 * p_2_pw3 * d_12 * b - 2 * p_1 * p_2 * d_12_pw2 * b;
//   factors[4] =
//       -2 * f_2 * p_2_pw2 * f_1 * p_1 * d_12 * b + f_2_pw2 * p_2_pw2 * d_12_pw2 +
//       2 * p_1_pw3 * d_12 - p_1_pw2 * d_12_pw2 + f_2_pw2 * p_2_pw2 * p_1_pw2 -
//       p_1_pw4 - 2 * f_2_pw2 * p_2_pw2 * p_1 * d_12 +
//       p_2_pw2 * f_1_pw2 * p_1_pw2 + f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2;

//   // Computation of roots
//   real_t real_roots[4] = {0};
//   kneip_solve_quadratic(factors, real_roots);

//   // Backsubstitution of each solution
//   for (int i = 0; i < 4; ++i) {
//     const real_t cot_alpha =
//         (-f_1 * p_1 / f_2 - real_roots[i] * p_2 + d_12 * b) /
//         (-f_1 * real_roots[i] * p_2 / f_2 + p_1 - d_12);
//     const real_t cos_theta = real_roots[i];
//     const real_t sin_theta = sqrt(1 - pow((real_t) real_roots[i], 2));
//     const real_t sin_alpha = sqrt(1 / (pow(cot_alpha, 2) + 1));
//     real_t cos_alpha = sqrt(1 - pow(sin_alpha, 2));
//     if (cot_alpha < 0) {
//       cos_alpha = -cos_alpha;
//     }

//     real_t C[3] = {0};
//     C[0] = d_12 * cos_alpha * (sin_alpha * b + cos_alpha);
//     C[1] = cos_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha);
//     C[2] = sin_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha);
//     // C = P1 + N.transpose() * C;
//     C[0] = P1[0] + (N[0] * C[0] + N[3] * C[1] + N[6] * C[2]);
//     C[1] = P1[1] + (N[1] * C[0] + N[4] * C[1] + N[7] * C[2]);
//     C[2] = P1[2] + (N[2] * C[0] + N[5] * C[1] + N[8] * C[2]);

//     real_t R[3 * 3] = {0};
//     R[0] = -cos_alpha;
//     R[1] = -sin_alpha * cos_theta;
//     R[2] = -sin_alpha * sin_theta;
//     R[3] = sin_alpha;
//     R[4] = -cos_alpha * cos_theta;
//     R[5] = -cos_alpha * sin_theta;
//     R[6] = 0;
//     R[7] = -sin_theta;
//     R[8] = cos_theta;
//     // R = N.transpose() * R.transpose() * T;
//     // clang-format off
//     {
//       real_t tmp[3 * 3] = {0};
//       tmp[0] = T[0]*(N[0]*R[0] + N[3]*R[1] + N[6]*R[2]) + T[3]*(N[0]*R[3] + N[3]*R[4] + N[6]*R[5]) + T[6]*(N[0]*R[6] + N[3]*R[7] + N[6]*R[8]);
//       tmp[1] = T[1]*(N[0]*R[0] + N[3]*R[1] + N[6]*R[2]) + T[4]*(N[0]*R[3] + N[3]*R[4] + N[6]*R[5]) + T[7]*(N[0]*R[6] + N[3]*R[7] + N[6]*R[8]);
//       tmp[2] = T[2]*(N[0]*R[0] + N[3]*R[1] + N[6]*R[2]) + T[5]*(N[0]*R[3] + N[3]*R[4] + N[6]*R[5]) + T[8]*(N[0]*R[6] + N[3]*R[7] + N[6]*R[8]);

//       tmp[3] = T[0]*(N[1]*R[0] + N[4]*R[1] + N[7]*R[2]) + T[3]*(N[1]*R[3] + N[4]*R[4] + N[7]*R[5]) + T[6]*(N[1]*R[6] + N[4]*R[7] + N[7]*R[8]);
//       tmp[4] = T[1]*(N[1]*R[0] + N[4]*R[1] + N[7]*R[2]) + T[4]*(N[1]*R[3] + N[4]*R[4] + N[7]*R[5]) + T[7]*(N[1]*R[6] + N[4]*R[7] + N[7]*R[8]);
//       tmp[5] = T[2]*(N[1]*R[0] + N[4]*R[1] + N[7]*R[2]) + T[5]*(N[1]*R[3] + N[4]*R[4] + N[7]*R[5]) + T[8]*(N[1]*R[6] + N[4]*R[7] + N[7]*R[8]);

//       tmp[6] = T[0]*(N[2]*R[0] + N[5]*R[1] + N[8]*R[2]) + T[3]*(N[2]*R[3] + N[5]*R[4] + N[8]*R[5]) + T[6]*(N[2]*R[6] + N[5]*R[7] + N[8]*R[8]);
//       tmp[7] = T[1]*(N[2]*R[0] + N[5]*R[1] + N[8]*R[2]) + T[4]*(N[2]*R[3] + N[5]*R[4] + N[8]*R[5]) + T[7]*(N[2]*R[6] + N[5]*R[7] + N[8]*R[8]);
//       tmp[8] = T[2]*(N[2]*R[0] + N[5]*R[1] + N[8]*R[2]) + T[5]*(N[2]*R[3] + N[5]*R[4] + N[8]*R[5]) + T[8]*(N[2]*R[6] + N[5]*R[7] + N[8]*R[8]);

//       mat3_copy(tmp, R);
//     }
//     // clang-format on

//     // solution.block<3, 3>(0, 0) = R;
//     // solution.col(3) = C;
//     // clang-format off
//     solutions[i][0] = R[0];
//     solutions[i][1] = R[1];
//     solutions[i][2]  = R[2];
//     solutions[i][3] = C[0];
//     solutions[i][4] = R[3];
//     solutions[i][5] = R[4];
//     solutions[i][6]  = R[5];
//     solutions[i][7] = C[1];
//     solutions[i][8] = R[3];
//     solutions[i][9] = R[4];
//     solutions[i][10] = R[5];
//     solutions[i][11] = C[2];
//     solutions[i][12] = 0.0;
//     solutions[i][13] = 0.0;
//     solutions[i][14] = 0.0;
//     solutions[i][15] = 1.0;
//     // clang-format on
//   }

//   return 0;
// }

static real_t *_solvepnp_residuals(const real_t *proj_params,
                                   const real_t *img_pts,
                                   const real_t *obj_pts,
                                   const int N,
                                   real_t *param) {
  POSE2TF(param, T_FC_est);
  TF_INV(T_FC_est, T_CF_est);
  real_t *r = malloc(sizeof(real_t) * 2 * N);

  for (int n = 0; n < N; n++) {
    // Calculate residual
    real_t z[2] = {img_pts[n * 2 + 0], img_pts[n * 2 + 1]};
    real_t p_F[3] = {obj_pts[n * 3 + 0],
                     obj_pts[n * 3 + 1],
                     obj_pts[n * 3 + 2]};
    TF_POINT(T_CF_est, p_F, p_C);
    real_t zhat[2] = {0};
    pinhole_project(proj_params, p_C, zhat);
    real_t res[2] = {z[0] - zhat[0], z[1] - zhat[1]};

    // Form R.H.S.Gauss Newton g
    r[n * 2 + 0] = res[0];
    r[n * 2 + 1] = res[1];
  }

  return r;
}

static real_t _solvepnp_cost(const real_t *proj_params,
                             const real_t *img_pts,
                             const real_t *obj_pts,
                             const int N,
                             real_t *param) {
  real_t *r = _solvepnp_residuals(proj_params, img_pts, obj_pts, N, param);
  real_t cost = 0;
  dot(r, 1, 2 * N, r, 2 * N, 1, &cost);
  free(r);

  return 0.5 * cost;
}

static void _solvepnp_linearize(const real_t *proj_params,
                                const real_t *img_pts,
                                const real_t *obj_pts,
                                const int N,
                                const real_t *param,
                                real_t *H,
                                real_t *g) {
  // Form Gauss-Newton system
  POSE2TF(param, T_FC_est);
  TF_INV(T_FC_est, T_CF_est);
  zeros(H, 6, 6);
  zeros(g, 6, 1);

  for (int i = 0; i < N; i++) {
    // Calculate residual
    const real_t z[2] = {img_pts[i * 2 + 0], img_pts[i * 2 + 1]};
    const real_t p_F[3] = {obj_pts[i * 3 + 0],
                           obj_pts[i * 3 + 1],
                           obj_pts[i * 3 + 2]};
    TF_POINT(T_CF_est, p_F, p_C);
    real_t zhat[2] = {0};
    pinhole_project(proj_params, p_C, zhat);
    const real_t r[2] = {z[0] - zhat[0], z[1] - zhat[1]};

    // Calculate Jacobian
    TF_DECOMPOSE(T_FC_est, C_FC, r_FC);
    TF_DECOMPOSE(T_CF_est, C_CF, r_CF);
    // -- Jacobian w.r.t 3D point p_C
    // clang-format off
    const real_t Jp[2 * 3] = {
      1.0 / p_C[2], 0.0, -p_C[0] / (p_C[2] * p_C[2]),
      0.0, 1.0 / p_C[2], -p_C[1] / (p_C[2] * p_C[2])
    };
    // clang-format on
    // -- Jacobian w.r.t 2D point x
    const real_t Jk[2 * 2] = {proj_params[0], 0, 0, proj_params[1]};
    // -- Pinhole projection Jacobian
    // Jh = -1 * Jk @ Jp
    real_t Jh[2 * 3] = {0};
    dot(Jk, 2, 2, Jp, 2, 3, Jh);
    for (int i = 0; i < 6; i++) {
      Jh[i] *= -1.0;
    }
    // -- Jacobian of reprojection w.r.t. pose T_FC
    real_t nC_CF[3 * 3] = {0};
    real_t nC_FC[3 * 3] = {0};
    mat3_copy(C_CF, nC_CF);
    mat3_copy(C_FC, nC_FC);
    mat_scale(nC_CF, 3, 3, -1.0);
    mat_scale(nC_FC, 3, 3, -1.0);
    // -- J_pos = Jh * -C_CF
    real_t J_pos[2 * 3] = {0};
    dot(Jh, 2, 3, nC_CF, 3, 3, J_pos);
    // -- J_rot = Jh * -C_CF * hat(p_F - r_FC) * -C_FC
    real_t J_rot[2 * 3] = {0};
    real_t A[3 * 3] = {0};
    real_t dp[3] = {0};
    real_t dp_hat[3 * 3] = {0};
    dp[0] = p_F[0] - r_FC[0];
    dp[1] = p_F[1] - r_FC[1];
    dp[2] = p_F[2] - r_FC[2];
    hat(dp, dp_hat);
    dot(dp_hat, 3, 3, nC_FC, 3, 3, A);
    dot(J_pos, 2, 3, A, 3, 3, J_rot);
    // -- J = [J_pos | J_rot]
    real_t J[2 * 6] = {0};
    real_t Jt[6 * 2] = {0};
    J[0] = J_pos[0];
    J[1] = J_pos[1];
    J[2] = J_pos[2];
    J[6] = J_pos[3];
    J[7] = J_pos[4];
    J[8] = J_pos[5];

    J[3] = J_rot[0];
    J[4] = J_rot[1];
    J[5] = J_rot[2];
    J[9] = J_rot[3];
    J[10] = J_rot[4];
    J[11] = J_rot[5];
    mat_transpose(J, 2, 6, Jt);

    // Form Hessian
    // H += J.T * J
    real_t Hi[6 * 6] = {0};
    dot(Jt, 6, 2, J, 2, 6, Hi);
    for (int i = 0; i < 36; i++) {
      H[i] += Hi[i];
    }

    // Form R.H.S. Gauss Newton g
    // g += -J.T @ r
    real_t gi[6] = {0};
    mat_scale(Jt, 6, 2, -1.0);
    dot(Jt, 6, 2, r, 2, 1, gi);
    g[0] += gi[0];
    g[1] += gi[1];
    g[2] += gi[2];
    g[3] += gi[3];
    g[4] += gi[4];
    g[5] += gi[5];
  }
}

static void _solvepnp_solve(real_t lambda_k, real_t *H, real_t *g, real_t *dx) {
  // Damp Hessian: H = H + lambda * I
  for (int i = 0; i < 6; i++) {
    H[(i * 6) + i] += lambda_k;
  }

  // Solve: H * dx = g
  chol_solve(H, g, dx, 6);
}

static void _solvepnp_update(const real_t *param_k,
                             const real_t *dx,
                             real_t *param_kp1) {
  param_kp1[0] = param_k[0];
  param_kp1[1] = param_k[1];
  param_kp1[2] = param_k[2];
  param_kp1[3] = param_k[3];
  param_kp1[4] = param_k[4];
  param_kp1[5] = param_k[5];
  param_kp1[6] = param_k[6];
  pose_update(param_kp1, dx);
}

/**
 * Solve the Perspective-N-Points problem.
 *
 * **IMPORTANT**: This function assumes that object points lie on the plane
 * because the initialization step uses DLT to estimate the homography between
 * camera and planar object, then the relative pose between them is recovered.
 */
int solvepnp(const real_t proj_params[4],
             const real_t *img_pts,
             const real_t *obj_pts,
             const int N,
             real_t T_CO[4 * 4]) {
  assert(proj_params != NULL);
  assert(img_pts != NULL);
  assert(obj_pts != NULL);
  assert(N > 0);
  assert(T_CO != NULL);

  const int verbose = 0;
  const int max_iter = 10;
  const real_t lambda_init = 1e4;
  const real_t lambda_factor = 10.0;
  const real_t dx_threshold = 1e-5;
  const real_t J_threshold = 1e-5;

  // Initialize pose with DLT
  if (homography_pose(proj_params, img_pts, obj_pts, N, T_CO) != 0) {
    return -1;
  }

  TF_INV(T_CO, T_OC);
  TF_VECTOR(T_OC, param_k);
  real_t lambda_k = lambda_init;
  real_t J_k = _solvepnp_cost(proj_params, img_pts, obj_pts, N, param_k);

  real_t H[6 * 6] = {0};
  real_t g[6 * 1] = {0};
  real_t dx[6 * 1] = {0};
  real_t dJ = 0;
  real_t dx_norm = 0;
  real_t J_kp1 = 0;
  real_t param_kp1[7] = {0};

  for (int iter = 0; iter < max_iter; iter++) {
    // Solve
    _solvepnp_linearize(proj_params, img_pts, obj_pts, N, param_k, H, g);
    _solvepnp_solve(lambda_k, H, g, dx);
    _solvepnp_update(param_k, dx, param_kp1);
    J_kp1 = _solvepnp_cost(proj_params, img_pts, obj_pts, N, param_kp1);

    // Accept or reject update
    dJ = J_kp1 - J_k;
    dx_norm = vec_norm(dx, 6);
    if (J_kp1 < J_k) {
      // Accept update
      J_k = J_kp1;
      vec_copy(param_kp1, 7, param_k);
      lambda_k /= lambda_factor;
    } else {
      // Reject update
      lambda_k *= lambda_factor;
    }

    // Display
    if (verbose) {
      printf("iter: %d, ", iter);
      printf("lambda_k: %.2e, ", lambda_k);
      printf("norm(dx): %.2e, ", dx_norm);
      printf("dcost: %.2e, ", dJ);
      printf("cost:  %.2e\n", J_k);
    }

    // Terminate?
    if (J_k < J_threshold) {
      break;
    } else if (dx_threshold > dx_norm) {
      break;
    }
  }

  // // Calculate reprojection errors
  // real_t *r = _solvepnp_residuals(proj_params, img_pts, obj_pts, N, param_kp1);
  // real_t *errors = malloc(sizeof(real_t) * N);
  // for (int i = 0; i < N; i++) {
  //   const real_t x = r[i * 2 + 0];
  //   const real_t y = r[i * 2 + 1];
  //   errors[i] = sqrt(x * x + y * y);
  // }

  // // Calculate RMSE
  // real_t sum = 0.0;
  // real_t sse = 0.0;
  // for (int i = 0; i < N; i++) {
  //   sum += errors[i];
  //   sse += errors[i] * errors[i];
  // }
  // const real_t reproj_rmse = sqrt(sse / N);
  // const real_t reproj_mean = sum / N;
  // const real_t reproj_median = median(errors, N);
  // printf("rmse: %f, mean: %f, median: %f\n", reproj_rmse, reproj_mean, reproj_median);

  // free(r);
  // free(errors);

  return 0;
}

