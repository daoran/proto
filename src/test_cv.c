#include "munit.h"
#include "xyz_cv.h"

int test_image_setup(void) {
  return 0;
}

int test_image_load(void) {
  return 0;
}

int test_image_print_properties(void) {
  return 0;
}

int test_image_free(void) {
  return 0;
}

int test_radtan4_distort(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t p_d[2] = {0};
  radtan4_distort(params, p, p_d);

  // print_vector("p", p, 2);
  // print_vector("p_d", p_d, 2);

  return 0;
}

int test_radtan4_undistort(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};

  real_t p_d[2] = {0};
  real_t p_out[2] = {0};
  radtan4_distort(params, p, p_d);
  radtan4_undistort(params, p_d, p_out);

  // print_vector("p", p, 2);
  // print_vector("p_d", p_d, 2);
  // print_vector("p_out", p_out, 2);
  // printf("dp[0]: %f\n", p[0] - p_out[0]);
  // printf("dp[1]: %f\n", p[1] - p_out[1]);

  MU_ASSERT(fltcmp(p[0], p_out[0]) == 0);
  MU_ASSERT(fltcmp(p[1], p_out[1]) == 0);

  return 0;
}

int test_radtan4_point_jacobian(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t J_point[2 * 2] = {0};
  radtan4_point_jacobian(params, p, J_point);

  /* Calculate numerical diff */
  const real_t step = 1e-4;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 2] = {0};
  {
    real_t p_d[2] = {0};
    radtan4_distort(params, p, p_d);

    for (int i = 0; i < 2; i++) {
      real_t p_diff[2] = {p[0], p[1]};
      p_diff[i] = p[i] + step;

      real_t p_d_prime[2] = {0};
      radtan4_distort(params, p_diff, p_d_prime);

      J_numdiff[i] = (p_d_prime[0] - p_d[0]) / step;
      J_numdiff[i + 2] = (p_d_prime[1] - p_d[1]) / step;
    }
  }

  /* Check jacobian */
  // print_vector("p", p, 2);
  // print_matrix("J_point", J_point, 2, 2);
  // print_matrix("J_numdiff", J_numdiff, 2, 2);
  MU_ASSERT(check_jacobian("J", J_numdiff, J_point, 2, 2, tol, 0) == 0);

  return 0;
}

int test_radtan4_params_jacobian(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t J_param[2 * 4] = {0};
  radtan4_params_jacobian(params, p, J_param);

  /* Calculate numerical diff */
  const real_t step = 1e-4;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 4] = {0};
  {
    real_t p_d[2] = {0};
    radtan4_distort(params, p, p_d);

    for (int i = 0; i < 4; i++) {
      real_t params_diff[4] = {params[0], params[1], params[2], params[3]};
      params_diff[i] = params[i] + step;

      real_t p_d_prime[2] = {0};
      radtan4_distort(params_diff, p, p_d_prime);

      J_numdiff[i] = (p_d_prime[0] - p_d[0]) / step;
      J_numdiff[i + 4] = (p_d_prime[1] - p_d[1]) / step;
    }
  }

  /* Check jacobian */
  // print_vector("p", p, 2);
  // print_matrix("J_param", J_param, 2, 4);
  // print_matrix("J_numdiff", J_numdiff, 2, 4);
  MU_ASSERT(check_jacobian("J", J_numdiff, J_param, 2, 4, tol, 0) == 0);

  return 0;
}

int test_equi4_distort(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t p_d[2] = {0};
  equi4_distort(params, p, p_d);

  // print_vector("p", p, 2);
  // print_vector("p_d", p_d, 2);

  return 0;
}

int test_equi4_undistort(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t p_d[2] = {0};
  real_t p_out[2] = {0};
  equi4_distort(params, p, p_d);
  equi4_undistort(params, p_d, p_out);

  // print_vector("p", p, 2);
  // print_vector("p_d", p_d, 2);
  // print_vector("p_out", p_out, 2);
  // printf("dp[0]: %f\n", p[0] - p_out[0]);
  // printf("dp[1]: %f\n", p[1] - p_out[1]);

  MU_ASSERT(fltcmp(p[0], p_out[0]) == 0);
  MU_ASSERT(fltcmp(p[1], p_out[1]) == 0);
  return 0;
}

int test_equi4_point_jacobian(void) {
  const real_t params[4] = {0.01, 0.001, 0.001, 0.001};
  const real_t p[2] = {0.1, 0.2};
  real_t J_point[2 * 2] = {0};
  equi4_point_jacobian(params, p, J_point);

  /* Calculate numerical diff */
  const real_t step = 1e-4;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 2] = {0};
  {
    real_t p_d[2] = {0};
    equi4_distort(params, p, p_d);

    for (int i = 0; i < 2; i++) {
      real_t p_diff[2] = {p[0], p[1]};
      p_diff[i] = p[i] + step;

      real_t p_d_prime[2] = {0};
      equi4_distort(params, p_diff, p_d_prime);

      J_numdiff[i] = (p_d_prime[0] - p_d[0]) / step;
      J_numdiff[i + 2] = (p_d_prime[1] - p_d[1]) / step;
    }
  }

  /* Check jacobian */
  // print_vector("p", p, 2);
  // print_matrix("J_point", J_point, 2, 2);
  // print_matrix("J_numdiff", J_numdiff, 2, 2);
  MU_ASSERT(check_jacobian("J", J_numdiff, J_point, 2, 2, tol, 0) == 0);

  return 0;
}

int test_equi4_params_jacobian(void) {
  const real_t params[4] = {0.01, 0.01, 0.01, 0.01};
  const real_t p[2] = {0.1, 0.2};
  real_t J_param[2 * 4] = {0};
  equi4_params_jacobian(params, p, J_param);

  /* Calculate numerical diff */
  const real_t step = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 4] = {0};
  {
    real_t p_d[2] = {0};
    equi4_distort(params, p, p_d);

    for (int i = 0; i < 4; i++) {
      real_t params_diff[4] = {params[0], params[1], params[2], params[3]};
      params_diff[i] = params[i] + step;

      real_t p_d_prime[2] = {0};
      equi4_distort(params_diff, p, p_d_prime);

      J_numdiff[i] = (p_d_prime[0] - p_d[0]) / step;
      J_numdiff[i + 4] = (p_d_prime[1] - p_d[1]) / step;
    }
  }

  /* Check jacobian */
  // print_vector("p", p, 2);
  // print_matrix("J_param", J_param, 2, 4);
  // print_matrix("J_numdiff", J_numdiff, 2, 4);
  MU_ASSERT(check_jacobian("J", J_numdiff, J_param, 2, 4, tol, 0) == 0);

  return 0;
}

int test_pinhole_focal(void) {
  const real_t focal = pinhole_focal(640, 90.0);
  MU_ASSERT(fltcmp(focal, 320.0) == 0);
  return 0;
}

int test_pinhole_K(void) {
  const real_t params[4] = {1.0, 2.0, 3.0, 4.0};
  real_t K[3 * 3] = {0};
  pinhole_K(params, K);

  MU_ASSERT(fltcmp(K[0], 1.0) == 0);
  MU_ASSERT(fltcmp(K[1], 0.0) == 0);
  MU_ASSERT(fltcmp(K[2], 3.0) == 0);

  MU_ASSERT(fltcmp(K[3], 0.0) == 0);
  MU_ASSERT(fltcmp(K[4], 2.0) == 0);
  MU_ASSERT(fltcmp(K[5], 4.0) == 0);

  MU_ASSERT(fltcmp(K[6], 0.0) == 0);
  MU_ASSERT(fltcmp(K[7], 0.0) == 0);
  MU_ASSERT(fltcmp(K[8], 1.0) == 0);

  return 0;
}

int test_pinhole_projection_matrix(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t params[4] = {fx, fy, cx, cy};

  /* Camera pose */
  const real_t ypr_WC0[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC0[3] = {0.0, 0.0, 0.0};
  real_t T_WC0[4 * 4] = {0};
  tf_euler_set(T_WC0, ypr_WC0);
  tf_trans_set(T_WC0, r_WC0);

  /* Camera projection matrix */
  real_t P[3 * 4] = {0};
  pinhole_projection_matrix(params, T_WC0, P);

  /* Project point using projection matrix */
  const real_t p_W[3] = {1.0, 0.1, 0.2};
  const real_t hp_W[4] = {p_W[0], p_W[1], p_W[2], 1.0};
  real_t hp[3] = {0};
  dot(P, 3, 4, hp_W, 4, 1, hp);
  real_t z[2] = {hp[0], hp[1]};

  /* Project point by inverting T_WC0 and projecting the point */
  real_t p_C[3] = {0};
  real_t T_C0W[4 * 4] = {0};
  real_t z_gnd[2] = {0};
  tf_inv(T_WC0, T_C0W);
  tf_point(T_C0W, p_W, p_C);
  pinhole_project(params, p_C, z_gnd);

  /* Assert */
  MU_ASSERT(fltcmp(z_gnd[0], z[0]) == 0);
  MU_ASSERT(fltcmp(z_gnd[1], z[1]) == 0);

  return 0;
}

int test_pinhole_project(void) {
  const real_t img_w = 640;
  const real_t img_h = 480;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t params[4] = {fx, fy, cx, cy};
  const real_t p_C[3] = {0.0, 0.0, 1.0};
  real_t z[2] = {0.0, 0.0};
  pinhole_project(params, p_C, z);

  /* print_vector("p_C", p_C, 3); */
  /* print_vector("z", z, 2); */
  MU_ASSERT(fltcmp(z[0], 320.0) == 0);
  MU_ASSERT(fltcmp(z[1], 240.0) == 0);

  return 0;
}

int test_pinhole_point_jacobian(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t params[4] = {fx, fy, cx, cy};

  /* Calculate analytical jacobian */
  real_t J_point[2 * 2] = {0};
  pinhole_point_jacobian(params, J_point);

  /* Numerical differentiation */
  const real_t p_C[3] = {0.1, 0.2, 1.0};
  real_t z[2] = {0};
  pinhole_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 2] = {0};

  for (size_t i = 0; i < 2; i++) {
    real_t z_fd[2] = {0};
    real_t p_C_fd[3] = {p_C[0], p_C[1], p_C[2]};
    p_C_fd[i] += h;
    pinhole_project(params, p_C_fd, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 2] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  MU_ASSERT(check_jacobian("J_point", J_numdiff, J_point, 2, 2, tol, 0) == 0);

  return 0;
}

int test_pinhole_params_jacobian(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t params[4] = {fx, fy, cx, cy};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 1.0};
  const real_t x[2] = {p_C[0] / p_C[2], p_C[1] / p_C[2]};
  real_t J_params[2 * 4] = {0};
  pinhole_params_jacobian(params, x, J_params);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 4] = {0};

  for (size_t i = 0; i < 4; i++) {
    real_t z_fd[2] = {0};
    real_t params_fd[4] = {params[0], params[1], params[2], params[3]};
    params_fd[i] += h;
    pinhole_project(params_fd, p_C, z_fd);

    J_numdiff[i + 0] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 4] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  MU_ASSERT(check_jacobian("J_params", J_numdiff, J_params, 2, 4, tol, 0) == 0);

  return 0;
}

int test_pinhole_radtan4_project(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.3;
  const real_t k2 = 0.01;
  const real_t p1 = 0.01;
  const real_t p2 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, p1, p2};

  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t x[2] = {0};
  pinhole_radtan4_project(params, p_C, x);

  /* print_vector("x", x, 2); */
  MU_ASSERT(fltcmp(x[0], 323.204000) == 0);
  MU_ASSERT(fltcmp(x[1], 166.406400) == 0);

  return 0;
}

int test_pinhole_radtan4_project_jacobian(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.3;
  const real_t k2 = 0.01;
  const real_t p1 = 0.01;
  const real_t p2 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, p1, p2};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t J[2 * 3] = {0};
  pinhole_radtan4_project_jacobian(params, p_C, J);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_radtan4_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 3] = {0};

  for (size_t i = 0; i < 3; i++) {
    real_t z_fd[2] = {0};
    real_t p_C_fd[3] = {p_C[0], p_C[1], p_C[2]};
    p_C_fd[i] += h;

    pinhole_radtan4_project(params, p_C_fd, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 3] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  /* print_matrix("J_numdiff", J_numdiff, 2, 3); */
  /* print_matrix("J", J, 2, 3); */
  MU_ASSERT(check_jacobian("J", J_numdiff, J, 2, 3, tol, 0) == 0);

  return 0;
}

int test_pinhole_radtan4_params_jacobian(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.3;
  const real_t k2 = 0.01;
  const real_t p1 = 0.01;
  const real_t p2 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, p1, p2};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t J_params[2 * 8] = {0};
  pinhole_radtan4_params_jacobian(params, p_C, J_params);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_radtan4_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 8] = {0};

  for (size_t i = 0; i < 8; i++) {
    real_t z_fd[2] = {0};

    real_t params_fd[8] = {0};
    memcpy(params_fd, params, sizeof(real_t) * 8);
    params_fd[i] += h;

    pinhole_radtan4_project(params_fd, p_C, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 8] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  /* print_matrix("J_numdiff", J_numdiff, 2, 8); */
  /* print_matrix("J_params", J_params, 2, 8); */
  MU_ASSERT(check_jacobian("J_params", J_numdiff, J_params, 2, 8, tol, 0) == 0);

  return 0;
}

int test_pinhole_equi4_project(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.1;
  const real_t k2 = 0.01;
  const real_t k3 = 0.01;
  const real_t k4 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, k3, k4};

  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t x[2] = {0};
  pinhole_equi4_project(params, p_C, x);

  /* print_vector("x", x, 2); */
  MU_ASSERT(fltcmp(x[0], 323.199627) == 0);
  MU_ASSERT(fltcmp(x[1], 166.399254) == 0);

  return 0;
}

int test_pinhole_equi4_project_jacobian(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.1;
  const real_t k2 = 0.01;
  const real_t k3 = 0.01;
  const real_t k4 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, k3, k4};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t J[2 * 3] = {0};
  pinhole_equi4_project_jacobian(params, p_C, J);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_equi4_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 3] = {0};

  for (size_t i = 0; i < 3; i++) {
    real_t z_fd[2] = {0};
    real_t p_C_fd[3] = {p_C[0], p_C[1], p_C[2]};
    p_C_fd[i] += h;

    pinhole_equi4_project(params, p_C_fd, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 3] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  /* print_matrix("J_numdiff", J_numdiff, 2, 3); */
  /* print_matrix("J", J, 2, 3); */
  MU_ASSERT(check_jacobian("J", J_numdiff, J, 2, 3, tol, 0) == 0);

  return 0;
}

int test_pinhole_equi4_params_jacobian(void) {
  /* Camera parameters */
  const int img_w = 640;
  const int img_h = 320;
  const real_t fx = pinhole_focal(img_w, 90.0);
  const real_t fy = pinhole_focal(img_w, 90.0);
  const real_t cx = img_w / 2.0;
  const real_t cy = img_h / 2.0;
  const real_t k1 = 0.1;
  const real_t k2 = 0.01;
  const real_t k3 = 0.01;
  const real_t k4 = 0.01;
  const real_t params[8] = {fx, fy, cx, cy, k1, k2, k3, k4};

  /* Calculate analytical jacobian */
  const real_t p_C[3] = {0.1, 0.2, 10.0};
  real_t J_params[2 * 8] = {0};
  pinhole_equi4_params_jacobian(params, p_C, J_params);

  /* Numerical differentiation */
  real_t z[2] = {0};
  pinhole_equi4_project(params, p_C, z);

  const real_t h = 1e-8;
  const real_t tol = 1e-4;
  real_t J_numdiff[2 * 8] = {0};

  for (size_t i = 0; i < 8; i++) {
    real_t z_fd[2] = {0};

    real_t params_fd[8] = {0};
    memcpy(params_fd, params, sizeof(real_t) * 8);
    params_fd[i] += h;

    pinhole_equi4_project(params_fd, p_C, z_fd);
    J_numdiff[i] = (z_fd[0] - z[0]) / h;
    J_numdiff[i + 8] = (z_fd[1] - z[1]) / h;
  }

  /* Assert */
  /* print_matrix("J_numdiff", J_numdiff, 2, 8); */
  /* print_matrix("J_params", J_params, 2, 8); */
  MU_ASSERT(check_jacobian("J_params", J_numdiff, J_params, 2, 8, tol, 0) == 0);

  return 0;
}

int test_linear_triangulation(void) {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2.0;
  const real_t cy = image_height / 2.0;
  const real_t proj_params[4] = {fx, fy, cx, cy};
  real_t K[3 * 3];
  pinhole_K(proj_params, K);

  // Setup camera pose T_WC0
  const real_t ypr_WC0[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC0[3] = {0.0, 0.0, 0.0};
  real_t T_WC0[4 * 4] = {0};
  tf_euler_set(T_WC0, ypr_WC0);
  tf_trans_set(T_WC0, r_WC0);

  // Setup camera pose T_WC1
  const real_t euler_WC1[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC1[3] = {0.1, 0.1, 0.0};
  real_t T_WC1[4 * 4] = {0};
  tf_euler_set(T_WC1, euler_WC1);
  tf_trans_set(T_WC1, r_WC1);

  // Setup projection matrices
  real_t P0[3 * 4] = {0};
  real_t P1[3 * 4] = {0};
  pinhole_projection_matrix(proj_params, T_WC0, P0);
  pinhole_projection_matrix(proj_params, T_WC1, P1);

  // Setup 3D and 2D correspondance points
  int num_tests = 100;
  for (int i = 0; i < num_tests; i++) {
    const real_t p_W[3] = {5.0, randf(-1.0, 1.0), randf(-1.0, 1.0)};

    real_t T_C0W[4 * 4] = {0};
    real_t T_C1W[4 * 4] = {0};
    tf_inv(T_WC0, T_C0W);
    tf_inv(T_WC1, T_C1W);

    real_t p_C0[3] = {0};
    real_t p_C1[3] = {0};
    tf_point(T_C0W, p_W, p_C0);
    tf_point(T_C1W, p_W, p_C1);

    real_t z0[2] = {0};
    real_t z1[2] = {0};
    pinhole_project(proj_params, p_C0, z0);
    pinhole_project(proj_params, p_C1, z1);

    // Test
    real_t p_W_est[3] = {0};
    linear_triangulation(P0, P1, z0, z1, p_W_est);

    // Assert
    real_t diff[3] = {0};
    vec_sub(p_W, p_W_est, diff, 3);
    const real_t norm = vec_norm(diff, 3);
    // print_vector("p_W [gnd]", p_W, 3);
    // print_vector("p_W [est]", p_W_est, 3);
    MU_ASSERT(norm < 1e-4);
    // break;
  }

  return 0;
}

int test_homography_find(void) {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2.0;
  const real_t cy = image_height / 2.0;
  const real_t proj_params[4] = {fx, fy, cx, cy};
  real_t K[3 * 3];
  pinhole_K(proj_params, K);

  // Setup camera pose T_WC0
  const real_t ypr_WC0[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC0[3] = {0.0, 0.0, 0.0};
  real_t T_WC0[4 * 4] = {0};
  tf_euler_set(T_WC0, ypr_WC0);
  tf_trans_set(T_WC0, r_WC0);

  // Setup camera pose T_WC1
  const real_t euler_WC1[3] = {-M_PI / 2.0, 0, -M_PI / 2.0 + 0.3};
  const real_t r_WC1[3] = {0.0, -0.3, 0.0};
  real_t T_WC1[4 * 4] = {0};
  tf_euler_set(T_WC1, euler_WC1);
  tf_trans_set(T_WC1, r_WC1);

  // Setup 3D and 2D correspondance points
  int num_points = 20;
  real_t *pts_i = MALLOC(real_t, num_points * 2);
  real_t *pts_j = MALLOC(real_t, num_points * 2);
  for (int i = 0; i < num_points; i++) {
    const real_t p_W[3] = {3.0, randf(-1.0, 1.0), randf(-1.0, 1.0)};

    real_t T_C0W[4 * 4] = {0};
    real_t T_C1W[4 * 4] = {0};
    tf_inv(T_WC0, T_C0W);
    tf_inv(T_WC1, T_C1W);

    real_t p_C0[3] = {0};
    real_t p_C1[3] = {0};
    tf_point(T_C0W, p_W, p_C0);
    tf_point(T_C1W, p_W, p_C1);

    real_t z0[2] = {0};
    real_t z1[2] = {0};
    pinhole_project(proj_params, p_C0, z0);
    pinhole_project(proj_params, p_C1, z1);
    real_t pt_i[2] = {(z0[0] - cx) / fx, (z0[1] - cy) / fy};
    real_t pt_j[2] = {(z1[0] - cx) / fx, (z1[1] - cy) / fy};

    pts_i[i * 2 + 0] = pt_i[0];
    pts_i[i * 2 + 1] = pt_i[1];
    pts_j[i * 2 + 0] = pt_j[0];
    pts_j[i * 2 + 1] = pt_j[1];
  }

  real_t H[3 * 3] = {0};
  int retval = homography_find(pts_i, pts_j, num_points, H);
  MU_ASSERT(retval == 0);

  for (int i = 0; i < num_points; i++) {
    const real_t p0[3] = {pts_i[i * 2 + 0], pts_i[i * 2 + 1], 1.0};
    const real_t p1[3] = {pts_j[i * 2 + 0], pts_j[i * 2 + 1], 1.0};

    real_t p1_est[3] = {0};
    dot(H, 3, 3, p0, 3, 1, p1_est);
    p1_est[0] /= p1_est[2];
    p1_est[1] /= p1_est[2];
    p1_est[2] /= p1_est[2];

    const real_t dx = p1[0] - p1_est[0];
    const real_t dy = p1[1] - p1_est[1];
    const real_t dz = p1[2] - p1_est[2];
    const real_t diff = sqrt(dx * dx + dy * dy + dz * dz);
    if (diff >= 1e-3) {
      print_vector("p1_gnd", p1, 3);
      print_vector("p1_est", p1_est, 3);
      printf("\n");
    }
    MU_ASSERT(diff < 1e-3);
  }

  // Clean up
  free(pts_i);
  free(pts_j);

  return 0;
}

int test_homography_pose(void) {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2.0;
  const real_t cy = image_height / 2.0;
  const real_t proj_params[4] = {fx, fy, cx, cy};

  // Setup camera pose T_WC
  const real_t ypr_WC[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC[3] = {0.0, 0.0, 0.0};
  real_t T_WC[4 * 4] = {0};
  tf_euler_set(T_WC, ypr_WC);
  tf_trans_set(T_WC, r_WC);

  // Calibration target pose T_WF
  const int num_rows = 4;
  const int num_cols = 4;
  const real_t tag_size = 0.1;
  const real_t target_x = ((num_cols - 1) * tag_size) / 2.0;
  const real_t target_y = -((num_rows - 1) * tag_size) / 2.0;
  const real_t ypr_WF[3] = {-M_PI / 2, 0.0, M_PI / 2};
  const real_t r_WF[3] = {0.5, target_x, target_y};
  TF_ER(ypr_WF, r_WF, T_WF);

  // Setup 3D and 2D correspondance points
  const int N = num_rows * num_cols;
  real_t *world_pts = MALLOC(real_t, N * 3);
  real_t *obj_pts = MALLOC(real_t, N * 3);
  real_t *img_pts = MALLOC(real_t, N * 2);

  int idx = 0;
  for (int i = 0; i < num_rows; i++) {
    for (int j = 0; j < num_cols; j++) {
      const real_t p_F[3] = {i * tag_size, j * tag_size, 0.0};
      TF_POINT(T_WF, p_F, p_W);
      TF_INV(T_WC, T_CW);
      TF_POINT(T_CW, p_W, p_C);

      real_t z[2] = {0};
      pinhole_project(proj_params, p_C, z);

      obj_pts[idx * 3 + 0] = p_F[0];
      obj_pts[idx * 3 + 1] = p_F[1];
      obj_pts[idx * 3 + 2] = p_F[2];

      world_pts[idx * 3 + 0] = p_W[0];
      world_pts[idx * 3 + 1] = p_W[1];
      world_pts[idx * 3 + 2] = p_W[2];

      img_pts[idx * 2 + 0] = z[0];
      img_pts[idx * 2 + 1] = z[1];

      idx++;
    }
  }

  // Find homography pose
  real_t T_CF_est[4 * 4] = {0};
  int retval = homography_pose(proj_params, img_pts, obj_pts, N, T_CF_est);
  MU_ASSERT(retval == 0);

  TF_INV(T_WC, T_CW);
  TF_CHAIN(T_CF_gnd, 2, T_CW, T_WF);
  // print_matrix("T_CF_gnd", T_CF_gnd, 4, 4);
  // print_matrix("T_CF_est", T_CF_est, 4, 4);

  // Cleanup
  free(obj_pts);
  free(world_pts);
  free(img_pts);

  return 0;
}

// int test_p3p_kneip(void) {
//   // Setup camera
//   const int image_width = 640;
//   const int image_height = 480;
//   const real_t fov = 120.0;
//   const real_t fx = pinhole_focal(image_width, fov);
//   const real_t fy = pinhole_focal(image_width, fov);
//   const real_t cx = image_width / 2;
//   const real_t cy = image_height / 2;
//   const real_t proj_params[4] = {fx, fy, cx, cy};
//   real_t K[3 * 3];
//   pinhole_K(proj_params, K);

//   // Setup camera pose T_WC
//   const real_t ypr_WC[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
//   const real_t r_WC[3] = {0.0, 0.0, 0.0};
//   real_t T_WC[4 * 4] = {0};
//   tf_euler_set(T_WC, ypr_WC);
//   tf_trans_set(T_WC, r_WC);
//   TF_INV(T_WC, T_CW);

//   // Setup points
//   real_t features[4][3] = {{1.0, -0.1, 0.1},
//                            {1.0, 0.1, 0.1},
//                            {1.0, 0.1, -0.1},
//                            {1.0, -0.1, -0.1}};
//   real_t points[4][3] = {0};
//   tf_point(T_CW, features[0], points[0]);
//   tf_point(T_CW, features[1], points[1]);
//   tf_point(T_CW, features[2], points[2]);

//   real_t solutions[4][4 * 4];
//   p3p_kneip(features, points, solutions);
//   // printf("retval: %d\n", retval);

//   return 0;
// }

int test_solvepnp(void) {
  // Setup camera
  const int image_width = 640;
  const int image_height = 480;
  const real_t fov = 120.0;
  const real_t fx = pinhole_focal(image_width, fov);
  const real_t fy = pinhole_focal(image_width, fov);
  const real_t cx = image_width / 2.0;
  const real_t cy = image_height / 2.0;
  const real_t proj_params[4] = {fx, fy, cx, cy};

  // Setup camera pose T_WC
  const real_t ypr_WC[3] = {-M_PI / 2.0, 0, -M_PI / 2.0};
  const real_t r_WC[3] = {0.0, 0.0, 0.0};
  real_t T_WC[4 * 4] = {0};
  tf_euler_set(T_WC, ypr_WC);
  tf_trans_set(T_WC, r_WC);

  // Calibration target pose T_WF
  const int num_rows = 4;
  const int num_cols = 4;
  const real_t tag_size = 0.1;
  const real_t target_x = ((num_cols - 1) * tag_size) / 2.0;
  const real_t target_y = -((num_rows - 1) * tag_size) / 2.0;
  const real_t ypr_WF[3] = {-M_PI / 2, 0.0, M_PI / 2};
  const real_t r_WF[3] = {0.5, target_x, target_y};
  TF_ER(ypr_WF, r_WF, T_WF);

  // Setup 3D and 2D correspondance points
  const int N = num_rows * num_cols;
  real_t *world_pts = MALLOC(real_t, N * 3);
  real_t *obj_pts = MALLOC(real_t, N * 3);
  real_t *img_pts = MALLOC(real_t, N * 2);

  int idx = 0;
  for (int i = 0; i < num_rows; i++) {
    for (int j = 0; j < num_cols; j++) {
      const real_t p_F[3] = {i * tag_size, j * tag_size, 0.0};
      TF_POINT(T_WF, p_F, p_W);
      TF_INV(T_WC, T_CW);
      TF_POINT(T_CW, p_W, p_C);

      real_t z[2] = {0};
      pinhole_project(proj_params, p_C, z);

      obj_pts[idx * 3 + 0] = p_F[0];
      obj_pts[idx * 3 + 1] = p_F[1];
      obj_pts[idx * 3 + 2] = p_F[2];

      world_pts[idx * 3 + 0] = p_W[0];
      world_pts[idx * 3 + 1] = p_W[1];
      world_pts[idx * 3 + 2] = p_W[2];

      img_pts[idx * 2 + 0] = z[0];
      img_pts[idx * 2 + 1] = z[1];

      idx++;
    }
  }

  // Find homography pose
  real_t T_CF_est[4 * 4] = {0};
  // struct timespec t_start = tic();
  int retval = solvepnp(proj_params, img_pts, obj_pts, N, T_CF_est);
  MU_ASSERT(retval == 0);
  // printf("time: %f\n", toc(&t_start));

  TF_INV(T_WC, T_CW);
  TF_CHAIN(T_CF_gnd, 2, T_CW, T_WF);

  real_t dr[3] = {0};
  real_t dr_norm = {0};
  real_t dtheta = 0;
  tf_diff2(T_CF_gnd, T_CF_est, dr, &dtheta);
  dr_norm = vec_norm(dr, 3);

  // printf("dr: %f, dtheta: %f\n", dr_norm, dtheta);
  MU_ASSERT(dr_norm < 1e-5);
  MU_ASSERT(dtheta < 1e-5);

  // print_matrix("T_CF_gnd", T_CF_gnd, 4, 4);
  // print_matrix("T_CF_est", T_CF_est, 4, 4);

  // Cleanup
  free(obj_pts);
  free(world_pts);
  free(img_pts);

  return 0;
}

void test_suite(void) {
  MU_ADD_TEST(test_image_setup);
  MU_ADD_TEST(test_image_load);
  MU_ADD_TEST(test_image_print_properties);
  MU_ADD_TEST(test_image_free);
  MU_ADD_TEST(test_radtan4_distort);
  MU_ADD_TEST(test_radtan4_undistort);
  MU_ADD_TEST(test_radtan4_point_jacobian);
  MU_ADD_TEST(test_radtan4_params_jacobian);
  MU_ADD_TEST(test_equi4_distort);
  MU_ADD_TEST(test_equi4_undistort);
  MU_ADD_TEST(test_equi4_point_jacobian);
  MU_ADD_TEST(test_equi4_params_jacobian);
  MU_ADD_TEST(test_pinhole_focal);
  MU_ADD_TEST(test_pinhole_K);
  MU_ADD_TEST(test_pinhole_projection_matrix);
  MU_ADD_TEST(test_pinhole_project);
  MU_ADD_TEST(test_pinhole_point_jacobian);
  MU_ADD_TEST(test_pinhole_params_jacobian);
  MU_ADD_TEST(test_pinhole_radtan4_project);
  MU_ADD_TEST(test_pinhole_radtan4_project_jacobian);
  MU_ADD_TEST(test_pinhole_radtan4_params_jacobian);
  MU_ADD_TEST(test_pinhole_equi4_project);
  MU_ADD_TEST(test_pinhole_equi4_project_jacobian);
  MU_ADD_TEST(test_pinhole_equi4_params_jacobian);
  MU_ADD_TEST(test_linear_triangulation);
  MU_ADD_TEST(test_homography_find);
  MU_ADD_TEST(test_homography_pose);
  // MU_ADD_TEST(test_p3p_kneip);
  MU_ADD_TEST(test_solvepnp);
}
MU_RUN_TESTS(test_suite)
