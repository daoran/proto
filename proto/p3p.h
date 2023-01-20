#ifndef P3P_H
#define P3P_H
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <assert.h>

int kneip_p3p(const double features[3][3],
              const double points[3][3],
              double solutions[4][4 * 4]);

#endif // P3P_H

//////////////////////////////////////////////////////////////////////////////
//                             IMPLEMENTATION                               //
//////////////////////////////////////////////////////////////////////////////

#ifdef P3P_IMPLEMENTATION

static void cross3(const double a[3], const double b[3], double c[3]) {
  assert(a != b);
  assert(a != c);

  // cx = ay * bz - az * by
  // cy = az * bx - ax * bz
  // cz = ax * by - ay * bx
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

static double norm3(const double x[3]) {
  return sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
}

static void normalize3(double x[3]) {
  const double n = norm3(x);
  x[0] = x[0] / n;
  x[1] = x[1] / n;
  x[2] = x[2] / n;
}

// static void vec3_copy(const double *src, double *dst) {
//   dst[0] = src[0];
//   dst[1] = src[1];
//   dst[2] = src[2];
// }

// static void vec3_add(const double x[3], const double y[3], double z[3]) {
//   z[0] = x[0] + y[0];
//   z[1] = x[1] + y[1];
//   z[2] = x[2] + y[2];
// }

static void vec3_sub(const double x[3], const double y[3], double z[3]) {
  z[0] = x[0] - y[0];
  z[1] = x[1] - y[1];
  z[2] = x[2] - y[2];
}

static void mat3_copy(const double *src, double *dst) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];

  dst[3] = src[3];
  dst[4] = src[4];
  dst[5] = src[5];

  dst[6] = src[6];
  dst[7] = src[7];
  dst[8] = src[8];
}

static int kneip_solve_quadratic(const double factors[5],
                                 double real_roots[4]) {
  const double A = factors[0];
  const double B = factors[1];
  const double C = factors[2];
  const double D = factors[3];
  const double E = factors[4];

  const double A_pw2 = A * A;
  const double B_pw2 = B * B;
  const double A_pw3 = A_pw2 * A;
  const double B_pw3 = B_pw2 * B;
  const double A_pw4 = A_pw3 * A;
  const double B_pw4 = B_pw3 * B;

  const double alpha = -3 * B_pw2 / (8 * A_pw2) + C / A;
  const double beta = B_pw3 / (8 * A_pw3) - B * C / (2 * A_pw2) + D / A;
  const double gamma = -3 * B_pw4 / (256 * A_pw4) + B_pw2 * C / (16 * A_pw3) -
                       B * D / (4 * A_pw2) + E / A;

  const double alpha_pw2 = alpha * alpha;
  const double alpha_pw3 = alpha_pw2 * alpha;

  const double complex P = (-alpha_pw2 / 12 - gamma);
  const double complex Q =
      -alpha_pw3 / 108 + alpha * gamma / 3 - pow(beta, 2) / 8;
  const double complex R =
      -Q / 2.0 + sqrt(pow(Q, 2.0) / 4.0 + pow(P, 3.0) / 27.0);

  const double complex U = pow(R, (1.0 / 3.0));
  double complex y;
  if (fabs(creal(U)) < 1e-10) {
    y = -5.0 * alpha / 6.0 - pow(Q, (1.0 / 3.0));
  } else {
    y = -5.0 * alpha / 6.0 - P / (3.0 * U) + U;
  }

  const double complex w = sqrt(alpha + 2.0 * y);
  const double m = -B / (4.0 * A);
  const double a = sqrt(-(3.0 * alpha + 2.0 * y + 2.0 * beta / w));
  const double b = sqrt(-(3.0 * alpha + 2.0 * y - 2.0 * beta / w));
  real_roots[0] = creal(m + 0.5 * (w + a));
  real_roots[1] = creal(m + 0.5 * (w - a));
  real_roots[2] = creal(m + 0.5 * (-w + b));
  real_roots[3] = creal(m + 0.5 * (-w - b));

  return 0;
}

int kneip_p3p(const double features[3][3],
              const double points[3][3],
              double solutions[4][4 * 4]) {
  assert(features != NULL);
  assert(points != NULL);
  assert(solutions != NULL);

  // Extract points
  double P1[3] = {points[0][0], points[0][1], points[0][2]};
  double P2[3] = {points[1][0], points[1][1], points[1][2]};
  double P3[3] = {points[2][0], points[2][1], points[2][2]};

  // Verify points are not colinear
  double temp1[3] = {P2[0] - P1[0], P2[1] - P1[1], P2[2] - P2[2]};
  double temp2[3] = {P3[0] - P1[0], P3[1] - P1[1], P3[2] - P2[2]};
  double temp3[3] = {0};
  cross3(temp1, temp2, temp3);
  if (fabs(norm3(temp3)) > 1e-10) {
    return -1;
  }

  // Extract feature vectors
  double f1[3] = {features[0][0], features[0][1], features[0][2]};
  double f2[3] = {features[1][0], features[1][1], features[1][2]};
  double f3[3] = {features[2][0], features[2][1], features[2][2]};

  // Creation of intermediate camera frame
  double e1[3] = {f1[0], f1[1], f1[2]};
  double e3[3] = {0};
  cross3(f1, f2, e3);
  normalize3(e3);
  double e2[3] = {0};
  cross3(e3, e1, e2);

  // clang-format off
  double T[3 * 3] = {
    e1[0], e1[1], e1[2],
    e2[0], e2[1], e2[2],
    e3[0], e3[1], e3[2]
  };
  // clang-format on

  // f3 = T * f3;
  {
    double x[3] = {0};
    x[0] = T[0] * f3[0] + T[1] * f3[1] + T[2] * f3[2];
    x[1] = T[3] * f3[0] + T[4] * f3[1] + T[5] * f3[2];
    x[2] = T[6] * f3[0] + T[7] * f3[1] + T[8] * f3[2];
    f3[0] = x[0];
    f3[1] = x[1];
    f3[2] = x[2];
  }

  // Reinforce that f3(2,0) > 0 for having theta in [0;pi]
  if (f3[2] > 0) {
    // f1 = features.col(1);
    f1[0] = features[0][0];
    f1[1] = features[0][1];
    f1[2] = features[0][2];

    // f2 = features.col(0);
    f2[0] = features[1][0];
    f2[1] = features[1][1];
    f2[2] = features[1][2];

    // f3 = features.col(2);
    f3[0] = features[2][0];
    f3[1] = features[2][1];
    f3[2] = features[2][2];

    // e1 = f1;
    e1[0] = f1[0];
    e1[1] = f1[1];
    e1[2] = f1[2];

    // e3 = f1.cross(f2);
    // e3 = e3 / e3.norm();
    cross3(f1, f2, e3);
    normalize3(e3);

    // e2 = e3.cross(e1);
    cross3(e3, e1, e2);

    // T.row(0) = e1.transpose();
    T[0] = e1[0];
    T[1] = e1[1];
    T[2] = e1[2];

    // T.row(1) = e2.transpose();
    T[3] = e2[0];
    T[4] = e2[1];
    T[5] = e2[2];

    // T.row(2) = e3.transpose();
    T[6] = e3[0];
    T[7] = e3[1];
    T[8] = e3[2];

    // f3 = T * f3;
    {
      double x[3] = {0};
      x[0] = T[0] * f3[0] + T[1] * f3[1] + T[2] * f3[2];
      x[1] = T[3] * f3[0] + T[4] * f3[1] + T[5] * f3[2];
      x[2] = T[6] * f3[0] + T[7] * f3[1] + T[8] * f3[2];
      f3[0] = x[0];
      f3[1] = x[1];
      f3[2] = x[2];
    }

    // P1 = points.col(1);
    P1[0] = points[0][0];
    P1[1] = points[0][1];
    P1[2] = points[0][2];

    // P2 = points.col(0);
    P2[0] = points[1][0];
    P2[1] = points[1][1];
    P2[2] = points[1][2];

    // P3 = points.col(2);
    P3[0] = points[2][0];
    P3[1] = points[2][1];
    P3[2] = points[2][2];
  }

  // Creation of intermediate world frame
  // n1 = P2 - P1;
  // n1 = n1 / n1.norm();
  double n1[3] = {0};
  vec3_sub(P2, P1, n1);
  normalize3(n1);

  // n3 = n1.cross(P3 - P1);
  // n3 = n3 / n3.norm();
  double n3[3] = {0};
  vec3_sub(P3, P1, n3);
  normalize3(n3);

  // n2 = n3.cross(n1);
  double n2[3] = {0};
  cross3(n3, n1, n2);

  // N.row(0) = n1.transpose();
  // N.row(1) = n2.transpose();
  // N.row(2) = n3.transpose();
  // clang-format off
  double N[3 * 3] = {
    n1[0], n1[1], n1[2],
    n2[0], n2[1], n2[2],
    n3[0], n3[1], n3[2]
  };
  // clang-format on

  // Extraction of known parameters
  // P3 = N * (P3 - P1);
  {
    double d[3] = {0};
    vec3_sub(P3, P1, d);
    P3[0] = N[0] * d[0] + N[1] * d[1] + N[2] * d[2];
    P3[1] = N[3] * d[0] + N[4] * d[1] + N[5] * d[2];
    P3[2] = N[6] * d[0] + N[7] * d[1] + N[8] * d[2];
  }

  double dP21[3] = {0};
  vec3_sub(P2, P1, dP21);
  double d_12 = norm3(dP21);
  double f_1 = f3[0] / f3[2];
  double f_2 = f3[1] / f3[2];
  double p_1 = P3[0];
  double p_2 = P3[1];

  // cos_beta = f1.dot(f2);
  // b = 1 / (1 - pow(cos_beta, 2)) - 1;
  const double cos_beta = f1[0] * f2[0] + f1[1] * f2[1] + f1[1] * f2[1];
  double b = 1 / (1 - pow(cos_beta, 2)) - 1;
  if (cos_beta < 0) {
    b = -sqrt(b);
  } else {
    b = sqrt(b);
  }

  // Definition of temporary variables for avoiding multiple computation
  const double f_1_pw2 = pow(f_1, 2);
  const double f_2_pw2 = pow(f_2, 2);
  const double p_1_pw2 = pow(p_1, 2);
  const double p_1_pw3 = p_1_pw2 * p_1;
  const double p_1_pw4 = p_1_pw3 * p_1;
  const double p_2_pw2 = pow(p_2, 2);
  const double p_2_pw3 = p_2_pw2 * p_2;
  const double p_2_pw4 = p_2_pw3 * p_2;
  const double d_12_pw2 = pow(d_12, 2);
  const double b_pw2 = pow(b, 2);

  // Computation of factors of 4th degree polynomial
  double factors[5] = {0};
  factors[0] = -f_2_pw2 * p_2_pw4 - p_2_pw4 * f_1_pw2 - p_2_pw4;
  factors[1] = 2 * p_2_pw3 * d_12 * b + 2 * f_2_pw2 * p_2_pw3 * d_12 * b -
               2 * f_2 * p_2_pw3 * f_1 * d_12;
  factors[2] =
      -f_2_pw2 * p_2_pw2 * p_1_pw2 - f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2 -
      f_2_pw2 * p_2_pw2 * d_12_pw2 + f_2_pw2 * p_2_pw4 + p_2_pw4 * f_1_pw2 +
      2 * p_1 * p_2_pw2 * d_12 + 2 * f_1 * f_2 * p_1 * p_2_pw2 * d_12 * b -
      p_2_pw2 * p_1_pw2 * f_1_pw2 + 2 * p_1 * p_2_pw2 * f_2_pw2 * d_12 -
      p_2_pw2 * d_12_pw2 * b_pw2 - 2 * p_1_pw2 * p_2_pw2;
  factors[3] = 2 * p_1_pw2 * p_2 * d_12 * b + 2 * f_2 * p_2_pw3 * f_1 * d_12 -
               2 * f_2_pw2 * p_2_pw3 * d_12 * b - 2 * p_1 * p_2 * d_12_pw2 * b;
  factors[4] =
      -2 * f_2 * p_2_pw2 * f_1 * p_1 * d_12 * b + f_2_pw2 * p_2_pw2 * d_12_pw2 +
      2 * p_1_pw3 * d_12 - p_1_pw2 * d_12_pw2 + f_2_pw2 * p_2_pw2 * p_1_pw2 -
      p_1_pw4 - 2 * f_2_pw2 * p_2_pw2 * p_1 * d_12 +
      p_2_pw2 * f_1_pw2 * p_1_pw2 + f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2;

  // Computation of roots
  double real_roots[4] = {0};
  kneip_solve_quadratic(factors, real_roots);

  // Backsubstitution of each solution
  for (int i = 0; i < 4; ++i) {
    const double cot_alpha =
        (-f_1 * p_1 / f_2 - real_roots[i] * p_2 + d_12 * b) /
        (-f_1 * real_roots[i] * p_2 / f_2 + p_1 - d_12);
    const double cos_theta = real_roots[i];
    const double sin_theta = sqrt(1 - pow((double) real_roots[i], 2));
    const double sin_alpha = sqrt(1 / (pow(cot_alpha, 2) + 1));
    double cos_alpha = sqrt(1 - pow(sin_alpha, 2));
    if (cot_alpha < 0) {
      cos_alpha = -cos_alpha;
    }

    double C[3] = {0};
    C[0] = d_12 * cos_alpha * (sin_alpha * b + cos_alpha);
    C[1] = cos_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha);
    C[2] = sin_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha);
    // C = P1 + N.transpose() * C;
    C[0] = P1[0] + (N[0] * C[0] + N[3] * C[1] + N[6] * C[2]);
    C[1] = P1[1] + (N[1] * C[0] + N[4] * C[1] + N[7] * C[2]);
    C[2] = P1[2] + (N[2] * C[0] + N[5] * C[1] + N[8] * C[2]);

    double R[3 * 3] = {0};
    R[0] = -cos_alpha;
    R[1] = -sin_alpha * cos_theta;
    R[2] = -sin_alpha * sin_theta;
    R[3] = sin_alpha;
    R[4] = -cos_alpha * cos_theta;
    R[5] = -cos_alpha * sin_theta;
    R[6] = 0;
    R[7] = -sin_theta;
    R[8] = cos_theta;
    // R = N.transpose() * R.transpose() * T;
    // clang-format off
    {
      double tmp[3 * 3] = {0};
      tmp[0] = T[0]*(N[0]*R[0] + N[3]*R[1] + N[6]*R[2]) + T[3]*(N[0]*R[3] + N[3]*R[4] + N[6]*R[5]) + T[6]*(N[0]*R[6] + N[3]*R[7] + N[6]*R[8]);
      tmp[1] = T[1]*(N[0]*R[0] + N[3]*R[1] + N[6]*R[2]) + T[4]*(N[0]*R[3] + N[3]*R[4] + N[6]*R[5]) + T[7]*(N[0]*R[6] + N[3]*R[7] + N[6]*R[8]);
      tmp[2] = T[2]*(N[0]*R[0] + N[3]*R[1] + N[6]*R[2]) + T[5]*(N[0]*R[3] + N[3]*R[4] + N[6]*R[5]) + T[8]*(N[0]*R[6] + N[3]*R[7] + N[6]*R[8]);

      tmp[3] = T[0]*(N[1]*R[0] + N[4]*R[1] + N[7]*R[2]) + T[3]*(N[1]*R[3] + N[4]*R[4] + N[7]*R[5]) + T[6]*(N[1]*R[6] + N[4]*R[7] + N[7]*R[8]);
      tmp[4] = T[1]*(N[1]*R[0] + N[4]*R[1] + N[7]*R[2]) + T[4]*(N[1]*R[3] + N[4]*R[4] + N[7]*R[5]) + T[7]*(N[1]*R[6] + N[4]*R[7] + N[7]*R[8]);
      tmp[5] = T[2]*(N[1]*R[0] + N[4]*R[1] + N[7]*R[2]) + T[5]*(N[1]*R[3] + N[4]*R[4] + N[7]*R[5]) + T[8]*(N[1]*R[6] + N[4]*R[7] + N[7]*R[8]);

      tmp[6] = T[0]*(N[2]*R[0] + N[5]*R[1] + N[8]*R[2]) + T[3]*(N[2]*R[3] + N[5]*R[4] + N[8]*R[5]) + T[6]*(N[2]*R[6] + N[5]*R[7] + N[8]*R[8]);
      tmp[7] = T[1]*(N[2]*R[0] + N[5]*R[1] + N[8]*R[2]) + T[4]*(N[2]*R[3] + N[5]*R[4] + N[8]*R[5]) + T[7]*(N[2]*R[6] + N[5]*R[7] + N[8]*R[8]);
      tmp[8] = T[2]*(N[2]*R[0] + N[5]*R[1] + N[8]*R[2]) + T[5]*(N[2]*R[3] + N[5]*R[4] + N[8]*R[5]) + T[8]*(N[2]*R[6] + N[5]*R[7] + N[8]*R[8]);

      mat3_copy(tmp, R);
    }
    // clang-format on

    // solution.block<3, 3>(0, 0) = R;
    // solution.col(3) = C;
    // clang-format off
    solutions[i][0] = R[0]; solutions[i][1] = R[1]; solutions[i][2]  = R[2]; solutions[i][3] = C[0];
    solutions[i][4] = R[3]; solutions[i][5] = R[4]; solutions[i][6]  = R[5]; solutions[i][7] = C[1];
    solutions[i][8] = R[3]; solutions[i][9] = R[4]; solutions[i][10] = R[5]; solutions[i][11] = C[2];
    solutions[i][12] = 0.0; solutions[i][13] = 0.0; solutions[i][14] = 0.0;  solutions[i][15] = 1.0;
    // clang-format on
  }

  return 0;
}

#endif // P3P_IMPLEMENTATION

//////////////////////////////////////////////////////////////////////////////
//                                UNITTESTS                                 //
//////////////////////////////////////////////////////////////////////////////

#ifdef P3P_UNITTEST

#include <stdio.h>
#include <math.h>

// UNITESTS GLOBAL VARIABLES
static int nb_tests = 0;
static int nb_passed = 0;
static int nb_failed = 0;

#define ENABLE_TERM_COLORS 0
#if ENABLE_TERM_COLORS == 1
#define TERM_RED "\x1B[1;31m"
#define TERM_GRN "\x1B[1;32m"
#define TERM_WHT "\x1B[1;37m"
#define TERM_NRM "\x1B[1;0m"
#else
#define TERM_RED
#define TERM_GRN
#define TERM_WHT
#define TERM_NRM
#endif

/**
 * Run unittests
 * @param[in] test_name Test name
 * @param[in] test_ptr Pointer to unittest
 */
void run_test(const char *test_name, int (*test_ptr)()) {
  if ((*test_ptr)() == 0) {
    printf("-> [%s] " TERM_GRN "OK!\n" TERM_NRM, test_name);
    fflush(stdout);
    nb_passed++;
  } else {
    printf(TERM_RED "FAILED!\n" TERM_NRM);
    fflush(stdout);
    nb_failed++;
  }
  nb_tests++;
}

/**
 * Add unittest
 * @param[in] TEST Test function
 */
#define TEST(TEST_FN) run_test(#TEST_FN, TEST_FN);

/**
 * Unit-test assert
 * @param[in] TEST Test condition
 */
#define TEST_ASSERT(TEST)                                                      \
  do {                                                                         \
    if ((TEST) == 0) {                                                         \
      printf(TERM_RED "ERROR!" TERM_NRM " [%s:%d] %s FAILED!\n",               \
             __func__,                                                         \
             __LINE__,                                                         \
             #TEST);                                                           \
      return -1;                                                               \
    }                                                                          \
  } while (0)

// static int fltcmp(const float x, const float y) {
//   if (fabs(x - y) < 1e-10) {
//     return 0;
//   } else if (x > y) {
//     return 1;
//   }

//   return -1;
// }

int test_kneip_p3p() {

  return 0;
}

int main(int argc, char *argv[]) {
  TEST(test_kneip_p3p);

  return (nb_failed) ? -1 : 0;
}

#endif // P3P_UNITTEST
