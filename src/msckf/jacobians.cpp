#include "prototype/msckf/jacobians.hpp"

namespace prototype {

vec3_t p_c2_f_jacobian_wrt_theta1(const vec3_t &p_c1_f,
                                const vec6_t &tau_s,
                                const vec6_t &tau_d,
																const double &Lambda1,
                                const vec3_t &w1,
																const double &Lambda2,
                                const vec3_t &w2,
																const double theta1_offset,
																const double theta2_offset) {
  // clang-format off
  const double px_c1_f = p_c1_f(0);
  const double py_c1_f = p_c1_f(1);
  const double pz_c1_f = p_c1_f(2);
  const double tx1 = tau_s(0);
  const double ty1 = tau_s(1);
  // const double tz1 = tau_s(2);
  const double r1 = tau_s(3);
  const double p1 = tau_s(4);
  const double y1 = tau_s(5);
  // const double tx2 = tau_d(0);
  // const double ty2 = tau_d(1);
  // const double tz2 = tau_d(2);
  const double r2 = tau_d(3);
  const double p2 = tau_d(4);
  const double y2 = tau_d(5);
  const double theta1 = Lambda1 + theta1_offset;
  // const double d1 = w1(0);
  // const double a1 = w1(1);
  const double alpha1 = w1(2);
  const double theta2 = Lambda2 + theta2_offset;
  // const double d2 = w2(0);
  // const double a2 = w2(1);
  const double alpha2 = w2(2);
  vec3_t p_c2_f_jacobian;

  double t2 = std::cos(p2);
  double t3 = std::cos(theta2);
  double t4 = std::cos(y2);
  double t5 = std::cos(theta1);
  double t6 = std::sin(alpha1);
  double t7 = std::sin(y2);
  double t8 = std::cos(alpha1);
  double t9 = std::sin(theta1);
  double t10 = std::sin(theta2);
  double t11 = std::cos(alpha2);
  double t12 = std::cos(r2);
  double t13 = std::sin(r2);
  double t14 = std::sin(alpha2);
  double t15 = std::sin(p2);
  double t16 = std::sin(y1);
  double t17 = std::cos(r1);
  double t18 = std::cos(p1);
  double t19 = std::cos(y1);
  double t20 = std::sin(r1);
  double t21 = std::sin(p1);
  double d0 = px_c1_f * t4;
  double b_d1 = px_c1_f * t4 * t5;
  double b_d2 = px_c1_f * t4 * t6 * t9;
  double d3 = px_c1_f * t4 * t5 * t6 * t11;
  double d4 = px_c1_f * t3 * t7 * t8 * t9 * t13;
  double d5 = px_c1_f * t3 * t5 * t7 * t8 * t13 * t14;
  double d6 = px_c1_f * t3 * t7 * t8 * t9 * t11 * t12 * t16;
  double d7 = px_c1_f * t3 * t5 * t7 * t8 * t11 * t12 * t18 * t19;
  double d8 = ((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((-t2 *
    t3 * t4 * t9 * tx1 + t2 * t3 * t4 * t5 * ty1) - t2 * t4 * t5 * t8 * t10 *
    tx1) + t5 * t6 * t7 * t11 * t13 * tx1) - t5 * t6 * t7 * t12 * t14 * tx1) -
    t7 * t9 * t10 * t11 * t12 * tx1) - t7 * t9 * t10 * t13 * t14 * tx1) - t2 *
    t4 * t8 * t9 * t10 * ty1) + t5 * t7 * t10 * t11 * t12 * ty1) + t6 * t7 * t9 *
    t11 * t13 * ty1) - t6 * t7 * t9 * t12 * t14 * ty1) + t5 * t7 * t10 * t13 *
    t14 * ty1) + px_c1_f * t2 * t3 * t4 * t5 * t16 * t18) - px_c1_f * t2 * t3 *
    t4 * t9 * t18 * t19) + py_c1_f * t2 * t3 * t4 * t5 * t17 * t19) + py_c1_f *
    t2 * t3 * t4 * t9 * t16 * t17) - pz_c1_f * t2 * t3 * t4 * t5 * t19 * t20) -
    pz_c1_f * t2 * t3 * t4 * t9 * t16 * t20) + t3 * t5 * t7 * t8 * t11 * t12 *
    tx1) + t3 * t5 * t7 * t8 * t13 * t14 * tx1) + t4 * t5 * t6 * t11 * t12 * t15
    * tx1) + t4 * t5 * t6 * t13 * t14 * t15 * tx1) + t4 * t9 * t10 * t11 * t13 *
    t15 * tx1) - t4 * t9 * t10 * t12 * t14 * t15 * tx1) + t3 * t7 * t8 * t9 *
    t11 * t12 * ty1) + t3 * t7 * t8 * t9 * t13 * t14 * ty1) + t4 * t6 * t9 * t11
    * t12 * t15 * ty1) - t4 * t5 * t10 * t11 * t13 * t15 * ty1) + t4 * t5 * t10 *
    t12 * t14 * t15 * ty1) + t4 * t6 * t9 * t13 * t14 * t15 * ty1) - px_c1_f *
    t2 * t4 * t5 * t8 * t10 * t18 * t19) - px_c1_f * t2 * t4 * t8 * t9 * t10 *
    t16 * t18) + px_c1_f * t5 * t6 * t7 * t11 * t13 * t18 * t19) + px_c1_f * t5 *
    t7 * t10 * t11 * t12 * t16 * t18) + px_c1_f * t6 * t7 * t9 * t11 * t13 * t16
    * t18) - px_c1_f * t5 * t6 * t7 * t12 * t14 * t18 * t19) - px_c1_f * t6 * t7
    * t9 * t12 * t14 * t16 * t18) + px_c1_f * t5 * t7 * t10 * t13 * t14 * t16 *
    t18) - px_c1_f * t7 * t9 * t10 * t11 * t12 * t18 * t19) - px_c1_f * t7 * t9 *
    t10 * t13 * t14 * t18 * t19) + py_c1_f * t2 * t4 * t5 * t8 * t10 * t16 * t17)
    - py_c1_f * t2 * t4 * t8 * t9 * t10 * t17 * t19) + py_c1_f * t2 * t3 * t4 *
    t5 * t16 * t20 * t21) - py_c1_f * t5 * t6 * t7 * t11 * t13 * t16 * t17) +
    py_c1_f * t5 * t6 * t7 * t12 * t14 * t16 * t17) - py_c1_f * t2 * t3 * t4 *
    t9 * t19 * t20 * t21) + py_c1_f * t5 * t7 * t10 * t11 * t12 * t17 * t19) +
    py_c1_f * t6 * t7 * t9 * t11 * t13 * t17 * t19) + py_c1_f * t7 * t9 * t10 *
    t11 * t12 * t16 * t17) - py_c1_f * t6 * t7 * t9 * t12 * t14 * t17 * t19) +
                        py_c1_f * t5 * t7 * t10 * t13 * t14 * t17 * t19) +
                       py_c1_f * t7 * t9 * t10 * t13 * t14 * t16 * t17) -
                      pz_c1_f * t2 * t4 * t5 * t8 * t10 * t16 * t20) + pz_c1_f *
                     t2 * t3 * t4 * t5 * t16 * t17 * t21) + pz_c1_f * t2 * t4 *
                    t8 * t9 * t10 * t19 * t20) - pz_c1_f * t2 * t3 * t4 * t9 *
                   t17 * t19 * t21) + pz_c1_f * t5 * t6 * t7 * t11 * t13 * t16 *
                  t20) - pz_c1_f * t5 * t6 * t7 * t12 * t14 * t16 * t20) -
                pz_c1_f * t5 * t7 * t10 * t11 * t12 * t19 * t20) - pz_c1_f * t6 *
               t7 * t9 * t11 * t13 * t19 * t20) - pz_c1_f * t7 * t9 * t10 * t11 *
              t12 * t16 * t20) + pz_c1_f * t6 * t7 * t9 * t12 * t14 * t19 * t20)
            - pz_c1_f * t5 * t7 * t10 * t13 * t14 * t19 * t20) - pz_c1_f * t7 *
           t9 * t10 * t13 * t14 * t16 * t20) - t3 * t4 * t5 * t8 * t11 * t13 *
          t15 * tx1) + t3 * t4 * t5 * t8 * t12 * t14 * t15 * tx1) - t3 * t4 * t8
        * t9 * t11 * t13 * t15 * ty1) + t3 * t4 * t8 * t9 * t12 * t14 * t15 *
    ty1;
  p_c2_f_jacobian(0) =
    (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((
    ((((((((((((((((((((((d8 + d7) + d6 * t18) + d5 * t18 * t19) + d4 * t14 *
    t16 * t18) + d3 * t12 * t15 * t18 * t19) + b_d2 * t11 * t12 * t15 * t16 *
    t18) - b_d1 * t10 * t11 * t13 * t15 * t16 * t18) + d0 * t5 * t6 * t13 * t14 *
    t15 * t18 * t19) + px_c1_f * t4 * t5 * t10 * t12 * t14 * t15 * t16 * t18) +
    px_c1_f * t4 * t6 * t9 * t13 * t14 * t15 * t16 * t18) + px_c1_f * t4 * t9 *
    t10 * t11 * t13 * t15 * t18 * t19) - px_c1_f * t4 * t9 * t10 * t12 * t14 *
    t15 * t18 * t19) - py_c1_f * t3 * t5 * t7 * t8 * t11 * t12 * t16 * t17) -
    py_c1_f * t3 * t5 * t7 * t8 * t13 * t14 * t16 * t17) + py_c1_f * t3 * t7 *
    t8 * t9 * t11 * t12 * t17 * t19) - py_c1_f * t4 * t5 * t6 * t11 * t12 * t15 *
    t16 * t17) - py_c1_f * t2 * t4 * t5 * t8 * t10 * t19 * t20 * t21) - py_c1_f *
    t2 * t4 * t8 * t9 * t10 * t16 * t20 * t21) + py_c1_f * t3 * t7 * t8 * t9 *
    t13 * t14 * t17 * t19) - py_c1_f * t4 * t5 * t6 * t13 * t14 * t15 * t16 *
    t17) + py_c1_f * t4 * t6 * t9 * t11 * t12 * t15 * t17 * t19) - py_c1_f * t4 *
    t5 * t10 * t11 * t13 * t15 * t17 * t19) - py_c1_f * t4 * t9 * t10 * t11 *
    t13 * t15 * t16 * t17) + py_c1_f * t4 * t5 * t10 * t12 * t14 * t15 * t17 *
    t19) + py_c1_f * t4 * t6 * t9 * t13 * t14 * t15 * t17 * t19) + py_c1_f * t4 *
    t9 * t10 * t12 * t14 * t15 * t16 * t17) + py_c1_f * t5 * t6 * t7 * t11 * t13
    * t19 * t20 * t21) + py_c1_f * t5 * t7 * t10 * t11 * t12 * t16 * t20 * t21)
    + py_c1_f * t6 * t7 * t9 * t11 * t13 * t16 * t20 * t21) - py_c1_f * t5 * t6 *
    t7 * t12 * t14 * t19 * t20 * t21) - py_c1_f * t6 * t7 * t9 * t12 * t14 * t16
    * t20 * t21) + py_c1_f * t5 * t7 * t10 * t13 * t14 * t16 * t20 * t21) -
    py_c1_f * t7 * t9 * t10 * t11 * t12 * t19 * t20 * t21) - py_c1_f * t7 * t9 *
    t10 * t13 * t14 * t19 * t20 * t21) + pz_c1_f * t3 * t5 * t7 * t8 * t11 * t12
    * t16 * t20) - pz_c1_f * t2 * t4 * t5 * t8 * t10 * t17 * t19 * t21) +
    pz_c1_f * t3 * t5 * t7 * t8 * t13 * t14 * t16 * t20) - pz_c1_f * t2 * t4 *
    t8 * t9 * t10 * t16 * t17 * t21) - pz_c1_f * t3 * t7 * t8 * t9 * t11 * t12 *
    t19 * t20) + pz_c1_f * t4 * t5 * t6 * t11 * t12 * t15 * t16 * t20) - pz_c1_f
    * t3 * t7 * t8 * t9 * t13 * t14 * t19 * t20) + pz_c1_f * t4 * t5 * t6 * t13 *
    t14 * t15 * t16 * t20) - pz_c1_f * t4 * t6 * t9 * t11 * t12 * t15 * t19 *
    t20) + pz_c1_f * t4 * t5 * t10 * t11 * t13 * t15 * t19 * t20) + pz_c1_f * t4
    * t9 * t10 * t11 * t13 * t15 * t16 * t20) - pz_c1_f * t4 * t5 * t10 * t12 *
    t14 * t15 * t19 * t20) + pz_c1_f * t5 * t6 * t7 * t11 * t13 * t17 * t19 *
    t21) + pz_c1_f * t5 * t7 * t10 * t11 * t12 * t16 * t17 * t21) - pz_c1_f * t4
    * t6 * t9 * t13 * t14 * t15 * t19 * t20) - pz_c1_f * t4 * t9 * t10 * t12 *
    t14 * t15 * t16 * t20) + pz_c1_f * t6 * t7 * t9 * t11 * t13 * t16 * t17 *
    t21) - pz_c1_f * t5 * t6 * t7 * t12 * t14 * t17 * t19 * t21) - pz_c1_f * t6 *
    t7 * t9 * t12 * t14 * t16 * t17 * t21) + pz_c1_f * t5 * t7 * t10 * t13 * t14
    * t16 * t17 * t21) - pz_c1_f * t7 * t9 * t10 * t11 * t12 * t17 * t19 * t21)
    - pz_c1_f * t7 * t9 * t10 * t13 * t14 * t17 * t19 * t21) - px_c1_f * t3 * t4
    * t5 * t8 * t11 * t13 * t15 * t18 * t19) - px_c1_f * t3 * t4 * t8 * t9 * t11
    * t13 * t15 * t16 * t18) + px_c1_f * t3 * t4 * t5 * t8 * t12 * t14 * t15 *
    t18 * t19) + px_c1_f * t3 * t4 * t8 * t9 * t12 * t14 * t15 * t16 * t18) +
    py_c1_f * t3 * t4 * t5 * t8 * t11 * t13 * t15 * t16 * t17) - py_c1_f * t3 *
    t4 * t5 * t8 * t12 * t14 * t15 * t16 * t17) - py_c1_f * t3 * t4 * t8 * t9 *
    t11 * t13 * t15 * t17 * t19) + py_c1_f * t3 * t4 * t8 * t9 * t12 * t14 * t15
    * t17 * t19) + py_c1_f * t3 * t5 * t7 * t8 * t11 * t12 * t19 * t20 * t21) +
    py_c1_f * t3 * t7 * t8 * t9 * t11 * t12 * t16 * t20 * t21) + py_c1_f * t3 *
    t5 * t7 * t8 * t13 * t14 * t19 * t20 * t21) + py_c1_f * t3 * t7 * t8 * t9 *
    t13 * t14 * t16 * t20 * t21) + py_c1_f * t4 * t5 * t6 * t11 * t12 * t15 *
    t19 * t20 * t21) + py_c1_f * t4 * t6 * t9 * t11 * t12 * t15 * t16 * t20 *
    t21) - py_c1_f * t4 * t5 * t10 * t11 * t13 * t15 * t16 * t20 * t21) +
    py_c1_f * t4 * t5 * t6 * t13 * t14 * t15 * t19 * t20 * t21) + py_c1_f * t4 *
    t5 * t10 * t12 * t14 * t15 * t16 * t20 * t21) + py_c1_f * t4 * t6 * t9 * t13
    * t14 * t15 * t16 * t20 * t21) + py_c1_f * t4 * t9 * t10 * t11 * t13 * t15 *
    t19 * t20 * t21) - py_c1_f * t4 * t9 * t10 * t12 * t14 * t15 * t19 * t20 *
    t21) - pz_c1_f * t3 * t4 * t5 * t8 * t11 * t13 * t15 * t16 * t20) + pz_c1_f *
    t3 * t4 * t5 * t8 * t12 * t14 * t15 * t16 * t20) + pz_c1_f * t3 * t4 * t8 *
    t9 * t11 * t13 * t15 * t19 * t20) + pz_c1_f * t3 * t5 * t7 * t8 * t11 * t12 *
    t17 * t19 * t21) - pz_c1_f * t3 * t4 * t8 * t9 * t12 * t14 * t15 * t19 * t20)
                      + pz_c1_f * t3 * t7 * t8 * t9 * t11 * t12 * t16 * t17 *
                      t21) + pz_c1_f * t3 * t5 * t7 * t8 * t13 * t14 * t17 * t19
                     * t21) + pz_c1_f * t3 * t7 * t8 * t9 * t13 * t14 * t16 *
                    t17 * t21) + pz_c1_f * t4 * t5 * t6 * t11 * t12 * t15 * t17 *
                   t19 * t21) + pz_c1_f * t4 * t6 * t9 * t11 * t12 * t15 * t16 *
                  t17 * t21) - pz_c1_f * t4 * t5 * t10 * t11 * t13 * t15 * t16 *
                 t17 * t21) + pz_c1_f * t4 * t5 * t6 * t13 * t14 * t15 * t17 *
                t19 * t21) + pz_c1_f * t4 * t5 * t10 * t12 * t14 * t15 * t16 *
               t17 * t21) + pz_c1_f * t4 * t6 * t9 * t13 * t14 * t15 * t16 * t17
              * t21) + pz_c1_f * t4 * t9 * t10 * t11 * t13 * t15 * t17 * t19 *
             t21) - pz_c1_f * t4 * t9 * t10 * t12 * t14 * t15 * t17 * t19 * t21)
           - py_c1_f * t3 * t4 * t5 * t8 * t11 * t13 * t15 * t19 * t20 * t21) -
          py_c1_f * t3 * t4 * t8 * t9 * t11 * t13 * t15 * t16 * t20 * t21) +
         py_c1_f * t3 * t4 * t5 * t8 * t12 * t14 * t15 * t19 * t20 * t21) +
        py_c1_f * t3 * t4 * t8 * t9 * t12 * t14 * t15 * t16 * t20 * t21) -
       pz_c1_f * t3 * t4 * t5 * t8 * t11 * t13 * t15 * t17 * t19 * t21) -
      pz_c1_f * t3 * t4 * t8 * t9 * t11 * t13 * t15 * t16 * t17 * t21) + pz_c1_f
     * t3 * t4 * t5 * t8 * t12 * t14 * t15 * t17 * t19 * t21) + pz_c1_f * t3 *
    t4 * t8 * t9 * t12 * t14 * t15 * t16 * t17 * t21;
  d0 = px_c1_f * t5;
  b_d1 = px_c1_f * t5 * t7;
  b_d2 = px_c1_f * t6 * t7 * t9;
  d3 = px_c1_f * t5 * t6 * t7 * t11;
  d4 = px_c1_f * t3 * t4 * t8 * t9 * t13;
  d5 = px_c1_f * t3 * t4 * t5 * t8 * t13 * t14;
  d6 = px_c1_f * t3 * t4 * t8 * t9 * t11 * t12 * t16;
  d7 = px_c1_f * t3 * t4 * t5 * t8 * t11 * t12 * t18 * t19;
  d8 = ((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((-t2 *
    t3 * t7 * t9 * tx1 + t2 * t3 * t5 * t7 * ty1) - t2 * t5 * t7 * t8 * t10 *
    tx1) - t4 * t5 * t6 * t11 * t13 * tx1) + t4 * t5 * t6 * t12 * t14 * tx1) +
    t4 * t9 * t10 * t11 * t12 * tx1) + t4 * t9 * t10 * t13 * t14 * tx1) - t2 *
    t7 * t8 * t9 * t10 * ty1) - t4 * t5 * t10 * t11 * t12 * ty1) - t4 * t6 * t9 *
    t11 * t13 * ty1) + t4 * t6 * t9 * t12 * t14 * ty1) - t4 * t5 * t10 * t13 *
    t14 * ty1) + px_c1_f * t2 * t3 * t5 * t7 * t16 * t18) - px_c1_f * t2 * t3 *
    t7 * t9 * t18 * t19) + py_c1_f * t2 * t3 * t5 * t7 * t17 * t19) + py_c1_f *
    t2 * t3 * t7 * t9 * t16 * t17) - pz_c1_f * t2 * t3 * t5 * t7 * t19 * t20) -
    pz_c1_f * t2 * t3 * t7 * t9 * t16 * t20) - t3 * t4 * t5 * t8 * t11 * t12 *
    tx1) - t3 * t4 * t5 * t8 * t13 * t14 * tx1) + t5 * t6 * t7 * t11 * t12 * t15
    * tx1) + t5 * t6 * t7 * t13 * t14 * t15 * tx1) + t7 * t9 * t10 * t11 * t13 *
    t15 * tx1) - t7 * t9 * t10 * t12 * t14 * t15 * tx1) - t3 * t4 * t8 * t9 *
    t11 * t12 * ty1) - t3 * t4 * t8 * t9 * t13 * t14 * ty1) + t6 * t7 * t9 * t11
    * t12 * t15 * ty1) - t5 * t7 * t10 * t11 * t13 * t15 * ty1) + t5 * t7 * t10 *
    t12 * t14 * t15 * ty1) + t6 * t7 * t9 * t13 * t14 * t15 * ty1) - px_c1_f *
    t2 * t5 * t7 * t8 * t10 * t18 * t19) - px_c1_f * t2 * t7 * t8 * t9 * t10 *
    t16 * t18) - px_c1_f * t4 * t5 * t6 * t11 * t13 * t18 * t19) - px_c1_f * t4 *
    t5 * t10 * t11 * t12 * t16 * t18) - px_c1_f * t4 * t6 * t9 * t11 * t13 * t16
    * t18) + px_c1_f * t4 * t5 * t6 * t12 * t14 * t18 * t19) + px_c1_f * t4 * t6
    * t9 * t12 * t14 * t16 * t18) - px_c1_f * t4 * t5 * t10 * t13 * t14 * t16 *
    t18) + px_c1_f * t4 * t9 * t10 * t11 * t12 * t18 * t19) + px_c1_f * t4 * t9 *
    t10 * t13 * t14 * t18 * t19) + py_c1_f * t2 * t5 * t7 * t8 * t10 * t16 * t17)
    - py_c1_f * t2 * t7 * t8 * t9 * t10 * t17 * t19) + py_c1_f * t4 * t5 * t6 *
    t11 * t13 * t16 * t17) + py_c1_f * t2 * t3 * t5 * t7 * t16 * t20 * t21) -
    py_c1_f * t4 * t5 * t6 * t12 * t14 * t16 * t17) - py_c1_f * t4 * t5 * t10 *
    t11 * t12 * t17 * t19) - py_c1_f * t4 * t6 * t9 * t11 * t13 * t17 * t19) -
    py_c1_f * t4 * t9 * t10 * t11 * t12 * t16 * t17) - py_c1_f * t2 * t3 * t7 *
    t9 * t19 * t20 * t21) + py_c1_f * t4 * t6 * t9 * t12 * t14 * t17 * t19) -
                        py_c1_f * t4 * t5 * t10 * t13 * t14 * t17 * t19) -
                       py_c1_f * t4 * t9 * t10 * t13 * t14 * t16 * t17) -
                      pz_c1_f * t2 * t5 * t7 * t8 * t10 * t16 * t20) + pz_c1_f *
                     t2 * t3 * t5 * t7 * t16 * t17 * t21) + pz_c1_f * t2 * t7 *
                    t8 * t9 * t10 * t19 * t20) - pz_c1_f * t4 * t5 * t6 * t11 *
                   t13 * t16 * t20) + pz_c1_f * t4 * t5 * t6 * t12 * t14 * t16 *
                  t20) - pz_c1_f * t2 * t3 * t7 * t9 * t17 * t19 * t21) +
                pz_c1_f * t4 * t5 * t10 * t11 * t12 * t19 * t20) + pz_c1_f * t4 *
               t6 * t9 * t11 * t13 * t19 * t20) + pz_c1_f * t4 * t9 * t10 * t11 *
              t12 * t16 * t20) - pz_c1_f * t4 * t6 * t9 * t12 * t14 * t19 * t20)
            + pz_c1_f * t4 * t5 * t10 * t13 * t14 * t19 * t20) + pz_c1_f * t4 *
           t9 * t10 * t13 * t14 * t16 * t20) - t3 * t5 * t7 * t8 * t11 * t13 *
          t15 * tx1) + t3 * t5 * t7 * t8 * t12 * t14 * t15 * tx1) - t3 * t7 * t8
        * t9 * t11 * t13 * t15 * ty1) + t3 * t7 * t8 * t9 * t12 * t14 * t15 *
    ty1;
  p_c2_f_jacobian(1) =
    (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((
    ((((((((((((((((((((((d8 - d7) - d6 * t18) - d5 * t18 * t19) - d4 * t14 *
    t16 * t18) + d3 * t12 * t15 * t18 * t19) + b_d2 * t11 * t12 * t15 * t16 *
    t18) - b_d1 * t10 * t11 * t13 * t15 * t16 * t18) + d0 * t6 * t7 * t13 * t14 *
    t15 * t18 * t19) + px_c1_f * t5 * t7 * t10 * t12 * t14 * t15 * t16 * t18) +
    px_c1_f * t6 * t7 * t9 * t13 * t14 * t15 * t16 * t18) + px_c1_f * t7 * t9 *
    t10 * t11 * t13 * t15 * t18 * t19) - px_c1_f * t7 * t9 * t10 * t12 * t14 *
    t15 * t18 * t19) + py_c1_f * t3 * t4 * t5 * t8 * t11 * t12 * t16 * t17) +
    py_c1_f * t3 * t4 * t5 * t8 * t13 * t14 * t16 * t17) - py_c1_f * t3 * t4 *
    t8 * t9 * t11 * t12 * t17 * t19) - py_c1_f * t3 * t4 * t8 * t9 * t13 * t14 *
    t17 * t19) - py_c1_f * t5 * t6 * t7 * t11 * t12 * t15 * t16 * t17) - py_c1_f
    * t2 * t5 * t7 * t8 * t10 * t19 * t20 * t21) - py_c1_f * t2 * t7 * t8 * t9 *
    t10 * t16 * t20 * t21) - py_c1_f * t5 * t6 * t7 * t13 * t14 * t15 * t16 *
    t17) + py_c1_f * t6 * t7 * t9 * t11 * t12 * t15 * t17 * t19) - py_c1_f * t5 *
    t7 * t10 * t11 * t13 * t15 * t17 * t19) - py_c1_f * t7 * t9 * t10 * t11 *
    t13 * t15 * t16 * t17) - py_c1_f * t4 * t5 * t6 * t11 * t13 * t19 * t20 *
    t21) - py_c1_f * t4 * t5 * t10 * t11 * t12 * t16 * t20 * t21) + py_c1_f * t5
    * t7 * t10 * t12 * t14 * t15 * t17 * t19) - py_c1_f * t4 * t6 * t9 * t11 *
    t13 * t16 * t20 * t21) + py_c1_f * t6 * t7 * t9 * t13 * t14 * t15 * t17 *
    t19) + py_c1_f * t7 * t9 * t10 * t12 * t14 * t15 * t16 * t17) + py_c1_f * t4
    * t5 * t6 * t12 * t14 * t19 * t20 * t21) + py_c1_f * t4 * t6 * t9 * t12 *
    t14 * t16 * t20 * t21) - py_c1_f * t4 * t5 * t10 * t13 * t14 * t16 * t20 *
    t21) + py_c1_f * t4 * t9 * t10 * t11 * t12 * t19 * t20 * t21) + py_c1_f * t4
    * t9 * t10 * t13 * t14 * t19 * t20 * t21) - pz_c1_f * t3 * t4 * t5 * t8 *
    t11 * t12 * t16 * t20) - pz_c1_f * t3 * t4 * t5 * t8 * t13 * t14 * t16 * t20)
    + pz_c1_f * t3 * t4 * t8 * t9 * t11 * t12 * t19 * t20) - pz_c1_f * t2 * t5 *
    t7 * t8 * t10 * t17 * t19 * t21) - pz_c1_f * t2 * t7 * t8 * t9 * t10 * t16 *
    t17 * t21) + pz_c1_f * t3 * t4 * t8 * t9 * t13 * t14 * t19 * t20) + pz_c1_f *
    t5 * t6 * t7 * t11 * t12 * t15 * t16 * t20) - pz_c1_f * t4 * t5 * t6 * t11 *
    t13 * t17 * t19 * t21) - pz_c1_f * t4 * t5 * t10 * t11 * t12 * t16 * t17 *
    t21) + pz_c1_f * t5 * t6 * t7 * t13 * t14 * t15 * t16 * t20) - pz_c1_f * t4 *
    t6 * t9 * t11 * t13 * t16 * t17 * t21) + pz_c1_f * t4 * t5 * t6 * t12 * t14 *
    t17 * t19 * t21) + pz_c1_f * t4 * t6 * t9 * t12 * t14 * t16 * t17 * t21) -
    pz_c1_f * t6 * t7 * t9 * t11 * t12 * t15 * t19 * t20) - pz_c1_f * t4 * t5 *
    t10 * t13 * t14 * t16 * t17 * t21) + pz_c1_f * t5 * t7 * t10 * t11 * t13 *
    t15 * t19 * t20) + pz_c1_f * t7 * t9 * t10 * t11 * t13 * t15 * t16 * t20) -
    pz_c1_f * t5 * t7 * t10 * t12 * t14 * t15 * t19 * t20) + pz_c1_f * t4 * t9 *
    t10 * t11 * t12 * t17 * t19 * t21) - pz_c1_f * t6 * t7 * t9 * t13 * t14 *
    t15 * t19 * t20) - pz_c1_f * t7 * t9 * t10 * t12 * t14 * t15 * t16 * t20) +
    pz_c1_f * t4 * t9 * t10 * t13 * t14 * t17 * t19 * t21) - px_c1_f * t3 * t5 *
    t7 * t8 * t11 * t13 * t15 * t18 * t19) - px_c1_f * t3 * t7 * t8 * t9 * t11 *
    t13 * t15 * t16 * t18) + px_c1_f * t3 * t5 * t7 * t8 * t12 * t14 * t15 * t18
    * t19) + px_c1_f * t3 * t7 * t8 * t9 * t12 * t14 * t15 * t16 * t18) +
    py_c1_f * t3 * t5 * t7 * t8 * t11 * t13 * t15 * t16 * t17) - py_c1_f * t3 *
    t5 * t7 * t8 * t12 * t14 * t15 * t16 * t17) - py_c1_f * t3 * t7 * t8 * t9 *
    t11 * t13 * t15 * t17 * t19) - py_c1_f * t3 * t4 * t5 * t8 * t11 * t12 * t19
    * t20 * t21) - py_c1_f * t3 * t4 * t8 * t9 * t11 * t12 * t16 * t20 * t21) +
    py_c1_f * t3 * t7 * t8 * t9 * t12 * t14 * t15 * t17 * t19) - py_c1_f * t3 *
    t4 * t5 * t8 * t13 * t14 * t19 * t20 * t21) - py_c1_f * t3 * t4 * t8 * t9 *
    t13 * t14 * t16 * t20 * t21) + py_c1_f * t5 * t6 * t7 * t11 * t12 * t15 *
    t19 * t20 * t21) + py_c1_f * t6 * t7 * t9 * t11 * t12 * t15 * t16 * t20 *
    t21) - py_c1_f * t5 * t7 * t10 * t11 * t13 * t15 * t16 * t20 * t21) +
    py_c1_f * t5 * t6 * t7 * t13 * t14 * t15 * t19 * t20 * t21) + py_c1_f * t5 *
    t7 * t10 * t12 * t14 * t15 * t16 * t20 * t21) + py_c1_f * t6 * t7 * t9 * t13
    * t14 * t15 * t16 * t20 * t21) + py_c1_f * t7 * t9 * t10 * t11 * t13 * t15 *
    t19 * t20 * t21) - py_c1_f * t7 * t9 * t10 * t12 * t14 * t15 * t19 * t20 *
    t21) - pz_c1_f * t3 * t5 * t7 * t8 * t11 * t13 * t15 * t16 * t20) - pz_c1_f *
    t3 * t4 * t5 * t8 * t11 * t12 * t17 * t19 * t21) + pz_c1_f * t3 * t5 * t7 *
    t8 * t12 * t14 * t15 * t16 * t20) - pz_c1_f * t3 * t4 * t8 * t9 * t11 * t12 *
    t16 * t17 * t21) - pz_c1_f * t3 * t4 * t5 * t8 * t13 * t14 * t17 * t19 * t21)
                      - pz_c1_f * t3 * t4 * t8 * t9 * t13 * t14 * t16 * t17 *
                      t21) + pz_c1_f * t3 * t7 * t8 * t9 * t11 * t13 * t15 * t19
                     * t20) - pz_c1_f * t3 * t7 * t8 * t9 * t12 * t14 * t15 *
                    t19 * t20) + pz_c1_f * t5 * t6 * t7 * t11 * t12 * t15 * t17 *
                   t19 * t21) + pz_c1_f * t6 * t7 * t9 * t11 * t12 * t15 * t16 *
                  t17 * t21) - pz_c1_f * t5 * t7 * t10 * t11 * t13 * t15 * t16 *
                 t17 * t21) + pz_c1_f * t5 * t6 * t7 * t13 * t14 * t15 * t17 *
                t19 * t21) + pz_c1_f * t5 * t7 * t10 * t12 * t14 * t15 * t16 *
               t17 * t21) + pz_c1_f * t6 * t7 * t9 * t13 * t14 * t15 * t16 * t17
              * t21) + pz_c1_f * t7 * t9 * t10 * t11 * t13 * t15 * t17 * t19 *
             t21) - pz_c1_f * t7 * t9 * t10 * t12 * t14 * t15 * t17 * t19 * t21)
           - py_c1_f * t3 * t5 * t7 * t8 * t11 * t13 * t15 * t19 * t20 * t21) -
          py_c1_f * t3 * t7 * t8 * t9 * t11 * t13 * t15 * t16 * t20 * t21) +
         py_c1_f * t3 * t5 * t7 * t8 * t12 * t14 * t15 * t19 * t20 * t21) +
        py_c1_f * t3 * t7 * t8 * t9 * t12 * t14 * t15 * t16 * t20 * t21) -
       pz_c1_f * t3 * t5 * t7 * t8 * t11 * t13 * t15 * t17 * t19 * t21) -
      pz_c1_f * t3 * t7 * t8 * t9 * t11 * t13 * t15 * t16 * t17 * t21) + pz_c1_f
     * t3 * t5 * t7 * t8 * t12 * t14 * t15 * t17 * t19 * t21) + pz_c1_f * t3 *
    t7 * t8 * t9 * t12 * t14 * t15 * t16 * t17 * t21;
  p_c2_f_jacobian(2) =
    (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((
    (((((((((((((((((t3 * t9 * t15 * tx1 - t3 * t5 * t15 * ty1) + t5 * t8 * t10 *
    t15 * tx1) + t8 * t9 * t10 * t15 * ty1) - px_c1_f * t3 * t5 * t15 * t16 *
    t18) + px_c1_f * t3 * t9 * t15 * t18 * t19) - py_c1_f * t3 * t5 * t15 * t17 *
    t19) - py_c1_f * t3 * t9 * t15 * t16 * t17) + pz_c1_f * t3 * t5 * t15 * t19 *
    t20) + pz_c1_f * t3 * t9 * t15 * t16 * t20) + t2 * t5 * t6 * t11 * t12 * tx1)
    + t2 * t5 * t6 * t13 * t14 * tx1) + t2 * t9 * t10 * t11 * t13 * tx1) - t2 *
    t9 * t10 * t12 * t14 * tx1) + t2 * t6 * t9 * t11 * t12 * ty1) - t2 * t5 *
    t10 * t11 * t13 * ty1) + t2 * t5 * t10 * t12 * t14 * ty1) + t2 * t6 * t9 *
    t13 * t14 * ty1) + px_c1_f * t5 * t8 * t10 * t15 * t18 * t19) + px_c1_f * t8
    * t9 * t10 * t15 * t16 * t18) - py_c1_f * t5 * t8 * t10 * t15 * t16 * t17) +
    py_c1_f * t8 * t9 * t10 * t15 * t17 * t19) - py_c1_f * t3 * t5 * t15 * t16 *
    t20 * t21) + py_c1_f * t3 * t9 * t15 * t19 * t20 * t21) + pz_c1_f * t5 * t8 *
    t10 * t15 * t16 * t20) - pz_c1_f * t3 * t5 * t15 * t16 * t17 * t21) -
    pz_c1_f * t8 * t9 * t10 * t15 * t19 * t20) + pz_c1_f * t3 * t9 * t15 * t17 *
    t19 * t21) - t2 * t3 * t5 * t8 * t11 * t13 * tx1) + t2 * t3 * t5 * t8 * t12 *
    t14 * tx1) - t2 * t3 * t8 * t9 * t11 * t13 * ty1) + t2 * t3 * t8 * t9 * t12 *
    t14 * ty1) + px_c1_f * t2 * t5 * t6 * t11 * t12 * t18 * t19) + px_c1_f * t2 *
    t6 * t9 * t11 * t12 * t16 * t18) - px_c1_f * t2 * t5 * t10 * t11 * t13 * t16
    * t18) + px_c1_f * t2 * t5 * t6 * t13 * t14 * t18 * t19) + px_c1_f * t2 * t5
    * t10 * t12 * t14 * t16 * t18) + px_c1_f * t2 * t6 * t9 * t13 * t14 * t16 *
    t18) + px_c1_f * t2 * t9 * t10 * t11 * t13 * t18 * t19) - px_c1_f * t2 * t9 *
    t10 * t12 * t14 * t18 * t19) - py_c1_f * t2 * t5 * t6 * t11 * t12 * t16 *
    t17) - py_c1_f * t2 * t5 * t6 * t13 * t14 * t16 * t17) + py_c1_f * t2 * t6 *
    t9 * t11 * t12 * t17 * t19) - py_c1_f * t2 * t5 * t10 * t11 * t13 * t17 *
    t19) - py_c1_f * t2 * t9 * t10 * t11 * t13 * t16 * t17) + py_c1_f * t2 * t5 *
    t10 * t12 * t14 * t17 * t19) + py_c1_f * t2 * t6 * t9 * t13 * t14 * t17 *
    t19) + py_c1_f * t2 * t9 * t10 * t12 * t14 * t16 * t17) + py_c1_f * t5 * t8 *
    t10 * t15 * t19 * t20 * t21) + py_c1_f * t8 * t9 * t10 * t15 * t16 * t20 *
    t21) + pz_c1_f * t2 * t5 * t6 * t11 * t12 * t16 * t20) + pz_c1_f * t2 * t5 *
    t6 * t13 * t14 * t16 * t20) - pz_c1_f * t2 * t6 * t9 * t11 * t12 * t19 * t20)
    + pz_c1_f * t2 * t5 * t10 * t11 * t13 * t19 * t20) + pz_c1_f * t2 * t9 * t10
    * t11 * t13 * t16 * t20) - pz_c1_f * t2 * t5 * t10 * t12 * t14 * t19 * t20)
    - pz_c1_f * t2 * t6 * t9 * t13 * t14 * t19 * t20) - pz_c1_f * t2 * t9 * t10 *
    t12 * t14 * t16 * t20) + pz_c1_f * t5 * t8 * t10 * t15 * t17 * t19 * t21) +
    pz_c1_f * t8 * t9 * t10 * t15 * t16 * t17 * t21) - px_c1_f * t2 * t3 * t5 *
    t8 * t11 * t13 * t18 * t19) - px_c1_f * t2 * t3 * t8 * t9 * t11 * t13 * t16 *
    t18) + px_c1_f * t2 * t3 * t5 * t8 * t12 * t14 * t18 * t19) + px_c1_f * t2 *
    t3 * t8 * t9 * t12 * t14 * t16 * t18) + py_c1_f * t2 * t3 * t5 * t8 * t11 *
    t13 * t16 * t17) - py_c1_f * t2 * t3 * t5 * t8 * t12 * t14 * t16 * t17) -
    py_c1_f * t2 * t3 * t8 * t9 * t11 * t13 * t17 * t19) + py_c1_f * t2 * t3 *
    t8 * t9 * t12 * t14 * t17 * t19) + py_c1_f * t2 * t5 * t6 * t11 * t12 * t19 *
    t20 * t21) + py_c1_f * t2 * t6 * t9 * t11 * t12 * t16 * t20 * t21) - py_c1_f
    * t2 * t5 * t10 * t11 * t13 * t16 * t20 * t21) + py_c1_f * t2 * t5 * t6 *
    t13 * t14 * t19 * t20 * t21) + py_c1_f * t2 * t5 * t10 * t12 * t14 * t16 *
    t20 * t21) + py_c1_f * t2 * t6 * t9 * t13 * t14 * t16 * t20 * t21) + py_c1_f
    * t2 * t9 * t10 * t11 * t13 * t19 * t20 * t21) - py_c1_f * t2 * t9 * t10 *
    t12 * t14 * t19 * t20 * t21) - pz_c1_f * t2 * t3 * t5 * t8 * t11 * t13 * t16
                       * t20) + pz_c1_f * t2 * t3 * t5 * t8 * t12 * t14 * t16 *
                      t20) + pz_c1_f * t2 * t3 * t8 * t9 * t11 * t13 * t19 * t20)
                    - pz_c1_f * t2 * t3 * t8 * t9 * t12 * t14 * t19 * t20) +
                   pz_c1_f * t2 * t5 * t6 * t11 * t12 * t17 * t19 * t21) +
                  pz_c1_f * t2 * t6 * t9 * t11 * t12 * t16 * t17 * t21) -
                 pz_c1_f * t2 * t5 * t10 * t11 * t13 * t16 * t17 * t21) +
                pz_c1_f * t2 * t5 * t6 * t13 * t14 * t17 * t19 * t21) + pz_c1_f *
               t2 * t5 * t10 * t12 * t14 * t16 * t17 * t21) + pz_c1_f * t2 * t6 *
              t9 * t13 * t14 * t16 * t17 * t21) + pz_c1_f * t2 * t9 * t10 * t11 *
             t13 * t17 * t19 * t21) - pz_c1_f * t2 * t9 * t10 * t12 * t14 * t17 *
            t19 * t21) - py_c1_f * t2 * t3 * t5 * t8 * t11 * t13 * t19 * t20 *
           t21) - py_c1_f * t2 * t3 * t8 * t9 * t11 * t13 * t16 * t20 * t21) +
         py_c1_f * t2 * t3 * t5 * t8 * t12 * t14 * t19 * t20 * t21) + py_c1_f *
        t2 * t3 * t8 * t9 * t12 * t14 * t16 * t20 * t21) - pz_c1_f * t2 * t3 *
       t5 * t8 * t11 * t13 * t17 * t19 * t21) - pz_c1_f * t2 * t3 * t8 * t9 *
      t11 * t13 * t16 * t17 * t21) + pz_c1_f * t2 * t3 * t5 * t8 * t12 * t14 *
     t17 * t19 * t21) + pz_c1_f * t2 * t3 * t8 * t9 * t12 * t14 * t16 * t17 *
    t21;
  // p_c2_f_jacobian(3) = 0.0;

  return p_c2_f_jacobian;
}

vec3_t p_c2_f_jacobian_wrt_theta2(const vec3_t &p_c1_f,
                                const vec6_t &tau_s,
                                const vec6_t &tau_d,
																const double &Lambda1,
                                const vec3_t &w1,
																const double &Lambda2,
                                const vec3_t &w2,
																const double theta1_offset,
																const double theta2_offset) {
	// clang-format off
  const double px_c1_f = p_c1_f(0);
  const double py_c1_f = p_c1_f(1);
  const double pz_c1_f = p_c1_f(2);
  const double tx1 = tau_s(0);
  const double ty1 = tau_s(1);
  const double tz1 = tau_s(2);
  const double r1 = tau_s(3);
  const double p1 = tau_s(4);
  const double y1 = tau_s(5);
  // const double tx2 = tau_d(0);
  // const double ty2 = tau_d(1);
  // const double tz2 = tau_d(2);
  const double r2 = tau_d(3);
  const double p2 = tau_d(4);
  const double y2 = tau_d(5);
  const double theta1 = Lambda1 + theta1_offset;
  const double d1 = w1(0);
  const double a1 = w1(1);
  const double alpha1 = w1(2);
  const double theta2 = Lambda2 + theta2_offset;
  // const double d2 = w2(0);
  // const double a2 = w2(1);
  const double alpha2 = w2(2);
  vec3_t p_c2_f_jacobian;

  double t2 = std::cos(p2);
  double t3 = std::cos(theta2);
  double t4 = std::cos(y2);
  double t5 = std::sin(alpha1);
  double t6 = std::sin(y2);
  double t7 = std::sin(theta2);
  double t8 = std::cos(alpha2);
  double t9 = std::sin(r2);
  double t10 = std::sin(alpha2);
  double t11 = std::cos(r2);
  double t12 = std::sin(p2);
  double t13 = std::sin(theta1);
  double t14 = std::cos(theta1);
  double t15 = std::cos(alpha1);
  double t16 = std::cos(r1);
  double t17 = std::sin(y1);
  double t18 = std::cos(y1);
  double t19 = std::sin(r1);
  double t20 = std::sin(p1);
  double t21 = std::cos(p1);
  double d0 = py_c1_f * t5;
  double b_d1 = py_c1_f * t3 * t6;
  double b_d2 = py_c1_f * t3 * t6 * t9;
  double d3 = py_c1_f * t3 * t6 * t8 * t11;
  double d4 = py_c1_f * t3 * t6 * t8 * t11 * t13;
  double d5 = py_c1_f * t2 * t3 * t4 * t14 * t15 * t16;
  double d6 = py_c1_f * t2 * t3 * t4 * t13 * t15 * t16 * t17;
  double d7 = ((((((((((((((((((((((((((((((((((((((((((((((((a1 * t2 * t4 * t7 - a1 *
    t3 * t6 * t8 * t11) - a1 * t3 * t6 * t9 * t10) - d1 * t2 * t3 * t4 * t5) -
    t2 * t4 * t7 * t14 * tx1) - t2 * t4 * t7 * t13 * ty1) + t2 * t3 * t4 * t5 *
    tz1) + a1 * t3 * t4 * t8 * t9 * t12) - a1 * t3 * t4 * t10 * t11 * t12) - d1 *
    t5 * t6 * t7 * t8 * t11) - d1 * t5 * t6 * t7 * t9 * t10) - px_c1_f * t2 * t3
    * t4 * t5 * t20) - t2 * t3 * t4 * t13 * t15 * tx1) + t3 * t6 * t8 * t11 *
    t14 * tx1) + t3 * t6 * t9 * t10 * t14 * tx1) + t2 * t3 * t4 * t14 * t15 *
    ty1) + t3 * t6 * t8 * t11 * t13 * ty1) + t3 * t6 * t9 * t10 * t13 * ty1) +
    t5 * t6 * t7 * t8 * t11 * tz1) + t5 * t6 * t7 * t9 * t10 * tz1) + d1 * t4 *
    t5 * t7 * t8 * t9 * t12) - d1 * t4 * t5 * t7 * t10 * t11 * t12) - px_c1_f *
    t5 * t6 * t7 * t8 * t11 * t20) - px_c1_f * t5 * t6 * t7 * t9 * t10 * t20) -
    px_c1_f * t2 * t4 * t7 * t13 * t17 * t21) - px_c1_f * t2 * t4 * t7 * t14 *
    t18 * t21) + py_c1_f * t2 * t3 * t4 * t5 * t19 * t21) - py_c1_f * t2 * t4 *
    t7 * t13 * t16 * t18) + py_c1_f * t2 * t4 * t7 * t14 * t16 * t17) + pz_c1_f *
    t2 * t3 * t4 * t5 * t16 * t21) + pz_c1_f * t2 * t4 * t7 * t13 * t18 * t19) -
    pz_c1_f * t2 * t4 * t7 * t14 * t17 * t19) - t3 * t4 * t8 * t9 * t12 * t14 *
                        tx1) + t3 * t4 * t10 * t11 * t12 * t14 * tx1) - t6 * t7 *
                      t8 * t11 * t13 * t15 * tx1) - t6 * t7 * t9 * t10 * t13 *
                     t15 * tx1) - t3 * t4 * t8 * t9 * t12 * t13 * ty1) + t3 * t4
                   * t10 * t11 * t12 * t13 * ty1) + t6 * t7 * t8 * t11 * t14 *
                  t15 * ty1) + t6 * t7 * t9 * t10 * t14 * t15 * ty1) - t4 * t5 *
                t7 * t8 * t9 * t12 * tz1) + t4 * t5 * t7 * t10 * t11 * t12 * tz1)
              + px_c1_f * t4 * t5 * t7 * t8 * t9 * t12 * t20) - px_c1_f * t4 *
             t5 * t7 * t10 * t11 * t12 * t20) - px_c1_f * t2 * t3 * t4 * t13 *
            t15 * t18 * t21) + px_c1_f * t2 * t3 * t4 * t14 * t15 * t17 * t21) +
          px_c1_f * t3 * t6 * t8 * t11 * t13 * t17 * t21) + px_c1_f * t3 * t6 *
         t9 * t10 * t13 * t17 * t21) + px_c1_f * t3 * t6 * t8 * t11 * t14 * t18 *
        t21) + px_c1_f * t3 * t6 * t9 * t10 * t14 * t18 * t21;
  p_c2_f_jacobian(0) =
    (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((
    ((((((((((((((((((((((d7 + d6) + d5 * t18) + d4 * t16 * t18) - d3 * t14 *
    t16 * t17) + b_d2 * t10 * t13 * t16 * t18) - b_d1 * t9 * t10 * t14 * t16 *
    t17) + d0 * t6 * t7 * t8 * t11 * t19 * t21) + py_c1_f * t5 * t6 * t7 * t9 *
    t10 * t19 * t21) - py_c1_f * t2 * t4 * t7 * t13 * t17 * t19 * t20) - py_c1_f
    * t2 * t4 * t7 * t14 * t18 * t19 * t20) - pz_c1_f * t2 * t3 * t4 * t13 * t15
    * t17 * t19) + pz_c1_f * t5 * t6 * t7 * t8 * t11 * t16 * t21) + pz_c1_f * t5
    * t6 * t7 * t9 * t10 * t16 * t21) - pz_c1_f * t2 * t3 * t4 * t14 * t15 * t18
    * t19) - pz_c1_f * t3 * t6 * t8 * t11 * t13 * t18 * t19) + pz_c1_f * t3 * t6
    * t8 * t11 * t14 * t17 * t19) - pz_c1_f * t3 * t6 * t9 * t10 * t13 * t18 *
    t19) + pz_c1_f * t3 * t6 * t9 * t10 * t14 * t17 * t19) - pz_c1_f * t2 * t4 *
    t7 * t13 * t16 * t17 * t20) - pz_c1_f * t2 * t4 * t7 * t14 * t16 * t18 * t20)
    + t4 * t7 * t8 * t9 * t12 * t13 * t15 * tx1) - t4 * t7 * t10 * t11 * t12 *
    t13 * t15 * tx1) - t4 * t7 * t8 * t9 * t12 * t14 * t15 * ty1) + t4 * t7 *
    t10 * t11 * t12 * t14 * t15 * ty1) - px_c1_f * t3 * t4 * t8 * t9 * t12 * t13
    * t17 * t21) - px_c1_f * t3 * t4 * t8 * t9 * t12 * t14 * t18 * t21) +
    px_c1_f * t3 * t4 * t10 * t11 * t12 * t13 * t17 * t21) + px_c1_f * t3 * t4 *
    t10 * t11 * t12 * t14 * t18 * t21) - px_c1_f * t6 * t7 * t8 * t11 * t13 *
    t15 * t18 * t21) + px_c1_f * t6 * t7 * t8 * t11 * t14 * t15 * t17 * t21) -
    px_c1_f * t6 * t7 * t9 * t10 * t13 * t15 * t18 * t21) + px_c1_f * t6 * t7 *
    t9 * t10 * t14 * t15 * t17 * t21) - py_c1_f * t3 * t4 * t8 * t9 * t12 * t13 *
    t16 * t18) + py_c1_f * t3 * t4 * t8 * t9 * t12 * t14 * t16 * t17) - py_c1_f *
    t4 * t5 * t7 * t8 * t9 * t12 * t19 * t21) + py_c1_f * t3 * t4 * t10 * t11 *
    t12 * t13 * t16 * t18) - py_c1_f * t3 * t4 * t10 * t11 * t12 * t14 * t16 *
    t17) + py_c1_f * t4 * t5 * t7 * t10 * t11 * t12 * t19 * t21) + py_c1_f * t6 *
    t7 * t8 * t11 * t13 * t15 * t16 * t17) + py_c1_f * t6 * t7 * t9 * t10 * t13 *
    t15 * t16 * t17) - py_c1_f * t2 * t3 * t4 * t13 * t15 * t18 * t19 * t20) +
    py_c1_f * t2 * t3 * t4 * t14 * t15 * t17 * t19 * t20) + py_c1_f * t6 * t7 *
    t8 * t11 * t14 * t15 * t16 * t18) + py_c1_f * t6 * t7 * t9 * t10 * t14 * t15
    * t16 * t18) + py_c1_f * t3 * t6 * t8 * t11 * t13 * t17 * t19 * t20) +
    py_c1_f * t3 * t6 * t9 * t10 * t13 * t17 * t19 * t20) + py_c1_f * t3 * t6 *
    t8 * t11 * t14 * t18 * t19 * t20) + py_c1_f * t3 * t6 * t9 * t10 * t14 * t18
    * t19 * t20) - pz_c1_f * t4 * t5 * t7 * t8 * t9 * t12 * t16 * t21) + pz_c1_f
    * t3 * t4 * t8 * t9 * t12 * t13 * t18 * t19) - pz_c1_f * t3 * t4 * t8 * t9 *
    t12 * t14 * t17 * t19) + pz_c1_f * t4 * t5 * t7 * t10 * t11 * t12 * t16 *
    t21) - pz_c1_f * t3 * t4 * t10 * t11 * t12 * t13 * t18 * t19) + pz_c1_f * t3
    * t4 * t10 * t11 * t12 * t14 * t17 * t19) - pz_c1_f * t2 * t3 * t4 * t13 *
    t15 * t16 * t18 * t20) + pz_c1_f * t2 * t3 * t4 * t14 * t15 * t16 * t17 *
    t20) + pz_c1_f * t3 * t6 * t8 * t11 * t13 * t16 * t17 * t20) + pz_c1_f * t3 *
    t6 * t9 * t10 * t13 * t16 * t17 * t20) + pz_c1_f * t3 * t6 * t8 * t11 * t14 *
    t16 * t18 * t20) + pz_c1_f * t3 * t6 * t9 * t10 * t14 * t16 * t18 * t20) -
    pz_c1_f * t6 * t7 * t8 * t11 * t13 * t15 * t17 * t19) - pz_c1_f * t6 * t7 *
    t9 * t10 * t13 * t15 * t17 * t19) - pz_c1_f * t6 * t7 * t8 * t11 * t14 * t15
    * t18 * t19) - pz_c1_f * t6 * t7 * t9 * t10 * t14 * t15 * t18 * t19) +
    px_c1_f * t4 * t7 * t8 * t9 * t12 * t13 * t15 * t18 * t21) - px_c1_f * t4 *
    t7 * t8 * t9 * t12 * t14 * t15 * t17 * t21) - px_c1_f * t4 * t7 * t10 * t11 *
    t12 * t13 * t15 * t18 * t21) + px_c1_f * t4 * t7 * t10 * t11 * t12 * t14 *
    t15 * t17 * t21) - py_c1_f * t4 * t7 * t8 * t9 * t12 * t13 * t15 * t16 * t17)
    - py_c1_f * t4 * t7 * t8 * t9 * t12 * t14 * t15 * t16 * t18) - py_c1_f * t3 *
    t4 * t8 * t9 * t12 * t13 * t17 * t19 * t20) + py_c1_f * t4 * t7 * t10 * t11 *
    t12 * t13 * t15 * t16 * t17) - py_c1_f * t3 * t4 * t8 * t9 * t12 * t14 * t18
    * t19 * t20) + py_c1_f * t4 * t7 * t10 * t11 * t12 * t14 * t15 * t16 * t18)
    + py_c1_f * t3 * t4 * t10 * t11 * t12 * t13 * t17 * t19 * t20) + py_c1_f *
    t3 * t4 * t10 * t11 * t12 * t14 * t18 * t19 * t20) - py_c1_f * t6 * t7 * t8 *
    t11 * t13 * t15 * t18 * t19 * t20) + py_c1_f * t6 * t7 * t8 * t11 * t14 *
    t15 * t17 * t19 * t20) - py_c1_f * t6 * t7 * t9 * t10 * t13 * t15 * t18 *
    t19 * t20) + py_c1_f * t6 * t7 * t9 * t10 * t14 * t15 * t17 * t19 * t20) -
                       pz_c1_f * t3 * t4 * t8 * t9 * t12 * t13 * t16 * t17 * t20)
                      - pz_c1_f * t3 * t4 * t8 * t9 * t12 * t14 * t16 * t18 *
                      t20) + pz_c1_f * t4 * t7 * t8 * t9 * t12 * t13 * t15 * t17
                     * t19) + pz_c1_f * t3 * t4 * t10 * t11 * t12 * t13 * t16 *
                    t17 * t20) + pz_c1_f * t4 * t7 * t8 * t9 * t12 * t14 * t15 *
                   t18 * t19) + pz_c1_f * t3 * t4 * t10 * t11 * t12 * t14 * t16 *
                  t18 * t20) - pz_c1_f * t4 * t7 * t10 * t11 * t12 * t13 * t15 *
                 t17 * t19) - pz_c1_f * t4 * t7 * t10 * t11 * t12 * t14 * t15 *
                t18 * t19) - pz_c1_f * t6 * t7 * t8 * t11 * t13 * t15 * t16 *
               t18 * t20) + pz_c1_f * t6 * t7 * t8 * t11 * t14 * t15 * t16 * t17
              * t20) - pz_c1_f * t6 * t7 * t9 * t10 * t13 * t15 * t16 * t18 *
             t20) + pz_c1_f * t6 * t7 * t9 * t10 * t14 * t15 * t16 * t17 * t20)
           + py_c1_f * t4 * t7 * t8 * t9 * t12 * t13 * t15 * t18 * t19 * t20) -
          py_c1_f * t4 * t7 * t8 * t9 * t12 * t14 * t15 * t17 * t19 * t20) -
         py_c1_f * t4 * t7 * t10 * t11 * t12 * t13 * t15 * t18 * t19 * t20) +
        py_c1_f * t4 * t7 * t10 * t11 * t12 * t14 * t15 * t17 * t19 * t20) +
       pz_c1_f * t4 * t7 * t8 * t9 * t12 * t13 * t15 * t16 * t18 * t20) -
      pz_c1_f * t4 * t7 * t8 * t9 * t12 * t14 * t15 * t16 * t17 * t20) - pz_c1_f
     * t4 * t7 * t10 * t11 * t12 * t13 * t15 * t16 * t18 * t20) + pz_c1_f * t4 *
    t7 * t10 * t11 * t12 * t14 * t15 * t16 * t17 * t20;
  d0 = py_c1_f * t4;
  b_d1 = py_c1_f * t2 * t3;
  b_d2 = py_c1_f * t3 * t4 * t9;
  d3 = py_c1_f * t3 * t4 * t9 * t10;
  d4 = py_c1_f * t3 * t4 * t8 * t11 * t14;
  d5 = py_c1_f * t3 * t4 * t8 * t11 * t13 * t16;
  d6 = py_c1_f * t2 * t3 * t6 * t13 * t15 * t16 * t17;
  d7 = ((((((((((((((((((((((((((((((((((((((((((((((((a1 * t2 * t6 * t7 + a1 *
    t3 * t4 * t8 * t11) + a1 * t3 * t4 * t9 * t10) - d1 * t2 * t3 * t5 * t6) -
    t2 * t6 * t7 * t14 * tx1) - t2 * t6 * t7 * t13 * ty1) + t2 * t3 * t5 * t6 *
    tz1) + a1 * t3 * t6 * t8 * t9 * t12) - a1 * t3 * t6 * t10 * t11 * t12) + d1 *
    t4 * t5 * t7 * t8 * t11) + d1 * t4 * t5 * t7 * t9 * t10) - px_c1_f * t2 * t3
    * t5 * t6 * t20) - t2 * t3 * t6 * t13 * t15 * tx1) - t3 * t4 * t8 * t11 *
    t14 * tx1) - t3 * t4 * t9 * t10 * t14 * tx1) - t3 * t4 * t8 * t11 * t13 *
    ty1) - t3 * t4 * t9 * t10 * t13 * ty1) + t2 * t3 * t6 * t14 * t15 * ty1) -
    t4 * t5 * t7 * t8 * t11 * tz1) - t4 * t5 * t7 * t9 * t10 * tz1) + d1 * t5 *
    t6 * t7 * t8 * t9 * t12) - d1 * t5 * t6 * t7 * t10 * t11 * t12) + px_c1_f *
    t4 * t5 * t7 * t8 * t11 * t20) + px_c1_f * t4 * t5 * t7 * t9 * t10 * t20) -
    px_c1_f * t2 * t6 * t7 * t13 * t17 * t21) - px_c1_f * t2 * t6 * t7 * t14 *
    t18 * t21) + py_c1_f * t2 * t3 * t5 * t6 * t19 * t21) - py_c1_f * t2 * t6 *
    t7 * t13 * t16 * t18) + py_c1_f * t2 * t6 * t7 * t14 * t16 * t17) + pz_c1_f *
    t2 * t3 * t5 * t6 * t16 * t21) + pz_c1_f * t2 * t6 * t7 * t13 * t18 * t19) -
    pz_c1_f * t2 * t6 * t7 * t14 * t17 * t19) - t3 * t6 * t8 * t9 * t12 * t14 *
                        tx1) + t3 * t6 * t10 * t11 * t12 * t14 * tx1) + t4 * t7 *
                      t8 * t11 * t13 * t15 * tx1) + t4 * t7 * t9 * t10 * t13 *
                     t15 * tx1) - t3 * t6 * t8 * t9 * t12 * t13 * ty1) + t3 * t6
                   * t10 * t11 * t12 * t13 * ty1) - t4 * t7 * t8 * t11 * t14 *
                  t15 * ty1) - t4 * t7 * t9 * t10 * t14 * t15 * ty1) - t5 * t6 *
                t7 * t8 * t9 * t12 * tz1) + t5 * t6 * t7 * t10 * t11 * t12 * tz1)
              + px_c1_f * t5 * t6 * t7 * t8 * t9 * t12 * t20) - px_c1_f * t5 *
             t6 * t7 * t10 * t11 * t12 * t20) - px_c1_f * t3 * t4 * t8 * t11 *
            t13 * t17 * t21) - px_c1_f * t3 * t4 * t9 * t10 * t13 * t17 * t21) -
          px_c1_f * t2 * t3 * t6 * t13 * t15 * t18 * t21) + px_c1_f * t2 * t3 *
         t6 * t14 * t15 * t17 * t21) - px_c1_f * t3 * t4 * t8 * t11 * t14 * t18 *
        t21) - px_c1_f * t3 * t4 * t9 * t10 * t14 * t18 * t21;
  p_c2_f_jacobian(1) =
    (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((
    ((((((((((((((((((((((d7 + d6) - d5 * t18) + d4 * t16 * t17) - d3 * t13 *
    t16 * t18) + b_d2 * t10 * t14 * t16 * t17) + b_d1 * t6 * t14 * t15 * t16 *
    t18) - d0 * t5 * t7 * t8 * t11 * t19 * t21) - py_c1_f * t4 * t5 * t7 * t9 *
    t10 * t19 * t21) - py_c1_f * t2 * t6 * t7 * t13 * t17 * t19 * t20) - py_c1_f
    * t2 * t6 * t7 * t14 * t18 * t19 * t20) - pz_c1_f * t4 * t5 * t7 * t8 * t11 *
    t16 * t21) - pz_c1_f * t4 * t5 * t7 * t9 * t10 * t16 * t21) - pz_c1_f * t2 *
    t3 * t6 * t13 * t15 * t17 * t19) + pz_c1_f * t3 * t4 * t8 * t11 * t13 * t18 *
    t19) - pz_c1_f * t3 * t4 * t8 * t11 * t14 * t17 * t19) + pz_c1_f * t3 * t4 *
    t9 * t10 * t13 * t18 * t19) - pz_c1_f * t3 * t4 * t9 * t10 * t14 * t17 * t19)
    - pz_c1_f * t2 * t3 * t6 * t14 * t15 * t18 * t19) - pz_c1_f * t2 * t6 * t7 *
    t13 * t16 * t17 * t20) - pz_c1_f * t2 * t6 * t7 * t14 * t16 * t18 * t20) +
    t6 * t7 * t8 * t9 * t12 * t13 * t15 * tx1) - t6 * t7 * t10 * t11 * t12 * t13
    * t15 * tx1) - t6 * t7 * t8 * t9 * t12 * t14 * t15 * ty1) + t6 * t7 * t10 *
    t11 * t12 * t14 * t15 * ty1) - px_c1_f * t3 * t6 * t8 * t9 * t12 * t13 * t17
    * t21) - px_c1_f * t3 * t6 * t8 * t9 * t12 * t14 * t18 * t21) + px_c1_f * t3
    * t6 * t10 * t11 * t12 * t13 * t17 * t21) + px_c1_f * t3 * t6 * t10 * t11 *
    t12 * t14 * t18 * t21) + px_c1_f * t4 * t7 * t8 * t11 * t13 * t15 * t18 *
    t21) - px_c1_f * t4 * t7 * t8 * t11 * t14 * t15 * t17 * t21) + px_c1_f * t4 *
    t7 * t9 * t10 * t13 * t15 * t18 * t21) - px_c1_f * t4 * t7 * t9 * t10 * t14 *
    t15 * t17 * t21) - py_c1_f * t3 * t6 * t8 * t9 * t12 * t13 * t16 * t18) +
    py_c1_f * t3 * t6 * t8 * t9 * t12 * t14 * t16 * t17) - py_c1_f * t5 * t6 *
    t7 * t8 * t9 * t12 * t19 * t21) + py_c1_f * t3 * t6 * t10 * t11 * t12 * t13 *
    t16 * t18) - py_c1_f * t3 * t6 * t10 * t11 * t12 * t14 * t16 * t17) -
    py_c1_f * t4 * t7 * t8 * t11 * t13 * t15 * t16 * t17) - py_c1_f * t4 * t7 *
    t9 * t10 * t13 * t15 * t16 * t17) + py_c1_f * t5 * t6 * t7 * t10 * t11 * t12
    * t19 * t21) - py_c1_f * t4 * t7 * t8 * t11 * t14 * t15 * t16 * t18) -
    py_c1_f * t4 * t7 * t9 * t10 * t14 * t15 * t16 * t18) - py_c1_f * t3 * t4 *
    t8 * t11 * t13 * t17 * t19 * t20) - py_c1_f * t3 * t4 * t9 * t10 * t13 * t17
    * t19 * t20) - py_c1_f * t2 * t3 * t6 * t13 * t15 * t18 * t19 * t20) +
    py_c1_f * t2 * t3 * t6 * t14 * t15 * t17 * t19 * t20) - py_c1_f * t3 * t4 *
    t8 * t11 * t14 * t18 * t19 * t20) - py_c1_f * t3 * t4 * t9 * t10 * t14 * t18
    * t19 * t20) - pz_c1_f * t5 * t6 * t7 * t8 * t9 * t12 * t16 * t21) + pz_c1_f
    * t3 * t6 * t8 * t9 * t12 * t13 * t18 * t19) - pz_c1_f * t3 * t6 * t8 * t9 *
    t12 * t14 * t17 * t19) + pz_c1_f * t5 * t6 * t7 * t10 * t11 * t12 * t16 *
    t21) - pz_c1_f * t3 * t4 * t8 * t11 * t13 * t16 * t17 * t20) - pz_c1_f * t3 *
    t4 * t9 * t10 * t13 * t16 * t17 * t20) - pz_c1_f * t3 * t6 * t10 * t11 * t12
    * t13 * t18 * t19) + pz_c1_f * t3 * t6 * t10 * t11 * t12 * t14 * t17 * t19)
    - pz_c1_f * t2 * t3 * t6 * t13 * t15 * t16 * t18 * t20) + pz_c1_f * t2 * t3 *
    t6 * t14 * t15 * t16 * t17 * t20) - pz_c1_f * t3 * t4 * t8 * t11 * t14 * t16
    * t18 * t20) - pz_c1_f * t3 * t4 * t9 * t10 * t14 * t16 * t18 * t20) +
    pz_c1_f * t4 * t7 * t8 * t11 * t13 * t15 * t17 * t19) + pz_c1_f * t4 * t7 *
    t9 * t10 * t13 * t15 * t17 * t19) + pz_c1_f * t4 * t7 * t8 * t11 * t14 * t15
    * t18 * t19) + pz_c1_f * t4 * t7 * t9 * t10 * t14 * t15 * t18 * t19) +
    px_c1_f * t6 * t7 * t8 * t9 * t12 * t13 * t15 * t18 * t21) - px_c1_f * t6 *
    t7 * t8 * t9 * t12 * t14 * t15 * t17 * t21) - px_c1_f * t6 * t7 * t10 * t11 *
    t12 * t13 * t15 * t18 * t21) + px_c1_f * t6 * t7 * t10 * t11 * t12 * t14 *
    t15 * t17 * t21) - py_c1_f * t6 * t7 * t8 * t9 * t12 * t13 * t15 * t16 * t17)
    - py_c1_f * t6 * t7 * t8 * t9 * t12 * t14 * t15 * t16 * t18) - py_c1_f * t3 *
    t6 * t8 * t9 * t12 * t13 * t17 * t19 * t20) + py_c1_f * t6 * t7 * t10 * t11 *
    t12 * t13 * t15 * t16 * t17) - py_c1_f * t3 * t6 * t8 * t9 * t12 * t14 * t18
    * t19 * t20) + py_c1_f * t6 * t7 * t10 * t11 * t12 * t14 * t15 * t16 * t18)
    + py_c1_f * t3 * t6 * t10 * t11 * t12 * t13 * t17 * t19 * t20) + py_c1_f *
    t3 * t6 * t10 * t11 * t12 * t14 * t18 * t19 * t20) + py_c1_f * t4 * t7 * t8 *
    t11 * t13 * t15 * t18 * t19 * t20) - py_c1_f * t4 * t7 * t8 * t11 * t14 *
    t15 * t17 * t19 * t20) + py_c1_f * t4 * t7 * t9 * t10 * t13 * t15 * t18 *
    t19 * t20) - py_c1_f * t4 * t7 * t9 * t10 * t14 * t15 * t17 * t19 * t20) -
                       pz_c1_f * t3 * t6 * t8 * t9 * t12 * t13 * t16 * t17 * t20)
                      - pz_c1_f * t3 * t6 * t8 * t9 * t12 * t14 * t16 * t18 *
                      t20) + pz_c1_f * t6 * t7 * t8 * t9 * t12 * t13 * t15 * t17
                     * t19) + pz_c1_f * t3 * t6 * t10 * t11 * t12 * t13 * t16 *
                    t17 * t20) + pz_c1_f * t6 * t7 * t8 * t9 * t12 * t14 * t15 *
                   t18 * t19) + pz_c1_f * t3 * t6 * t10 * t11 * t12 * t14 * t16 *
                  t18 * t20) - pz_c1_f * t6 * t7 * t10 * t11 * t12 * t13 * t15 *
                 t17 * t19) + pz_c1_f * t4 * t7 * t8 * t11 * t13 * t15 * t16 *
                t18 * t20) - pz_c1_f * t4 * t7 * t8 * t11 * t14 * t15 * t16 *
               t17 * t20) + pz_c1_f * t4 * t7 * t9 * t10 * t13 * t15 * t16 * t18
              * t20) - pz_c1_f * t4 * t7 * t9 * t10 * t14 * t15 * t16 * t17 *
             t20) - pz_c1_f * t6 * t7 * t10 * t11 * t12 * t14 * t15 * t18 * t19)
           + py_c1_f * t6 * t7 * t8 * t9 * t12 * t13 * t15 * t18 * t19 * t20) -
          py_c1_f * t6 * t7 * t8 * t9 * t12 * t14 * t15 * t17 * t19 * t20) -
         py_c1_f * t6 * t7 * t10 * t11 * t12 * t13 * t15 * t18 * t19 * t20) +
        py_c1_f * t6 * t7 * t10 * t11 * t12 * t14 * t15 * t17 * t19 * t20) +
       pz_c1_f * t6 * t7 * t8 * t9 * t12 * t13 * t15 * t16 * t18 * t20) -
      pz_c1_f * t6 * t7 * t8 * t9 * t12 * t14 * t15 * t16 * t17 * t20) - pz_c1_f
     * t6 * t7 * t10 * t11 * t12 * t13 * t15 * t16 * t18 * t20) + pz_c1_f * t6 *
    t7 * t10 * t11 * t12 * t14 * t15 * t16 * t17 * t20;
  p_c2_f_jacobian(2) =
    (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((
    (((((((((((-a1 * t7 * t12 + d1 * t3 * t5 * t12) + t7 * t12 * t14 * tx1) + t7
    * t12 * t13 * ty1) - t3 * t5 * t12 * tz1) + a1 * t2 * t3 * t8 * t9) - a1 *
    t2 * t3 * t10 * t11) + px_c1_f * t3 * t5 * t12 * t20) + t3 * t12 * t13 * t15
    * tx1) - t3 * t12 * t14 * t15 * ty1) + d1 * t2 * t5 * t7 * t8 * t9) - d1 *
    t2 * t5 * t7 * t10 * t11) + px_c1_f * t7 * t12 * t13 * t17 * t21) + px_c1_f *
    t7 * t12 * t14 * t18 * t21) - py_c1_f * t3 * t5 * t12 * t19 * t21) + py_c1_f
    * t7 * t12 * t13 * t16 * t18) - py_c1_f * t7 * t12 * t14 * t16 * t17) -
    pz_c1_f * t3 * t5 * t12 * t16 * t21) - pz_c1_f * t7 * t12 * t13 * t18 * t19)
    + pz_c1_f * t7 * t12 * t14 * t17 * t19) - t2 * t3 * t8 * t9 * t14 * tx1) +
    t2 * t3 * t10 * t11 * t14 * tx1) - t2 * t3 * t8 * t9 * t13 * ty1) + t2 * t3 *
    t10 * t11 * t13 * ty1) - t2 * t5 * t7 * t8 * t9 * tz1) + t2 * t5 * t7 * t10 *
    t11 * tz1) + px_c1_f * t2 * t5 * t7 * t8 * t9 * t20) - px_c1_f * t2 * t5 *
    t7 * t10 * t11 * t20) + px_c1_f * t3 * t12 * t13 * t15 * t18 * t21) -
    px_c1_f * t3 * t12 * t14 * t15 * t17 * t21) - py_c1_f * t3 * t12 * t13 * t15
    * t16 * t17) - py_c1_f * t3 * t12 * t14 * t15 * t16 * t18) + py_c1_f * t7 *
    t12 * t13 * t17 * t19 * t20) + py_c1_f * t7 * t12 * t14 * t18 * t19 * t20) +
    pz_c1_f * t3 * t12 * t13 * t15 * t17 * t19) + pz_c1_f * t3 * t12 * t14 * t15
    * t18 * t19) + pz_c1_f * t7 * t12 * t13 * t16 * t17 * t20) + pz_c1_f * t7 *
    t12 * t14 * t16 * t18 * t20) + t2 * t7 * t8 * t9 * t13 * t15 * tx1) - t2 *
    t7 * t10 * t11 * t13 * t15 * tx1) - t2 * t7 * t8 * t9 * t14 * t15 * ty1) +
    t2 * t7 * t10 * t11 * t14 * t15 * ty1) - px_c1_f * t2 * t3 * t8 * t9 * t13 *
    t17 * t21) - px_c1_f * t2 * t3 * t8 * t9 * t14 * t18 * t21) + px_c1_f * t2 *
    t3 * t10 * t11 * t13 * t17 * t21) + px_c1_f * t2 * t3 * t10 * t11 * t14 *
    t18 * t21) - py_c1_f * t2 * t3 * t8 * t9 * t13 * t16 * t18) + py_c1_f * t2 *
    t3 * t8 * t9 * t14 * t16 * t17) - py_c1_f * t2 * t5 * t7 * t8 * t9 * t19 *
    t21) + py_c1_f * t2 * t3 * t10 * t11 * t13 * t16 * t18) - py_c1_f * t2 * t3 *
    t10 * t11 * t14 * t16 * t17) + py_c1_f * t2 * t5 * t7 * t10 * t11 * t19 *
    t21) + py_c1_f * t3 * t12 * t13 * t15 * t18 * t19 * t20) - py_c1_f * t3 *
    t12 * t14 * t15 * t17 * t19 * t20) - pz_c1_f * t2 * t5 * t7 * t8 * t9 * t16 *
    t21) + pz_c1_f * t2 * t3 * t8 * t9 * t13 * t18 * t19) - pz_c1_f * t2 * t3 *
    t8 * t9 * t14 * t17 * t19) + pz_c1_f * t2 * t5 * t7 * t10 * t11 * t16 * t21)
    - pz_c1_f * t2 * t3 * t10 * t11 * t13 * t18 * t19) + pz_c1_f * t2 * t3 * t10
    * t11 * t14 * t17 * t19) + pz_c1_f * t3 * t12 * t13 * t15 * t16 * t18 * t20)
    - pz_c1_f * t3 * t12 * t14 * t15 * t16 * t17 * t20) + px_c1_f * t2 * t7 * t8
    * t9 * t13 * t15 * t18 * t21) - px_c1_f * t2 * t7 * t8 * t9 * t14 * t15 *
    t17 * t21) - px_c1_f * t2 * t7 * t10 * t11 * t13 * t15 * t18 * t21) +
    px_c1_f * t2 * t7 * t10 * t11 * t14 * t15 * t17 * t21) - py_c1_f * t2 * t7 *
    t8 * t9 * t13 * t15 * t16 * t17) - py_c1_f * t2 * t7 * t8 * t9 * t14 * t15 *
    t16 * t18) - py_c1_f * t2 * t3 * t8 * t9 * t13 * t17 * t19 * t20) + py_c1_f *
    t2 * t7 * t10 * t11 * t13 * t15 * t16 * t17) - py_c1_f * t2 * t3 * t8 * t9 *
                       t14 * t18 * t19 * t20) + py_c1_f * t2 * t7 * t10 * t11 *
                      t14 * t15 * t16 * t18) + py_c1_f * t2 * t3 * t10 * t11 *
                     t13 * t17 * t19 * t20) + py_c1_f * t2 * t3 * t10 * t11 *
                    t14 * t18 * t19 * t20) - pz_c1_f * t2 * t3 * t8 * t9 * t13 *
                   t16 * t17 * t20) - pz_c1_f * t2 * t3 * t8 * t9 * t14 * t16 *
                  t18 * t20) + pz_c1_f * t2 * t7 * t8 * t9 * t13 * t15 * t17 *
                 t19) + pz_c1_f * t2 * t3 * t10 * t11 * t13 * t16 * t17 * t20) +
               pz_c1_f * t2 * t7 * t8 * t9 * t14 * t15 * t18 * t19) + pz_c1_f *
              t2 * t3 * t10 * t11 * t14 * t16 * t18 * t20) - pz_c1_f * t2 * t7 *
             t10 * t11 * t13 * t15 * t17 * t19) - pz_c1_f * t2 * t7 * t10 * t11 *
            t14 * t15 * t18 * t19) + py_c1_f * t2 * t7 * t8 * t9 * t13 * t15 *
           t18 * t19 * t20) - py_c1_f * t2 * t7 * t8 * t9 * t14 * t15 * t17 *
          t19 * t20) - py_c1_f * t2 * t7 * t10 * t11 * t13 * t15 * t18 * t19 *
         t20) + py_c1_f * t2 * t7 * t10 * t11 * t14 * t15 * t17 * t19 * t20) +
       pz_c1_f * t2 * t7 * t8 * t9 * t13 * t15 * t16 * t18 * t20) - pz_c1_f * t2
      * t7 * t8 * t9 * t14 * t15 * t16 * t17 * t20) - pz_c1_f * t2 * t7 * t10 *
     t11 * t13 * t15 * t16 * t18 * t20) + pz_c1_f * t2 * t7 * t10 * t11 * t14 *
    t15 * t16 * t17 * t20;
  // p_c2_f_jacobian(3) = 0.0;

  return p_c2_f_jacobian;
  // clang-format on
}

} //  namespace prototype
