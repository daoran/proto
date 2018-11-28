#ifndef PROTOTYPE_TF_HPP
#define PROTOTYPE_TF_HPP

namespace prototype {

// Frames
struct fr_world {};
struct fr_body {};
struct fr_cam0 {};
struct fr_cam1 {};
struct fr_imu0 {};
struct fr_na {};

template <typename FRAME_IN, typename FRAME_OF = fr_na>
struct point_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FRAME_IN fr_in;
  FRAME_OF fr_of;

  vec3_t p = zeros(3, 1);

  point_t() {}
  point_t(const vec3_t &p_) : p{p_} {}
  ~point_t() {}
};

template <typename FRAME_IN, typename FRAME_OF = fr_na>
struct vel_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FRAME_IN fr_in;
  FRAME_OF fr_of;

  vec3_t v = zeros(3, 1);

  vel_t() {}
  vel_t(const vec3_t &v_) : v{v_} {}
  ~vel_t() {}
};

template <typename FRAME_TO, typename FRAME_FROM>
struct tf_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FRAME_TO fr_to;
  FRAME_FROM fr_from;

  mat3_t C = I(3);
  vec3_t r = zeros(3, 1);

  tf_t() {}
  tf_t(const mat3_t &C_, const vec3_t &r_) : C{C_}, r{r_} {}
  ~tf_t() {}

  template <typename POINT_IN, typename POINT_OF>
  point_t<decltype(fr_to), POINT_OF>
      operator *(const point_t<POINT_IN, POINT_OF> &point) {
    static_assert(std::is_same<decltype(fr_from), POINT_IN>::value,
                  "Frames do not match!");
    return point_t<decltype(fr_to), POINT_OF>{C * point.p + r};
  }

  template <typename VEL_IN, typename VEL_OF>
  vel_t<decltype(fr_to), VEL_OF>
      operator *(const vel_t<VEL_IN, VEL_OF> &vel) {
    static_assert(std::is_same<decltype(fr_from), VEL_IN>::value,
                  "Frames do not match!");
    return vel_t<decltype(fr_to), VEL_OF>{C * vel.v + r};
  }
};

template <typename TO, typename FROM>
tf_t<TO, FROM> tf_inv(const tf_t<TO, FROM> &tf) {
  tf_t<TO, FROM> ret = tf;
  ret.C = ret.C.transpose();
  ret.r = -ret.C * ret.r;
  return ret;
}

static void vombie_function() {
  // Transform point in world frame to camera frame
  tf_t<fr_cam0, fr_world> T_CW;
  point_t<fr_world> p_W;
  auto p_C = T_CW * p_W;

  // Transform velocity in body frame to world frame
  tf_t<fr_world, fr_body> T_WB;
  vel_t<fr_body> v_B;
  auto v_W = T_WB * v_B;
}

} //  namespace prototype
#endif // PROTOTYPE_TF_HPP
