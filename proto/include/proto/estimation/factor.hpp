#ifndef PROTO_ESTIMATION_FACTOR_HPP
#define PROTO_ESTIMATION_FACTOR_HPP

#include "proto/core/core.hpp"

using namespace proto;

struct variable_t {
  timestamp_t ts = 0;
  size_t id = 0;

  variable_t() {}
  variable_t(const timestamp_t &ts_, const size_t id_) :
    ts{ts_}, id{id_} {}
  virtual ~variable_t() {}
  virtual double *data() = 0;
};

struct landmark_t : variable_t {
  vec3_t p_W;

  landmark_t(const timestamp_t &ts_, const size_t id_, const vec3_t &p_W_) :
    variable_t{ts_, id_}, p_W{p_W_} {}

  double *data() { return p_W.data(); };
};
typedef std::vector<landmark_t *> landmarks_t;

struct pose_t : variable_t {
  // qw, qx, qy, qz, x, y, z
  double param[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

  pose_t() {}
  pose_t(const timestamp_t &ts_, const size_t id_, const mat4_t &pose_)
      : variable_t{ts_, id_} {
    const quat_t q{pose_.block<3, 3>(0, 0)};
    param[0] = q.w();
    param[1] = q.x();
    param[2] = q.y();
    param[3] = q.z();

    const vec3_t r{pose_.block<3, 1>(0, 3)};
    param[4] = r(0);
    param[5] = r(1);
    param[6] = r(2);
  }
  virtual ~pose_t() {}

  mat3_t rot() {
    const quat_t q{param[0], param[1], param[2], param[3]};
    return q.toRotationMatrix();
  }

  mat4_t T() {
    const quat_t q{param[0], param[1], param[2], param[3]};
    const vec3_t r{param[4], param[5], param[6]};
    return tf(q, r);
  }

  double *data() { return param; }
};

struct factor_t {
  timestamp_t ts = 0;
  size_t id = 0;

  factor_t() {}
  factor_t(const timestamp_t &ts_, const size_t id_) :
    ts{ts_}, id{id_} {}
  virtual ~factor_t() {}
  virtual void eval(vecx_t &r, matx_t &J) const = 0;
};

struct cam_error_t : factor_t {
  vec2s_t z;
  const landmarks_t &landmarks;
  pose_t *sensor_pose = nullptr;
  pose_t *sensor_camera_extrinsic = nullptr;

  cam_error_t(const timestamp_t &ts_,
              const size_t id_,
              const vec2s_t &z_,
              landmarks_t &landmarks_) :
    factor_t{ts_, id_},
    z{z_},
    landmarks{landmarks_} {}

  /**
   * Intrinsics Jacobian: dK / dx_C
   */
  mat2_t intrinsics_point_jacobian(const mat3_t &K) const;

  /**
   * Project Jacobian: dp(p_C) / dp_C
   */
  matx_t project_jacobian(const vec3_t &p_C) const;

  /**
   * Camera Rotation Jacobian: dC_WC / dp_W
   */
  mat3_t camera_rotation_jacobian(const quat_t &q_WC,
                                  const vec3_t &r_WC,
                                  const vec3_t &p_W) const;

  /**
   * Camera Translation Jacobian: dr_WC / dp_W
   */
  mat3_t camera_translation_jacobian(const quat_t &q_WC) const;

  /**
   * Target Point Jacobian: dp_C / dp_W
   */
  mat3_t target_point_jacobian(const quat_t &q_WC) const;

  /**
   * Evaluate
   */
  void eval(vecx_t &r, matx_t &J) const;
};

// struct imu_error_t : factor_t {
//   timestamps_t ts;
//   vec3s_t gyro;
//   vec3s_t accel;
//
//   pose_t *T_WS_0 = nullptr;
//   pose_t *T_WS_1 = nullptr;
//
//   imu_error_t() {}
//   virtual ~imu_error_t() {}
// };

struct graph_t {
  std::map<size_t, landmark_t *> landmarks;
  std::map<size_t, pose_t *> T_WS;
  std::map<size_t, pose_t *> T_SC;
  std::map<int, std::map<size_t, const cam_error_t *>> cam_errors;

  graph_t() {}
  virtual ~graph_t() {}
};

void graph_delete(graph_t &graph);
void graph_set_sensor_camera_extrinsic(graph_t &graph,
                                       const int cam_idx,
                                       const mat4_t &T_SC);
size_t graph_add_cam_error(graph_t &graph,
                           const timestamp_t &ts,
                           const int cam_idx,
                           const vec2s_t &z,
                           const vec3s_t &p_W,
                           const mat4_t &T_WS);
void graph_solve(graph_t &graph, int max_iter=30);

#endif // PROTO_ESTIMATION_FACTOR_HPP
