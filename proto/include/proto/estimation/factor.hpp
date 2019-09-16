#ifndef PROTO_ESTIMATION_FACTOR_HPP
#define PROTO_ESTIMATION_FACTOR_HPP

#include "proto/core/core.hpp"
#include "proto/vision/camera/camera_geometry.hpp"

namespace proto {

/*****************************************************************************
 * Variable
 ****************************************************************************/

struct variable_t {
  timestamp_t ts = 0;
  size_t id = 0;
  size_t size = 0;
  size_t local_size = 0;

  variable_t() {}

  variable_t(const size_t id_, const size_t size_)
    : id{id_}, size{size_}, local_size{size_} {}

  variable_t(const size_t id_,
             const size_t size_,
             const size_t local_size_)
    : id{id_}, size{size_}, local_size{local_size_} {}

  variable_t(const timestamp_t &ts_, const size_t id_, const size_t size_)
    : ts{ts_}, id{id_}, size{size_}, local_size{size_} {}

  variable_t(const timestamp_t &ts_,
             const size_t id_,
             const size_t size_,
             const size_t local_size_)
    : ts{ts_}, id{id_}, size{size_}, local_size{local_size_} {}

  virtual ~variable_t() {}

  virtual double *data() = 0;

  virtual void plus(const vecx_t &) = 0;
};

/*****************************************************************************
 * Landmark Variable
 ****************************************************************************/

struct landmark_t : variable_t {
  double param[3] = {0.0, 0.0, 0.0};

  landmark_t(const size_t id_, const vec3_t &p_W_) :
    variable_t{id_, 3}, param{p_W_(0), p_W_(1), p_W_(2)} {}

  vec3_t vec() { return map_vec_t<3>(param); };

  double *data() { return param; };

  void plus(const vecx_t &delta) {
    param[0] = param[0] + delta[0];
    param[1] = param[1] + delta[1];
    param[2] = param[2] + delta[2];
  }
};
typedef std::vector<landmark_t *> landmarks_t;

/*****************************************************************************
 * Pose Variable
 ****************************************************************************/

struct pose_t : variable_t {
  // (qw, qx, qy, qz), (x, y, z)
  double param[7] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  pose_t() {}
  pose_t(const timestamp_t &ts_, const size_t id_, const mat4_t &pose_)
      : variable_t{ts_, id_, 7, 6} {
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

  void plus(const vecx_t &delta) {
    // Rotation component
    double half_norm = 0.5 * delta.head<3>().norm();
    double dq_w = cos(half_norm);
    double dq_x = sinc(half_norm) * 0.5 * delta(0);
    double dq_y = sinc(half_norm) * 0.5 * delta(1);
    double dq_z = sinc(half_norm) * 0.5 * delta(2);
    quat_t dq{dq_w, dq_x, dq_y, dq_z};
    quat_t q{param[0], param[1], param[2], param[3]};
    quat_t q_updated = q * dq;
    param[0] = q_updated.w();
    param[1] = q_updated.x();
    param[2] = q_updated.y();
    param[3] = q_updated.z();

    // Translation component
    param[3] = param[3] - delta[3];
    param[4] = param[4] - delta[4];
    param[5] = param[5] - delta[5];
  }
};

/*****************************************************************************
 * Factor
 ****************************************************************************/

struct factor_t {
  timestamp_t ts = 0;
  size_t id = 0;
  size_t residual_size = 0;
  std::vector<variable_t *> param_blocks;
  std::vector<size_t> param_sizes;

  factor_t() {}
  factor_t(const timestamp_t &ts_,
           const size_t id_,
           const size_t residual_size_,
           const std::vector<variable_t *> param_blocks_,
           const std::vector<size_t> param_sizes_) :
    ts{ts_}, id{id_}, residual_size{residual_size_},
    param_blocks{param_blocks_}, param_sizes{param_sizes_}  {}
  virtual ~factor_t() {}

  virtual int eval(double *residuals, double **jacobians) const = 0;
};

/*****************************************************************************
 * Bundle Adjustment Factor
 ****************************************************************************/

struct ba_factor_t : factor_t {
  vec2_t z = zeros(2, 1);
  landmark_t *p_W = nullptr;
  pose_t *T_WC = nullptr;

  mat2_t info = I(2);
  mat2_t sq_info = zeros(2, 2);

  ba_factor_t(const timestamp_t &ts_,
              const size_t id_,
              const vec2_t &z_,
              landmark_t *p_W_,
              pose_t *T_WC_,
              mat2_t info_=I(2))
      : factor_t{ts_, id_, 2, {p_W_, T_WC_}, {3, 6}},
        z{z_},
        p_W{p_W_},
        T_WC{T_WC_},
        info{info_} {
    Eigen::LLT<mat2_t> llt_info(info);
    sq_info = llt_info.matrixL().transpose();
  }

  int eval(double *residuals, double **jacobians) const;
};

/*****************************************************************************
 * Camera Factor
 ****************************************************************************/

// template <typename CM, typename DM>
struct cam_factor_t : factor_t {
	// camera_geometry_t<CM, DM> camera;
  vec2_t z = zeros(2, 1);
  landmark_t *p_W = nullptr;
  pose_t *T_WS = nullptr;
  pose_t *T_SC = nullptr;

  mat2_t info = I(2);
  mat2_t sq_info = zeros(2, 2);

  cam_factor_t(const timestamp_t &ts_,
               const size_t id_,
               const vec2_t &z_,
               landmark_t *p_W_,
               pose_t *T_WS_=nullptr,
               pose_t *T_SC_=nullptr,
               mat2_t info_=I(2))
      : factor_t{ts_, id_, 2, {p_W_, T_WS_, T_SC_}, {3, 6, 6}},
        z{z_},
        p_W{p_W_},
        T_WS{T_WS_},
        T_SC{T_SC_},
        info{info_} {
    Eigen::LLT<mat2_t> llt_info(info);
    sq_info = llt_info.matrixL().transpose();
  }

  /**
   * Evaluate
   */
  int eval(double *residuals, double **jacobians) const;
};

/*****************************************************************************
 * IMU Factor
 ****************************************************************************/

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

/*****************************************************************************
 * Factor Graph
 ****************************************************************************/

struct graph_t {
  std::unordered_map<size_t, variable_t *> variables;
  std::vector<factor_t *> factors;

  size_t residual_size = 0;
  size_t nb_poses = 0;
  size_t nb_landmarks = 0;

  std::unordered_map<factor_t *, vecx_t> residuals;
  std::unordered_map<factor_t *, std::vector<matx_t>> jacobians;
  std::unordered_map<variable_t *, size_t> param_index;
};

void graph_free(graph_t &graph);
size_t graph_next_variable_id(graph_t &graph);
size_t graph_next_factor_id(graph_t &graph);
size_t graph_add_pose(graph_t &graph, const timestamp_t &ts, const mat4_t &pose);
size_t graph_add_landmark(graph_t &graph, const vec3_t &landmark);
size_t graph_add_factor(graph_t &graph, factor_t *factor);
size_t graph_add_ba_factor(graph_t &graph,
                           const timestamp_t &ts,
                           const vec2_t &z,
                           landmark_t *p_W,
                           pose_t *T_WS);
// size_t graph_add_camera_factor(graph_t &graph,
//                                const timestamp_t &ts,
//                                const int cam_idx,
//                                const vec2_t &z,
//                                const vec3_t &p_W,
//                                const mat4_t &T_WC);
int graph_eval(graph_t &graph);
void graph_setup_problem(graph_t &graph, matx_t &J, vecx_t &r);
void graph_update(graph_t &graph, const vecx_t &dx);
int graph_solve(graph_t &graph, int max_iter=30);

} // namespace proto
#endif // PROTO_ESTIMATION_FACTOR_HPP
