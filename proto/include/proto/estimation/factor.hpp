#ifndef PROTO_ESTIMATION_FACTOR_HPP
#define PROTO_ESTIMATION_FACTOR_HPP

#include <ceres/ceres.h>

#include "proto/core/core.hpp"
#include "proto/vision/vision.hpp"

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

  variable_t(const size_t id_, const size_t size_, const size_t local_size_)
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

  virtual size_t dimensions() = 0;

  virtual void plus(const vecx_t &) = 0;
};

/*****************************************************************************
 * Landmark Variable
 ****************************************************************************/

struct landmark_t : variable_t {
  double param[3] = {0.0, 0.0, 0.0};

  landmark_t(const size_t id_, const vec3_t &p_W_)
      : variable_t{id_, 3}, param{p_W_(0), p_W_(1), p_W_(2)} {}

  vec3_t vec() { return map_vec_t<3>(param); };

  double *data() { return param; };

  size_t dimensions() { return 3; };

  void plus(const vecx_t &dx) {
    param[0] = param[0] + dx[0];
    param[1] = param[1] + dx[1];
    param[2] = param[2] + dx[2];
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

  size_t dimensions() { return 7; };

  void plus(const vecx_t &dx) {
    // Rotation component
    double half_norm = 0.5 * dx.head<3>().norm();
    double dq_w = cos(half_norm);
    double dq_x = sinc(half_norm) * 0.5 * dx(0);
    double dq_y = sinc(half_norm) * 0.5 * dx(1);
    double dq_z = sinc(half_norm) * 0.5 * dx(2);
    quat_t dq{dq_w, dq_x, dq_y, dq_z};
    quat_t q{param[0], param[1], param[2], param[3]};
    quat_t q_updated = q * dq;
    param[0] = q_updated.w();
    param[1] = q_updated.x();
    param[2] = q_updated.y();
    param[3] = q_updated.z();

    // Translation component
    param[3] = param[3] - dx[3];
    param[4] = param[4] - dx[4];
    param[5] = param[5] - dx[5];
  }
};

/*****************************************************************************
 * Factor
 ****************************************************************************/

struct factor_t : public ceres::CostFunction {
  timestamp_t ts = 0;
  size_t id = 0;

  factor_t() {}
  factor_t(const timestamp_t &ts_, const size_t id_) : ts{ts_}, id{id_} {}
  virtual ~factor_t() {}
};

/*****************************************************************************
 * Bundle Adjustment Factor
 ****************************************************************************/

struct ba_factor_t : public factor_t {
  vec2_t z = zeros(2, 1);
  mat2_t info = I(2);
  mat2_t sq_info = zeros(2, 2);

  ba_factor_t(const timestamp_t &ts_,
              const size_t id_,
              const vec2_t &z_,
              mat2_t info_ = I(2))
      : factor_t{ts_, id_}, z{z_}, info{info_} {
    Eigen::LLT<mat2_t> llt_info(info);
    sq_info = llt_info.matrixL().transpose();
  }

  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const;
};

/*****************************************************************************
 * Camera Factor
 ****************************************************************************/

// template <typename CM, typename DM>
struct cam_factor_t : factor_t {
  // camera_geometry_t<CM, DM> camera;
  vec2_t z = zeros(2, 1);
  mat2_t info = I(2);
  mat2_t sq_info = zeros(2, 2);

  cam_factor_t(const timestamp_t &ts_,
               const size_t id_,
               const vec2_t &z_,
               mat2_t info_ = I(2))
      : factor_t{ts_, id_}, z{z_}, info{info_} {
    Eigen::LLT<mat2_t> llt_info(info);
    sq_info = llt_info.matrixL().transpose();
  }

  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const;
};

/*****************************************************************************
 * IMU Factor
 ****************************************************************************/

// struct imu_error_t : factor_t {
//   timestamps_t ts;
//   vec3s_t gyro;
//   vec3s_t accel;
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

  ceres::Problem problem;
  ceres::Problem::Options problem_options;
  ceres::Solver::Options solver_options;
  ceres::Solver::Summary solver_summary;
};

void graph_free(graph_t &graph);
size_t graph_next_variable_id(graph_t &graph);
size_t graph_next_factor_id(graph_t &graph);
size_t graph_add_pose(graph_t &graph,
                      const timestamp_t &ts,
                      const mat4_t &pose);
size_t graph_add_landmark(graph_t &graph, const vec3_t &landmark);
size_t graph_add_factor(graph_t &graph,
                        factor_t *factor,
                        const std::vector<size_t> param_block_ids);
size_t graph_add_ba_factor(graph_t &graph,
                           const timestamp_t &ts,
                           const vec2_t &z,
                           const size_t point_id,
                           const size_t pose_id);
void graph_setup_problem(graph_t &graph);
int graph_solve(graph_t &graph);

} // namespace proto
#endif // PROTO_ESTIMATION_FACTOR_HPP
