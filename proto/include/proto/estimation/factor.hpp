#ifndef PROTO_ESTIMATION_FACTOR_HPP
#define PROTO_ESTIMATION_FACTOR_HPP

#include <ceres/ceres.h>

#include "proto/core/core.hpp"
#include "proto/vision/vision.hpp"

namespace proto {

/*****************************************************************************
 * Factor
 ****************************************************************************/

struct factor_t {
  size_t id = 0;
  timestamp_t ts = 0;
  std::vector<size_t> param_ids;

  factor_t();
  factor_t(const size_t id_, const timestamp_t &ts_);
  virtual ~factor_t();

  void addParam(size_t &id);

  virtual bool eval(double const *const *parameters,
                    double *residuals,
                    double **jacobians) const = 0;
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
               mat2_t info_ = I(2));

  virtual bool eval(double const *const *parameters,
                    double *residuals,
                    double **jacobians) const;
};

/*****************************************************************************
 * IMU Factor
 ****************************************************************************/

struct imu_error_t : factor_t {
  imu_error_t() {}
  virtual ~imu_error_t() {}
};

/*****************************************************************************
 * Factor Graph
 ****************************************************************************/

struct graph_t {
  std::unordered_map<size_t, param_t *> params;
  std::vector<factor_t *> factors;

  size_t poses = 0;
  size_t landmarks = 0;
  size_t param_size = 0;
  size_t residual_size = 0;

  graph_t();
  ~graph_t();

  size_t next_param_id();
  size_t next_factor_id();
};

size_t graph_add_pose(graph_t &graph,
                      const timestamp_t &ts,
                      const mat4_t &pose);
size_t graph_add_landmark(graph_t &graph, const vec3_t &landmark);
size_t graph_add_factor(graph_t &graph, factor_t *factor);
void graph_eval(graph_t &graph);
// int graph_solve(graph_t &graph);

} // namespace proto
#endif // PROTO_ESTIMATION_FACTOR_HPP
