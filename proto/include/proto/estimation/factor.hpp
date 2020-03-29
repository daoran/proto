#ifndef PROTO_ESTIMATION_FACTOR_HPP
#define PROTO_ESTIMATION_FACTOR_HPP

#include <ceres/ceres.h>

#include "proto/core/core.hpp"
#include "proto/vision/vision.hpp"

namespace proto {

/*****************************************************************************
 *                                FACTOR
 ****************************************************************************/

struct factor_t {
  size_t id = 0;
  timestamp_t ts = 0;
  std::vector<size_t> param_ids;

  factor_t() {}

  factor_t(const size_t id_, const timestamp_t &ts_)
    : id{id_}, ts{ts_} {}
  virtual ~factor_t() {}

  virtual bool eval(real_t const *const *parameters) = 0;
};

/*****************************************************************************
 *                             CAMERA FACTOR
 ****************************************************************************/

template <typename CM, typename DM>
struct cam_factor_t : factor_t {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const int cam_index = 0;
  const int img_w = 0;
  const int img_h = 0;
  const vec2_t z{0.0, 0.0};
  const mat2_t info = I(2);

  bool evaluated = false;
  vec2_t residuals{0, 0};
  matxs_t jacobians;

  cam_factor_t(const timestamp_t &ts_,
               const size_t id_,
               const int cam_index_,
               const int img_w_,
               const int img_h_,
               const vec2_t &z_,
               const mat2_t &info_ = I(2))
      : factor_t{ts_, id_}, cam_index{cam_index_},
        img_w{img_w_}, img_h{img_h_},
        z{z_}, info{info_} {
    jacobians.push_back(mat_t<2, 6>());  // T_WS
    jacobians.push_back(mat_t<2, 6>());  // T_SC
    jacobians.push_back(mat_t<2, 3>());  // p_W
    jacobians.push_back(mat_t<2, 4>());  // Camera model
    jacobians.push_back(mat_t<2, 4>());  // Distortion model
  }

  bool eval(real_t const *const *params) {
    assert(param_ids.size() == 5);

    // Map out parameters
    const mat4_t T_WS = tf(params[0]);
    const mat4_t T_SC = tf(params[1]);
    const vec3_t p_W{params[2]};
    const CM cam_model{params[3]};
    const DM dist_model{params[4]};

    // Transform point from world to camera frame
    const mat4_t T_WC = T_WS * T_SC;
    const mat4_t T_CW = T_WC.inverse();
    const vec3_t p_C = tf_point(T_CW, p_W);

    // Project point in camera frame to image plane
    vec2_t z_hat;
    mat_t<2, 3> J_h;
    int retval = project(img_w, img_h, cam_model, dist_model, p_C, z_hat, J_h);
    if (retval != 0) {
      LOG_ERROR("Failed to project point!");
      switch (retval) {
      case -1: LOG_ERROR("Point is not infront of camera!"); break;
      case -2: LOG_ERROR("Projected point is outside the image plane!"); break;
      }
      return false;
    }

    // Calculate residual
    residuals = z - z_hat;

    // Calculate Jacobians
    const vec2_t p{p_C(0) / p_C(2), p_C(1) / p_C(2)};
    const vec2_t p_dist = dist_model.distort(p);
    const vec3_t p_S = tf_point(T_SC, p_C);
    const mat3_t C_SC = tf_rot(T_SC);
    const mat3_t C_CS = C_SC.transpose();
    const mat3_t C_WS = tf_rot(T_WS);
    const mat3_t C_CW = C_CS * C_WS.transpose();

    // -- Jacobian w.r.t. sensor pose T_WS
    jacobians[0].block(0, 0, 2, 3) = -1 * J_h * -C_CW * -skew(C_WS * p_S);
    jacobians[0].block(0, 3, 2, 3) = -1 * J_h * -C_CW;
    // -- Jacobian w.r.t. sensor-camera extrinsic pose T_SCi
    jacobians[1].block(0, 0, 2, 3) = -1 * J_h * C_CS * skew(C_SC * p_C);
    jacobians[1].block(0, 3, 2, 3) = -1 * J_h * -C_CS;
    // -- Jacobian w.r.t. landmark
    jacobians[2] = -1 * J_h * C_CW;
    // -- Jacobian w.r.t. camera model
    jacobians[3] = -1 * cam_model.J_param(p_dist);
    // -- Jacobian w.r.t. distortion model
    jacobians[4] = -1 * cam_model.J_point() * dist_model.J_param(p);

    return 0;
  }
};

/*****************************************************************************
 *                              IMU FACTOR
 ****************************************************************************/

struct imu_error_t : factor_t {
  imu_error_t() {}

  bool eval(real_t const *const *params) {
    UNUSED(params);
    return true;
  }
};

/*****************************************************************************
 *                              FACTOR GRAPH
 ****************************************************************************/

struct graph_t {
  std::unordered_map<size_t, param_t *> params;
  std::deque<factor_t *> factors;
  size_t param_size = 0;
  size_t residual_size = 0;

  graph_t() {}

  ~graph_t() {
    for (const auto &factor : factors) {
      delete factor;
    }
    factors.clear();

    for (const auto &kv : params) {
      delete kv.second;
    }
    params.clear();
  }
};

size_t graph_add_pose(graph_t &graph,
                      const timestamp_t &ts,
                      const mat4_t &pose) {
  const auto id = graph.params.size();
  const auto pose_param = new pose_t{id, ts, pose};
  graph.params.insert({id, pose_param});
  return id;
}

size_t graph_add_landmark(graph_t &graph, const vec3_t &landmark) {
  const auto id = graph.params.size();
  const auto landmark_param = new landmark_t{id, landmark};
  graph.params.insert({id, landmark_param});
  return id;
}

size_t graph_add_cam_params(graph_t &graph,
                            const int cam_index,
                            const vecx_t &params) {
  const auto id = graph.params.size();

  return id;
}

template <typename CM, typename DM>
size_t graph_add_cam_factor(graph_t &graph,
                            const timestamp_t &ts,
                            const int cam_idx,
                            const int img_w,
                            const int img_h,
                            const CM &cm,
                            const DM &dm,
                            const mat4_t &T_WS,
                            const mat4_t &T_SC,
                            const vec3_t &p_W,
                            const vec2_t &z,
                            const mat2_t &info) {
  // Create cam_factor_t parameters
  const auto pose_id = graph_add_pose(graph, ts, T_WS);
  const auto sc_id = graph_add_pose(graph, ts, T_SC);
  const auto landmark_id = graph_add_landmark(graph, p_W);

  // Create cam_factor_t
  const auto f_id = graph.factors.size();
  auto factor = new cam_factor_t<CM, DM>{
    ts, f_id,
    cam_idx, img_w, img_h, cm, dm,
    z, info
  };
  factor->param_ids.push_back(pose_id);
  factor->param_ids.push_back(sc_id);
  factor->param_ids.push_back(landmark_id);

  // Add cam_factor_t to graph
  graph.factors.push_back(factor);

  return f_id;
}

void graph_eval(graph_t &graph) {
  vecx_t r = zeros(graph.residual_size);
  matx_t J = zeros(graph.residual_size, graph.param_size);

  // Determine parameter order
  std::map<real_t *, size_t> param_idx;
  std::map<real_t *, size_t> pose_blocks;
  std::map<real_t *, size_t> calib_blocks;
  std::map<real_t *, size_t> landmark_blocks;
  // size_t param_counter = 0;

  for (const auto &factor : graph.factors) {
    for (size_t i = 0; i < factor->param_ids.size(); i++) {
      const auto param_id = factor->param_ids[i];
      const auto param = graph.params[param_id];

      //   if (typeid(param) == typeid(pose_t)) {
      //     // temporal_blocks
      //
      //
      //   } else if (typeid(param) == typeid(landmark_t)) {
      //
      //
      //   }

      // param_idx[param->data()] = param_counter;
      // param_counter += param->local_size;
    }
  }

  // // Loop over time
  // real_t *params[10] = {0};
  // for (const auto &factor : graph.factors) {
  //   for (size_t i = 0; i < factor->param_ids.size(); i++) {
  //     const auto param_id = factor->param_ids[i];
  //     params[i] = graph.params[param_id]->data();
  //   }
  //
  //   // factor->eval(params, );
  //   memset(params, '\0', sizeof(real_t *) * 10);
  // }
}

int graph_solve(graph_t &graph) {
  return 0;
}

} // namespace proto
#endif // PROTO_ESTIMATION_FACTOR_HPP
