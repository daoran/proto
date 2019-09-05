#include "proto/estimation/factor.hpp"

namespace proto {

/*****************************************************************************
 * Camera Factor
 ****************************************************************************/

static mat3_t camera_rotation_jacobian(const quat_t &q_WC,
																							const vec3_t &r_WC,
																							const vec3_t &p_W) {
  return q_WC.toRotationMatrix().transpose() * skew(p_W - r_WC);
}

static mat3_t camera_translation_jacobian(const quat_t &q_WC) {
  return -q_WC.toRotationMatrix().transpose();
}

static mat3_t target_point_jacobian(const quat_t &q_WC) {
  return q_WC.toRotationMatrix().transpose();
}

int cam_factor_t::eval(double *residuals, double **jacobians) const {
  // Transform point from world to camera frame
  const mat4_t T_WS = sensor_pose->T();
  const mat4_t T_SC = sensor_camera_extrinsic->T();
  const mat4_t T_WC = T_WS * T_SC;
  const mat4_t T_CW = T_WC.inverse();
  const vec3_t p_W = landmark->p_W;
  const vec3_t p_C = (T_CW * p_W.homogeneous()).head(3);

  // Calculate residual
  const vec3_t x_C = normalize(p_C);
  const mat3_t K = I(3);
  const vec2_t pixel = (K * x_C).head(2);
  Eigen::Map<Eigen::Matrix<double, 2, 1>> r(residuals);
  r.segment<2>(0) = z - pixel;

  // // Weight
  // measurement_t weighted_error = squareRootInformation_ * error;
  // residuals[0] = weighted_error[0];
  // residuals[1] = weighted_error[1];

  // Calculate Jacobian
  if (jacobians == nullptr) {
    return 0;
  }

  // Check validity of the point, simple depth test.
  // bool valid = true;
  // if (fabs(hp_C[3]) > 1.0e-8) {
  //   Eigen::Vector3d p_C = hp_C.template head<3>() / hp_C[3];
  //   // if (p_C[2] < 0.2) { // 20 cm - not very generic... but reasonable
  //   if (p_C[2] < 0.0) { // 20 cm - not very generic... but reasonable
  //     // std::cout<<"INVALID POINT"<<std::endl;
  //     valid = false;
  //   }
  // }

	// // Jacobian w.r.t. sensor pose T_WS
	// if (jacobians[0] != NULL) {
	// 	Eigen::Matrix<double, 3, 3> dp_C__dp_W = C_SC.transpose() * C_WS.transpose();
	// 	Eigen::Matrix<double, 3, 3> dp_W__dr_WS = I(3);
	// 	Eigen::Matrix<double, 3, 3> dp_W__dtheta = -skew(C_WS * hp_S.head(3));
	// 	Eigen::Matrix<double, 2, 3> dh_dr_WS = Jh_weighted * dp_C__dp_W * dp_W__dr_WS;
	// 	Eigen::Matrix<double, 2, 3> dh_dtheta = Jh_weighted * dp_C__dp_W * dp_W__dtheta;
  //
	// 	// Compute the minimal version
	// 	Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J0(jacobians[0]);
	// 	J0.block(0, 0, 2, 3) = dh_dr_WS;
	// 	J0.block(0, 3, 2, 3) = dh_dtheta;
	// 	if (!valid) {
	// 		J0.setZero();
	// 	}
	// }

  // // Jacobian w.r.t. fiducial pose T_WF
  // if (jacobians[1] != NULL) {
	// 	Eigen::Matrix<double, 3, 3> dp_C__dp_W = C_SC.transpose() * C_WS.transpose();
	// 	Eigen::Matrix<double, 3, 3> dp_W__dr_WF = I(3);
	// 	Eigen::Matrix<double, 3, 3> dp_W__dtheta = -skew(C_WF * p_F_);
	// 	Eigen::Matrix<double, 2, 3> dh__dr_WF = -1 * Jh_weighted * dp_C__dp_W * dp_W__dr_WF;
	// 	Eigen::Matrix<double, 2, 3> dh__dtheta = -1 * Jh_weighted * dp_C__dp_W * dp_W__dtheta;
  //
	// 	// Compute the minimal version
	// 	Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J1(jacobians[1]);
	// 	J1.block(0, 0, 2, 3) = dh__dr_WF;
	// 	J1.block(0, 3, 2, 3) = dh__dtheta;
	// 	if (!valid) {
	// 		J1.setZero();
	// 	}
	// }

	// // Jacobian w.r.t. sensor-camera extrinsic pose T_SCi
	// if (jacobians[2] != NULL) {
	// 	Eigen::Matrix<double, 3, 3> dp_C__dp_S = C_SC.transpose();
	// 	Eigen::Matrix<double, 3, 3> dp_S__dr_SC = I(3);
	// 	Eigen::Matrix<double, 3, 3> dp_S__dtheta = -skew(C_SC * hp_C.head(3));
	// 	Eigen::Matrix<double, 2, 3> dh__dr_SC = Jh_weighted * dp_C__dp_S * dp_S__dr_SC;
	// 	Eigen::Matrix<double, 2, 3> dh__dtheta = Jh_weighted * dp_C__dp_S * dp_S__dtheta;
  //
	// 	// Compute the minimal version
	// 	Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J2(jacobians[2]);
	// 	J2.block(0, 0, 2, 3) = dh__dr_SC;
	// 	J2.block(0, 3, 2, 3) = dh__dtheta;
	// 	if (!valid) {
	// 		J2.setZero();
	// 	}
	// }

	// // Jacobian w.r.t. camera intrinsics + distortion params
	// if (jacobians[3] != NULL) {
	// 	Eigen::Map<Eigen::Matrix<double, 2, 8, Eigen::RowMajor>> J3(jacobians[3]);
	// 	J3 = -1 * J_cam_params;
	// }

  return 0;
}

/*****************************************************************************
 * Factor Graph
 ****************************************************************************/

void graph_set_sensor_camera_extrinsic(graph_t &graph,
                                       const int cam_idx,
                                       const mat4_t &T_SC) {
  graph.T_SC[0] = pose_t(0, cam_idx, T_SC);
}

size_t graph_add_camera_factor(graph_t &graph,
                              const timestamp_t &ts,
                              const int cam_idx,
                              const vec2_t &z,
                              const vec3_t &p_W,
                              const mat4_t &T_WS) {
  // Add landmarks
  const size_t lm_id = graph.landmarks.size();
  graph.landmarks.insert({lm_id, {ts, lm_id, p_W}});

  // Add sensor pose
  const size_t pose_id = graph.T_WS.size();
  graph.T_WS.insert({pose_id, {ts, pose_id, T_WS}});

  // Add cam error
  const size_t error_id = graph.cam_factors[cam_idx].size();
  graph.cam_factors[cam_idx].insert(
    {error_id, {ts, error_id, z, lookup(graph.landmarks, lm_id)}}
  );
  return error_id;
}

int graph_solve(graph_t &graph, int max_iter) {
  // Calculate number of cameras and measurements
  size_t nb_meas = 0;
  size_t nb_cams = graph.T_SC.size();
  size_t nb_poses = graph.T_WS.size();
  for (size_t cam_idx = 0; cam_idx < nb_cams; cam_idx++) {
    nb_meas += graph.cam_factors[cam_idx].size();
  }

  // Setup residuals
  const size_t r_rows = nb_meas * 2;
  vecx_t r = zeros(r_rows, 1);

  // Setup Jacobian
  const size_t J_rows = nb_meas * 2;
  const size_t J_cols = (nb_poses * 6) + (nb_meas * 3);
  matx_t J = zeros(J_rows, J_cols);

  // Gauss-Newton
  for (int iter = 0; iter < max_iter; iter++) {
    // Evaluate and form global Jacobian
    // for (const auto &cam_factors : graph.cam_factors) {
    //   for (const auto &kv : cam_factors.second) {
    //     const cam_factor_t &error = kv.second;
    //     // error.eval(r, J);
    //   }
    // }

    // Calculate state update dx
    const matx_t H = J.transpose() * J;
    const vecx_t b = -J.transpose() * r;
    const vecx_t dx = -H.inverse() * b;
    const double cost = 0.5 * r.transpose() * r;
    printf("iter: %d\tcost: %f\n", iter, cost);

    // Update state vector
    // -- Sensor poses
    // for (size_t k = 0; k < nb_poses; k+=6) {
    //   // Update sensor rotation
    //   // const vec3_t dalpha = dx.segment<3>(k);
    //   // const quat_t dq = quat_delta(dalpha);
    //   // graph.T_WS[k]->q = dq * graph.T_WS[k]->q;
    //
    //   // Update sensor position
    //   // const vec3_t dr_WS = dx.segment<3>(k + 3);
    //   // graph.T_WS[k]->r += dr_WS;
    // }
    // -- Landmarks
    // for (size_t i = 0; i < nb_landmarks; i++) {
    //   const vec3_t dp_W = dx.segment<3>(i);
    //   graph.landmarks[i]->p_W += dp_W;
    // }

  }

  return 0;
}

} // namespace proto
