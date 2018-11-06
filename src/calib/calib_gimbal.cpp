#include "prototype/calib/calib_gimbal.hpp"

namespace prototype {

calib_gimbal_data_t::calib_gimbal_data_t() {}

calib_gimbal_data_t::~calib_gimbal_data_t() {}

calib_gimbal_params_t::calib_gimbal_params_t() {}

calib_gimbal_params_t::~calib_gimbal_params_t() {
  if (this->tau_s != nullptr)
    free(this->tau_s);

  if (this->tau_d != nullptr)
    free(this->tau_d);

  if (this->w1 != nullptr)
    free(this->w1);

  if (this->w2 != nullptr)
    free(this->w2);

  if (this->theta1_offset != nullptr)
    free(this->theta1_offset);

  if (this->theta2_offset != nullptr)
    free(this->theta2_offset);

  if (this->Lambda1 != nullptr)
    free(this->Lambda1);

  if (this->Lambda2 != nullptr)
    free(this->Lambda2);
}

calib_gimbal_t::calib_gimbal_t() {}

calib_gimbal_t::~calib_gimbal_t() {}

std::ostream &operator<<(std::ostream &os, const calib_gimbal_data_t &m) {
  os << "nb_measurements: " << m.nb_measurements << std::endl;
  return os;
}

std::ostream &operator<<(std::ostream &os,
                         const calib_gimbal_params_t &params) {
  os << "tau_s: " << array2str(params.tau_s, 6) << std::endl;
  os << "tau_d: " << array2str(params.tau_d, 6) << std::endl;
  os << "w1: " << array2str(params.w1, 3) << std::endl;
  os << "w2: " << array2str(params.w2, 3) << std::endl;
  os << "theta1_offset: " << *params.theta1_offset << std::endl;
  os << "theta2_offset: " << *params.theta2_offset << std::endl;

  // os << "Lamba 1 and 2: " << std::endl;
  // for (int i = 0; i < params.nb_measurements; i++) {
  //   os << params.Lambda1[i] << " " << params.Lambda2[i] << std::endl;
  // }

  return os;
}

int calib_gimbal_data_load(calib_gimbal_data_t &data,
                           const std::string &data_dir) {
  // Load joint data
  const std::string joint_filepath = data_dir + "/joint.csv";
  if (csv2mat(joint_filepath, false, data.joint_data) != 0) {
    LOG_ERROR("Failed to load joint data [%s]!", joint_filepath.c_str());
    return -1;
  }

  // Load data
  data.nb_measurements = data.joint_data.rows();
  for (int i = 0; i < data.nb_measurements; i++) {
    const std::string set_path = data_dir + "/set" + std::to_string(i);
    std::cout << "Loading measurement set " << i << " ";
    std::cout << "[" << set_path << "]" << std::endl;

    // P_s_i
    matx_t P_s_i;
    const std::string P_s_fp = set_path + "/P_s";
    if (csv2mat(P_s_fp, false, P_s_i) != 0) {
      LOG_ERROR("Failed to P_s data [%s]!", P_s_fp.c_str());
      return -1;
    }

    // P_d_i
    matx_t P_d_i;
    const std::string P_d_fp = set_path + "/P_d";
    if (csv2mat(P_d_fp, false, P_d_i) != 0) {
      LOG_ERROR("Failed to P_d data [%s]!", P_d_fp.c_str());
      return -1;
    }

    // Q_s_i
    matx_t Q_s_i;
    const std::string Q_s_fp = set_path + "/Q_s";
    if (csv2mat(Q_s_fp, false, Q_s_i) != 0) {
      LOG_ERROR("Failed to Q_s data [%s]!", Q_s_fp.c_str());
      return -1;
    }

    // Q_d_i
    matx_t Q_d_i;
    const std::string Q_d_fp = set_path + "/Q_d";
    if (csv2mat(Q_d_fp, false, Q_d_i) != 0) {
      LOG_ERROR("Failed to Q_d data [%s]!", Q_d_fp.c_str());
      return -1;
    }

    // Store it
    data.P_s.emplace_back(P_s_i);
    data.P_d.emplace_back(P_d_i);
    data.Q_s.emplace_back(Q_s_i);
    data.Q_d.emplace_back(Q_d_i);
  }

  data.ok = true;
  return 0;
}

int calib_gimbal_params_load(calib_gimbal_params_t &data,
                             const std::string &camchain_file,
                             const std::string &joint_file) {
  // Parse camchain file
  // if (this->camchain.load(3, camchain_file) != 0) {
  //   LOG_ERROR("Failed to load camchain file [%s]!", camchain_file.c_str());
  //   return -1;
  // }

  // Load joint angles file
  matx_t joint_data;
  if (csv2mat(joint_file, false, joint_data) != 0) {
    LOG_ERROR("Failed to load joint angles file [%s]!", joint_file.c_str());
    return -1;
  }
  data.nb_measurements = joint_data.rows();

  // Initialize optimization params
  // data.tau_s = vec2array(data.camchain.tau_s);
  // data.tau_d = vec2array(data.camchain.tau_d);
  // data.w1 = vec2array(data.camchain.w1);
  // data.w2 = vec2array(data.camchain.w2);

  data.theta1_offset = (double *) malloc(sizeof(double) * 1);
  data.theta2_offset = (double *) malloc(sizeof(double) * 1);
  // *data.theta1_offset = data.camchain.theta1_offset;
  // *data.theta2_offset = data.camchain.theta2_offset;

  data.Lambda1 = vec2array(joint_data.col(0));
  data.Lambda2 = vec2array(joint_data.col(1));

  return 0;
}

int calib_gimbal_load(calib_gimbal_t &calib,
                      const std::string &data_dir) {
  // Load calibration data
  calib.data_dir = data_dir;
  if (calib_gimbal_data_load(calib.data, data_dir) != 0) {
    LOG_ERROR("Failed to load calibration data [%s]!", data_dir.c_str());
    return -1;
  }

  // Load optimization params
  const std::string config_file = data_dir + "/camchain.yaml";
  const std::string joint_file = data_dir + "/joint.csv";
  if (calib_gimbal_params_load(calib.params, config_file, joint_file) != 0) {
    LOG_ERROR("Failed to load optimization params!");
    return -1;
  }

  // Setup optimization problem
  // const mat3_t K_s = calib.params.camchain.cam[0].K();
  // const mat3_t K_d = calib.params.camchain.cam[2].K();
  // const vec4_t D_s = calib.params.camchain.cam[0].D();
  // const vec4_t D_d = calib.params.camchain.cam[2].D();
  const mat3_t K_s;
  const mat3_t K_d;
  const vec4_t D_s;
  const vec4_t D_d;
  const double theta1_offset = *calib.params.theta1_offset;
  const double theta2_offset = *calib.params.theta2_offset;

  // const std::string dmodel_s = calib.params.camchain.cam[0].distortion_model;
  // const std::string dmodel_d = calib.params.camchain.cam[2].distortion_model;
  const std::string dmodel_s;
  const std::string dmodel_d;

  // Form residual blocks
  for (int i = 0; i < calib.data.nb_measurements; i++) {
    for (int j = 0; j < calib.data.P_s[i].rows(); j++) {
      const vec3_t P_s = calib.data.P_s[i].row(j);
      const vec3_t P_d = calib.data.P_d[i].row(j);

      // Undistort pixel measurements from static camera
      const vec2_t p_s = calib.data.Q_s[i].row(j);
      cv::Point2f pt_s(p_s(0), p_s(1));
      // pt_s = calib.params.camchain.cam[0].undistortPoint(pt_s);
      vec3_t x_s{pt_s.x, pt_s.y, 1.0};
      // x_s = calib.params.camchain.cam[0].K() * x_s;
      const vec2_t Q_s{x_s(0), x_s(1)};

      // Undistort pixel measurements from dynamic camera
      const vec2_t p_d = calib.data.Q_d[i].row(j);
      cv::Point2f pt_d(p_d(0), p_d(1));
      // pt_d = calib.params.camchain.cam[2].undistortPoint(pt_d);
      vec3_t x_d{pt_d.x, pt_d.y, 1.0};
      // x_d = calib.params.camchain.cam[2].K() * x_d;
      const vec2_t Q_d{x_d(0), x_d(1)};

      // auto residual =
      //     new GimbalCalibResidual(P_s, P_d,
      //                             Q_s, Q_d,
      //                             K_s, K_d,
      //                             D_s, D_d,
      //                             theta1_offset, theta2_offset);

      // Build cost function
      // auto cost_func =
      //     new ceres::AutoDiffCostFunction<GimbalCalibResidual, // Residual
      //     type
      //                                     4, // Size of residual
      //                                     6, // Size of: tau_s
      //                                     6, // Size of: tau_d
      //                                     3, // Size of: w1
      //                                     3, // Size of: w2
      //                                     1, // Size of: Lambda1
      //                                     1  // Size of: Lambda2
      //                                     >(residual);

      // Add residual block to problem
      // calib.problem.AddResidualBlock(cost_func, // Cost function
      //                                NULL,      // Loss function
      //                                calib.params.tau_s,
      //                                calib.params.tau_d,
      //                                calib.params.w1,
      //                                calib.params.w2,
      //                                &calib.params.Lambda1[i],
      //                                &calib.params.Lambda2[i]);
      // calib.problem.SetParameterBlockConstant(&calib.params.Lambda1[i]);
      // calib.problem.SetParameterBlockConstant(&calib.params.Lambda2[i]);
    }
  }
  calib.problem.SetParameterBlockConstant(calib.params.w2);

  return 0;
}

int calib_gimbal_calc_reprojection_errors(calib_gimbal_t &calib) {
  // Form gimbal model
  gimbal_model_t gimbal_model;
  gimbal_model.tau_s = vec6_t{calib.params.tau_s};
  gimbal_model.tau_d = vec6_t{calib.params.tau_d};
  gimbal_model.w1 = vec3_t{calib.params.w1};
  gimbal_model.w2 = vec3_t{calib.params.w2};
  gimbal_model.theta1_offset = *calib.params.theta1_offset;
  gimbal_model.theta2_offset = *calib.params.theta2_offset;

  // Load joint data
  matx_t joint_data;
  const std::string joint_filepath = calib.data_dir + "/joint.csv";
  if (csv2mat(joint_filepath, false, joint_data) != 0) {
    LOG_ERROR("Failed to load joint data [%s]!", joint_filepath.c_str());
    return -1;
  }

  // // Camera matrix for both static and dynamic camera
  // const mat3_t K_s = calib.params.camchain.cam[0].K();
  // const mat3_t K_d = calib.params.camchain.cam[2].K();

  //   // Calculate reprojection error
  //   double sse_s = 0.0;
  //   double sse_d = 0.0;
  //   int points = 0;
  //
  //   for (int i = 0; i < calib.data.nb_measurements; i++) {
  //     for (int j = 0; j < calib.data.P_s[i].rows(); j++) {
  //       // Get 3D point from static and dyanmic camera
  //       const vec3_t P_s = calib.data.P_s[i].row(j);
  //       const vec3_t P_d = calib.data.P_d[i].row(j);
  //
  //       // Undistort pixel measurements from static camera
  //       const vec2_t p_s = calib.data.Q_s[i].row(j);
  //       cv::Point2f pt_s(p_s(0), p_s(1));
  //       pt_s = calib.params.camchain.cam[0].undistortPoint(pt_s);
  //       vec3_t x_s{pt_s.x, pt_s.y, 1.0};
  //       x_s = calib.params.camchain.cam[0].K() * x_s;
  //       const vec2_t Q_s{x_s(0), x_s(1)};
  //
  //       // Undistort pixel measurements from dynamic camera
  //       const vec2_t p_d = calib.data.Q_d[i].row(j);
  //       cv::Point2f pt_d(p_d(0), p_d(1));
  //       pt_d = calib.params.camchain.cam[2].undistortPoint(pt_d);
  //       vec3_t x_d{pt_d.x, pt_d.y, 1.0};
  //       x_d = calib.params.camchain.cam[2].K() * x_d;
  //       const vec2_t Q_d{x_d(0), x_d(1)};
  //
  //       // Get gimbal roll and pitch angles then form T_ds transform
  //       const double joint_roll = calib.params.Lambda1[i];
  //       const double joint_pitch = calib.params.Lambda2[i];
  //
  //       // std::cout << joint_roll << " " << joint_pitch << std::endl;
  //       // std::cout << joint_data(i, 0) << " " << joint_data(i, 1) <<
  //       std::endl;
  //       // std::cout << "Roll Error: " << joint_roll - joint_data(i, 0) <<
  //       std::endl;
  //       // std::cout << "Pitch Error: " << joint_pitch - joint_data(i, 1) <<
  //       std::endl;
  //       // // exit(0);
  //       // std::cout << std::endl;
  //       // std::cout << std::endl;
  //
  //       gimbal_model.setAttitude(joint_roll, joint_pitch);
  //       const mat4_t T_ds = gimbal_model.T_ds();
  //
  //       // Project 3D point observed from static to dynamic camera
  //       const vec3_t P_d_cal = (T_ds * P_s.homogeneous()).head(3);
  //       const vec3_t X{P_d_cal(0) / P_d_cal(2), P_d_cal(1) /
  //       P_d_cal(2), 1.0}; const vec2_t Q_d_cal = (K_d * X).head(2);
  //
  //       // Project 3D point observed from dynamic to static camera
  //       const vec3_t P_s_cal = (T_ds.inverse() * P_d.homogeneous()).head(3);
  //       const vec3_t X_s{P_s_cal(0) / P_s_cal(2), P_s_cal(1) /
  //       P_s_cal(2), 1.0}; const vec2_t Q_s_cal = (K_s * X_s).head(2);
  //
  //       // Calculate reprojection error in static camera
  //       const vec2_t diff_s = Q_s_cal - Q_s;
  //       const double squared_error_s = diff_s.norm();
  //       sse_s += squared_error_s;
  //
  //       // Calculate reprojection error in dynamic camera
  //       const vec2_t diff_d = Q_d_cal - Q_d;
  //       const double squared_error_d = diff_d.norm();
  //       sse_d += squared_error_d;
  //
  //       points++;
  //     }
  //   }
  //
  //   // Print reprojection errors
  //   std::cout << "Static camera RMSE: " << sqrt(sse_s / points) << std::endl;
  //   std::cout << "Dynamic camera RMSE: " << sqrt(sse_d / points) <<
  //   std::endl;

  return 0;
}

int calib_gimbal_solve(calib_gimbal_t &calib) {
  // Set options
  calib.options.max_num_iterations = 1000;
  calib.options.function_tolerance = 1e-12;
  calib.options.parameter_tolerance = 1e-12;
  calib.options.minimizer_progress_to_stdout = true;

  // Print gimal params before
  std::cout << "Gimbal params before: " << std::endl;
  std::cout << calib.params << std::endl;

  // Solve
  ceres::Solve(calib.options, &calib.problem, &calib.summary);
  std::cout << calib.summary.FullReport() << std::endl;

  // Print gimal params after
  std::cout << "Gimbal params after: " << std::endl;
  std::cout << calib.params << std::endl;

  // // Update camchain with optimized gimbal params
  // calib.params.camchain.tau_s = vec6_t{calib.params.tau_s};
  // calib.params.camchain.tau_d = vec6_t{calib.params.tau_d};
  // calib.params.camchain.w1 = vec3_t{calib.params.w1};
  // calib.params.camchain.w2 = vec3_t{calib.params.w2};
  // calib.params.camchain.theta1_offset = *calib.params.theta1_offset;
  // calib.params.camchain.theta2_offset = *calib.params.theta2_offset;

  // Calculate reprojection errors
  calib_gimbal_calc_reprojection_errors(calib);

  // // Save optimized camchain file
  // calib.params.camchain.save(calib.data_dir + "/camchain_optimized.yaml");

  return 0;
}

GimbalCalibResidual::GimbalCalibResidual() {}

GimbalCalibResidual::GimbalCalibResidual(const vec3_t &P_s,
                                         const vec3_t &P_d,
                                         const vec2_t &Q_s,
                                         const vec2_t &Q_d,
                                         const mat3_t &K_s,
                                         const mat3_t &K_d,
                                         const vec4_t &D_s,
                                         const vec4_t &D_d,
                                         const double theta1_offset,
                                         const double theta2_offset) {
  // Observed 3d point in static camera
  this->P_s[0] = P_s(0);
  this->P_s[1] = P_s(1);
  this->P_s[2] = P_s(2);

  // Observed 3d point in dynamic camera
  this->P_d[0] = P_d(0);
  this->P_d[1] = P_d(1);
  this->P_d[2] = P_d(2);

  // Observed pixel in static camera
  this->Q_s[0] = Q_s(0);
  this->Q_s[1] = Q_s(1);

  // Observed pixel in dynamic camera
  this->Q_d[0] = Q_d(0);
  this->Q_d[1] = Q_d(1);

  // Theta1 and Theta2 offsets
  this->theta1_offset = theta1_offset;
  this->theta2_offset = theta2_offset;

  // Static camera intrinsics
  this->fx_s = K_s(0, 0);
  this->fy_s = K_s(1, 1);
  this->cx_s = K_s(0, 2);
  this->cy_s = K_s(1, 2);

  // Dynamic camera intrinsics
  this->fx_d = K_d(0, 0);
  this->fy_d = K_d(1, 1);
  this->cx_d = K_d(0, 2);
  this->cy_d = K_d(1, 2);

  // Static camera distortion coefficients
  this->k1_s = D_s(0);
  this->k2_s = D_s(1);
  this->k3_s = D_s(2);
  this->k4_s = D_s(2);

  // Dynamic camera distortion coefficients
  this->k1_d = D_d(0);
  this->k2_d = D_d(1);
  this->k3_d = D_d(2);
  this->k4_d = D_d(2);
}

std::ostream &operator<<(std::ostream &os,
                         const GimbalCalibResidual &residual) {
  os << "P_s: " << array2str(residual.P_s, 3) << std::endl;
  os << "P_d: " << array2str(residual.P_d, 3) << std::endl;
  os << "Q_s: " << array2str(residual.Q_s, 2) << std::endl;
  os << "Q_d: " << array2str(residual.Q_d, 2) << std::endl;
  return os;
}

std::ostream &operator<<(std::ostream &os,
                         const GimbalCalibResidual *residual) {
  os << "P_s: " << array2str(residual->P_s, 3) << std::endl;
  os << "P_d: " << array2str(residual->P_d, 3) << std::endl;
  os << "Q_s: " << array2str(residual->Q_s, 2) << std::endl;
  os << "Q_d: " << array2str(residual->Q_d, 2) << std::endl;
  return os;
}

} //  namespace prototype
