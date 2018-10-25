#include "calibration/calibration.hpp"

namespace prototype {

GimbalCalib::GimbalCalib() {}

GimbalCalib::~GimbalCalib() {}

int GimbalCalib::load(const std::string &data_dir) {
  // Load calibration data
  this->data_dir = data_dir;
  if (this->data.load(data_dir) != 0) {
    LOG_ERROR("Failed to load calibration data [%s]!", data_dir.c_str());
    return -1;
  }

  // Load optimization params
  const std::string config_file = data_dir + "/camchain.yaml";
  const std::string joint_file = data_dir + "/joint.csv";
  if (this->params.load(config_file, joint_file) != 0) {
    LOG_ERROR("Failed to load optimization params!");
    return -1;
  }

  // Setup optimization problem
  const mat3_t K_s = this->params.camchain.cam[0].K();
  const mat3_t K_d = this->params.camchain.cam[2].K();
  const vec4_t D_s = this->params.camchain.cam[0].D();
  const vec4_t D_d = this->params.camchain.cam[2].D();
  const double theta1_offset = *this->params.theta1_offset;
  const double theta2_offset = *this->params.theta2_offset;

  const std::string dmodel_s = this->params.camchain.cam[0].distortion_model;
  const std::string dmodel_d = this->params.camchain.cam[2].distortion_model;

  // Form residual blocks
  for (int i = 0; i < this->data.nb_measurements; i++) {
    for (int j = 0; j < this->data.P_s[i].rows(); j++) {
      const vec3_t P_s = this->data.P_s[i].row(j);
      const vec3_t P_d = this->data.P_d[i].row(j);

      // Undistort pixel measurements from static camera
      const vec2_t p_s = this->data.Q_s[i].row(j);
      cv::Point2f pt_s(p_s(0), p_s(1));
      pt_s = this->params.camchain.cam[0].undistortPoint(pt_s);
      vec3_t x_s{pt_s.x, pt_s.y, 1.0};
      x_s = this->params.camchain.cam[0].K() * x_s;
      const vec2_t Q_s{x_s(0), x_s(1)};

      // Undistort pixel measurements from dynamic camera
      const vec2_t p_d = this->data.Q_d[i].row(j);
      cv::Point2f pt_d(p_d(0), p_d(1));
      pt_d = this->params.camchain.cam[2].undistortPoint(pt_d);
      vec3_t x_d{pt_d.x, pt_d.y, 1.0};
      x_d = this->params.camchain.cam[2].K() * x_d;
      const vec2_t Q_d{x_d(0), x_d(1)};

      auto residual =
          new GimbalCalibResidual(P_s, P_d,
                                  Q_s, Q_d,
                                  K_s, K_d,
                                  D_s, D_d,
                                  theta1_offset, theta2_offset);

      // Build cost function
      auto cost_func =
          new ceres::AutoDiffCostFunction<GimbalCalibResidual, // Residual type
                                          4, // Size of residual
                                          6, // Size of: tau_s
                                          6, // Size of: tau_d
                                          3, // Size of: w1
                                          3, // Size of: w2
                                          1, // Size of: Lambda1
                                          1  // Size of: Lambda2
                                          >(residual);

      // Add residual block to problem
      this->problem.AddResidualBlock(cost_func, // Cost function
                                     NULL,      // Loss function
                                     this->params.tau_s,
                                     this->params.tau_d,
                                     this->params.w1,
                                     this->params.w2,
                                     &this->params.Lambda1[i],
                                     &this->params.Lambda2[i]);
    // this->problem.SetParameterBlockConstant(&this->params.Lambda1[i]);
    // this->problem.SetParameterBlockConstant(&this->params.Lambda2[i]);
    }
  }
  this->problem.SetParameterBlockConstant(this->params.w2);

  return 0;
}

int GimbalCalib::calculateReprojectionErrors() {
  // Form gimbal model
  GimbalModel gimbal_model;
  gimbal_model.tau_s = vec6_t{this->params.tau_s};
  gimbal_model.tau_d = vec6_t{this->params.tau_d};
  gimbal_model.w1 = vec3_t{this->params.w1};
  gimbal_model.w2 = vec3_t{this->params.w2};
  gimbal_model.theta1_offset = *this->params.theta1_offset;
  gimbal_model.theta2_offset = *this->params.theta2_offset;


  // Load joint data
  matx_t joint_data;
  const std::string joint_filepath = this->data_dir + "/joint.csv";
  if (csv2mat(joint_filepath, false, joint_data) != 0) {
    LOG_ERROR("Failed to load joint data [%s]!", joint_filepath.c_str());
    return -1;
  }

  // Camera matrix for both static and dynamic camera
  const mat3_t K_s = this->params.camchain.cam[0].K();
  const mat3_t K_d = this->params.camchain.cam[2].K();

  // Calculate reprojection error
  double sse_s = 0.0;
  double sse_d = 0.0;
  int points = 0;

  for (int i = 0; i < this->data.nb_measurements; i++) {
    for (int j = 0; j < this->data.P_s[i].rows(); j++) {
      // Get 3D point from static and dyanmic camera
      const vec3_t P_s = this->data.P_s[i].row(j);
      const vec3_t P_d = this->data.P_d[i].row(j);

      // Undistort pixel measurements from static camera
      const vec2_t p_s = this->data.Q_s[i].row(j);
      cv::Point2f pt_s(p_s(0), p_s(1));
      pt_s = this->params.camchain.cam[0].undistortPoint(pt_s);
      vec3_t x_s{pt_s.x, pt_s.y, 1.0};
      x_s = this->params.camchain.cam[0].K() * x_s;
      const vec2_t Q_s{x_s(0), x_s(1)};

      // Undistort pixel measurements from dynamic camera
      const vec2_t p_d = this->data.Q_d[i].row(j);
      cv::Point2f pt_d(p_d(0), p_d(1));
      pt_d = this->params.camchain.cam[2].undistortPoint(pt_d);
      vec3_t x_d{pt_d.x, pt_d.y, 1.0};
      x_d = this->params.camchain.cam[2].K() * x_d;
      const vec2_t Q_d{x_d(0), x_d(1)};

      // Get gimbal roll and pitch angles then form T_ds transform
      const double joint_roll = this->params.Lambda1[i];
      const double joint_pitch = this->params.Lambda2[i];

      // std::cout << joint_roll << " " << joint_pitch << std::endl;
      // std::cout << joint_data(i, 0) << " " << joint_data(i, 1) << std::endl;
      // std::cout << "Roll Error: " << joint_roll - joint_data(i, 0) << std::endl;
      // std::cout << "Pitch Error: " << joint_pitch - joint_data(i, 1) << std::endl;
      // // exit(0);
      // std::cout << std::endl;
      // std::cout << std::endl;

      gimbal_model.setAttitude(joint_roll, joint_pitch);
      const mat4_t T_ds = gimbal_model.T_ds();

      // Project 3D point observed from static to dynamic camera
      const vec3_t P_d_cal = (T_ds * P_s.homogeneous()).head(3);
      const vec3_t X{P_d_cal(0) / P_d_cal(2), P_d_cal(1) / P_d_cal(2), 1.0};
      const vec2_t Q_d_cal = (K_d * X).head(2);

      // Project 3D point observed from dynamic to static camera
      const vec3_t P_s_cal = (T_ds.inverse() * P_d.homogeneous()).head(3);
      const vec3_t X_s{P_s_cal(0) / P_s_cal(2), P_s_cal(1) / P_s_cal(2), 1.0};
      const vec2_t Q_s_cal = (K_s * X_s).head(2);

      // Calculate reprojection error in static camera
      const vec2_t diff_s = Q_s_cal - Q_s;
      const double squared_error_s = diff_s.norm();
      sse_s += squared_error_s;

      // Calculate reprojection error in dynamic camera
      const vec2_t diff_d = Q_d_cal - Q_d;
      const double squared_error_d = diff_d.norm();
      sse_d += squared_error_d;

      points++;
    }
  }

  // Print reprojection errors
  std::cout << "Static camera RMSE: " << sqrt(sse_s / points) << std::endl;
  std::cout << "Dynamic camera RMSE: " << sqrt(sse_d / points) << std::endl;

  return 0;
}

int GimbalCalib::calibrate() {
  // Set options
  this->options.max_num_iterations = 1000;
  this->options.function_tolerance = 1e-12;
  this->options.parameter_tolerance = 1e-12;
  this->options.minimizer_progress_to_stdout = true;

  // Print gimal params before
  std::cout << "Gimbal params before: " << std::endl;
  std::cout << this->params << std::endl;

  // Solve
  ceres::Solve(this->options, &this->problem, &this->summary);
  std::cout << summary.FullReport() << std::endl;

  // Print gimal params after
  std::cout << "Gimbal params after: " << std::endl;
  std::cout << this->params << std::endl;

  // Update camchain with optimized gimbal params
  this->params.camchain.tau_s = vec6_t{this->params.tau_s};
  this->params.camchain.tau_d = vec6_t{this->params.tau_d};
  this->params.camchain.w1 = vec3_t{this->params.w1};
  this->params.camchain.w2 = vec3_t{this->params.w2};
  this->params.camchain.theta1_offset = *this->params.theta1_offset;
  this->params.camchain.theta2_offset = *this->params.theta2_offset;

  // Calculate reprojection errors
  this->calculateReprojectionErrors();

  // Save optimized camchain file
  this->params.camchain.save(this->data_dir + "/camchain_optimized.yaml");

  return 0;
}

} //  namespace prototype
