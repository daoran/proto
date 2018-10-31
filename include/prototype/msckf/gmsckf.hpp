/**
 * @file
 * @defgroup msckf libr
 */
#ifndef PROTOTYPE_VISION_MSCKF_GMSCKF_HPP
#define PROTOTYPE_VISION_MSCKF_GMSCKF_HPP

#include <algorithm>
#include <chrono>
#include <random>
#include <vector>

#include <Eigen/QR>
#include <Eigen/SPQRSupport>
#include <Eigen/SVD>
#include <Eigen/SparseCore>
#include <boost/math/distributions/chi_squared.hpp>

#include "prototype/core.hpp"
#include "prototype/core/quaternion/jpl.hpp"
#include "prototype/vision/feature2d/feature_tracker.hpp"

#include "prototype/msckf/camera_state.hpp"
#include "prototype/msckf/feature_estimator.hpp"
#include "prototype/msckf/jacobians.hpp"

namespace prototype {
/**
 * @addtogroup msckf
 * @{
 */

/**
 * Gimbal Multi-State Constraint Kalman Filter (GMSCKF)
 */
class GMSCKF {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool rk4 = false; ///< Runge-Kutta 4th order integration
  const int x_imu_sz = 15;
  const int x_cam_sz = 6;

  // Covariance matrices
  matx_t P_imu = 1e-2 * I(x_imu_sz, x_imu_sz);
  matx_t Q_imu = 1e-2 * I(12, 12);
  matx_t P_cam = 1e-2 * I(x_cam_sz, x_cam_sz);
  matx_t P_imu_cam = 1e-2 * I(x_imu_sz, x_cam_sz);

  // IMU
  // -- State
  vec4_t q_IG = vec4_t{0.0, 0.0, 0.0, 1.0}; ///< JPL Quaternion in Global frame
  vec3_t b_g = zeros(3, 1);                 ///< Bias of gyroscope
  vec3_t v_G = zeros(3, 1);                 ///< Velocity in Global frame
  vec3_t b_a = zeros(3, 1);                 ///< Bias of accelerometer
  vec3_t p_G = zeros(3, 1);                 ///< Position in Global frame
  // -- IMU-Camera extrinsics
  vec4_t q_CI = vec4_t{0.0, 0.0, 0.0, 1.0};
  vec3_t p_IC = zeros(3, 1);

  // Constants
  vec3_t g_G = vec3_t{0.0, 0.0, -9.81}; ///< Gravitational acceleration

  // Gimbal
  GimbalModel gimbal_model;

  // Camera
  // -- State
  CameraStates cam_states;
  CameraStates old_cam_states;
  FrameID counter_frame_id = 0;
  // -- Measurement noise
  double img_var = 1e-1;

  // Misc
  std::map<int, double> chi_squared_table;
  long timestamp_first = 0;
  std::vector<double> imu_timestamps = {0.0};
  std::vector<double> cam_timestamps = {0.0};
  long last_updated = 0;

  // Settings
  int max_window_size = 30;
  int min_track_length = 10;
  int max_nb_tracks = 10;
  bool enable_chisq_test = false;
  bool enable_ns_trick = true;
  bool enable_qr_trick = true;
  std::string qr_mode = "sparse";

  GMSCKF();

  /**
   * Configure
   *
   * @param config_file Path to configuration file
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Get State
   *
   * @returns State (p_G, v_G, rpy_G)
   */
  vecx_t getState();

  /**
   * Transition F matrix
   *
   * @param w_hat Estimated angular velocity
   * @param q_hat Estimated quaternion (x, y, z, w)
   * @param a_hat Estimated acceleration
   * @returns Transition jacobian matrix F
   */
  matx_t F(const vec3_t &w_hat, const vec4_t &q_hat, const vec3_t &a_hat);

  /**
   * Input G matrix
   *
   * A matrix that maps the input vector (IMU gaussian noise) to the state
   * vector (IMU error state vector), it tells us how the inputs affect the
   * state vector.
   *
   * @param q_hat Estimated quaternion (x, y, z, w)
   * @returns Input jacobian matrix G
   */
  matx_t G(const vec4_t &q_hat);

  /**
   * Return covariance matrix P
   */
  matx_t P();

  /**
   * Jacobian J matrix
   *
   * @param cam_q_CI Rotation from IMU to camera frame in quaternion (x,y,z,w)
   * @param cam_p_IC Position of camera in IMU frame
   * @param q_hat_IG Rotation from global to IMU frame
   * @param N Number of camera states
   *
   * @returns Jacobian J matrix
   */
  matx_t J(const vec4_t &cam_q_CI,
           const vec3_t &cam_p_IC,
           const vec4_t &q_hat_IG,
           const int N);

  /**
   * Return length of sliding window
   */
  int N();

  /**
   * Return measurement matrix H
   *
   * @param track Feature track
   * @param track_cam_states List of camera states
   * @param p_G_f Feature position in the global frame
   * @param H_f_j Measurement matrix
   * @param H_x_j Measurement matrix
   */
  void H(const FeatureTrack &track,
         const CameraStates &track_cam_states,
         const vec3_t &p_G_f,
         matx_t &H_f_j,
         matx_t &H_x_j);

  /**
   * Initialize
   *
   * @param ts timestamp in nano-seconds
   * @param q_IG Quaternion denoting IMU orientation in global frame
   * @param v_G Velocity in global frame
   * @param p_G Position in global frame
   *
   * @returns 0 for success, -1 for failure
   */
  int initialize(const long ts,
                 const vec4_t &q_IG = vec4_t{0.0, 0.0, 0.0, 1.0},
                 const vec3_t &v_G = vec3_t::Zero(),
                 const vec3_t &p_G = vec3_t::Zero(),
                 const vec2_t &theta = vec2_t::Zero());

  /**
   * Initialize
   *
   * @param ts timestamp in nano-seconds
   * @param imu_gyro_buffer IMU gyro buffer
   * @param imu_accel_buffer IMU accel buffer
   *
   * @returns 0 for success, -1 for failure
   */
  int initialize(const long ts,
                 const std::vector<vec3_t> &imu_gyro_buffer,
                 const std::vector<vec3_t> &imu_accel_buffer,
                 const vec2_t &theta);

  /**
   * Augment state vector with new camera state
   *
   * Augment state and covariance matrix with a copy of the current camera
   * pose estimate when a new image is recorded
   */
  void augmentState(const vec2_t &theta);

  /**
   * Get camera states the feature track was observed in
   *
   * @param track Feature track
   * @returns Camera states where feature track was observed
   */
  CameraStates getTrackCameraStates(const FeatureTrack &track);

  /**
   * Get all camera states including old and current
   *
   * @returns All camera states (old + current)
   */
  CameraStates getAllCameraStates();

  /**
   * Prediction update
   *
   * @param a_m Measured acceleration in body frame
   * @param w_m Measured angular velicty in body frame
   * @param ts Timestamp in nano-seconds
   *
   * @returns 0 for success, -1 for failure
   */
  int predictionUpdate(const vec3_t &a_m, const vec3_t &w_m, const long ts);

  /**
   * Chi-squared test
   */
  int chiSquaredTest(const matx_t &H, const vecx_t &r, const int dof);

  /**
   * Residualize track
   *
   * @param track Feature track
   * @param H_j Measurement jacobian matrix
   * @param r_j Residuals vector
   *
   * @returns
   *  - -1: Track length < Min track length
   *  - -2: Failed to estimate feature position
   */
  int residualizeTrack(const FeatureTrack &track, matx_t &H_j, vecx_t &r_j);

  /**
   * Calculate residuals
   *
   * @param tracks Feature tracks
   * @param T_H
   * @param r_n Residuals vector
   *
   * @returns
   *  - -1: Track length < Min track length
   *  - -2: Failed to estimate feature position
   */
  int calcResiduals(const FeatureTracks &tracks, matx_t &T_H, vecx_t &r_n);

  /**
   * Correct IMU state
   *
   * @param dx Correction vector
   */
  void correctIMUState(const vecx_t &dx);

  /**
   * Correct camera states
   *
   * @param dx Correction vector
   */
  void correctCameraStates(const vecx_t &dx);

  /**
   * Prune old camera state to maintain sliding window size
   */
  CameraStates pruneCameraState();

  /**
   * Filter tracks
   *
   * @param tracks Feature tracks
   * @returns 0 for success, -1 for failure
   */
  FeatureTracks filterTracks(const FeatureTracks &tracks);

  /**
   * Measurmement update
   *
   * @param tracks Feature tracks
   * @returns 0 for success, -1 for failure
   */
  int measurementUpdate(const FeatureTracks &tracks, const vec2_t &theta);
};

/** @} group msckf */
} //  namespace prototype
#endif // PROTOTYPE_VISION_MSCKF_GMSCKF_HPP
