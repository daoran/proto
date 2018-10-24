/**
 * @file
 * @defgroup msckf libr
 */
#ifndef PROTOTYPE_VISION_MSCKF_GMSCKF_HPP
#define PROTOTYPE_VISION_MSCKF_GMSCKF_HPP

#include <vector>
#include <algorithm>
#include <chrono>
#include <random>

#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/SparseCore>
#include <Eigen/SPQRSupport>
#include <boost/math/distributions/chi_squared.hpp>

#include "prototype/core.hpp"
#include "prototype/core/quaternion/jpl.hpp"
#include "prototype/vision/feature2d/feature_tracker.hpp"

#include "prototype/msckf/jacobians.hpp"
#include "prototype/msckf/camera_state.hpp"
#include "prototype/msckf/feature_estimator.hpp"

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
  bool rk4 = false;  ///< Runge-Kutta 4th order integration
  const int x_imu_sz = 15;
  const int x_cam_sz = 6;

  // Covariance matrices
  MatX P_imu = 1e-2 * I(x_imu_sz, x_imu_sz);
  MatX Q_imu = 1e-2 * I(12, 12);
  MatX P_cam = 1e-2 * I(x_cam_sz, x_cam_sz);
  MatX P_imu_cam = 1e-2 * I(x_imu_sz, x_cam_sz);

  // IMU
  // -- State
  Vec4 q_IG = Vec4{0.0, 0.0, 0.0, 1.0}; ///< JPL Quaternion in Global frame
  Vec3 b_g = zeros(3, 1);               ///< Bias of gyroscope
  Vec3 v_G = zeros(3, 1);               ///< Velocity in Global frame
  Vec3 b_a = zeros(3, 1);               ///< Bias of accelerometer
  Vec3 p_G = zeros(3, 1);               ///< Position in Global frame
  // -- IMU-Camera extrinsics
  Vec4 q_CI = Vec4{0.0, 0.0, 0.0, 1.0};
  Vec3 p_IC = zeros(3, 1);

  // Constants
  Vec3 g_G = Vec3{0.0, 0.0, -9.81}; ///< Gravitational acceleration

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
  VecX getState();

  /**
   * Transition F matrix
   *
   * @param w_hat Estimated angular velocity
   * @param q_hat Estimated quaternion (x, y, z, w)
   * @param a_hat Estimated acceleration
   * @returns Transition jacobian matrix F
   */
  MatX F(const Vec3 &w_hat, const Vec4 &q_hat, const Vec3 &a_hat);

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
  MatX G(const Vec4 &q_hat);

  /**
   * Return covariance matrix P
   */
  MatX P();

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
  MatX J(const Vec4 &cam_q_CI,
         const Vec3 &cam_p_IC,
         const Vec4 &q_hat_IG,
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
         const Vec3 &p_G_f,
         MatX &H_f_j,
         MatX &H_x_j);

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
                 const Vec4 &q_IG = Vec4{0.0, 0.0, 0.0, 1.0},
                 const Vec3 &v_G = Vec3::Zero(),
                 const Vec3 &p_G = Vec3::Zero(),
                 const Vec2 &theta = Vec2::Zero());

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
                 const std::vector<Vec3> &imu_gyro_buffer,
                 const std::vector<Vec3> &imu_accel_buffer,
                 const Vec2 &theta);

  /**
   * Augment state vector with new camera state
   *
   * Augment state and covariance matrix with a copy of the current camera
   * pose estimate when a new image is recorded
   */
  void augmentState(const Vec2 &theta);

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
  int predictionUpdate(const Vec3 &a_m, const Vec3 &w_m, const long ts);

  /**
   * Chi-squared test
   */
  int chiSquaredTest(const MatX &H, const VecX &r, const int dof);

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
  int residualizeTrack(const FeatureTrack &track, MatX &H_j, VecX &r_j);

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
  int calcResiduals(const FeatureTracks &tracks, MatX &T_H, VecX &r_n);

  /**
   * Correct IMU state
   *
   * @param dx Correction vector
   */
  void correctIMUState(const VecX &dx);

  /**
   * Correct camera states
   *
   * @param dx Correction vector
   */
  void correctCameraStates(const VecX &dx);

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
  int measurementUpdate(const FeatureTracks &tracks, const Vec2 &theta);
};

/** @} group msckf */
} //  namespace prototype
#endif // PROTOTYPE_VISION_MSCKF_GMSCKF_HPP
