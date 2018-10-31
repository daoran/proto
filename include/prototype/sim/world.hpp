/**
 * @file
 * @defgroup sim sim
 */
#ifndef PROTOTYPE_VISION_SIM_WORLD_HPP
#define PROTOTYPE_VISION_SIM_WORLD_HPP

#include <map>
#include <vector>

#include "camera/pinhole_model.hpp"
#include "feature2d/feature_container.hpp"
#include "prototype/core.hpp"
#include "sim/camera.hpp"
#include "sim/camera_motion.hpp"
#include "sim/gimbal_motion.hpp"
#include "sim/twowheel.hpp"

namespace prototype {
/**
 * @addtogroup sim
 * @{
 */

/**
 * Feature bounds
 */
struct feature_bounds {
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
  double z_min = 0.0;
  double z_max = 0.0;
};

/**
 * Simulation world
 */
class SimWorld {
public:
  std::vector<double> timestamps;
  std::vector<double> camera_timestamps;
  double t = 0.0;
  double t_end = 0.0;
  double dt = 0.01;
  size_t time_index = 0;
  size_t frame_index = 0;

  std::string camera_type = "MONO_CAMERA";
  int camera_rate = 10;
  VirtualCamera mono_camera;
  VirtualStereoCamera stereo_camera;
  CameraMotion camera_motion;
  GimbalMotion gimbal_motion;

  vec3_t origin{0.0, 0.0, 0.0};
  vec3_t dimensions{60.0, 60.0, 30.0};
  size_t nb_features = 10000;
  size_t max_track_length = 20;
  matx_t features3d;

  long track_id_counter = 0;
  size_t min_track_length = 10;
  std::vector<int> features_tracking;
  std::map<int, FeatureTrack> tracks_tracking;
  FeatureTracks tracks_lost;

  std::string output_path = "/tmp/sim";
  std::ofstream gnd_file;
  std::ofstream est_file;
  std::ofstream cam_file;
  std::ofstream mea_file;
  std::ofstream jnt_file;
  std::ofstream cam0_idx_file;

  SimWorld();
  virtual ~SimWorld();

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Setup output directory and files
   * @returns 0 for success, -1 for failure
   */
  int setupOutput();

  /**
   * Create 3D features
   *
   * @param bounds Feature bounds
   * @param nb_features Number of 3D features
   * @returns N Features as a Nx3 matrix
   */
  matx_t create3DFeatures(const struct feature_bounds &bounds,
                          const size_t nb_features);

  /**
   * Create 3D feature perimeter around a defined origin in
   * a square like manner
   *
   * @param origin Origin of feature perimeter
   * @param dimensions Diemnsions of feature perimeter
   * @param nb_features Number of 3D features
   * @returns N Features as a Nx3 matrix
   */
  matx_t create3DFeaturePerimeter(const vec3_t &origin,
                                  const vec3_t &dimensions,
                                  const size_t nb_features);

  /**
   * Detect features with mono camera
   */
  void detectFeaturesWithMonoCamera();

  /**
   * Detect features with stereo camera
   */
  void detectFeaturesWithStereoCamera();

  /**
   * Detect features
   */
  void detectFeatures();

  /**
   * Get lost feature tracks
   *
   * @returns Lost feature tracks
   */
  FeatureTracks getLostTracks();

  /**
   * Record ground truth
   *
   * @param time Relative time in seconds (where 0 is start)
   * @param p_G Ground Truth position in global frame
   * @param v_G Ground Truth velocity in global frame
   * @param rpy_G Ground Truth roll, pitch, and yaw in global frame
   *
   * @returns 0 for success, -1 for failure
   */
  int recordGroundTruth(const double time,
                        const vec3_t &p_G,
                        const vec3_t &v_G,
                        const vec3_t &rpy_G);

  /**
   * Record measurement
   *
   * @param time Relative time in seconds (where 0 is start)
   * @param measurement_a_B Accelerometer measurement
   * @param measurement_w_B Gyroscope measurement
   *
   * @returns 0 for success, -1 for failure
   */
  int recordMeasurement(const double time,
                        const vec3_t &measurement_a_B,
                        const vec3_t &measurement_w_B);

  /**
   * Record camera observation
   *
   * @param feature_ids Feature IDs
   * @param keypoints Observed keypoints
   * @param landmarks Observed landmarks
   *
   * @returns 0 for success, -1 for failure
   */
  int recordCameraObservation(const std::vector<size_t> &feature_ids,
                              const std::vector<vec2_t> &keypoints,
                              const std::vector<vec3_t> &landmarks);

  /**
   * Record estimate
   *
   * @param time Relative time in seconds (where 0 is start)
   * @param p_G Position in global frame
   * @param v_G Velocity in global frame
   * @param rpy_G Roll, pitch and yaw in global frame
   *
   * @returns 0 for success, -1 for failure
   */
  int recordEstimate(const double time,
                     const vec3_t &p_G,
                     const vec3_t &v_G,
                     const vec3_t &rpy_G);

  /**
   * Record estimate
   *
   * @param time Relative time in seconds (where 0 is start)
   * @param p_G Position in global frame
   * @param v_G Velocity in global frame
   * @param rpy_G Roll, pitch and yaw in global frame
   * @param p_G_cov Covariance of position in global frame
   * @param v_G_cov Covariance of velocity in global frame
   * @param rpy_G_cov Covariance of roll, pitch and yaw in global frame
   *
   * @returns 0 for success, -1 for failure
   */
  int recordEstimate(const double time,
                     const vec3_t &p_G,
                     const vec3_t &v_G,
                     const vec3_t &rpy_G,
                     const vec3_t &p_G_cov,
                     const vec3_t &v_G_cov,
                     const vec3_t &rpy_G_cov);

  /**
   * Record camera pose
   *
   * @param time Relative time in seconds (where 0 is start)
   * @param p_G Position in global frame
   * @param v_G Velocity in global frame
   * @param rpy_G Roll, pitch and yaw in global frame
   *
   * @returns 0 for success, -1 for failure
   */
  int recordCameraStates(const std::vector<double> time,
                         const std::vector<vec3_t> &p_G,
                         const std::vector<vec3_t> &rpy_G);

  /**
   * Record gimbal joint angles
   *
   * @param time Relative time in seconds (where 0 is start)
   * @param joint_angle Gimbal joint angles
   *
   * @returns 0 for success, -1 for failure
   */
  int recordJointAngles(const double time, const vec2_t &joint_angles);

  /**
   * Generate initialization data for GMSKCF
   *
   * @param nb_samples Number of IMU measurements
   * @param imu_gyro_buffer IMU gyro buffer
   * @param imu_accel_buffer IMU accel buffer
   */
  void generateInitializationData(const int nb_samples,
                                  std::vector<vec3_t> &imu_gyro_buffer,
                                  std::vector<vec3_t> &imu_accel_buffer);

  /**
   * Step simulation
   *
   * @returns 0 for success, -1 for failure
   */
  int step();
};

/** @} group sim */
} //  namespace prototype
#endif // PROTOTYPE_VISION_SIM_WORLD_HPP
