/**
 * @file
 * @ingroup msckf
 */
#ifndef PROTOTYPE_VISION_MSCKF_FEATURE_ESTIMATOR_HPP
#define PROTOTYPE_VISION_MSCKF_FEATURE_ESTIMATOR_HPP

#include <cmath>
#include <cfloat>

#include <ceres/ceres.h>

#include "prototype/core.hpp"
#include "prototype/core/quaternion/jpl.hpp"
#include "prototype/msckf/msckf.hpp"
#include "prototype/model/gimbal.hpp"
#include "prototype/vision/camera/pinhole.hpp"

namespace prototype {
/**
 * @addtogroup msckf
 * @{
 */

/**
 * Linear Least Squares Triangulation
 *
 * Source:
 *     Hartley, R.I. and Sturm, P., "Triangulation", Computer vision and image
 *     understanding, 1997
 *
 * @param u1 homogenous image point (u,v,1)
 * @param P1 camera 1 matrix
 * @param u2 homogenous image point in 2nd camera
 * @param P2 camera 2 matrix
 *
 * @returns Estimated feature position
 */
vec3_t lls_triangulation(const vec3_t &u1,
                       const mat34_t &P1,
                       const vec3_t &u2,
                       const mat34_t &P2);

/**
 * Linear Least Squares Triangulation
 *
 * @param z1 Feature point 1 in image coordinates
 * @param z2 Feature point 2 in image coordinates
 * @param T_C1_C0 Transformation matrix from camera 0 to camera 1
 *
 * @returns Estimated feature position
 */
vec3_t lls_triangulation(const vec2_t &z1, const vec2_t &z2, const mat4_t T_C1_C0);

/**
 * Linear Least Squares Triangulation
 *
 * @param z1 Feature point 1 in image coordinates
 * @param z2 Feature point 2 in image coordinates
 * @param C_C0C1 Rotation matrix from frame C1 to C0
 * @param t_C0_C0C1 Translation vector from frame C0 to C1 expressed in C0
 *
 * @returns Estimated feature position
 */
vec3_t lls_triangulation(const vec2_t &z1,
                       const vec2_t &z2,
                       const mat3_t &C_C0C1,
                       const vec3_t &t_C0_C0C1);

/**
 * Triangulate monocular feature tracks
 *
 * This function uses OpenCV's `cv::triangulatePoints()` to initialize the
 * feature position in 3D.
 *
 * @param T_C1_C0 Transformation matrix from camera 0 to camera 1
 * @param tracks Feature tracks to triangulate
 */
void triangulate_mono_tracks(const mat4_t &T_cam1_cam0, FeatureTracks &tracks);

/**
 * Group tracks according to when they were first captured (frame start)
 *
 * @param tracks Feature tracks to group
 * @param track_groups Feature tracks in groups
 * @param frame_starts Frame starts
 */
void group_tracks(const FeatureTracks &tracks,
                  std::map<FrameID, FeatureTracks> &track_groups,
                  std::set<FrameID> &frame_starts);

/**
 * Triangulate stereo feature tracks
 *
 * This function uses OpenCV's `cv::triangulatePoints()` to initialize the
 * feature position in 3D.
 *
 * @param T_C1_C0 Transformation matrix from camera 0 to camera 1
 * @param tracks Feature tracks to triangulate
 */
FeatureTracks triangulate_stereo_tracks(const mat4_t &T_cam1_cam0,
                                        FeatureTracks &tracks);

/**
 * Feature estimator
 */
class FeatureEstimator {
public:
  FeatureTrack track;
  CameraStates track_cam_states;
  GimbalModel gimbal_model;

  bool debug_mode = false;
  int max_iter = 30;

  /**
   * Optimation configuration
   */
  struct OptimizationConfig {
    double translation_threshold = 0.2;
    double huber_epsilon = 0.01;
    double estimation_precision = 5e-7;
    double initial_damping = 1e-3;
    int outer_loop_max_iteration = 10;
    int inner_loop_max_iteration = 10;
    OptimizationConfig() {}
  };

  FeatureEstimator(const FeatureTrack &track,
                   const CameraStates &track_cam_states);

  FeatureEstimator(const FeatureTrack &track,
                   const CameraStates &track_cam_states,
                   const GimbalModel &gimbal_model);

  /**
   * Triangulate feature observed from camera C0 and C1 and return the
   * position relative to camera C0
   *
   * @param p1 Feature point 1 in pixel coordinates
   * @param p2 Feature point 2 in pixel coordinates
   * @param C_C0C1 Rotation matrix from frame C1 to C0
   * @param t_C0_C0C1 Translation vector from frame C0 to C1 expressed in C0
   * @param p_C0_f Initial feature position
   *
   * @returns 0 for success, -1 for failure
   */
  static int triangulate(const vec2_t &p1,
                         const vec2_t &p2,
                         const mat3_t &C_C0C1,
                         const vec3_t &t_C0_C0C1,
                         vec3_t &p_C0_f);

  /**
   * Calculate initial estimate
   *
   * @param p_C0_f Initial feature position
   * @returns 0 for success, -1 for failure
   */
  int initialEstimate(vec3_t &p_C0_f);

  /**
   * Check estimate
   *
   * @param p_G_f Feature position in global frame
   * @returns 0 for success, -1 for failure
   */
  int checkEstimate(const vec3_t &p_G_f);

  /**
   * Transform feature position from camera to global frame
   *
   * @param alpha Inverse depth parameter x
   * @param beta Inverse depth parameter y
   * @param rho Inverse depth parameter z
   * @param p_G_f Feature position in global frame
   */
  void transformEstimate(const double alpha,
                         const double beta,
                         const double rho,
                         vec3_t &p_G_f);

  /**
   * Reprojection error
   *
   * @param T_Ci_C0 Relative transform from camera 0 to i-th camera
   * @param z Measurement
   * @param x Optimization parameters
   * @returns Jacobian
   */
  vec2_t residual(const mat4_t &T_Ci_C0, const vec2_t &z, const vec3_t &x);

  /**
   * Jacobian
   *
   * @param x Optimization parameters
   * @returns Jacobian
   */
  matx_t jacobian(const mat4_t &T_Ci_C0, const vecx_t &x);

  /**
   * Estimate feature position in global frame
   *
   * @param p_G_f Feature position in global frame
   * @returns 0 for success, -1 for failure
   */
  virtual int estimate(vec3_t &p_G_f);
};

/**
 * Auto-diff reprojection error
 */
struct AutoDiffReprojectionError {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Camera extrinsics
  double C_CiC0[9];
  double t_Ci_CiC0[3];

  // Measurement
  double u = 0.0;
  double v = 0.0;

  AutoDiffReprojectionError(const mat3_t &C_CiC0,
                            const vec3_t &t_Ci_CiC0,
                            const vec2_t &kp);

  /**
   * JPL Quaternion to rotation matrix R
   *
   * Page 9. of Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect
   * Kalman filter for 3D attitude estimation." University of Minnesota,
   * Dept. of Comp. Sci. & Eng., Tech. Rep 2 (2005): 2005.
   *
   * @param q JPL quaternion (x, y, z, w)
   * @returns Rotation matrix
   */
  template <typename T>
  Eigen::Matrix<T, 3, 3> quatToRot(const Eigen::Matrix<T, 4, 1> &q) const;

  /**
   * Calculate Bundle Adjustment Residual
   *
   * @param x Inverse depth parameters (alpha, beta, rho)
   * @param residual Calculated residual
   **/
  template <typename T>
  bool operator()(const T *const x, T *residual) const;
};

/**
 * Analytical reprojection error
 */
struct AnalyticalReprojectionError : public ceres::SizedCostFunction<2, 3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Camera extrinsics
  mat3_t C_CiC0;
  vec3_t t_Ci_CiC0;

  // Measurement
  vec2_t keypoint;

  AnalyticalReprojectionError(const mat3_t &C_CiC0,
                              const vec3_t &t_Ci_CiC0,
                              const vec2_t &keypoint)
      : C_CiC0{C_CiC0}, t_Ci_CiC0{t_Ci_CiC0}, keypoint{keypoint} {}

  /**
   * Calculate reprojection residual
   *
   * @param parameters Optimization parameters
   * @param residuals Residuals
   * @param jacobians Jacobians
   **/
  virtual bool Evaluate(double const *const *parameters,
                        double *residuals,
                        double **jacobians) const;
};

/**
 * Ceres-Solver based feature estimator
 */
class CeresFeatureEstimator : public FeatureEstimator {
public:
  std::string method = "ANALYTICAL";
  // std::string method = "AUTODIFF";
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  double x[3] = {0.0, 0.0, 0.0};

  CeresFeatureEstimator(const FeatureTrack &track,
                        const CameraStates &track_cam_states)
      : FeatureEstimator{track, track_cam_states} {}

  CeresFeatureEstimator(const FeatureTrack &track,
                        const CameraStates &track_cam_states,
                        const GimbalModel &gimbal_model)
      : FeatureEstimator{track, track_cam_states, gimbal_model} {}

  /**
   * Add residual block
   *
   * @param kp Keypoint
   * @param C_CiC0 Rotation matrix from frame C0 to Ci
   * @param t_Ci_CiC0 Translation vector from frame C0 to Ci expressed in Ci
   * @param x Optimization parameters
   */
  void addResidualBlock(const vec2_t &kp,
                        const mat3_t &C_CiC0,
                        const vec3_t &t_Ci_CiC0,
                        double *x);

  /**
   * Setup the optimization problem
   *
   * It performs the following 3 tasks:
   *
   * 1. Calculates an initial estimate of the landmark position
   * 2. Setup camera poses such that the first camera state is the origin,
   * since here we are performing a bundle adjustment of a feature track
   * relative to the first camera pose.
   * 3. Setup residual blocks
   *
   * @returns 0 for success, -1 for failure
   */
  int setupProblem();

  /**
   * Estimate feature position in global frame
   *
   * @param p_G_f Feature position in global frame
   * @returns 0 for success, -1 for failure
   */
  int estimate(vec3_t &p_G_f);
};

/** @} group msckf */
} //  namespace prototype
#include "feature_estimator_impl.hpp"
#endif // PROTOTYPE_VISION_MSCKF_FEATURE_ESTIMATOR_HPP
