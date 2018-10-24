/**
 * @file
 * @ingroup msckf
 */
#ifndef PROTOTYPE_VISION_MSCKF_CAMERA_STATE_HPP
#define PROTOTYPE_VISION_MSCKF_CAMERA_STATE_HPP

#include <vector>

#include "prototype/core.hpp"
#include "prototype/core/quaternion/jpl.hpp"
#include "prototype/vision/feature2d/feature_tracker.hpp"

namespace prototype {
/**
 * @addtogroup msckf
 * @{
 */

/**
 * Camera state
 */
class CameraState {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const int size = 6;            ///< Size of error-state vector
  FrameID frame_id = -1;                ///< Camera frame id
  Vec3 p_G = zeros(3, 1);               ///< Position in Global frame
  Vec4 q_CG = Vec4{0.0, 0.0, 0.0, 1.0}; ///< Orientation in Global frame

  // Gimbal joint angle
  Vec2 theta = Vec2::Zero();

  CameraState();
  CameraState(const Vec3 &p_G, const Vec4 &q_CG);
  CameraState(const FrameID frame_id, const Vec3 &p_G, const Vec4 &q_CG);
  CameraState(const FrameID frame_id, const Vec3 &p_G, const Vec4 &q_CG, const Vec2 &theta);

  /**
   * Correct camera state
   * @param dx Correction vector
   */
  void correct(const VecX &dx);

  /**
   * Set frame id
   * @param frame_id Frame ID
   */
  void setFrameID(const FrameID &frame_id);
};

/**
 * Camera states
 */
using CameraStates = std::vector<CameraState>;

/**
 * Get camera states the feature track was observed in
 *
 * @param cam_states Camera states
 * @param track Feature track
 * @returns Camera states where feature track was observed
 */
CameraStates get_track_camera_states(const CameraStates &cam_states,
                                     const FeatureTrack &track);

/**
 * Camera states to CSV file
 *
 * @param states Camera states
 * @param output_path Output path
 */
int save_camera_states(const CameraStates &states,
                       const std::string &output_path);

/**
 * Convert camera states to vector of position and euler angles
 *
 * @param states Camera states
 * @param p_G Positions
 * @param rpy_G Euler angles
 */
void convert_camera_states(const CameraStates &states,
                           std::vector<Vec3> &p_G,
                           std::vector<Vec3> &rpy_G);

/**
  * CameraState to string
  */
std::ostream &operator<<(std::ostream &os, const CameraState &state);

/** @} group msckf */
} //  namespace prototype
#endif // PROTOTYPE_VISION_MSCKF_CAMERA_STATE_HPP
