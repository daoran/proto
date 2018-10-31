/**
 * @file
 * @ingroup msckf
 */
#ifndef PROTOTYPE_VISION_MSCKF_CAMERA_STATE_HPP
#define PROTOTYPE_VISION_MSCKF_CAMERA_STATE_HPP

#include <vector>

#include "prototype/core.hpp"
#include "prototype/core/quaternion/jpl.hpp"

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

  static const int size = 6;                ///< Size of error-state vector
  FrameID frame_id = -1;                    ///< Camera frame id
  vec3_t p_G = zeros(3, 1);                 ///< Position in Global frame
  vec4_t q_CG = vec4_t{0.0, 0.0, 0.0, 1.0}; ///< Orientation in Global frame

  // Gimbal joint angle
  vec2_t theta = vec2_t::Zero();

  CameraState();
  CameraState(const vec3_t &p_G, const vec4_t &q_CG);
  CameraState(const FrameID frame_id, const vec3_t &p_G, const vec4_t &q_CG);
  CameraState(const FrameID frame_id,
              const vec3_t &p_G,
              const vec4_t &q_CG,
              const vec2_t &theta);

  /**
   * Correct camera state
   * @param dx Correction vector
   */
  void correct(const vecx_t &dx);

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
                           std::vector<vec3_t> &p_G,
                           std::vector<vec3_t> &rpy_G);

/**
 * CameraState to string
 */
std::ostream &operator<<(std::ostream &os, const CameraState &state);

/** @} group msckf */
} //  namespace prototype
#endif // PROTOTYPE_VISION_MSCKF_CAMERA_STATE_HPP
