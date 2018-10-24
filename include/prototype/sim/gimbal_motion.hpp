/**
 * @file
 * @defgroup sim sim
 */
#ifndef PROTOTYPE_SIM_GIMBAL_MOTION_HPP
#define PROTOTYPE_SIM_GIMBAL_MOTION_HPP

#include <vector>

#include "prototype/sim/bezier.hpp"

namespace prototype {
/**
 * @addtogroup sim
 * @{
 */

/**
 * Gimbal motion
 */
class GimbalMotion {
public:
  // Simulation settings
  std::vector<Vec2> joint_setpoints; ///< Bezier curve attitude control points
  double time_dt = 0.01;        ///< Time difference [s]
  double time = 0.0;            ///< Time [s]
  double time_end = 0.0;        ///< Time end [s]
  double bezier_t = 0.0;        ///< Bezier curve parameter between 0.0 to 1.0
  double bezier_dt = 0.0;       ///< Bezier curve parameter increment
  double scale = 0.0;           ///< Scale bezier dt back to real time dt

  // Gimbal joint angle
  Vec2 joint_angles = Vec2::Zero();  ///< Gimbal joint angle [rad]

  GimbalMotion();

  GimbalMotion(const std::vector<Vec2> &joint_setpoints,
               const double time_dt,
               const double time_end);

  virtual ~GimbalMotion();

  /**
   * Update
   *
   * @returns
   *   - 0 for success
   *   - -1 for failure
   *   - 1 for camera motion complete
   */
  int update();
};

/**
 * GimbalMotion to string
 */
std::ostream &operator<<(std::ostream &os, const GimbalMotion &camera_motion);

/** @} group sim */
} //  namespace prototype
#endif // PROTOTYPE_VISION_SIM_GIMBAL_MOTION_HPP
