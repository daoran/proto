/**
 * @file
 * @defgroup sim sim
 */
#ifndef PROTOTYPE_VISION_SIM_CAMERA_MOTION_HPP
#define PROTOTYPE_VISION_SIM_CAMERA_MOTION_HPP

#include <random>
#include <vector>

#include "prototype/sim/bezier.hpp"

namespace prototype {
/**
 * @addtogroup sim
 * @{
 */

/**
 * Camera motion
 */
class CameraMotion {
public:
  // Simulation settings
  std::vector<vec3_t> pos_points; ///< Bezier curve position control points
  std::vector<vec3_t> att_points; ///< Bezier curve attitude control points
  double time_dt = 0.01;          ///< Time difference [s]
  double time = 0.0;              ///< Time [s]
  double time_end = 0.0;          ///< Time end [s]
  double bezier_t = 0.0;          ///< Bezier curve parameter between 0.0 to 1.0
  double bezier_dt = 0.0;         ///< Bezier curve parameter increment
  double scale = 0.0;             ///< Scale bezier dt back to real time dt
  bool add_noise = true;

  // Camera position velocity and accelation
  vec3_t p_G = vec3_t::Zero();   ///< Global position [m]
  vec3_t v_G = vec3_t::Zero();   ///< Global velocity [m s^-1]
  vec3_t a_G = vec3_t::Zero();   ///< Global acceleration [m s^-2]
  vec3_t rpy_G = vec3_t::Zero(); ///< Global attitude (roll, pitch, yaw) [rad]
  vec3_t w_G = vec3_t::Zero();   ///< Global angular velocity [rad]

  // Emulate IMU measurements
  vec3_t a_B = vec3_t::Zero(); ///< Body acceleration [m s^-2]
  vec3_t w_B = vec3_t::Zero(); ///< Body angular velocity [rad s^-1]

  // Random
  // std::random_device rd;
  std::mt19937 gen{1};

  CameraMotion();

  CameraMotion(const std::vector<vec3_t> &pos_points,
               const std::vector<vec3_t> &att_points,
               const double time_dt,
               const double time_end);

  virtual ~CameraMotion();

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
 * CameraMotion to string
 */
std::ostream &operator<<(std::ostream &os, const CameraMotion &camera_motion);

/** @} group sim */
} //  namespace prototype
#endif // PROTOTYPE_VISION_SIM_CAMERA_MOTION_HPP
