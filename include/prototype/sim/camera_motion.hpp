/**
 * @file
 * @defgroup sim sim
 */
#ifndef PROTOTYPE_VISION_SIM_CAMERA_MOTION_HPP
#define PROTOTYPE_VISION_SIM_CAMERA_MOTION_HPP

#include <vector>
#include <random>

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
  std::vector<Vec3> pos_points; ///< Bezier curve position control points
  std::vector<Vec3> att_points; ///< Bezier curve attitude control points
  double time_dt = 0.01;        ///< Time difference [s]
  double time = 0.0;            ///< Time [s]
  double time_end = 0.0;        ///< Time end [s]
  double bezier_t = 0.0;        ///< Bezier curve parameter between 0.0 to 1.0
  double bezier_dt = 0.0;       ///< Bezier curve parameter increment
  double scale = 0.0;           ///< Scale bezier dt back to real time dt
  bool add_noise = true;

  // Camera position velocity and accelation
  Vec3 p_G = Vec3::Zero();   ///< Global position [m]
  Vec3 v_G = Vec3::Zero();   ///< Global velocity [m s^-1]
  Vec3 a_G = Vec3::Zero();   ///< Global acceleration [m s^-2]
  Vec3 rpy_G = Vec3::Zero(); ///< Global attitude (roll, pitch, yaw) [rad]
  Vec3 w_G = Vec3::Zero();   ///< Global angular velocity [rad]

  // Emulate IMU measurements
  Vec3 a_B = Vec3::Zero(); ///< Body acceleration [m s^-2]
  Vec3 w_B = Vec3::Zero(); ///< Body angular velocity [rad s^-1]

  // Random
  // std::random_device rd;
  std::mt19937 gen{1};

  CameraMotion();

  CameraMotion(const std::vector<Vec3> &pos_points,
               const std::vector<Vec3> &att_points,
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
