/**
 * @file
 * @ingroup control
 */
#ifndef PROTOTYPE_CONTROL_POS_CTRL_HPP
#define PROTOTYPE_CONTROL_POS_CTRL_HPP

#include "prototype/core.hpp"
#include "prototype/control/pid.hpp"

namespace prototype {
/**
 * @addtogroup control
 * @{
 */

/**
 * Position control
 */
struct pos_ctrl {
  double dt = 0.0;
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};

  double roll_limit[2] = {-30, 30};
  double pitch_limit[2] = {-30, 30};
  double hover_throttle = 0.5;

  // PID tunes for dt = 0.01 (i.e. 100Hz)
  struct pid x_ctrl = pid_setup(0.5, 0.0, 0.35);
  struct pid y_ctrl = pid_setup(0.5, 0.0, 0.35);
  struct pid z_ctrl = pid_setup(0.3, 0.0, 0.1);
};

/**
 * Configure position control
 *
 * @param[in,out] pc Position control
 * @param[in] config_file Configuration file
 *
 * @returns 0 or -1 for success or failure
 */
int pos_ctrl_configure(struct pos_ctrl &pc,
                       const std::string &config_file);

/**
  * Update position control
  *
  * @param[in,out] pc Position control
  * @param[in] setpoints Setpoints (x, y, z)
  * @param[in] actual Actual (x, y, z)
  * @param[in] yaw (radians)
  * @param[in] dt Time difference (s)
  *
  * @returns Attitude command (roll, pitch, yaw, thrust)
  */
Vec4 pos_ctrl_update(struct pos_ctrl &pc,
                     const Vec3 &setpoints,
                     const Vec4 &actual,
                     const double yaw,
                     const double dt);

/**
  * Reset position control
  */
void pos_ctrl_reset(struct pos_ctrl &pc);

/** @} group control */
} //  namespace prototype
#endif // PROTOTYPE_CONTROL_POS_CTRL_HPP
