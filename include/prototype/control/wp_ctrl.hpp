/**
 * @file
 * @ingroup control
 */
#ifndef PROTOTYPE_CONTROL_WP_CTRL_HPP
#define PROTOTYPE_CONTROL_WP_CTRL_HPP

#include <deque>
#include <iomanip>
#include <libgen.h>
#include <vector>

#include "prototype/core.hpp"
#include "prototype/control/pid.hpp"
#include "prototype/control/mission.hpp"

namespace prototype {
/**
 * @addtogroup control
 * @{
 */

/**
 * Waypoint Control
 */
struct wp_ctrl {
  bool configured = false;
  double dt = 0.0;

  struct pid at_controller = pid_setup(0.5, 0.0, 0.035);
  struct pid ct_controller = pid_setup(0.5, 0.0, 0.035);
  struct pid z_controller = pid_setup(0.3, 0.0, 0.1);
  struct pid yaw_controller = pid_setup(2.0, 0.0, 0.1);

  double roll_limit[2] = {deg2rad(-30.0), deg2rad(30.0)};
  double pitch_limit[2] = {deg2rad(-30.0), deg2rad(30.0)};
  double hover_throttle = 0.5;

  Vec3 setpoints{0.0, 0.0, 0.0};
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};
};

/**
 * Configure waypoint control
 *
 * @param[in] config_file Path to config file
 * @return
 *    - 0: Success
 *    - -1: Failed to load config file
 *    - -2: Failed to load mission file
 */
int wp_ctrl_configure(struct wp_ctrl &wc, const std::string &config_file);

/**
  * Update controller
  *
  * @param[in,out] wc Waypoint controller
  * @param[in,out] m Mission
  * @param[in] p_G Actual position in global frame
  * @param[in] v_G Actual velocity in global frame
  * @param[in] rpy_G Actual roll, pitch and yaw in global frame
  * @param[in] dt Time difference in seconds
  *
  * @return
  *   - 0: Success
  *   - -1: Not configured
  *   - -2: No more waypoints
  */
int wp_ctrl_update(struct wp_ctrl &wc,
                   struct wp_mission &m,
                   const Vec3 &p_G,
                   const Vec3 &v_G,
                   const Vec3 &rpy_G,
                   const double dt);

/**
 * Reset controller errors to 0
 */
void wp_ctrl_reset(struct wp_ctrl &wc);

/** @} group control */
} //  namespace prototype
#endif // PROTOTYPE_CONTROL_WP_CTRL_HPP
