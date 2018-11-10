#ifndef PROTOTYPE_CONTROL_WP_CTRL_HPP
#define PROTOTYPE_CONTROL_WP_CTRL_HPP

#include <deque>
#include <iomanip>
#include <libgen.h>
#include <vector>

#include "prototype/core/core.hpp"
#include "prototype/control/mission.hpp"
#include "prototype/control/pid.hpp"

namespace prototype {

/**
 * Waypoint Control
 */
struct wp_ctrl_t {
  bool configured = false;
  double dt = 0.0;

  pid_t at_controller{0.5, 0.0, 0.035};
  pid_t ct_controller{0.5, 0.0, 0.035};
  pid_t z_controller{0.3, 0.0, 0.1};
  pid_t yaw_controller{2.0, 0.0, 0.1};

  double roll_limit[2] = {deg2rad(-30.0), deg2rad(30.0)};
  double pitch_limit[2] = {deg2rad(-30.0), deg2rad(30.0)};
  double hover_throttle = 0.5;

  vec3_t setpoints{0.0, 0.0, 0.0};
  vec4_t outputs{0.0, 0.0, 0.0, 0.0};
};

/**
 * Configure waypoint control
 *
 * @param[in,out] wc Waypoint controller
 * @param[in] config_file Path to config file
 * @return
 *    - 0: Success
 *    - -1: Failed to load config file
 *    - -2: Failed to load mission file
 */
int wp_ctrl_configure(wp_ctrl_t &wc, const std::string &config_file);

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
int wp_ctrl_update(wp_ctrl_t &wc,
                   wp_mission_t &m,
                   const vec3_t &p_G,
                   const vec3_t &v_G,
                   const vec3_t &rpy_G,
                   const double dt);

/**
 * Reset controller errors to 0
 */
void wp_ctrl_reset(wp_ctrl_t &wc);

} //  namespace prototype
#endif // PROTOTYPE_CONTROL_WP_CTRL_HPP
