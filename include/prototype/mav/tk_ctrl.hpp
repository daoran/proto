#ifndef PROTOTYPE_MAV_TK_CTRL_HPP
#define PROTOTYPE_MAV_TK_CTRL_HPP

#include "prototype/core/core.hpp"
#include "prototype/control/pid.hpp"
#include "prototype/mav/lz.hpp"

namespace proto {

/**
 * Tracking control
 */
struct tk_ctrl_t {
  double dt = 0.0;
  vec4_t outputs{0.0, 0.0, 0.0, 0.0};

  vec2_t roll_limits{-30, 30};
  vec2_t pitch_limits{-30, 30};
  double hover_throttle = 0.5;

  pid_t x_ctrl;
  pid_t y_ctrl;
  pid_t z_ctrl;
};

/**
 * Configure tracking control
 *
 * @returns 0 or -1 for success or failure
 */
int tk_ctrl_configure(tk_ctrl_t &ctrl,
                      const std::string &config_file,
                      const std::string &prefix = "");

/**
 * Update tracking control
 *
 * - `setpoints`: Setpoints (x, y, z)
 * - `actual`: Actual Pose T_WB
 * - `dt`: Time step [s]
 *
 * @returns Attitude command (roll, pitch, yaw, thrust)
 */
vec4_t tk_ctrl_update(tk_ctrl_t &ctrl,
                      const mat4_t &T_WZ,
                      const mat4_t &T_WB,
                      const double desired_height,
                      const double desired_yaw,
                      const double dt);

/**
 * Reset tk control
 */
void tk_ctrl_reset(tk_ctrl_t &ctrl);

} //  namespace proto
#endif // PROTOTYPE_MAV_TK_CTRL_HPP
