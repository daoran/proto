#ifndef PROTOTYPE_MAV_POS_CTRL_HPP
#define PROTOTYPE_MAV_POS_CTRL_HPP

#include "prototype/core/core.hpp"
#include "prototype/control/pid.hpp"

namespace proto {

/**
 * Position control
 */
struct pos_ctrl_t {
  bool ok = false;

  double dt = 0.0;
  vec4_t outputs{0.0, 0.0, 0.0, 0.0};

  vec2_t roll_limits{-30, 30};
  vec2_t pitch_limits{-30, 30};
  double hover_throttle = 0.5;

  // PID tunes for dt = 0.01 (i.e. 100Hz)
  pid_t x_ctrl{0.5, 0.0, 0.35};
  pid_t y_ctrl{0.5, 0.0, 0.35};
  pid_t z_ctrl{0.3, 0.0, 0.1};
};

/**
 * Configure position control
 *
 * @returns 0 or -1 for success or failure
 */
int pos_ctrl_configure(pos_ctrl_t &ctrl,
                       const std::string &config_file,
                       const std::string &prefix = "");

/**
 * Update position control
 *
 * - `setpoints`: Setpoints (x, y, z)
 * - `actual`: Actual Pose T_WB
 * - `yaw`: [rads]
 * - `dt`: Time step [s]
 *
 * @returns Attitude command (roll, pitch, yaw, thrust)
 */
vec4_t pos_ctrl_update(pos_ctrl_t &ctrl,
                       const vec3_t &setpoints,
                       const mat4_t &T_WB,
                       const double yaw_setpoint,
                       const double dt);

/**
 * Reset position control
 */
void pos_ctrl_reset(pos_ctrl_t &ctrl);

} //  namespace proto
#endif // PROTOTYPE_MAV_POS_CTRL_HPP
