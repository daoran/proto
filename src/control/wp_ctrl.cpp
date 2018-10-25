#include "prototype/control/wp_ctrl.hpp"

namespace prototype {

int wp_ctrl_update(wp_ctrl_t &wc,
                   wp_mission_t &m,
                   const vec3_t &p_G,
                   const vec3_t &v_G,
                   const vec3_t &rpy_G,
                   const double dt) {
  // Check rate
  wc.dt += dt;
  if (wc.dt < 0.01) {
    return 0;
  }

  // Current waypoint
  vec3_t wp_G = vec3_t::Zero();
  int retval = wp_mission_update(m, p_G, wp_G);
  if (retval != 0) {
    return retval;
  }

  // // Calculate waypoint relative to quadrotor
  // mat4_t T_P_W = zeros(4, 4);
  // T_P_W.block(0, 0, 3, 3) = euler123ToRot(yaw(rpy_G));
  // T_P_W(3, 3) = 1.0;
  // const vec4_t wp_B_homo{wp_G(0) - p_G(0),
  //                      wp_G(1) - p_G(1),
  //                      wp_G(2) - p_G(2),
  //                      1.0};
  // const vec4_t errors = T_P_W * wp_B_homo;

  // std::cout << "T_P_W:\n" << T_P_W << std::endl;
  // std::cout << "wp_G: " << wp_G.transpose() << std::endl;
  // std::cout << "p_G: " << p_G.transpose() << std::endl;
  // std::cout << "wp_B_homo: " << wp_B_homo.transpose() << std::endl;
  // std::cout << std::endl;

  // // Calculate velocity relative to quadrotor
  // const vec4_t v_G_homo{v_G(0), v_G(1), v_G(2), 1.0};
  // const vec4_t v_B = T_P_W * v_G_homo;

  // Calculate RPY errors relative to quadrotor by incorporating yaw
  vec3_t errors{wp_G(0) - p_G(0), wp_G(1) - p_G(1), wp_G(2) - p_G(2)};
  const vec3_t euler{0.0, 0.0, rpy_G(2)};
  const mat3_t R = euler123ToRot(euler);
  errors = R * errors;

  // Roll
  double r = - pid_update(wc.ct_controller, errors(1), wc.dt);

  // Pitch
  // double error_forward = m.desired_velocity - v_B(0);
  // double p = wc.at_controller.update(error_forward, wc.dt);
  double p = pid_update(wc.at_controller, errors(0), wc.dt);

  // Yaw
  // double y = 0.2 * mission_waypoint_heading(m);
  double y = 0.0;

  // Throttle
  double t = wc.hover_throttle;
  t += pid_update(wc.z_controller, errors(2), wc.dt);
  t /= fabs(cos(r) * cos(p)); // adjust throttle for roll and pitch

  // Limit roll, pitch and throttle
  r = (r < wc.roll_limit[0]) ? wc.roll_limit[0] : r;
  r = (r > wc.roll_limit[1]) ? wc.roll_limit[1] : r;
  p = (p < wc.pitch_limit[0]) ? wc.pitch_limit[0] : p;
  p = (p > wc.pitch_limit[1]) ? wc.pitch_limit[1] : p;
  t = (t < 0.0) ? 0.0 : t;
  t = (t > 1.0) ? 1.0 : t;

  // Keep track of setpoints and outputs
  wc.setpoints = wp_G;
  wc.outputs << r, p, y, t;
  wc.dt = 0.0;

  return 0;
}

void wp_ctrl_reset(wp_ctrl_t &wc) {
  pid_reset(wc.at_controller);
  pid_reset(wc.ct_controller);
  pid_reset(wc.z_controller);
  pid_reset(wc.yaw_controller);
}

} //  namespace prototype
