#include "prototype/model/mav.hpp"

namespace prototype {

int mav_model_update(mav_model_t &qm,
                     const vec4_t &motor_inputs,
                     const double dt) {
  const double ph = qm.rpy_G(0);
  const double th = qm.rpy_G(1);
  const double ps = qm.rpy_G(2);

  const double p = qm.w_G(0);
  const double q = qm.w_G(1);
  const double r = qm.w_G(2);

  const double x = qm.p_G(0);
  const double y = qm.p_G(1);
  const double z = qm.p_G(2);

  const double vx = qm.v_G(0);
  const double vy = qm.v_G(1);
  const double vz = qm.v_G(2);

  const double Ix = qm.Ix;
  const double Iy = qm.Iy;
  const double Iz = qm.Iz;

  const double kr = qm.kr;
  const double kt = qm.kt;

  const double m = qm.m;
  const double g = qm.g;

  // convert motor inputs to angular p, q, r and total thrust
  // clang-format off
  mat4_t A;
  A << 1.0, 1.0, 1.0, 1.0,
       0.0, -qm.l, 0.0, qm.l,
       -qm.l, 0.0, qm.l, 0.0,
       -qm.d, qm.d, -qm.d, qm.d;
  // clang-format on

  vec4_t tau = A * motor_inputs;
  const double tauf = tau(0);
  const double taup = tau(1);
  const double tauq = tau(2);
  const double taur = tau(3);

  // Update
  // clang-format off
  qm.rpy_G(0) = ph + (p + q * sin(ph) * tan(th) + r * cos(ph) * tan(th)) * dt;
  qm.rpy_G(1) = th + (q * cos(ph) - r * sin(ph)) * dt;
  qm.rpy_G(2) = ps + ((1 / cos(th)) * (q * sin(ph) + r * cos(ph))) * dt;

  qm.w_G(0) = p + (-((Iz - Iy) / Ix) * q * r - (kr * p / Ix) + (1 / Ix) * taup) * dt;
  qm.w_G(1) = q + (-((Ix - Iz) / Iy) * p * r - (kr * q / Iy) + (1 / Iy) * tauq) * dt;
  qm.w_G(2) = r + (-((Iy - Ix) / Iz) * p * q - (kr * r / Iz) + (1 / Iz) * taur) * dt;

  qm.p_G(0) = x + vx * dt;
  qm.p_G(1) = y + vy * dt;
  qm.p_G(2) = z + vz * dt;

	qm.a_G(0) = ((-kt * vx / m) + (1 / m) * (cos(ph) * sin(th) * cos(ps) + sin(ph) * sin(ps)) * tauf);
	qm.a_G(1) = ((-kt * vy / m) + (1 / m) * (cos(ph) * sin(th) * sin(ps) - sin(ph) * cos(ps)) * tauf);
	qm.a_G(2) = (-(kt * vz / m) + (1 / m) * (cos(ph) * cos(th)) * tauf - g);

  qm.v_G(0) = vx + qm.a_G(0) * dt;
  qm.v_G(1) = vy + qm.a_G(1) * dt;
  qm.v_G(2) = vz + qm.a_G(2) * dt;
  // clang-format on

  // Constrain yaw to be [-180, 180]
  qm.rpy_G(2) = wrapToPi(qm.rpy_G(2));

  // Calculate body acceleration and angular velocity
  const mat3_t R_BG = euler321ToRot(qm.rpy_G);
  const vec3_t g_B = euler321ToRot(qm.rpy_G) * vec3_t{0.0, 0.0, qm.g};
  qm.w_B = R_BG * qm.w_G;
  qm.a_B = (R_BG * qm.a_G) + g_B;

  return 0;
}

// int mav_model_update(const double dt) {
//   vec4_t motor_inputs;
//   if (this->ctrl_mode == "POS_CTRL_MODE") {
//     motor_inputs = this->positionControllerControl(dt);
//
//   } else if (this->ctrl_mode == "ATT_CTRL_MODE") {
//     motor_inputs = this->attitudeControllerControl(dt);
//
//   } else if (this->ctrl_mode == "WP_CTRL_MODE") {
//     if (this->mission.configured == false) {
//       LOG_ERROR("Mission is not configured!");
//       return -1;
//     }
//
//     motor_inputs = this->waypointControllerControl(dt);
//   }
//
//   return this->update(motor_inputs, dt);
// }

// vec4_t QuadrotorModel::attitudeControllerControl(const double dt) {
//   const vec4_t actual_attitude{this->rpy_G(0), // roll
//                              this->rpy_G(1), // pitch
//                              this->rpy_G(2), // yaw
//                              this->p_G(2)};  // z
//
//   const vec4_t motor_inputs =
//       this->attitude_controller.update(this->attitude_setpoints,
//                                        actual_attitude,
//                                        dt);
//
//   return motor_inputs;
// }
//
// vec4_t QuadrotorModel::positionControllerControl(const double dt) {
//   // Position controller
//   const vec4_t actual_position{this->p_G(0),    // x
//                              this->p_G(1),    // y
//                              this->p_G(2),    // z
//                              this->rpy_G(2)}; // yaw
//   this->attitude_setpoints =
//       this->position_controller.update(this->position_setpoints,
//                                        actual_position,
//                                        0.0,
//                                        dt);
//
//   // Attitude controller
//   const vec4_t actual_attitude{this->rpy_G(0), // roll
//                              this->rpy_G(1), // pitch
//                              this->rpy_G(2), // yaw
//                              this->p_G(2)};  // z
//
//   const vec4_t motor_inputs =
//       this->attitude_controller.update(this->attitude_setpoints,
//                                        actual_attitude,
//                                        dt);
//
//   return motor_inputs;
// }
//
// vec4_t QuadrotorModel::waypointControllerControl(const double dt) {
//   // Waypoint controller
//   int retval = this->waypoint_controller.update(this->mission,
//                                                 this->p_G,
//                                                 this->v_G,
//                                                 this->rpy_G,
//                                                 dt);
//   if (retval != 0) {
//     this->attitude_setpoints = vec4_t{0.0, 0.0, 0.0, 0.5};
//   } else {
//     this->attitude_setpoints = this->waypoint_controller.outputs;
//   }
//
//   // Attitude controller
//   const vec4_t actual_attitude{this->rpy_G(0), // roll
//                              this->rpy_G(1), // pitch
//                              this->rpy_G(2), // yaw
//                              this->p_G(2)};  // z
//   const vec4_t motor_inputs =
//       this->attitude_controller.update(this->attitude_setpoints,
//                                        actual_attitude,
//                                        dt);
//
//   return motor_inputs;
// }

void mav_model_set_attitude(mav_model_t &qm,
                            const double roll,
                            const double pitch,
                            const double yaw) {
  qm.rpy_G = vec3_t{roll, pitch, yaw};
}

void mav_model_set_position(mav_model_t &qm, const vec3_t &p_G) {
  qm.p_G = p_G;
}

void mav_model_print(mav_model_t &qm) {
  printf("x: %f\t", qm.p_G(0));
  printf("y: %f\t", qm.p_G(1));
  printf("z: %f\t\t", qm.p_G(2));

  printf("phi: %f\t", qm.rpy_G(0));
  printf("theta: %f\t", qm.rpy_G(1));
  printf("psi: %f\n", qm.rpy_G(2));
}

} //  namespace prototype
