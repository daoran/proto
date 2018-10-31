#include "sim/camera_motion.hpp"

namespace prototype {

CameraMotion::CameraMotion() {}

CameraMotion::CameraMotion(const std::vector<vec3_t> &pos_points,
                           const std::vector<vec3_t> &att_points,
                           const double time_dt,
                           const double time_end)
    : pos_points{pos_points}, att_points{att_points}, time_dt{time_dt},
      time_end{time_end}, bezier_dt{time_dt / time_end}, scale{bezier_dt /
                                                               time_dt} {
  this->update();
}

CameraMotion::~CameraMotion() {}

int CameraMotion::update() {
  std::normal_distribution<> gyro_x_dist{0.0, 1.0};
  std::normal_distribution<> gyro_y_dist{0.0, 1.0};
  std::normal_distribution<> gyro_z_dist{0.0, 1.0};
  std::normal_distribution<> accel_x_dist{0.0, 1.0};
  std::normal_distribution<> accel_y_dist{0.0, 1.0};
  std::normal_distribution<> accel_z_dist{0.0, 1.0};

  // Calculate pos, vel and accel on the Bezier curve at t
  this->p_G = bezier(this->pos_points, this->bezier_t);
  this->v_G =
      bezier_derivative(this->pos_points, this->bezier_t, 1) * this->scale;
  this->a_G = bezier_derivative(this->pos_points, this->bezier_t, 2) *
              this->scale * this->scale;

  // Calculate attitude
  this->rpy_G = bezier(this->att_points, this->bezier_t);
  this->w_G =
      bezier_derivative(this->att_points, this->bezier_t, 1) * this->scale;

  // Calculate IMU measurements
  const mat3_t R_BG = euler123ToRot(this->rpy_G);
  const vec3_t gravity{0.0, 0.0, 9.81};
  if (this->add_noise) {
    const vec3_t n_g{gyro_x_dist(this->gen),
                     gyro_y_dist(this->gen),
                     gyro_z_dist(this->gen)};
    const vec3_t n_a{accel_x_dist(this->gen),
                     accel_y_dist(this->gen),
                     accel_z_dist(this->gen)};
    this->a_B = R_BG * (this->a_G + gravity) + n_a;
    this->w_B = R_BG * this->w_G + n_g;
  } else {
    this->a_B = R_BG * (this->a_G + gravity);
    this->w_B = R_BG * this->w_G;
  }

  // Update time
  this->time += this->time_dt;
  this->bezier_t = this->time / this->time_end;
  if (this->bezier_t > 1.0) {
    return 1;
  }

  return 0;
}

std::ostream &operator<<(std::ostream &os, const CameraMotion &camera_motion) {
  os << "camera_motion:" << std::endl;
  os << "\t p_G: " << camera_motion.p_G.transpose() << std::endl;
  os << "\t v_G: " << camera_motion.v_G.transpose() << std::endl;
  os << "\t a_G: " << camera_motion.a_G.transpose() << std::endl;
  os << "\t rpy_G: " << camera_motion.rpy_G.transpose() << std::endl;
  os << "\t a_B: " << camera_motion.a_B.transpose() << std::endl;
  os << "\t w_B: " << camera_motion.w_B.transpose() << std::endl;
  return os;
}

} //  namespace prototype
