#pragma once

#include "../Core.hpp"

namespace xyz {

/********************************* RADTAN4 ************************************/

Vec2 radtan4_distort(const Vec4 &dist_params, const Vec2 &p);
Vec2 radtan4_undistort(const Vec4 &dist_params, const Vec2 &p0);
mat2_t radtan4_point_jacobian(const Vec4 &dist_params, const Vec2 &p);
matx_t radtan4_params_jacobian(const Vec4 &dist_params, const Vec2 &p);

/********************************** EQUI4 *************************************/

Vec2 equi4_distort(const Vec4 &dist_params, const Vec2 &p);
Vec2 equi4_undistort(const Vec4 &dist_params, const Vec2 &p0);
mat2_t equi4_point_jacobian(const Vec4 &dist_params, const Vec2 &p);
matx_t equi4_params_jacobian(const Vec4 &dist_params, const Vec2 &p);

/********************************* PROJECT ************************************/

Vec2 project_point(const Vec3 &p_C);
mat_t<2, 3> project_jacobian(const Vec3 &p_C);

/********************************* PINHOLE ************************************/

double pinhole_focal(const int image_size, const double fov);
mat3_t
pinhole_K(const double fx, const double fy, const double cx, const double cy);
int pinhole_project(const Vec2i &res,
                    const Vec4 &proj_params,
                    const Vec3 &p,
                    Vec2 &z_hat);
mat2_t pinhole_point_jacobian(const Vec4 &proj_params);
mat_t<2, 4> pinhole_params_jacobian(const Vec4 &proj_params, const Vec2 &p);

#define PINHOLE_PROJECT(FUNC_NAME, DIST_FUNC)                                  \
  int FUNC_NAME(const Vec2i &res,                                            \
                const VecX &params,                                          \
                const Vec3 &p_C,                                             \
                Vec2 &z_hat) {                                               \
    /* Setup */                                                                \
    const Vec4 proj_params = params.head(4);                                 \
    const VecX dist_params = params.tail(params.size() - 4);                 \
                                                                               \
    /* Project, distort and then scale and center */                           \
    const double fx = proj_params(0);                                          \
    const double fy = proj_params(1);                                          \
    const double cx = proj_params(2);                                          \
    const double cy = proj_params(3);                                          \
    const Vec2 p = project_point(p_C);                                       \
    const Vec2 p_d = DIST_FUNC(dist_params, p);                              \
    z_hat.x() = fx * p_d.x() + cx;                                             \
    z_hat.y() = fy * p_d.y() + cy;                                             \
                                                                               \
    /* Check projection is within image frame */                               \
    const bool x_ok = (z_hat.x() >= 0 && z_hat.x() < res.x());                 \
    const bool y_ok = (z_hat.y() >= 0 && z_hat.y() < res.y());                 \
    const bool z_ok = (p_C.z() > 0.0);                                         \
    const bool valid = (x_ok && y_ok && z_ok) ? true : false;                  \
                                                                               \
    return (valid) ? 0 : -1;                                                   \
  }

#define PINHOLE_PROJECT_J(FUNC_NAME, DIST_POINT_JAC_FUNC)                      \
  matx_t FUNC_NAME(const VecX &params, const Vec3 &p_C) {                  \
    const Vec4 proj_params = params.head(4);                                 \
    const VecX dist_params = params.tail(params.size() - 4);                 \
    const Vec2 p = project_point(p_C);                                       \
                                                                               \
    const mat_t<2, 2> J_k = pinhole_point_jacobian(proj_params);               \
    const mat_t<2, 2> J_d = DIST_POINT_JAC_FUNC(dist_params, p);               \
    const matx_t J_p = project_jacobian(p_C);                                  \
    matx_t J_proj = J_k * J_d * J_p;                                           \
                                                                               \
    return J_proj;                                                             \
  }

#define PINHOLE_PARAMS_J(FUNC_NAME, DIST_PARAMS_JAC_FUNC)                      \
  matx_t FUNC_NAME(const VecX &params, const Vec3 &p_C) {                  \
    const Vec4 proj_params = params.head(4);                                 \
    const VecX dist_params = params.tail(params.size() - 4);                 \
                                                                               \
    const Vec2 p = project_point(p_C);                                       \
    const Vec2 p_d = radtan4_distort(dist_params, p);                        \
                                                                               \
    const matx_t J_proj_params = pinhole_params_jacobian(proj_params, p_d);    \
    const matx_t J_proj_point = pinhole_point_jacobian(proj_params);           \
    const matx_t J_dist_params = DIST_PARAMS_JAC_FUNC(dist_params, p);         \
                                                                               \
    const int d_size = dist_params.size();                                     \
    matx_t J_params;                                                           \
    J_params.resize(2, 4 + d_size);                                            \
    J_params.block(0, 0, 2, 4) = J_proj_params;                                \
    J_params.block(0, 4, 2, d_size) = J_proj_point * J_dist_params;            \
                                                                               \
    return J_params;                                                           \
  }

#define PINHOLE_BACK_PROJECT(FUNC_NAME, DIST_UNDISTORT_FUNC)                   \
  int FUNC_NAME(const VecX &params, const Vec2 &x, Vec3 &ray) {          \
    const double fx = params(0);                                               \
    const double fy = params(1);                                               \
    const double cx = params(2);                                               \
    const double cy = params(3);                                               \
    const double px = (x.x() - cx) / fx;                                       \
    const double py = (x.y() - cy) / fy;                                       \
    const Vec2 p{px, py};                                                    \
                                                                               \
    const VecX dist_params = params.tail(params.size() - 4);                 \
    const Vec2 kp = DIST_UNDISTORT_FUNC(dist_params, p);                     \
    ray.x() = kp.x();                                                          \
    ray.y() = kp.y();                                                          \
    ray.z() = 1.0;                                                             \
                                                                               \
    return 0;                                                                  \
  }

#define PINHOLE_UNDISTORT(FUNC_NAME, DIST_UNDISTORT_FUNC)                      \
  Vec2 FUNC_NAME(const VecX &params, const Vec2 &z) {                    \
    /* Back-project and undistort */                                           \
    const double fx = params(0);                                               \
    const double fy = params(1);                                               \
    const double cx = params(2);                                               \
    const double cy = params(3);                                               \
    const double px = (z.x() - cx) / fx;                                       \
    const double py = (z.y() - cy) / fy;                                       \
    const Vec2 p{px, py};                                                    \
                                                                               \
    const VecX dist_params = params.tail(params.size() - 4);                 \
    const Vec2 p_undist = DIST_UNDISTORT_FUNC(dist_params, p);               \
                                                                               \
    /* Project undistorted point to image plane */                             \
    const double x = p_undist.x() * fx + cx;                                   \
    const double y = p_undist.y() * fy + cy;                                   \
    const Vec2 z_undist = {x, y};                                            \
                                                                               \
    return z_undist;                                                           \
  }

/***************************** PINHOLE-RADTAN4 ********************************/

int pinhole_radtan4_project(const Vec2i &res,
                            const VecX &params,
                            const Vec3 &p_C,
                            Vec2 &z_hat);
matx_t pinhole_radtan4_project_jacobian(const VecX &params,
                                        const Vec3 &p_C);
matx_t pinhole_radtan4_params_jacobian(const VecX &params, const Vec3 &p_C);
int pinhole_radtan4_back_project(const VecX &params,
                                 const Vec2 &x,
                                 Vec3 &ray);
Vec2 pinhole_radtan4_undistort(const VecX &params, const Vec2 &z);

/****************************** PINHOLE-EQUI4 *********************************/

int pinhole_equi4_project(const Vec2i &res,
                          const VecX &params,
                          const Vec3 &p_C,
                          Vec2 &z_hat);
matx_t pinhole_equi4_project_jacobian(const VecX &params, const Vec3 &p_C);
matx_t pinhole_equi4_params_jacobian(const VecX &params, const Vec3 &p_C);
int pinhole_equi4_back_project(const VecX &params,
                               const Vec2 &x,
                               Vec3 &ray);
Vec2 pinhole_equi4_undistort(const VecX &params, const Vec2 &z);

/****************************** CAMERA GEOMETRY *******************************/

struct CameraModel {
  std::string type;

  CameraModel(const std::string &type_) : type{type_} {}
  virtual ~CameraModel() = default;

  virtual int project(const Vec2i &res,
                      const VecX &params,
                      const Vec3 &p_C,
                      Vec2 &z_hat) const = 0;

  virtual matx_t projectJacobian(const VecX &params,
                                 const Vec3 &p_C) const = 0;

  virtual matx_t paramsJacobian(const VecX &params,
                                const Vec3 &p_C) const = 0;

  virtual int backProject(const VecX &params,
                          const Vec2 &x,
                          Vec3 &ray) const = 0;

  virtual Vec2 undistort(const VecX &params, const Vec2 &z) const = 0;
};

struct PinholeRadtan4 : CameraModel {
  PinholeRadtan4() : CameraModel{"pinhole-radtan4"} {}

  int project(const Vec2i &res,
              const VecX &params,
              const Vec3 &p_C,
              Vec2 &z_hat) const override {
    return pinhole_radtan4_project(res, params, p_C, z_hat);
  }

  matx_t projectJacobian(const VecX &params,
                         const Vec3 &p_C) const override {
    return pinhole_radtan4_project_jacobian(params, p_C);
  }

  matx_t paramsJacobian(const VecX &params,
                        const Vec3 &p_C) const override {
    return pinhole_radtan4_params_jacobian(params, p_C);
  }

  int backProject(const VecX &params,
                  const Vec2 &x,
                  Vec3 &ray) const override {
    return pinhole_radtan4_back_project(params, x, ray);
  }

  Vec2 undistort(const VecX &params, const Vec2 &z) const override {
    return pinhole_radtan4_undistort(params, z);
  }
};

struct PinholeEqui4 : CameraModel {
  PinholeEqui4() : CameraModel{"pinhole-equi4"} {}

  int project(const Vec2i &res,
              const VecX &params,
              const Vec3 &p_C,
              Vec2 &z_hat) const override {
    return pinhole_equi4_project(res, params, p_C, z_hat);
  }

  matx_t projectJacobian(const VecX &params,
                         const Vec3 &p_C) const override {
    return pinhole_equi4_project_jacobian(params, p_C);
  }

  matx_t paramsJacobian(const VecX &params,
                        const Vec3 &p_C) const override {
    return pinhole_equi4_params_jacobian(params, p_C);
  }

  int backProject(const VecX &params,
                  const Vec2 &x,
                  Vec3 &ray) const override {
    return pinhole_equi4_back_project(params, x, ray);
  }

  Vec2 undistort(const VecX &params, const Vec2 &z) const override {
    return pinhole_equi4_undistort(params, z);
  }
};

} // namespace xyz
