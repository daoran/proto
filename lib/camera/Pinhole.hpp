#pragma once
#include "../Core.hpp"
#include "CameraModel.hpp"
#include "distortion/Distortion.hpp"

namespace xyz {

///////////////////////////////////////////////////////////////////////////////
// PINHOLE UTILS                                                             //
///////////////////////////////////////////////////////////////////////////////

/**
 * Estimate pinhole focal length using image size in pixels and field of view
 * in radians.
 */
double pinhole_focal(const int image_size, const double fov);

/**
 * Form pinhole camera matrix K.
 */
Mat3 pinhole_K(const double fx,
               const double fy,
               const double cx,
               const double cy);

/**
 * Project 3D point p_C in the camera frame, using proj_params (fx, fy, cx, cy)
 * where the camera resolution is res. Returns 0 or 1 for success or failure.
 */
int pinhole_project(const Vec2i &res,
                    const Vec4 &proj_params,
                    const Vec3 &p_C,
                    Vec2 &z_hat);

/**
 * Form pinhole projection jacobian
 */
Mat<2, 3> pinhole_project_jacobian(const Vec3 &p_C);

/**
 * Form pinhole point jacobian
 */
Mat2 pinhole_point_jacobian(const Vec4 &proj_params);

/**
 * Form pinhole params jacobian
 */
Mat<2, 4> pinhole_params_jacobian(const Vec2 &x);

///////////////////////////////////////////////////////////////////////////////
// PINHOLE                                                                   //
///////////////////////////////////////////////////////////////////////////////

/**
 * Pinhole Model
 */
template <typename DISTORTION>
struct Pinhole : CameraModel {
  Pinhole() = default;
  virtual ~Pinhole() = default;

  virtual std::string type() const override {
    std::string typestr = "pinhole-" + DISTORTION::type();
    return typestr;
  }

  virtual int project(const Vec2i &res,
                      const VecX &params,
                      const Vec3 &p_C,
                      Vec2 &z_hat) const override {
    // Setup
    const Vec4 proj_params = params.head(4);
    const VecX dist_params = params.tail(params.size() - 4);

    // Project, distort and then scale and center
    const double fx = proj_params(0);
    const double fy = proj_params(1);
    const double cx = proj_params(2);
    const double cy = proj_params(3);
    const Vec2 p{p_C.x() / p_C.z(), p_C.y() / p_C.z()};
    const Vec2 p_d = DISTORTION::distort(dist_params, p);
    z_hat.x() = fx * p_d.x() + cx;
    z_hat.y() = fy * p_d.y() + cy;

    // Check projection is within image frame
    const bool x_ok = (z_hat.x() >= 0 && z_hat.x() < res.x());
    const bool y_ok = (z_hat.y() >= 0 && z_hat.y() < res.y());
    const bool z_ok = (p_C.z() > 0.0);
    const bool valid = (x_ok && y_ok && z_ok) ? true : false;

    return (valid) ? 0 : -1;
  }

  virtual MatX project_jacobian(const VecX &params,
                                const Vec3 &p_C) const override {
    const Vec4 proj_params = params.head(4);
    const VecX dist_params = params.tail(params.size() - 4);
    const Vec2 p{p_C.x() / p_C.z(), p_C.y() / p_C.z()};

    const Mat<2, 2> J_k = pinhole_point_jacobian(proj_params);
    const Mat<2, 2> J_d = DISTORTION::point_jacobian(dist_params, p);
    const MatX J_p = pinhole_project_jacobian(p_C);
    MatX J_proj = J_k * J_d * J_p;

    return J_proj;
  }

  virtual MatX params_jacobian(const VecX &params,
                               const Vec3 &p_C) const override {
    const Vec4 proj_params = params.head(4);
    const VecX dist_params = params.tail(params.size() - 4);

    const Vec2 p{p_C.x() / p_C.z(), p_C.y() / p_C.z()};
    const Vec2 p_d = DISTORTION::distort(dist_params, p);

    const MatX J_proj_params = pinhole_params_jacobian(p_d);
    const MatX J_proj_point = pinhole_point_jacobian(proj_params);
    const MatX J_dist_params = DISTORTION::params_jacobian(dist_params, p);

    const int d_size = dist_params.size();
    MatX J_params;
    J_params.resize(2, 4 + d_size);
    J_params.block(0, 0, 2, 4) = J_proj_params;
    J_params.block(0, 4, 2, d_size) = J_proj_point * J_dist_params;

    return J_params;
  }

  virtual void back_project(const VecX &params,
                            const Vec2 &x,
                            Vec3 &ray) const override {
    // Back-project and undistort
    const double fx = params(0);
    const double fy = params(1);
    const double cx = params(2);
    const double cy = params(3);
    const double px = (x.x() - cx) / fx;
    const double py = (x.y() - cy) / fy;
    const Vec2 p{px, py};

    const VecX dist_params = params.tail(params.size() - 4);
    const Vec2 kp = DISTORTION::undistort(dist_params, p);
    ray.x() = kp.x();
    ray.y() = kp.y();
    ray.z() = 1.0;
  }

  virtual Vec2 undistort(const VecX &params, const Vec2 &z) const override {
    // Back-project and undistort
    const double fx = params(0);
    const double fy = params(1);
    const double cx = params(2);
    const double cy = params(3);
    const double px = (z.x() - cx) / fx;
    const double py = (z.y() - cy) / fy;
    const Vec2 p{px, py};

    const VecX dist_params = params.tail(params.size() - 4);
    const Vec2 p_undist = DISTORTION::undistort(dist_params, p);

    // Project undistorted point to image plane
    const double x = p_undist.x() * fx + cx;
    const double y = p_undist.y() * fy + cy;
    const Vec2 z_undist = {x, y};

    return z_undist;
  }
};

} // namespace xyz
