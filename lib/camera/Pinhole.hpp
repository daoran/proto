#pragma once
#include "../Core.hpp"
#include "CameraModel.hpp"

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
struct Pinhole : CameraModel {
  Pinhole() = default;
  virtual ~Pinhole() = default;

  virtual std::string type() const override;
  virtual int project(const Vec2i &res,
                      const VecX &params,
                      const Vec3 &p_C,
                      Vec2 &z_hat) const override;
  virtual MatX project_jacobian(const VecX &params,
                                const Vec3 &p_C) const override;
  virtual MatX params_jacobian(const VecX &params,
                               const Vec3 &p_C) const override;
  virtual void back_project(const VecX &params,
                            const Vec2 &x,
                            Vec3 &ray) const override;
  virtual Vec2 undistort(const VecX &params, const Vec2 &z) const override;
};

} // namespace xyz
