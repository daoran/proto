#include "CameraModel.hpp"
#include "Pinhole.hpp"

namespace cartesian {

///////////////////////////////////////////////////////////////////////////////
// RADIAL-TANGENTIAL DISTORTION                                              //
///////////////////////////////////////////////////////////////////////////////

Vec2 radtan4_distort(const Vec4 &dist_params, const Vec2 &p);
Vec2 radtan4_undistort(const Vec4 &dist_params, const Vec2 &p0);
Mat2 radtan4_point_jacobian(const Vec4 &dist_params, const Vec2 &p);
MatX radtan4_params_jacobian(const Vec4 &dist_params, const Vec2 &p);

///////////////////////////////////////////////////////////////////////////////
// BROWN-CONRADY4 CAMERA MODEL                                               //
///////////////////////////////////////////////////////////////////////////////

/**
 * Brown-Conrady Camera Model
 *
 * Source:
 *
 *   Brown, D., 1996. Decentering distortion of lenses. Photogrammetric
 *   engineering, 32(3), pp.444-462.
 *
 *   Conrady, A.E., 1919. Decentred lens-systems. Monthly notices of the
 *   royal astronomical society, 79(5), pp.384-390.
 *
 */
struct BrownConrady4 : CameraModel {
  BrownConrady4() = default;
  virtual ~BrownConrady4() = default;

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

} // namespace cartesian
