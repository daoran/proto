#ifndef PROTO_CORE_SPLINE_HPP
#define PROTO_CORE_SPLINE_HPP

#include <vector>

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

#include "proto/core/math.hpp"
#include "proto/core/time.hpp"
#include "proto/core/tf.hpp"

namespace proto {

typedef Eigen::Spline<double, 1> Spline1D;
typedef Eigen::Spline<double, 2> Spline2D;
typedef Eigen::Spline<double, 3> Spline3D;

#define SPLINE1D(X, Y, DEG) \
  Eigen::SplineFitting<Spline1D>::Interpolate(X, DEG, Y)

#define SPLINE2D(X, Y, DEG) \
  Eigen::SplineFitting<Spline2D>::Interpolate(X, DEG, Y)

#define SPLINE3D(X, Y, DEG) \
  Eigen::SplineFitting<Spline3D>::Interpolate(X, DEG, Y)

/**
 * Continuous trajectory generator
 */
struct ctraj_t {
  const timestamps_t timestamps;
  const vec3s_t positions;
  const quats_t orientations;

  const double ts_s_start;
  const double ts_s_end;
  const double ts_s_gap;

  Spline3D pos_spline;
  Spline3D rvec_spline;

  ctraj_t(const timestamps_t &timestamps,
          const vec3s_t &positions,
          const quats_t &orientations);
};

/**
 * Initialize continuous trajectory.
 */
void ctraj_init(ctraj_t &ctraj);

/**
 * Calculate pose `T_WB` at timestamp `ts`.
 */
mat4_t ctraj_get_pose(const ctraj_t &ctraj, const timestamp_t ts);

/**
 * Calculate velocity `v_WB` at timestamp `ts`.
 */
vec3_t ctraj_get_velocity(const ctraj_t &ctraj, const timestamp_t ts);

/**
 * Calculate acceleration `a_WB` at timestamp `ts`.
 */
vec3_t ctraj_get_acceleration(const ctraj_t &ctraj, const timestamp_t ts);

/**
 * Calculate angular velocity `w_WB` at timestamp `ts`.
 */
vec3_t ctraj_get_angular_velocity(const ctraj_t &ctraj, const timestamp_t ts);

} //  namespace proto
#endif // PROTO_CORE_SPLINE_HPP
