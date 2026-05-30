#pragma once
#include "Core.hpp"

namespace cartesian {

class Logger;

/**
 * Plane represented in Hessian normal form: n * x + d = 0.
 *
 * The plane consists of a unit normal `n` pointing away from the origin and a
 * signed distance `d` such that for any point `x` on the plane, `n * x + d = 0`.
 */
struct Plane {
  Vec3 normal{0.0, 0.0, 0.0};
  double d = 0.0;

  Plane() = default;
  Plane(const Vec3 &normal_, const double d_);
  Plane(const Vec3 &p1, const Vec3 &p2, const Vec3 &p3);
  ~Plane() = default;

  /** Signed distance of point `p` from the plane. Always non-negative. */
  double dist(const Vec3 &p) const;
};

/**
 * Fit a plane to `pts` using RANSAC.
 *
 * @param pts            Input point cloud.
 * @param iterations     Number of RANSAC iterations.
 * @param threshold      Inlier threshold (distance from plane).
 * @param best_inliers   Output list of inlier indices.
 * @return Best plane found by RANSAC.
 */
Plane plane_ransac(const std::vector<Vec3> &pts,
                   const int iterations,
                   const double threshold,
                   std::vector<int> &best_inliers);

/**
 * Refine a plane estimate by least-squares fit to known inliers.
 *
 * Uses PCA on the inlier points. The normal is the direction of smallest
 * variance. This is a closed-form (non-iterative) refinement.
 *
 * @param inliers  Points known to lie on the plane.
 * @return Refined plane.
 */
Plane plane_refine(const std::vector<Vec3> &inliers);

} // namespace cartesian
