#include "Plane.hpp"
#include "Logger.hpp"

namespace cartesian {

Plane::Plane(const Vec3 &normal_, const double d_) : normal{normal_}, d{d_} {}

Plane::Plane(const Vec3 &p1, const Vec3 &p2, const Vec3 &p3) {
  const Vec3 v1 = p2 - p1;
  const Vec3 v2 = p3 - p1;
  normal = v1.cross(v2).normalized();
  d = -normal.dot(p1);
}

double Plane::dist(const Vec3 &p) const { return std::abs(normal.dot(p) + d); }

Plane plane_ransac(const std::vector<Vec3> &pts,
                   const int iterations,
                   const double threshold,
                   std::vector<int> &best_inliers) {
  std::mt19937 gen(0);
  std::uniform_int_distribution<> dis(0, pts.size() - 1);

  size_t best_count = 0;
  Plane best_plane;
  for (int i = 0; i < iterations; ++i) {
    // Pick three random points to define a candidate plane
    const int i1 = dis(gen);
    const int i2 = dis(gen);
    const int i3 = dis(gen);

    // Count inliers within threshold distance of the candidate
    std::vector<int> inliers;
    Plane plane{pts[i1], pts[i2], pts[i3]};
    for (size_t j = 0; j < pts.size(); ++j) {
      if (plane.dist(pts[j]) < threshold) {
        inliers.push_back(j);
      }
    }

    // Keep the plane with the most inliers
    if (inliers.size() > best_count) {
      best_count = inliers.size();
      best_plane = plane;
      best_inliers = inliers;
    }
  }

  return best_plane;
}

Plane plane_refine(const std::vector<Vec3> &inliers) {
  // Find centroid
  Vec3 centroid(0, 0, 0);
  for (auto &p : inliers) {
    centroid += p;
  }
  centroid /= inliers.size();

  // Form data matrix for PCA (Principal Component Analysis)
  Eigen::MatrixXd A(inliers.size(), 3);
  for (size_t i = 0; i < inliers.size(); ++i) {
    A.row(i) = inliers[i] - centroid;
  }

  // Find direction with least variance in points
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  Vec3 normal = svd.matrixV().col(2);
  double d = -normal.dot(centroid);

  return Plane{normal, d};
}

}
