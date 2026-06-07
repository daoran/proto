#include "SO3.hpp"

namespace cartesian {
namespace SO3 {

Vec3 log(const Mat3 &R) {
  const double cos_theta = (R.trace() - 1.0) * 0.5;
  const double theta = std::acos(std::min(1.0, std::max(-1.0, cos_theta)));
  if (theta < 1e-8) {
    return Vec3::Zero();
  }

  Vec3 omega{R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)};
  omega *= (theta / (2.0 * sin(theta)));
  return omega;
}

Mat3 exp(const Vec3 &w) {
  const double theta = w.norm();
  if (theta < 1e-8) {
    return Mat3::Identity() + skew(w);
  }
  const Mat3 wx = skew(w);
  return Mat3::Identity() + (std::sin(theta) / theta) * wx
         + ((1.0 - std::cos(theta)) / (theta * theta)) * wx * wx;
}

Mat3 right_jacobian(const Vec3 &w) {
  const double theta = w.norm();
  const Mat3 wx = skew(w);
  if (theta < 1e-8) {
    return Mat3::Identity() - 0.5 * wx + (1.0 / 6.0) * wx * wx;
  }
  const double theta2 = theta * theta;
  const double theta3 = theta2 * theta;
  return Mat3::Identity()
         - ((1.0 - std::cos(theta)) / theta2) * wx
         + ((theta - std::sin(theta)) / theta3) * wx * wx;
}

Mat3 right_jacobian_inv(const Vec3 &w) {
  const double theta = w.norm();
  const Mat3 wx = skew(w);
  if (theta < 1e-8) {
    return Mat3::Identity() + 0.5 * wx + (1.0 / 12.0) * wx * wx;
  }
  const double theta2 = theta * theta;
  return Mat3::Identity() + 0.5 * wx
         + ((1.0 / theta2) - (1.0 + std::cos(theta))
            / (2.0 * theta * std::sin(theta))) * wx * wx;
}

Mat3 solve_handeye(const std::map<timestamp_t, Mat4> &A,
                   const std::map<timestamp_t, Mat4> &B) {
  // Extract timestamps from A and B
  std::vector<timestamp_t> timestamps_a;
  std::vector<timestamp_t> timestamps_b;
  for (const auto &[ts, _] : A) {
    timestamps_a.push_back(ts);
  }
  for (const auto &[ts, _] : B) {
    timestamps_b.push_back(ts);
  }

  // Perform a intersection between timestamps_a and timestamps_b
  std::vector<timestamp_t> timestamps;
  std::set_intersection(timestamps_a.begin(),
                        timestamps_a.end(),
                        timestamps_b.begin(),
                        timestamps_b.end(),
                        std::back_inserter(timestamps));

  // Form linear system - find the best linear system to solve
  Mat3 H = Mat3::Zero();
  double best_cond = std::numeric_limits<double>::max();
  Mat3 best_H = Mat3::Zero();
  for (const auto ts : timestamps) {
    const Vec3 a = log(tf_rot(A.at(ts)));
    const Vec3 b = log(tf_rot(B.at(ts)));
    const Mat3 H_k = a * b.transpose();

    Eigen::SelfAdjointEigenSolver<Mat3> es(H_k);
    const auto evals = es.eigenvalues().cwiseAbs();
    const double cond = evals.maxCoeff() / evals.minCoeff();

    const bool mincoeff_ok = evals.minCoeff() < 1.0e-3;
    const bool cond_ok = cond > 0.0 && cond < 1000.0;
    if (mincoeff_ok && cond_ok && cond < best_cond) {
      best_cond = cond;
      best_H = H_k;
    }
  }
  H = best_H;
  // Eigen::SelfAdjointEigenSolver<Mat3> es(H);
  // const auto evals = es.eigenvalues().cwiseAbs();
  // const double cond = evals.maxCoeff() / evals.minCoeff();
  // printf("-> cond: %e\n", cond);

  // Solve linear system
  Eigen::JacobiSVD<Mat3> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Mat3 U = svd.matrixU();
  Mat3 V = svd.matrixV();
  Mat3 X = U * V.transpose();

  // Ensure right-handed rotation
  if (X.determinant() < 0) {
    U.col(2) *= -1;
    X = U * V.transpose();
  }

  return X;
}

} // namespace SO3
} // namespace cartesian
