#pragma once
#include <deque>
#include <vector>
#include <unordered_map>

#include "CalibProblem.hpp"
#include "../core/Core.hpp"

namespace cartesian {

/** Calibration Target Query Tool */
struct CalibTargetChain {
  std::map<int, std::map<int, std::vector<Mat4>>> adjlist;

  CalibTargetChain() = default;
  CalibTargetChain(const std::map<int, CameraGeometryPtr> &camera_geometries,
                   const std::map<int, CameraData> &camera_measurements);
  virtual ~CalibTargetChain() = default;

  /** Insert link beteen target i and j */
  void insert(const int i, const int j, const Mat4 &T_ij);

  /** Find target extrinsics `T_ij` between `i` and `j` */
  int find(const int i, const int j, Mat4 &T_ij) const;
};

} // namespace cartesian
