#pragma once
#include <deque>
#include <vector>
#include <unordered_map>

#include "CalibData.hpp"
#include "../Core.hpp"

namespace xyz {

/** Camera Chain Query Tool */
class CameraChain {
private:
  std::map<int, std::map<int, std::vector<Mat4>>> adjlist_;

public:
  CameraChain() = default;
  CameraChain(const std::map<int, CameraGeometryPtr> &camera_geometries,
              const std::map<int, CameraData> &camera_measurements);
  virtual ~CameraChain() = default;

  /** Insert link beteen camera i and j */
  void insert(const int i, const int j, const Mat4 &T_ij);

  /** Find camera extrinsics `T_ij` between `i` and `j` */
  int find(const int i, const int j, Mat4 &T_ij) const;
};

} // namespace xyz
