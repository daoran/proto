#pragma once
#include "CameraModel.hpp"

namespace xyz {

// Forward declaration
struct CameraGeometry;
using CameraGeometryPtr = std::shared_ptr<CameraGeometry>;

/** Camera Geometry */
struct CameraGeometry {
  int camera_id;
  std::shared_ptr<CameraModel> camera_model;
  Vec2i resolution;
  VecX intrinsic;
  VecX extrinsic;

  CameraGeometry() = delete;
  CameraGeometry(const int camera_id_,
                 const std::string &camera_model_,
                 const Vec2i &resolution_,
                 const VecX &intrinsic_,
                 const VecX &extrinsic_);
  virtual ~CameraGeometry() = default;

  /** Set Extrinsic */
  void setExtrinsic(const Mat4 &transform);
};

} // namespace xyz
