#pragma once
#include "CameraModel.hpp"

namespace xyz {

/** Camera Geometry */
class CameraGeometry {
private:
  int camera_index_;
  std::shared_ptr<CameraModel> camera_model_;
  Vec2i resolution_;
  VecX intrinsic_;
  VecX extrinsic_;

public:
  CameraGeometry() = delete;
  CameraGeometry(const int camera_index,
                 const std::string &camera_model,
                 const Vec2i &resolution,
                 const VecX &intrinsic,
                 const VecX &extrinsic);
  virtual ~CameraGeometry();

  /** Get camera index **/
  int getCameraIndex() const;

  /** Get camera model **/
  CameraModel *getCameraModel() const;

  /** Get camera model string **/
  std::string getCameraModelString() const;

  /** Get resoultion **/
  Vec2i getResolution() const;

  /** Get intrinsic **/
  VecX getIntrinsic() const;

  /** Get extrinsic **/
  VecX getExtrinsic() const;

  /** Get intrinsic pointer **/
  double *getIntrinsicPtr();

  /** Get extrinsic **/
  double *getExtrinsicPtr();

  /** Get transform T_body_camera **/
  Mat4 getTransform() const;

  /** Set extrinsic */
  void setExtrinsic(const Mat4 &extrinsic);
};

} // namespace xyz
