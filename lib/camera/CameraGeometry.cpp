#include "CameraGeometry.hpp"

namespace xyz {

CameraGeometry::CameraGeometry(const int camera_index,
                               const std::string &camera_model,
                               const Vec2i &resolution,
                               const VecX &intrinsic,
                               const VecX &extrinsic)
    : camera_index_{camera_index}, resolution_{resolution},
      intrinsic_{intrinsic}, extrinsic_{extrinsic} {
  // Initialize camera model
  if (camera_model == "pinhole-radtan4") {
    camera_model_ = std::make_shared<PinholeRadtan4>();
  } else if (camera_model == "pinhole-equi4") {
    camera_model_ = std::make_shared<PinholeEqui4>();
  } else {
    FATAL("Unsupported camera model [%s]", camera_model.c_str());
  }

  // Initialize camera intrinsic
  if (intrinsic.size() != 0) {
    return;
  }
  const double fx = pinhole_focal(resolution.x(), 90.0);
  const double fy = pinhole_focal(resolution.x(), 90.0);
  const double cx = resolution.x() / 2.0;
  const double cy = resolution.y() / 2.0;

  int intrinsic_size = 0;
  if (camera_model == "pinhole-radtan4") {
    intrinsic_size = 8;
  } else if (camera_model == "pinhole-equi4") {
    intrinsic_size = 8;
  }

  intrinsic_.resize(intrinsic_size);
  intrinsic_.setZero();
  intrinsic_[0] = fx;
  intrinsic_[1] = fy;
  intrinsic_[2] = cx;
  intrinsic_[3] = cy;
}

int CameraGeometry::getCameraIndex() const { return camera_index_; }

std::shared_ptr<CameraModel> CameraGeometry::getCameraModel() const {
  return camera_model_;
}

std::string CameraGeometry::getCameraModelString() const {
  return camera_model_->type;
}

Vec2i CameraGeometry::getResolution() const { return resolution_; }

VecX CameraGeometry::getIntrinsic() const { return intrinsic_; }

VecX CameraGeometry::getExtrinsic() const { return extrinsic_; }

double *CameraGeometry::getIntrinsicPtr() { return intrinsic_.data(); }

double *CameraGeometry::getExtrinsicPtr() { return extrinsic_.data(); }

Mat4 CameraGeometry::getTransform() const { return tf(extrinsic_); }

void CameraGeometry::setExtrinsic(const Mat4 &extrinsic) {
  extrinsic_ = tf_vec(extrinsic);
}

} // namespace xyz
