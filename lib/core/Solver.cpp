#include "Solver.hpp"

namespace xyz {

Solver::setVerbose(const bool verbose) { verbose_ = verbose; }

Solver::setMaxIter(const int max_iter) { max_iter_ = max_iter; }

size_t Solver::getNumResidualBlocks() const { return residual_blocks_.size(); }

void Solver::addCameraGeometry(
    std::shared_ptr<CameraGeometry> &camera_geometry) {
  const int camera_index = camera_geometry->getCameraIndex();
  camera_geometry_[camera_index] = camera_geometry;
}

void Solver::addImuGeometry(std::shared_ptr<ImuGeometry> &imu_geometry) {
  const int imu_index = imu_geometry->getImuIndex();
  imu_geometry_[imu_index] = imu_geometry;
}

} // namespace xyz
