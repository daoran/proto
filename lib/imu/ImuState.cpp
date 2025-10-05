#include "ImuState.hpp"

namespace xyz {

ImuState::ImuState(const timestamp_t ts, const VecX &pose, const Vec3 &vel)
    : ts_{ts}, pose_{pose} {
  speed_biases_.resize(9);
  speed_biases_.segment<3>(0) = vel;
  speed_biases_.segment<3>(3) = Vec3{0.0, 0.0, 0.0};
  speed_biases_.segment<3>(6) = Vec3{0.0, 0.0, 0.0};
}

std::shared_ptr<ImuState> ImuState::create(const timestamp_t ts,
                                           const VecX &pose,
                                           const Vec3 &vel) {
  return std::make_shared<ImuState>(ts, pose, vel);
}

timestamp_t ImuState::getTimestamp() const { return ts_; }

VecX ImuState::getPose() const { return pose_; }

VecX ImuState::getSpeedBiases() const { return speed_biases_; }

double *ImuState::getPosePtr() { return pose_.data(); }

double *ImuState::getSpeedBiasesPtr() { return speed_biases_.data(); }

} // namespace xyz
