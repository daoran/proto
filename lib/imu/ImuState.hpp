#pragma once
#include "Core.hpp"

namespace xyz {

class ImuState {
private:
  timestamp_t ts_;
  VecX pose_;
  VecX speed_biases_;

public:
  ImuState(const timestamp_t ts, const VecX &pose, const Vec3 &vel);

  /** Create shared_ptr */
  static std::shared_ptr<ImuState> create(const timestamp_t ts,
                                          const VecX &pose,
                                          const Vec3 &vel);

  /** Return timestamp */
  timestamp_t getTimestamp() const;

  /** Return pose */
  VecX getPose() const;

  /** Return speed and biases */
  VecX getSpeedBiases() const;

  /** Return pose pointer */
  double *getPosePtr();

  /** Return speed biases pointer */
  double *getSpeedBiasesPtr();
};

} // namespace xyz
