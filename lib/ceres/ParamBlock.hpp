#pragma once
#include "../core/Core.hpp"

namespace xyz {

/** Parameter Block **/
struct ParamBlock {
  enum Type {
    POSE,
    EXTRINSIC,
    POINT,
    VELOCITY,
    BIAS,
    TIME_DELAY,
    INTRINSIC8,
    SPEED_BIASES,
    FIDUCIAL,
  };

  /** Get Parameter Size */
  static int getParamSize(const ParamBlock::Type type);

  /** Get Local Size */
  static int getLocalSize(const ParamBlock::Type type);

  /** Perturb parameter based on type */
  static void perturb(const ParamBlock::Type type,
                      const int i,
                      const double step,
                      double *ptr);
};

} // namespace xyz
