#pragma once
#include "Core.hpp"

namespace cartesian {
namespace SO3 {

Vec3 log(const Mat3 &R);
Mat3 exp(const Vec3 &w);
Mat3 solve_handeye(const std::map<timestamp_t, Mat4> &A,
                   const std::map<timestamp_t, Mat4> &B);

} // namespace SO3
} // namespace cartesian
