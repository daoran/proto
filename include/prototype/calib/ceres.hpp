#ifndef PROTOTYPE_CALIB_CERES_HPP
#define PROTOTYPE_CALIB_CERES_HPP

#include "prototype/core/core.hpp"

namespace prototype {

/**
 * Form a 4x4 homogeneous transformation matrix from a
 * rotation matrix R and translation vector t.
 *
 * @param C Rotation matrix
 * @param r Translation vector
 * @return T Transformation matrix
 */
template <typename T>
Eigen::Matrix<T, 4, 4> tf(const Eigen::Matrix<T, 3, 3> &C,
                          const Eigen::Matrix<T, 3, 1> &r);

/**
 * Pinhole camera matrix K
 */
template <typename T>
static Eigen::Matrix<T, 3, 3> pinhole_K(const T *intrinsics);

/**
 * Create Equidistant distortion vector
 */
template <typename T>
Eigen::Matrix<T, 4, 1> equi4_D(const T *distortion);

/**
 * Create Radial-tangential distortion vector
 */
template <typename T>
Eigen::Matrix<T, 4, 1> radtan4_D(const T *distortion);

/**
 * Project point using pinhole radial-tangential
 */
template <typename T>
static Eigen::Matrix<T, 2, 1>
pinhole_radtan4_project(const Eigen::Matrix<T, 3, 3> &K,
                        const Eigen::Matrix<T, 4, 1> &D,
                        const Eigen::Matrix<T, 3, 1> &point);

}  // namespace prototype
#include "ceres_impl.hpp"
#endif // PROTOTYPE_CALIB_CERES_HPP
