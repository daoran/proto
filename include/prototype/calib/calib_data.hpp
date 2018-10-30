/**
 * @file
 * @ingroup calib
 */
#ifndef PROTOTYPE_CALIB_CALIB_DATA_HPP
#define PROTOTYPE_CALIB_CALIB_DATA_HPP

#include <string>

#include <opencv2/calib3d/calib3d.hpp>

#include "prototype/core.hpp"
#include "prototype/calib/aprilgrid.hpp"
#include "prototype/calib/calib_target.hpp"
#include "prototype/vision/camera/pinhole.hpp"

namespace prototype {
/**
 * @addtogroup calib
 * @{
 */

int process_mono_data(const calib_target_t &target,
                      const std::string &image_dir,
                      const vec2_t image_size,
                      const double lens_hfov,
                      const double lens_vfov,
                      const std::string &output_dir);

/** @} group calib */
} //  namespace prototype
#endif // PROTOTYPE_CALIB_CALIB_DATA_HPP
