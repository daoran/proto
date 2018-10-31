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

/**
 * Preprocess mono camera image data and output AprilGrid detection data as csv
 *
 * @param[in] target Calibration target properties
 * @param[in] image_dir Path to camera images
 * @param[in] image_size Image resolution
 * @param[in] lens_hfov Horizontal fov of camera lens
 * @param[in] lens_vfov Vertical fov of camera lens
 * @param[in] output_dir Output path
 *
 * @returns 0 or -1 for success or failure
 */
int preprocess_mono_data(const calib_target_t &target,
                         const std::string &image_dir,
                         const vec2_t image_size,
                         const double lens_hfov,
                         const double lens_vfov,
                         const std::string &output_dir);

/**
 * Load preprocess-ed mono camera calibration data
 *
 * @param[in] data_dir Path where calibration data was saved
 * @param[out] aprilgrids Detected aprilgrids
 * @returns 0 or -1 for success or failure
 */
int load_mono_calib_data(const std::string &data_dir,
                         std::vector<aprilgrid_t> &aprilgrids);

/** @} group calib */
} //  namespace prototype
#endif // PROTOTYPE_CALIB_CALIB_DATA_HPP
