/**
 * @file
 * @ingroup calib
 */
#ifndef PROTOTYPE_CALIB_CALIB_HPP
#define PROTOTYPE_CALIB_CALIB_HPP

#include <string>

#include <opencv2/calib3d/calib3d.hpp>

#include "prototype/calib/aprilgrid.hpp"
#include "prototype/core.hpp"
#include "prototype/vision/util.hpp"
#include "prototype/vision/camera/camera_geometry.hpp"
#include "prototype/vision/camera/pinhole.hpp"
#include "prototype/vision/camera/radtan.hpp"
#include "prototype/vision/camera/equi.hpp"

namespace prototype {
/**
 * @addtogroup calib
 * @{
 */

/**
 * Calibration target
 */
struct calib_target_t {
  std::string target_type;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;

  calib_target_t();
  ~calib_target_t();
};

/**
 * calib_target_t to output stream
 */
std::ostream &operator<<(std::ostream &os, const calib_target_t &target);

/**
 * Load calibration target
 *
 * @param[in,out] ct Calibration target
 * @param[in] target_file Target file
 * @returns 0 and -1 for success or failure
 */
int calib_target_load(calib_target_t &ct, const std::string &target_file);

/**
 * Preprocess camera image data and output AprilGrid detection data as csv
 *
 * @param[in] target Calibration target properties
 * @param[in] image_dir Path to camera images
 * @param[in] image_size Image resolution [px]
 * @param[in] lens_hfov Horizontal fov of camera lens [deg]
 * @param[in] lens_vfov Vertical fov of camera lens [deg]
 * @param[in] output_dir Output path
 *
 * @returns 0 or -1 for success or failure
 */
int preprocess_camera_data(const calib_target_t &target,
                           const std::string &image_dir,
                           const vec2_t &image_size,
                           const double lens_hfov,
                           const double lens_vfov,
                           const std::string &output_dir);

/**
 * Load preprocess-ed camera calibration data
 *
 * @param[in] data_dir Path where calibration data was saved
 * @param[out] aprilgrids Detected aprilgrids
 * @returns 0 or -1 for success or failure
 */
int load_camera_calib_data(const std::string &data_dir,
                           std::vector<aprilgrid_t> &aprilgrids);

/**
 * Draw measured and projected pixel points
 *
 * @param[in] image Input image
 * @param[in] measured Measured image pixels
 * @param[in] projected Projected image pixels
 * @param[in] measured_color Measured color
 * @param[in] projected_color Projected color
 *
 * @returns Image
 */
cv::Mat draw_calib_validation(const cv::Mat &image,
                              const std::vector<vec2_t> &measured,
                              const std::vector<vec2_t> &projected,
                              const cv::Scalar &measured_color,
                              const cv::Scalar &projected_color);

/**
 * Validate calibration
 *
 * @param camera_index Camera index
 * @param image Input image
 * @returns Validation image for visual inspection
 */
template <typename CM, typename DM>
cv::Mat validate_intrinsics(const cv::Mat &image,
                            const std::vector<vec2_t> &keypoints,
                            const std::vector<vec3_t> &points,
                            const camera_geometry_t<CM, DM> &camera_geometry);

// /**
//   * Validate stereo calibration
//   *
//   * @param img0 Input image from cam0
//   * @param img1 Input image from cam1
//   * @returns Validation image for visual inspection
//   */
// cv::Mat validateStereo(const cv::Mat &img0, const cv::Mat &img1);

/** @} group calib */
} //  namespace prototype
#include "calib_impl.hpp"
#endif // PROTOTYPE_CALIB_CALIB_HPP
