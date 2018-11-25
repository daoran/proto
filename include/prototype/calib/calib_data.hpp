#ifndef PROTOTYPE_CALIB_CALIB_DATA_HPP
#define PROTOTYPE_CALIB_CALIB_DATA_HPP

#include <string>

#include <opencv2/calib3d/calib3d.hpp>

#include "prototype/calib/aprilgrid.hpp"
#include "prototype/core/core.hpp"
#include "prototype/vision/util.hpp"
#include "prototype/vision/camera/camera_geometry.hpp"
#include "prototype/vision/camera/pinhole.hpp"
#include "prototype/vision/camera/radtan.hpp"
#include "prototype/vision/camera/equi.hpp"

namespace prototype {

/**
 * Pose parameter block
 */
struct calib_pose_param_t {
  quat_t q;
  vec3_t t;

  calib_pose_param_t(const mat4_t &T);
  ~calib_pose_param_t();
};

/**
 * Calibration target.
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
 * Load calibration target.
 * @returns 0 or -1 for success or failure
 */
int calib_target_load(calib_target_t &ct, const std::string &target_file);

/**
 * Preprocess camera image data and output AprilGrid detection data as
 * csv. The data is initialized with `image_size` in pixels, the horizontal
 * lens fov `lens_hfov` and vertical lens fov `lens_vfov` in degrees.
 * @returns 0 for Success, -1 for failure, and 1 where the output directory
 * contains data.
 */
int preprocess_camera_data(const calib_target_t &target,
                           const std::string &image_dir,
                           const vec2_t &image_size,
                           const double lens_hfov,
                           const double lens_vfov,
                           const std::string &output_dir);

// TODO: Need to preprocess stereo camera data

/**
 * Load preprocess-ed camera calibration data.
 * @returns 0 or -1 for success or failure
 */
int load_camera_calib_data(const std::string &data_dir,
                           std::vector<aprilgrid_t> &aprilgrids);

/**
 * Draw measured and projected pixel points.
 * @returns Image
 */
cv::Mat draw_calib_validation(const cv::Mat &image,
                              const std::vector<vec2_t> &measured,
                              const std::vector<vec2_t> &projected,
                              const cv::Scalar &measured_color,
                              const cv::Scalar &projected_color);

/**
 * Validate calibration.
 * @returns Validation image for visual inspection
 */
template <typename CM, typename DM>
cv::Mat validate_intrinsics(const cv::Mat &image,
                            const std::vector<vec2_t> &keypoints,
                            const std::vector<vec3_t> &points,
                            const camera_geometry_t<CM, DM> &camera_geometry);

/**
 * Validate stereo extrinsics.
 * @returns Validation image for visual inspection
 */
template <typename CM, typename DM>
cv::Mat validate_stereo(const cv::Mat &image0,
                        const cv::Mat &image1,
                        const std::vector<vec2_t> &kps0,
                        const std::vector<vec3_t> &points0,
                        const std::vector<vec2_t> &kps1,
                        const std::vector<vec3_t> &points1,
                        const camera_geometry_t<CM, DM> &cam0,
                        const camera_geometry_t<CM, DM> &cam1,
                        const mat4_t &T_C1C0);

/**
 * `calib_target_t` to output stream.
 */
std::ostream &operator<<(std::ostream &os, const calib_target_t &target);

} // namespace prototype
#include "calib_data_impl.hpp"
#endif // PROTOTYPE_CALIB_CALIB_DATA_HPP
