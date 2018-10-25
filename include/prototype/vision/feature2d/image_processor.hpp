#ifndef PROTOTYPE_VISION_IMAGE_PROCESSOR_HPP
#define PROTOTYPE_VISION_IMAGE_PROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>

#include <map>
#include <vector>
#include <random>
#include <iostream>
#include <algorithm>

#include "prototype/core.hpp"
#include "prototype/vision/feature2d/util.hpp"
#include "prototype/vision/feature2d/grid_fast.hpp"
#include "prototype/vision/calib/calib.hpp"

namespace prototype {

/**
 * An alias for unsigned long long int.
 */
typedef unsigned long long int FeatureIDType;

/**
 * Meta information of a feature
 */
struct FeatureMetaData {
  FeatureIDType id;
  cv::Point2f cam0_point;
  cv::Point2f cam1_point;
  int lifetime;
  float response;

  FeatureMetaData() {}

  FeatureMetaData(const cv::Point2f &cam0_point,
                  const cv::Point2f &cam1_point,
                  const float response)
      : cam0_point{cam0_point}, cam1_point{cam1_point}, response{response} {}

  FeatureMetaData(const FeatureIDType &id,
                  const cv::Point2f &cam0_point,
                  const cv::Point2f &cam1_point,
                  int lifetime,
                  const float response)
      : id{id}, cam0_point{cam0_point}, cam1_point{cam1_point},
        lifetime{lifetime}, response{response} {}
};

/**
 * Organize features based on the grid they belong to. Note that the key is
 * encoded by the grid index.
 */
typedef std::map<int, std::vector<FeatureMetaData>> GridFeatures;

/**
 * Extract grid feature
 * - IDs
 * - Lifetimes
 * - Camera0 points
 * - Camera1 points
 *
 * @param grid_features Grid features
 * @param ids Feature IDs
 * @param lifetime Feature lifetimes
 * @param cam0_points Camera0 points
 * @param cam1_points Camera1 points
 */
void grid_features_extract_all(const GridFeatures &grid_features,
                               std::vector<FeatureIDType> &ids,
                               std::vector<int> &lifetimes,
                               std::vector<cv::Point2f> &cam0_points,
                               std::vector<cv::Point2f> &cam1_points);

/**
 * Extract grid feature ids
 *
 * @param grid_features Grid features
 * @param ids Feature IDs
 */
void grid_features_extract_ids(const GridFeatures &grid_features,
                               std::vector<FeatureIDType> &ids);

/**
 * Extract grid feature
 * - Camera0 points
 * - Camera1 points
 *
 * @param grid_features Grid features
 * @param cam0_points Camera0 points
 * @param cam1_points Camera1 points
 */
void grid_features_extract_points(const GridFeatures &grid_features,
                                  std::vector<cv::Point2f> &cam0_points,
                                  std::vector<cv::Point2f> &cam1_points);

/**
 * Extract grid feature
 * - Camera0 points
 * - Camera1 points
 *
 * @param grid_features Grid features
 * @param cam0_points Camera0 points
 * @param cam1_points Camera1 points
 */
void grid_features_extract_points(
    const GridFeatures &grid_features,
    std::map<FeatureIDType, cv::Point2f> &cam0_points,
    std::map<FeatureIDType, cv::Point2f> &cam1_points);

/**
 * KLT feature tracker
 *
 * @param cam0_img_pyramid Camera0 image pyramids
 * @param cam1_img_pyramid Camera1 image pyramids
 * @param cam0_points Camera0 points
 * @param cam1_points Camera1 points
 * @param max_iteration Max iteration
 * @param track_precision Track precision
 * @param patch_size Patch size
 * @param pyramid_levels pyramid_levels
 * @param image_width Image width
 * @param image_height Image height
 * @param inlier_markers Inliers
 */
void klt_track_features(const std::vector<cv::Mat> &cam0_img_pyramid,
                        const std::vector<cv::Mat> &cam1_img_pyramid,
                        const std::vector<cv::Point2f> &cam0_points,
                        const std::vector<cv::Point2f> &cam1_points,
                        const int max_iterations,
                        const int track_precision,
                        const int patch_size,
                        const int pyramid_levels,
                        const int image_width,
                        const int image_height,
                        std::vector<uchar> &inlier_markers);

/**
 * Draw tracked stereo points
 *
 * @param stereo_image Stereo image
 * @param prev_cam0_points Previous camera0 points
 * @param prev_cam1_points Previous camera1 points
 * @param curr_cam0_points Current camera0 points
 * @param curr_cam1_points Current camera1 points
 * @param color Color
 *
 * @returns Output image
 */
cv::Mat draw_tracked_stereo_points(
    const cv::Mat &stereo_image,
    const std::vector<FeatureIDType> &prev_ids,
    std::map<FeatureIDType, cv::Point2f> &prev_cam0_points,
    std::map<FeatureIDType, cv::Point2f> &prev_cam1_points,
    std::map<FeatureIDType, cv::Point2f> &curr_cam0_points,
    std::map<FeatureIDType, cv::Point2f> &curr_cam1_points,
    const cv::Scalar color = cv::Scalar(0, 255, 0));

/**
 * Draw new stereo points
 *
 * @param stereo_image Stereo image
 * @param cam0_points Camera0 points
 * @param cam1_points Camera1 points
 * @param color Color
 *
 * @returns Output image
 */
cv::Mat
draw_new_stereo_points(const cv::Mat &stereo_image,
                       const std::map<FeatureIDType, cv::Point2f> &cam0_points,
                       const std::map<FeatureIDType, cv::Point2f> &cam1_points,
                       const cv::Scalar &color = cv::Scalar(0, 255, 255));

/**
 * Compare two features based on the response.
 */
static bool feature_compare_by_response(const FeatureMetaData &f1,
                                        const FeatureMetaData &f2) {
  // Features with higher response will be at the beginning of the vector
  return f1.response > f2.response;
}

/**
 * Compare two features based on the lifetime.
 */
static bool feature_compare_by_lifetime(const FeatureMetaData &f1,
                                        const FeatureMetaData &f2) {
  // Features with longer lifetime will be at the beginning of the vector
  return f1.lifetime > f2.lifetime;
}

/**
 * Image processor
 */
class ImageProcessor {
public:
  /**
  * IMU data
  */
  struct IMUData {
    vec3_t a_B = vec3_t::Zero();
    vec3_t w_B = vec3_t::Zero();
    long ts = 0;

    IMUData() {}
    IMUData(const vec3_t &a_B, const vec3_t &w_B, const long ts)
        : a_B{a_B}, w_B{w_B}, ts{ts} {}
  };

  /**
  * Camera frame
  */
  struct CameraFrame {
    cv::Mat data;
    long ts = 0;

    CameraFrame() {}
    CameraFrame(const cv::Mat &data, const long ts) : data{data}, ts{ts} {}
  };

  CameraProperty cam0;
  CameraProperty cam1;
  mat4_t T_imu_cam0 = mat4_t::Zero();
  mat4_t T_cam1_cam0 = mat4_t::Zero();

  double fast_threshold = 0;
  int fast_max_corners = 0;
  bool fast_nonmax_suppression = false;
  bool fast_debug = false;

  int max_cell_features = 10;
  int min_cell_features = 5;

  int grid_rows = 0;
  int grid_cols = 0;
  int nb_grid_cells = 0;
  int pyramid_levels = 0;
  int patch_size = 0;
  int max_iterations = 0;
  double track_precision = 0.0;
  double ransac_threshold = 0.0;
  double stereo_threshold = 0.0;

  int image_width = 0;
  int image_height = 0;
  CameraFrame prev_cam0_img;
  CameraFrame prev_cam1_img;
  std::vector<cv::Mat> prev_cam0_img_pyramid;
  std::vector<cv::Mat> curr_cam0_img_pyramid;
  std::vector<cv::Mat> curr_cam1_img_pyramid;

  std::vector<IMUData> imu_buffer;

  ///< Intialized
  bool initialized = false;

  ///< ID for next new feature
  FeatureIDType next_feature_id;

  ///< Features in previous image
  GridFeatures prev_features;

  ///< Features in current image
  GridFeatures curr_features;

  // Tracking stats
  int before_tracking;
  int after_tracking;
  int after_matching;
  int after_ransac;

  // Debugging
  std::map<FeatureIDType, int> feature_lifetime;

  ImageProcessor();
  ImageProcessor(const ImageProcessor &) = delete;
  ImageProcessor operator=(const ImageProcessor &) = delete;
  ~ImageProcessor();

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Create image pyramids
   *
   * @param cam0_img Camera0 image
   * @param cam1_img Camera1 image
   */
  void createImagePyramids(const cv::Mat &cam0_img, const cv::Mat &cam1_img);

  /**
   * Project points from camera0 to camera1
   *
   * @param cam0_points cam0 points
   * @return cam1_points projected cam1 points
   */
  std::vector<cv::Point2f> projectPointsFromCam0ToCam1(
      const std::vector<cv::Point2f> &cam0_points, const mat4_t &T_cam1_cam0);

  /**
   * Initialize
   *
   * @param cam0_img Camera0 image
   */
  void initialize(const cv::Mat &cam0_img);

  /**
   * Matches features with stereo image pairs
   *
   * @param cam0_points points in the primary image
   * @param cam1_points points in the secondary image
   * @return inlier_markers 1 if the match is valid, 0 otherwise
   */
  void stereoMatch(const std::vector<cv::Point2f> &cam0_points,
                   std::vector<cv::Point2f> &cam1_points,
                   std::vector<unsigned char> &inlier_markers);

  /**
   * Tracker features on the newly received stereo images
   *
   * @param ts Timestamp
   */
  void trackFeatures(const long ts);

  /**
   * Detect new features on the image to ensure features are uniformly
   * distributed on the image
   */
  void addNewFeatures(const cv::Mat &cam0_img);

  /**
   * Remove excess features in each grid cell to ensure number is bounded
   */
  void pruneGridFeatures();

  /**
   * Integrate IMU gyro readings between the two consecutive images, which is
   * used for both tracking prediction and 2-point RANSAC.
   *
   * @param curr_ts Current timestamp
   * @param cam0_R_p_c Rotation matrix from previous to current cam0 frame
   * @param cam1_R_p_c Rotation matrix from previous to current cam1 frame
   */
  void integrateImuData(const long curr_ts,
                        cv::Matx33f &cam0_R_p_c,
                        cv::Matx33f &cam1_R_p_c);

  /**
   * Compensate the rotation between consecutive camera frames for robust and
   * fast feature tracking
   *
   * Note: that the input and output points are of pixel coordinates
   *
   * @param input_pts Features in the previous image to be tracked
   * @param R_p_c Rotation matrix from previous to current camera frame
   * @param intrinsics intrinsic matrix of the camera
   * @return compensated_pts Predicted feature locations in current image
   */
  void predictFeatures(const std::vector<cv::Point2f> &input_pts,
                       const cv::Matx33f &R_p_c,
                       const cv::Vec4d &intrinsics,
                       std::vector<cv::Point2f> &compenstated_pts);

  /**
   * Rescale points
   *
   * @param pts1 Points 1
   * @param pts2 Points 2
   * @param scaling_factor Scaling factor
   */
  void rescalePoints(std::vector<cv::Point2f> &pts1,
                     std::vector<cv::Point2f> &pts2,
                     float &scaling_factor);

  /**
   * Remove unmarked elements within a vector
   *
   * Note: that the order of the inliers in the raw_vec is perserved in the
   * refined_vec
   *
   * @param raw_vec vector with outliers
   * @param markers 0 will represent a outlier, 1 will be an inlier
   * @return refined_vec Vector without outliers
   */
  template <typename T>
  void removeUnmarkedElements(const std::vector<T> &raw_vec,
                              const std::vector<unsigned char> &markers,
                              std::vector<T> &refined_vec) {
    assert(raw_vec.size() == markers.size());

    for (size_t i = 0; i < markers.size(); ++i) {
      if (markers[i] == 0) {
        continue;
      }
      refined_vec.push_back(raw_vec[i]);
    }
  }

  /**
   * Update feature lifetime
   */
  void updateFeatureLifetime();

  /**
   * Feature lifetime statistics
   */
  void printFeatureLifetimeStatistics();

  /**
   * Callback for IMU measurement
   *
   * @param a_m Measured acceleration
   * @param w_m Measured angular velocity
   * @param ts Timestamp
   */
  void imuCallback(const vec3_t &a_m, const vec3_t &w_m, const long ts);

  /**
   * Callback function for the stereo images
   *
   * @param cam0_img Camera0 image
   * @param cam1_img Camera1 image
   * @param ts Timestamp
   */
  void stereoCallback(const cv::Mat &cam0_img,
                      const cv::Mat &cam1_img,
                      const long ts);

  /**
   * Draw tracked and newly detected features
   *
   * @param cam0_img Camera0 image
   * @param cam1_img Camera1 image
   */
  void drawFeatures(const cv::Mat &cam0_img, const cv::Mat &cam1_img);
};

} //  namespace prototype
#endif
