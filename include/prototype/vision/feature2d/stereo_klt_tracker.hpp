/**
 * @file
 * @ingroup feature2d
 */
#ifndef PROTOTYPE_VISION_FEATURE2D_STEREO_KLT_TRACKER_HPP
#define PROTOTYPE_VISION_FEATURE2D_STEREO_KLT_TRACKER_HPP

#include "prototype/model/gimbal.hpp"
#include "prototype/vision/util.hpp"
#include "prototype/vision/camera/pinhole_model.hpp"
#include "prototype/vision/feature2d/grid_fast.hpp"
#include "prototype/vision/feature2d/grid_good.hpp"
#include "prototype/vision/feature2d/klt_tracker.hpp"
#include "prototype/vision/feature2d/feature_tracker.hpp"
#include "prototype/vision/feature2d/feature_container.hpp"

namespace prototype {
/**
 * @addtogroup feature2d
 * @{
 */

/**
 * Stereo feature tracker
 */
class StereoKLTTracker {
public:
  int type = -1;
  CameraProperty cam0;
  CameraProperty cam1;

  mat4_t T_cam1_cam0 = mat4_t::Zero();
  GimbalModel gimbal_model;
  vec2_t joint_angles_prev = vec2_t::Zero();
  vec2_t joint_angles_curr = vec2_t::Zero();

  TrackID counter_frame_id = -1;
  FeatureContainer features;
  std::vector<cv::Point2f> cam0_pts;
  std::vector<cv::Point2f> cam1_pts;
  std::vector<int> track_ids;
  cv::Mat prev_cam0_img;
  cv::Mat prev_cam1_img;
  std::vector<cv::Point2f> pts0_new;
  std::vector<cv::Point2f> pts1_new;

  // Detector settings
  int max_corners = 1000;
  double threshold = 3.0;
  int grid_rows = 5;
  int grid_cols = 5;
  double quality_level = 0.01;
  double min_distance = 3.0;

  // LK settings
  int pyramid_levels = 3;
  int max_iteration = 30;
  double track_precision = 0.01;

  // Misc settings
  int image_width = 0;
  int image_height = 0;
  bool show_matches = false;
  bool show_tracking = false;

  // Stats
  struct FeatureContainerStats stats;

  StereoKLTTracker();

  StereoKLTTracker(const CameraProperty &camprop0,
                   const CameraProperty &camprop1,
                   const mat4_t &T_cam1_cam0,
                   const size_t min_track_length,
                   const size_t max_track_length);

  StereoKLTTracker(const CameraProperty &camprop0,
                   const CameraProperty &camprop1,
                   const GimbalModel &gimbal_model,
                   const size_t min_track_length,
                   const size_t max_track_length);

  virtual ~StereoKLTTracker();

  /**
   * Detect corners in image
   *
   * @param image Input image
   * @param mask Mask
   *
   * @returns Corners detected [px]
   */
  std::vector<cv::Point2f> detect(const cv::Mat &image,
                                  const cv::Mat &mask = cv::Mat());

  /**
   * Match features spacially
   *
   * @param img0 First image
   * @param img1 Second image
   * @param pts0 Input points [px]
   * @param pts1 Output points [px]
   * @param mask Mask
   */
  void match(const cv::Mat &img0,
             const cv::Mat &img1,
             std::vector<cv::Point2f> &pts0,
             std::vector<cv::Point2f> &pts1,
             std::vector<uchar> &mask);

  /**
   * Update a feature track
   *
   * @param index Index of feature in StereoKLTTracker.track_ids
   * @param is_inlier Is feature an inlier
   * @param pt0_ref Feature point in camera 0 in previous frame
   * @param pt0_cur Feature point in camera 0 in current frame
   * @param pt1_ref Feature point in camera 1 in previous frame
   * @param pt1_cur Feature point in camera 1 in current frame
   *
   * @returns Track ID if updated or added to container, else returns -1 for
   * removed
   */
  int updateTrack(const int index,
                  const bool is_inlier,
                  const cv::Point2f &pt0_ref,
                  const cv::Point2f &pt0_cur,
                  const cv::Point2f &pt1_ref,
                  const cv::Point2f &pt1_cur);

  /**
   * Track features
   *
   * The idea is that with the current features, we want to match it against
   * the current list of FeatureTrack.
   *
   * @param cam0_img Camera 0 input image
   * @param cam1_img Camera 1 input image
   * @returns 0 for success, -1 for failure
   */
  void trackFeatures(const cv::Mat &img_ref, const cv::Mat &img_cur);

  /**
   * Initialize stereo feature tracker
   *
   * @param img0_cur Current image frame from camera0
   * @param img1_cur Current image frame from camera1
   *
   * @returns 0 for success, -1 for failure
   */
  int initialize(const cv::Mat &img0_cur, const cv::Mat &img1_cur);

  /**
   * Initialize stereo feature tracker
   *
   * @param img0_cur Current image frame from camera0
   * @param img1_cur Current image frame from camera1
   * @param gimbal_roll Gimbal roll angle [rad]
   * @param gimbal_pitch Gimbal pitch angle [rad]
   *
   * @returns 0 for success, -1 for failure
   */
  int initialize(const cv::Mat &img0_cur,
                 const cv::Mat &img1_cur,
                 const double gimbal_roll,
                 const double gimbal_pitch);

  /**
   * Replenish features
   *
   * @param image Image
   * @returns 0 for success, -1 for failure
   */
  void replenishFeatures(const cv::Mat &image);

  /**
   * Update feature tracker
   *
   * @param img0_cur Current image frame from camera0
   * @param img1_cur Current image frame from camera1
   * @returns 0 for success, -1 for failure
   */
  int update(const cv::Mat &img0_cur, const cv::Mat &img1_cur);

  /**
   * Update feature tracker
   *
   * @param img0_cur Current image frame from camera0
   * @param img1_cur Current image frame from camera1
   * @param gimbal_roll Gimbal roll angle [rad]
   * @param gimbal_pitch Gimbal pitch angle [rad]
   *
   * @returns 0 for success, -1 for failure
   */
  int update(const cv::Mat &img0_cur,
             const cv::Mat &img1_cur,
             const double gimbal_roll,
             const double gimbal_pitch);

  /**
   * Get lost feature tracks
   *
   * @returns List of feature track
   */
  std::vector<FeatureTrack> getLostTracks();

  /**
   * Show matches
   *
   * @param cam0_img Camera0 image
   * @param cam1_img Camera0 image
   */
  void showMatches(const cv::Mat &cam0_img, const cv::Mat &cam1_img);

  /**
   * Show tracking
   *
   * @param cam0_img Camera0 image
   */
  void showTracking(const cv::Mat &cam0_img);
};

/** @} group feature2d */
} //  namespace prototype
#endif // PROTOTYPE_VISION_FEATURE2D_STEREO_KLT_TRACKER_HPP
