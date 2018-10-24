/**
 * @file
 * @ingroup feature2d
 */
#ifndef PROTOTYPE_VISION_FEATURE2D_STEREO_ORB_TRACKER_HPP
#define PROTOTYPE_VISION_FEATURE2D_STEREO_ORB_TRACKER_HPP

#include "prototype/vision/camera/pinhole_model.hpp"
#include "prototype/vision/feature2d/feature_tracker.hpp"
#include "prototype/vision/feature2d/feature_container.hpp"
#include "prototype/vision/feature2d/orb_tracker.hpp"

namespace prototype {
/**
 * @addtogroup feature2d
 * @{
 */

/**
 * Stereo feature tracker
 */
class StereoORBTracker {
public:
  ORBTracker tracker0;
  ORBTracker tracker1;
  TrackID counter_track_id = 0;

  size_t min_track_length = 10;
  size_t max_track_length = 20;
  bool show_matches = false;

  StereoORBTracker();
  StereoORBTracker(const CameraProperty &camprop0,
                   const CameraProperty &camprop1,
                   const size_t min_track_length,
                   const size_t max_track_length);
  virtual ~StereoORBTracker();

  /**
   * Initialize stereo feature tracker
   *
   * @param cam0_img Current image frame from camera0
   * @param cam1_img Current image frame from camera1
   * @returns 0 for success, -1 for failure
   */
  int initialize(const cv::Mat &cam0_img, const cv::Mat &cam1_img);

  /**
   * Update feature tracker
   *
   * @param cam0_img Current image frame from camera0
   * @param cam1_img Current image frame from camera1
   * @returns 0 for success, -1 for failure
   */
  int update(const cv::Mat &cam0_img, const cv::Mat &cam1_img);

  /**
   * Update feature tracker
   *
   * @param cam0_img Current image frame from camera0
   * @param cam1_img Current image frame from camera1
   * @returns 0 for success, -1 for failure
   */
  int update2(const cv::Mat &cam0_img, const cv::Mat &cam1_img, long ts);

  /**
   * Get lost feature tracks
   *
   * @returns List of feature track
   */
  std::vector<FeatureTrack> getLostTracks();
};

/** @} group feature2d */
} //  namespace prototype
#endif // PROTOTYPE_VISION_FEATURE2D_STEREO_ORB_TRACKER_HPP
