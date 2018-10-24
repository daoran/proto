/**
 * @file
 * @ingroup feature2d
 */
#ifndef PROTOTYPE_VISION_FEATURE2D_ORB_TRACKER_HPP
#define PROTOTYPE_VISION_FEATURE2D_ORB_TRACKER_HPP

#include "prototype/vision/feature2d/feature_tracker.hpp"
#include "prototype/vision/feature2d/grid_fast.hpp"

namespace prototype {
/**
 * @addtogroup feature2d
 * @{
 */

/**
 * ORB based feature tracker
 */
class ORBTracker : public FeatureTracker {
public:
  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  bool use_grid_fast = false;

  ORBTracker();
  ORBTracker(const CameraProperty &camera_property);
  ORBTracker(const CameraProperty &camera_property,
             const size_t min_track_length,
             const size_t max_track_length);
  virtual ~ORBTracker();

  /**
   * Detect features
   *
   * @param image Input image
   * @param features List of features
   * @returns 0 for success, -1 for failure
   */
  virtual int detect(const cv::Mat &image, Features &features) override;
};

/** @} group feature2d */
} //  namespace prototype
#endif // PROTOTYPE_VISION_FEATURE2D_ORB_TRACKER_HPP
