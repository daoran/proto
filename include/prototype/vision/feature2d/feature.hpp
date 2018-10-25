/**
 * @file
 * @ingroup feature2d
 */
#ifndef PROTOTYPE_VISION_FEATURE2D_FEATURE_HPP
#define PROTOTYPE_VISION_FEATURE2D_FEATURE_HPP

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "prototype/core.hpp"

namespace prototype {
/**
 * @addtogroup feature2d
 * @{
 */

/**
 * Track ID
 */
using TrackID = long int;

/**
 * Feature
 */
struct Feature {
  TrackID track_id = -1;
  cv::KeyPoint kp;
  cv::Mat desc;

  vec3_t ground_truth = vec3_t::Zero();

  Feature();
  Feature(const vec2_t &pt);
  Feature(const vec2_t &pt, const vec3_t &ground_truth);
  Feature(const cv::Point2f &pt);
  Feature(const cv::KeyPoint &kp);
  Feature(const cv::KeyPoint &kp, const cv::Mat &desc);

  /**
   * Set feature track ID
   *
   * @param track_id Track ID
   */
  void setTrackID(const TrackID &track_id);

  /**
   * Return feature as vec2_t
   */
  vec2_t getKeyPoint();

  /**
   * Return feature as vec2_t
   */
  vec2_t getKeyPoint() const;
};

/**
  * Feature to string
  */
std::ostream &operator<<(std::ostream &os, const Feature &f);

/**
 * Features
 */
using Features = std::vector<Feature>;

/** @} group feature2d */
} //  namespace prototype
#endif // PROTOTYPE_VISION_FEATURE2D_FEATURE_HPP
