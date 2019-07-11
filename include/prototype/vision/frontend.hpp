#ifndef PROTOTYPE_VISION_FRONTEND_HPP
#define PROTOTYPE_VISION_FRONTEND_HPP

#include "prototype/core/core.hpp"
#include "prototype/vision/vision_common.hpp"

namespace proto {

struct frontend_t {
  std::vector<cv::Point2f> keypoints;
  cv::Mat image_prev;

  frontend_t();
  ~frontend_t();
};

int frontend_update(frontend_t &frontend, const cv::Mat &image);

} //  namespace proto
#endif // PROTOTYPE_VISION_FRONTEND_HPP
