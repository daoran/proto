#ifndef PROTO_VISION_FRONTEND_HPP
#define PROTO_VISION_FRONTEND_HPP

#include "proto/core/core.hpp"
#include "proto/vision/vision.hpp"

namespace proto {

struct frontend_t {
  std::vector<cv::Point2f> keypoints;
  cv::Mat image_prev;

  frontend_t();
  ~frontend_t();
};

int frontend_update(frontend_t &fe,
                    const cv::Mat &image,
                    const bool debug = false);

} //  namespace proto
#endif // PROTO_VISION_FRONTEND_HPP
