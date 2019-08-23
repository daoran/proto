#ifndef PROTO_VISION_FEATURE2D_GRID_FAST_HPP
#define PROTO_VISION_FEATURE2D_GRID_FAST_HPP

#include <cstring>
#include <math.h>
#include <vector>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "proto/core/core.hpp"
#include "proto/vision/vision_common.hpp"

namespace proto {

/**
 * Grid fast
 *
 * @param[in] image Input image
 * @param[in] max_corners Max number of corners
 * @param[in] grid_rows Number of grid rows
 * @param[in] grid_cols Number of grid cols
 * @param[in] threshold Fast threshold
 * @param[in] nonmax_suppression Nonmax Suppression
 *
 * @returns List of keypoints
 */
std::vector<cv::KeyPoint> grid_fast(const cv::Mat &image,
                                    const int max_corners = 100,
                                    const int grid_rows = 5,
                                    const int grid_cols = 5,
                                    const double threshold = 10.0,
                                    const bool nonmax_suppression = true);

} //  namespace proto
#endif // PROTO_VISION_FEATURE2D_GRID_FAST_HPP
