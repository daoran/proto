#pragma once
#include <cassert>

#include "Core.hpp"

#include <rerun.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/core.hpp>

namespace xyz {

class Logger {
private:
  std::unique_ptr<rerun::RecordingStream> rec_;

  /** Convert timestamp to std::chrono::time_point **/
  std::chrono::time_point<std::chrono::system_clock>
  convert(const timestamp_t ts);

  /** Convert cv::Mat to rerun::Image **/
  rerun::Image convert(const cv::Mat &image) const;

  /** Convert Mat4 to rerun::Transform3D **/
  rerun::Transform3D convert(const Mat4 &pose,
                             const float axis_length = 1.0) const;

  /** Convert std::vector<Vec3> to rerun::Points3D **/
  rerun::Points3D convert(const std::vector<Vec3> &points,
                          const std::vector<Vec3> &colors,
                          const std::vector<double> &radii) const;

public:
  Logger(const std::string session_name = "xyz");
  virtual ~Logger() = default;

  /** Log Image */
  void log_image(const std::string &topic,
                 const timestamp_t ts,
                 const cv::Mat &image);

  /** Log scalar */
  void log_scalar(const std::string &topic,
                  const timestamp_t ts,
                  const double value);

  /** Log Transform */
  void log_pose(const std::string &topic,
                const Mat4 &pose,
                const float axis_length = 1.0);
  void log_pose(const std::string &topic,
                const timestamp_t ts,
                const Mat4 &pose,
                const float axis_length = 1.0);

  /** Log Points */
  void log_points(const std::string &topic,
                  const std::vector<Vec3> &points,
                  const std::vector<Vec3> &colors,
                  const std::vector<double> &radii);
  void log_points(const std::string &topic,
                  const timestamp_t ts,
                  const std::vector<Vec3> &points,
                  const std::vector<Vec3> &colors,
                  const std::vector<double> &radii);
};

} // namespace xyz
