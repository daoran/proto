#pragma once
#include <cassert>

#include "Core.hpp"
#include "calib/AprilGridConfig.hpp"

#include <rerun.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/core.hpp>

namespace cartesian {

class Logger {
private:
  std::unique_ptr<rerun::RecordingStream> rec_;

  /** Convert timestamp to std::chrono::time_point **/
  std::chrono::time_point<std::chrono::system_clock>
  convert_timestamp(const timestamp_t ts);

  /** Convert cv::Mat to rerun::Image **/
  rerun::Image convert_image(const cv::Mat &image) const;

  /** Convert Mat4 to rerun::Transform3D **/
  rerun::Transform3D convert_pose(const Mat4 &pose,
                                  const float axis_length = 1.0) const;

  /** Convert std::vector<Vec3> to rerun::Points3D **/
  rerun::Points3D convert_points(const std::vector<Vec3> &points,
                                 const std::vector<Vec3> &colors,
                                 const std::vector<double> &radii) const;

  /** Convert std::vector<Vec3> to rerun::LineStrips3D **/
  rerun::LineStrips3D convert_line(const std::vector<Vec3> &points,
                                   const Vec3 &colors,
                                   const float &radii);

public:
  Logger(const std::string session_name = "cartesian");
  virtual ~Logger() = default;

  /** Log Image */
  void log_image(const std::string &topic,
                 const timestamp_t ts,
                 const cv::Mat &image);

  /** Log scalar */
  void init_series_line(const std::string &topic,
                        const Vec3 color,
                        const float line_width);
  void log_scalar(const std::string &topic,
                  const timestamp_t ts,
                  const double value);

  /** Log line */
  void log_line(const std::string &topic,
                const std::vector<Vec3> &points,
                const Vec3 &color = Vec3{255.0, 0.0, 0.0},
                const float radii = 0.01);

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

  /** Log pose */
  void log_pose(const std::string &topic,
                const Mat4 &pose,
                const float axis_length = 1.0);
  void log_pose(const std::string &topic,
                const timestamp_t ts,
                const Mat4 &pose,
                const float axis_length = 1.0);
  void log_poses(const std::string &topic,
                 const std::map<timestamp_t, Mat4> &poses,
                 const float axis_length = 1.0);

  /** Log trajectory */
  void log_trajectory(const std::string &topic,
                      const std::map<timestamp_t, Mat4> &poses,
                      const Vec3 &color = Vec3{255.0, 0.0, 0.0},
                      const float radii = 0.005);

  /** Log Calibration Target */
  void log_target(const std::string &topic,
                  const AprilGridConfig &config,
                  const Mat4 &T_WT);
};

} // namespace cartesian
