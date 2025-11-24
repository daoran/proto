#include "Logger.hpp"

namespace xyz {

Logger::Logger(const std::string session_name) {
  rec_ = std::make_unique<rerun::RecordingStream>(session_name);
  rec_->spawn().exit_on_failure();
  rec_->log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
}

std::chrono::time_point<std::chrono::system_clock>
Logger::convert(const timestamp_t ts) {
  return std::chrono::time_point<std::chrono::system_clock>{
      std::chrono::nanoseconds{ts}};
}

rerun::Image Logger::convert(const cv::Mat &image) const {
  rerun::ColorModel color = rerun::ColorModel::BGR;
  if (image.channels() == 1) {
    color = rerun::ColorModel::L;
  } else if (image.channels() == 3) {
    color = rerun::ColorModel::BGR;
  }

  const auto data = rerun::borrow(image.data, image.total() * image.elemSize());
  const rerun::WidthHeight image_size(image.cols, image.rows);
  const auto rr_image = rerun::Image(data, image_size, color);
  return rr_image;
}

rerun::Transform3D Logger::convert(const Mat4 &pose,
                                   const float axis_length) const {
  const Eigen::Vector3f p = pose.block<3, 1>(0, 3).cast<float>();
  const Eigen::Matrix3f R = pose.block<3, 3>(0, 0).cast<float>();
  const rerun::Vec3D rr_position(p.data());
  const rerun::Mat3x3 rr_rotation(R.data());
  const rerun::components::Scale3D rr_scale{1.0, 1.0, 1.0};
  const rerun::components::AxisLength rr_length{axis_length};

  return rerun::Transform3D{rr_position, rr_rotation}
      .with_scale(rr_scale)
      .with_axis_length(rr_length);
}

rerun::Points3D Logger::convert(const std::vector<Vec3> &points,
                                const std::vector<Vec3> &colors,
                                const std::vector<double> &radii) const {

  std::vector<rerun::Position3D> rerun_positions;
  std::vector<rerun::Color> rerun_colors;
  std::vector<rerun::Radius> rerun_radii;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto p = points[i];
    const auto c = colors[i];
    rerun_positions.emplace_back(p.x(), p.y(), p.z());
    rerun_colors.emplace_back(c.x(), c.y(), c.z());
    rerun_radii.push_back(radii[i]);
  }

  return rerun::Points3D(rerun_positions)
      .with_colors(rerun_colors)
      .with_radii(rerun_radii);
}

void Logger::log_image(const std::string &topic,
                       const timestamp_t ts,
                       const cv::Mat &image) {
  rec_->set_time_timestamp("time", convert(ts));
  rec_->log(topic, convert(image));
}

void Logger::log_scalar(const std::string &topic,
                        const timestamp_t ts,
                        const double value) {
  rec_->set_time_timestamp("time", convert(ts));
  rec_->log(topic, rerun::Scalars{value});
}

void Logger::log_pose(const std::string &topic, const Mat4 &pose) {
  rec_->log(topic, convert(pose));
}

void Logger::log_pose(const std::string &topic,
                      const timestamp_t ts,
                      const Mat4 &pose) {
  rec_->set_time_timestamp("time", convert(ts));
  rec_->log(topic, convert(pose));
}

void Logger::log_points(const std::string &topic,
                        const std::vector<Vec3> &points,
                        const std::vector<Vec3> &colors,
                        const std::vector<double> &radii) {
  assert(points.size() == colors.size());
  rec_->log(topic, convert(points, colors, radii));
}

void Logger::log_points(const std::string &topic,
                        const timestamp_t ts,
                        const std::vector<Vec3> &points,
                        const std::vector<Vec3> &colors,
                        const std::vector<double> &radii) {
  assert(points.size() == colors.size());
  rec_->set_time_timestamp("time", convert(ts));
  rec_->log(topic, convert(points, colors, radii));
}

} // namespace xyz
