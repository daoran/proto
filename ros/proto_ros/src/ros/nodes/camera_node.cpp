#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "proto/ros/node.hpp"
#include "proto/ros/msgs.hpp"

namespace proto {

struct camera_node_t : ros_node_t {
  cv::VideoCapture capture_;
	int camera_index_ = 0;
  std::string topic_;

  camera_node_t(int argc, char **argv) : ros_node_t(argc, argv) {}

  int configure() {
    // Setup ROS
    ros_node_t::configure();
    ROS_GET_PARAM(node_name_ + "/topic", topic_);
    ROS_GET_OPTIONAL_PARAM(node_name_ + "/camera_index", camera_index_, 0);

    // Configure publishers and loop callbacks
    add_image_publisher(topic_);
    add_loop_callback(std::bind(&camera_node_t::loop_callback, this));

    // Open camera
    capture_ = cv::VideoCapture(camera_index_);
    if (capture_.isOpened() == false) {
      ROS_FATAL("Failed to open camera index[%d]!", camera_index_);
      return -1;
    }

    return 0;
  }

  int loop_callback() {
    // Get camera image
    cv::Mat image;
    if (capture_.read(image) == false) {
      ROS_FATAL("Failed to obtain image data from camera!");
      return -1;
    }

    // Publish image
    const auto header = std_msgs::Header();
    const auto image_type = "bgr8";
    const auto ros_image = cv_bridge::CvImage(header, image_type, image);
    img_pubs_[topic_].publish(ros_image.toImageMsg());

    return 0;
  }
};

} // namespace proto
RUN_ROS_NODE(proto::camera_node_t);
