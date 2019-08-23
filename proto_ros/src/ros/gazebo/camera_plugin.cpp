#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <ignition/math/Vector3.hh>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

namespace gazebo {

struct camera_plugin_t : public SensorPlugin {
  // Gazebo
  sdf::ElementPtr sdf_;
  event::ConnectionPtr conn_;
  rendering::ScenePtr scene_;
  sensors::CameraSensorPtr sensor_;
  rendering::CameraPtr camera_;

  // Image data
  size_t seq_ = 0;
  int image_width_ = 0;
  int image_height_ = 0;
  int image_depth_ = 0;
  std::string image_format_;

  // ROS
  std::string ros_topic_name_ = "camera";
  std::thread ros_thread_;
  ros::NodeHandle *ros_nh_;
  image_transport::ImageTransport *img_transport_;
  image_transport::Publisher img_pub_;

  camera_plugin_t() {}
  ~camera_plugin_t() {
    delete ros_nh_;
    delete img_transport_;
  }

  void Load(sensors::SensorPtr sptr, sdf::ElementPtr sdf) {
    sdf_ = sdf;

    // Load sensor pointer
    sensor_ = std::dynamic_pointer_cast<sensors::CameraSensor>(sptr);
    if (!sensor_) {
      gzerr << "camera_plugin_t requires a CameraSensor.\n";
    }

    // Load camera
    camera_ = sensor_->Camera();
    if (!camera_) {
      gzerr << "camera_plugin_t not attached to a camera sensor!\n";
      return;
    }

    // Keep track of image width, height and depth
    seq_ = 0;
    image_width_ = camera_->ImageWidth();
    image_height_ = camera_->ImageHeight();
    image_depth_ = camera_->ImageDepth();
    image_format_ = camera_->ImageFormat();

    // Register image callback
    conn_ = camera_->ConnectNewImageFrame(
        std::bind(&camera_plugin_t::on_new_frame,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3,
                  std::placeholders::_4,
                  std::placeholders::_5));

    // Create ROS thread
    ros_thread_ = std::thread(&camera_plugin_t::ros_thread, this);
  }

  void ros_thread() {
    // Initialize ros node
    if (ros::isInitialized() == false) {
      ROS_FATAL("ROS is not initialized!");
    }
    ros_nh_ = new ros::NodeHandle();

    // Register publisher
    int queue_size = 1;
    if (sdf_->HasElement("topic_name")) {
      ros_topic_name_ = sdf_->Get<std::string>("topic_name");
    }
    img_transport_ = new image_transport::ImageTransport(*ros_nh_);
    img_pub_ = img_transport_->advertise(ros_topic_name_, 1);
  }

  void on_new_frame(const unsigned char *image_raw,
                    const int image_width,
                    const int image_height,
                    const int image_depth,
                    const std::string &format) {
    // Get sim time
    rendering::ScenePtr scene = camera_->GetScene();
    common::Time timestamp = scene->SimTime();

    // Convert img message to cv::Mat
    const int buffer_size = image_width * image_height * 3;
    unsigned char *buffer = new unsigned char[buffer_size + 1];
    memcpy((char *) buffer, image_raw, buffer_size);
    const cv::Mat image(image_height, image_width, CV_8UC3, buffer);

    // Build std_msgs::Header
    std_msgs::Header header;
    header.seq = seq_;
    header.stamp = ros::Time(timestamp.sec, timestamp.nsec);
    header.frame_id = ros_topic_name_;

    // Build sensor_msgs::Image
    const auto img_msg = cv_bridge::CvImage(header, "rgb8", image).toImageMsg();

    // Publish image and clean up
    img_pub_.publish(img_msg);
    delete []buffer;
  }
};

GZ_REGISTER_SENSOR_PLUGIN(camera_plugin_t)
} // namespace gazebo
