#include <iostream>
#include <memory>
#include <thread>
#include <unistd.h>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

// clang-format off
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using std::placeholders::_1;
using std::placeholders::_2;
typedef message_filters::sync_policies::ApproximateTime<Image, Image> approx_policy;
bool keep_running = true;
// clang-format on

/**
 * Signal handler
 */
void signal_handler(int signum) {
  keep_running = false;
}

/**
 * Degrees to radians.
 * @returns Radians
 */
static double deg2rad(const double d) {
  return d * (M_PI / 180.0);
}

/**
 * Radians to degrees.
 * @returns Degrees
 */
static double rad2deg(const double r) {
  return r * (180.0 / M_PI);
}

/**
 * Clip vector `x` to be between `val_min` and `val_max`.
 */
void clip(double *x, const size_t n, const double vmin, const double vmax) {
  for (size_t i = 0; i < n; i++) {
    x[i] = (x[i] > vmax) ? vmax : x[i];
    x[i] = (x[i] < vmin) ? vmin : x[i];
  }
}

/**
 * Gimbal calibration dance.
 */
class GimbalNode : public rclcpp::Node {
private:
  // Publishers / Subscribers
  // clang-format off
  rclcpp::Publisher<Float64>::SharedPtr joint0_cmd_;
  rclcpp::Publisher<Float64>::SharedPtr joint1_cmd_;
  rclcpp::Publisher<Float64>::SharedPtr joint2_cmd_;
  rclcpp::Subscription<JointState>::SharedPtr joint0_state_;
  rclcpp::Subscription<JointState>::SharedPtr joint1_state_;
  rclcpp::Subscription<JointState>::SharedPtr joint2_state_;
  message_filters::Subscriber<Image> cam0_;
  message_filters::Subscriber<Image> cam1_;
  std::shared_ptr<message_filters::Synchronizer<approx_policy>> sync_;
  // clang-format on

  // Joint limits
  double jlims0_[2] = {deg2rad(-60.0), deg2rad(60.0)};
  double jlims1_[2] = {deg2rad(-45.0), deg2rad(45.0)};
  double jlims2_[2] = {deg2rad(-45.0), deg2rad(45.0)};

  // Dance limits
  double dlims0_[2] = {deg2rad(-10.0), deg2rad(10.0)};
  double dlims1_[2] = {deg2rad(-45.0), deg2rad(45.0)};
  double dlims2_[2] = {deg2rad(-20.0), deg2rad(20.0)};

  int num_yaw_ = 3;
  int num_roll_ = 4;
  int num_pitch_ = 4;

  // State
  double joint0_;
  double joint1_;
  double joint2_;

  /* Joint0 Callback */
  void joint0Callback(const JointState::SharedPtr msg) {
    joint0_ = msg->position[0];
  }

  /* Joint1 Callback */
  void joint1Callback(const JointState::SharedPtr msg) {
    joint1_ = msg->position[0];
  }

  /* Joint2 Callback */
  void joint2Callback(const JointState::SharedPtr msg) {
    joint2_ = msg->position[0];
  }

  /* Stereo Callback */
  void stereoCallback(const Image::ConstSharedPtr &cam0_msg,
                      const Image::ConstSharedPtr &cam1_msg) {
    const auto ptr0 = cv_bridge::toCvCopy(cam0_msg, cam0_msg->encoding);
    const auto ptr1 = cv_bridge::toCvCopy(cam1_msg, cam1_msg->encoding);
    const auto img0 = ptr0->image;
    const auto img1 = ptr1->image;

    cv::Mat viz = img0;
    cv::hconcat(viz, img1, viz);
    cv::imshow("Viz", viz);
    cv::waitKey(1);
  }

  std::thread calib_dance_thread_;

public:
  GimbalNode() : Node("GimbalNode") {
    // Setup publishers / subscribers
    // clang-format off
    auto j0cb = std::bind(&GimbalNode::joint0Callback, this, _1);
    auto j1cb = std::bind(&GimbalNode::joint1Callback, this, _1);
    auto j2cb = std::bind(&GimbalNode::joint2Callback, this, _1);
    joint0_cmd_ = create_publisher<Float64>("/gimbal/joint0_cmd", 1);
    joint1_cmd_ = create_publisher<Float64>("/gimbal/joint1_cmd", 1);
    joint2_cmd_ = create_publisher<Float64>("/gimbal/joint2_cmd", 1);
    joint0_state_ = create_subscription<JointState>("/gimbal/joint0_state", 1, j0cb);
    joint1_state_ = create_subscription<JointState>("/gimbal/joint1_state", 1, j1cb);
    joint2_state_ = create_subscription<JointState>("/gimbal/joint2_state", 1, j2cb);
    cam0_.subscribe(this, "/gimbal/camera0");
    cam1_.subscribe(this, "/gimbal/camera1");
    sync_ = std::make_shared<message_filters::Synchronizer<approx_policy>>(approx_policy(1), cam0_, cam1_);
    sync_->setMaxIntervalDuration(rclcpp::Duration(0, 0.01 * 1e9));
    sync_->registerCallback(std::bind(&GimbalNode::stereoCallback, this, _1, _2));
    // clang-format on

    // Start calibration dance
    calib_dance_thread_ = std::thread(&GimbalNode::calibrationDance, this);
  }

  virtual ~GimbalNode() {
    calib_dance_thread_.join();
  }

  /** Perform gimbal calibration dance. **/
  void calibrationDance() {
    // Reset
    reset();

    // clang-format off
    double dyaw = (dlims0_[1] - dlims0_[0]) / (num_yaw_ - 1);
    double dpitch = (dlims1_[1] - dlims1_[0]) / (num_roll_ - 1);
    double droll = (dlims2_[1] - dlims2_[0]) / (num_pitch_ - 1);
    double yaw = dlims0_[0];
    double pitch = dlims1_[0];
    double roll = dlims2_[0];
    // clang-format on

    int view_idx = 0;
    for (int i = 0; i < num_yaw_; i++) {
      roll = dlims1_[0];
      for (int j = 0; j < num_roll_; j++) {
        pitch = dlims1_[0];
        for (int k = 0; k < num_pitch_; k++) {
          publishJointCommand(0, yaw);
          publishJointCommand(1, roll);
          publishJointCommand(2, pitch);

          printf("view_idx: %d, ", view_idx);
          printf("yaw: %f, ", yaw);
          printf("roll: %f, ", roll);
          printf("pitch: %f ", pitch);
          printf("\n");
          fflush(stdout);

          view_idx += 1;
          pitch += dpitch;
          sleep(2);

          if (keep_running == false) {
            i = num_yaw_;
            j = num_roll_;
            k = num_pitch_;
          }
        }
        roll += droll;
      }
      yaw += dyaw;
    }

    // Reset
    reset();
  }

  /** Reset gimbal angles to 0, 0, 0 **/
  void reset() {
    sleep(2);
    publishJointCommand(0, 0);
    publishJointCommand(1, 0);
    publishJointCommand(2, 0);
    sleep(2);
  }

  /** Publish joint angle command **/
  void publishJointCommand(const int joint_idx, const double cmd) {
    double val = cmd;
    Float64 msg{};

    switch (joint_idx) {
      case 0:
        clip(&val, 1, jlims0_[0], jlims0_[1]);
        msg.data = val;
        joint0_cmd_->publish(msg);
        break;
      case 1:
        clip(&val, 1, jlims1_[0], jlims1_[1]);
        msg.data = val;
        joint1_cmd_->publish(msg);
        break;
      case 2:
        clip(&val, 1, jlims2_[0], jlims2_[1]);
        msg.data = val;
        joint2_cmd_->publish(msg);
        break;
    }
  }
};

int main(int argc, char *argv[]) {
  // Setup signal handler
  signal(SIGINT, signal_handler);

  // Setup ROS2
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalNode>());
  rclcpp::shutdown();

  return 0;
}
