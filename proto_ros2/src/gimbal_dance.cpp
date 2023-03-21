#include <iostream>
#include <memory>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using std::placeholders::_1;

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
  rclcpp::Publisher<Float64>::SharedPtr joint0_cmd_;
  rclcpp::Publisher<Float64>::SharedPtr joint1_cmd_;
  rclcpp::Publisher<Float64>::SharedPtr joint2_cmd_;
  rclcpp::Subscription<JointState>::SharedPtr joint0_state_;
  rclcpp::Subscription<JointState>::SharedPtr joint1_state_;
  rclcpp::Subscription<JointState>::SharedPtr joint2_state_;

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
    // clang-format on

    calibrationDance();
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
          sleep(5);
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
    sleep(5);
    publishJointCommand(0, 0);
    publishJointCommand(1, 0);
    publishJointCommand(2, 0);
    sleep(8);
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
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalNode>());
  rclcpp::shutdown();
  return 0;
}
