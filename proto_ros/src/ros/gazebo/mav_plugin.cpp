#include <functional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <proto/proto.hpp>
#include "proto/ros/msgs.hpp"
#include "proto/ros/gazebo/common.hpp"

namespace gazebo {

struct mav_plugin_t : ModelPlugin {
  // Gazebo
  physics::ModelPtr model_;
  event::ConnectionPtr conn_;
  sdf::ElementPtr sdf_;
  double prev_sim_time_ = 0; // [seconds]

  proto::vec4_t motor_inputs_{0.0, 0.0, 0.0, 0.0};
  proto::mav_model_t mav_;

  // ROS
  std::thread ros_th_;
  ros::NodeHandle *ros_nh_;
  ros::Subscriber input_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher twist_pub_;
  size_t ros_seq_ = 0;

  mav_plugin_t() {}
  ~mav_plugin_t() { delete ros_nh_; }

  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
    // Gazebo things
    model_ = parent;
    sdf_ = sdf;
    conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&mav_plugin_t::on_update, this));
    prev_sim_time_ = model_->GetWorld()->SimTime().Double();

    // Start ROS thread
    ros_th_ = std::thread(&mav_plugin_t::ros_thread, this);
    sleep(1);

    // MAV
    const ignition::math::Pose3d T_WR = model_->WorldPose();
    const proto::quat_t q_WR = convert(T_WR.Rot());
    const proto::vec3_t r_WR = convert(T_WR.Pos());
    mav_.attitude = proto::quat2euler(q_WR);
    mav_.position = r_WR;
  }

  void ros_thread() {
    // Initialize ros node
    if (ros::isInitialized() == false) {
      ROS_FATAL("ROS is not initialized!");
    }
    ros_nh_ = new ros::NodeHandle();

    // Register input subscriber
    {
      std::string topic_name = "mav/motor_inputs";
      if (sdf_->HasElement("motor_inputs_topic")) {
        topic_name = sdf_->Get<std::string>("motor_inputs_topic");
      }
      input_sub_ = ros_nh_->subscribe(topic_name,
                                      1,
                                      &mav_plugin_t::motor_input_callback,
                                      this);
    }

    // Register pose publisher
    {
      std::string topic_name = "mav/pose";
      if (sdf_->HasElement("pose_topic")) {
        topic_name = sdf_->Get<std::string>("pose_topic");
      }
      pose_pub_ = ros_nh_->advertise<geometry_msgs::PoseStamped>(topic_name, 1);
    }

    // Register twist publisher
    {
      std::string topic_name = "mav/twist";
      if (sdf_->HasElement("twist_topic")) {
        topic_name = sdf_->Get<std::string>("twist_topic");
      }
      twist_pub_ =
          ros_nh_->advertise<geometry_msgs::TwistStamped>(topic_name, 1);
    }
  }

  void on_update() {
    // Calculate dt
    const double curr_sim_time = model_->GetWorld()->SimTime().Double();
    const double dt = curr_sim_time - prev_sim_time_;
    prev_sim_time_ = curr_sim_time;

    // Set model
    // {
    //   const ignition::math::Pose3d T_WR = model_->WorldPose();
    //   const proto::quat_t q_WR = convert(T_WR.Rot());
    //   const proto::vec3_t r_WR = convert(T_WR.Pos());
    //   const ignition::math::Vector3d w_WR = model_->WorldAngularVel();
    //   const ignition::math::Vector3d v_WR = model_->WorldLinearVel();
    //   mav_.attitude = proto::quat2euler(q_WR);
    //   mav_.angular_velocity = convert(w_WR);
    //   mav_.position = r_WR;
    //   mav_.linear_velocity = convert(v_WR);
    // }

    // Update model
    proto::mav_model_update(mav_, motor_inputs_, dt);
    const proto::vec3_t r_WR = mav_.position;
    const double x = r_WR(0);
    const double y = r_WR(1);
    const double z = r_WR(2);
    const proto::vec3_t rpy_WR = mav_.attitude;
    const double roll = rpy_WR(0);
    const double pitch = rpy_WR(1);
    const double yaw = rpy_WR(2);

    const ignition::math::Pose3d T_WR{x, y, z, roll, pitch, yaw};
    model_->SetWorldPose(T_WR);
    // model_->SetLinearVel(convert(mav_.linear_velocity));
    // model_->SetAngularVel(convert(mav_.angular_velocity));

    // Publish pose
    publish_pose();
    publish_twist();
    ros_seq_++;
  }

  void motor_input_callback(const sensor_msgs::Joy::ConstPtr &msg) {
    motor_inputs_(0) = msg->axes[0];
    motor_inputs_(1) = msg->axes[1];
    motor_inputs_(2) = msg->axes[2];
    motor_inputs_(3) = msg->axes[3];
  }

  void publish_pose() {
    const auto time = ros::Time(model_->GetWorld()->SimTime().Double());
    const proto::mat4_t T_WR = convert(model_->WorldPose());
    const auto msg = proto::msg_build(ros_seq_, time, "mav", T_WR);
    pose_pub_.publish(msg);
  }

  void publish_twist() {
    const auto time = ros::Time(model_->GetWorld()->SimTime().Double());
    const proto::vec3_t linear_velocity = convert(model_->WorldLinearVel());
    const proto::vec3_t angular_velocity = convert(model_->WorldAngularVel());

    const auto msg = proto::msg_build(ros_seq_,
                                      time,
                                      "mav",
                                      linear_velocity,
                                      angular_velocity);

    twist_pub_.publish(msg);
  }
};

GZ_REGISTER_MODEL_PLUGIN(mav_plugin_t)
} // namespace gazebo
