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
#include "ros/ros.hpp"
#include "gazebo/gazebo.hpp"

namespace gazebo {

struct gimbal_2axis_t {
  proto::vec4_t states = proto::zeros(4, 1);
  double Ix = 0.01;
  double Iy = 0.01;

  // Gimbal2AxisController joint_controller;
  proto::quat_t frame_orientation;
  proto::quat_t joint_orientation;
  proto::vec2_t joint_setpoints;

  void update(const proto::vec2_t &u, const double dt) {
    const float ph = states(0);     // Roll
    const float ph_vel = states(1); // Roll velocity
    const float th = states(2);     // Pitch
    const float th_vel = states(3); // Pitch velocity

    states(0) = ph + ph_vel * dt;
    states(1) = ph_vel + (u(0) / Ix) * dt;
    states(2) = th + th_vel * dt;
    states(3) = th_vel + (u(1) / Ix) * dt;

    // Update joint orientation
    proto::vec3_t euler, target;
    euler << states(0), states(2), 0.0;
    joint_orientation = proto::euler2quat(euler);

    // Track target attitude
    euler = proto::quat2euler(frame_orientation);
    // joint_setpoints(0) = target_attitude_W(0) - euler(0);
    // joint_setpoints(1) = target_attitude_W(1) - euler(1);
  }
};

struct gimbal_2axis_controller_t {
  double dt_ = 0.0;
  proto::vec2_t outputs_;

  proto::vec2_t update(const proto::vec2_t &setpoints,
                       const proto::vec2_t &actual,
                       const double dt) {
    // Limit rate to 100Hz
    // increment counter and only take action when 0.001s accumulates
    dt_ += dt;
    if (dt_ < 0.001) {
      return outputs_;
    }

    // Calculate roll and pitch outputs using independent controllers
    // auto roll = roll_ctrlr_.update(setpoints(0), actual(0), dt);
    // auto pitch = pitch_ctrlr_.update(setpoints(1), actual(1), dt);

    // Keep track and return outputs
    // outputs_ << roll, pitch;
    dt_ = 0.0; // Reset counter
    return outputs_;
  }
};

struct gimbal_plugin_t : ModelPlugin {
  // Gazebo
  physics::ModelPtr model_;
  event::ConnectionPtr conn_;
  sdf::ElementPtr sdf_;
  double prev_sim_time_ = 0; // [seconds]

  proto::vec4_t inputs_{0.0, 0.0, 0.0, 0.0};
  gimbal_2axis_t gimbal_;

  // ROS
  std::thread ros_th_;
  ros::NodeHandle *ros_nh_;
  ros::Subscriber input_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher twist_pub_;
  size_t ros_seq_ = 0;

  gimbal_plugin_t() {}
  ~gimbal_plugin_t() { delete ros_nh_; }

  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
    // Gazebo things
    model_ = parent;
    sdf_ = sdf;
    conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&gimbal_plugin_t::on_update, this));
    prev_sim_time_ = model_->GetWorld()->SimTime().Double();

    // Start ROS thread
    ros_th_ = std::thread(&gimbal_plugin_t::ros_thread, this);
  }

  void ros_thread() {
    // Initialize ros node
    if (ros::isInitialized() == false) {
      ROS_FATAL("ROS is not initialized!");
    }
    ros_nh_ = new ros::NodeHandle();

    // // Register input subscriber
    // {
    //   std::string topic_name = "gimbal/motor_input";
    //   if (sdf_->HasElement("motor_input_topic")) {
    //     topic_name = sdf_->Get<std::string>("motor_input_topic");
    //   }
    //   input_sub_ = ros_nh_->subscribe(
    //     topic_name,
    //     1,
    //     &gimbal_plugin_t::motor_input_callback,
    //     this
    //   );
    // }

    // Register pose publisher
    {
      std::string topic_name = "gimbal/pose";
      if (sdf_->HasElement("pose_topic")) {
        topic_name = sdf_->Get<std::string>("pose_topic");
      }
      pose_pub_ = ros_nh_->advertise<geometry_msgs::PoseStamped>(topic_name, 1);
    }

    // Register twist publisher
    {
      std::string topic_name = "gimbal/twist";
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

    // Update model

    // Publish pose
    publish_pose();
    publish_twist();
    ros_seq_++;
  }

  void
  gimbal_world_attitude_callback(const geometry_msgs::Vector3::ConstPtr &msg) {
    // motor_inputs_(0) = msg->axes[0];
    // motor_inputs_(1) = msg->axes[1];
    // motor_inputs_(2) = msg->axes[2];
  }

  void publish_pose() {
    const double time = model_->GetWorld()->SimTime().Double();
    const ignition::math::Pose3d T_WG = model_->WorldPose();
    const proto::vec3_t r_WG = convert(T_WG.Pos());
    const proto::quat_t q_WG = convert(T_WG.Rot());

    geometry_msgs::PoseStamped msg;
    msg.header.seq = ros_seq_;
    msg.header.stamp = ros::Time(time);
    msg.header.frame_id = "gimbal";
    msg.pose.position.x = r_WG(0);
    msg.pose.position.y = r_WG(1);
    msg.pose.position.z = r_WG(2);
    msg.pose.orientation.w = q_WG.w();
    msg.pose.orientation.x = q_WG.x();
    msg.pose.orientation.y = q_WG.y();
    msg.pose.orientation.z = q_WG.z();

    pose_pub_.publish(msg);
  }

  void publish_twist() {
    const double time = model_->GetWorld()->SimTime().Double();
    const proto::vec3_t linear_velocity = convert(model_->WorldLinearVel());
    const proto::vec3_t angular_velocity = convert(model_->WorldAngularVel());

    geometry_msgs::TwistStamped msg;
    msg.header.seq = ros_seq_;
    msg.header.stamp = ros::Time(time);
    msg.header.frame_id = "gimbal";
    msg.twist.linear.x = linear_velocity(0);
    msg.twist.linear.y = linear_velocity(1);
    msg.twist.linear.z = linear_velocity(2);
    msg.twist.angular.x = angular_velocity(0);
    msg.twist.angular.y = angular_velocity(1);
    msg.twist.angular.z = angular_velocity(2);

    twist_pub_.publish(msg);
  }
};

GZ_REGISTER_MODEL_PLUGIN(gimbal_plugin_t)
} // namespace gazebo
