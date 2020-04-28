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

struct landing_target_plugin_t : ModelPlugin {
  // Gazebo
  physics::ModelPtr model_;
  event::ConnectionPtr conn_;
  sdf::ElementPtr sdf_;
  double prev_sim_time_ = 0; // [seconds]

  // ROS
  std::thread ros_th_;
  ros::NodeHandle *ros_nh_;
  ros::Subscriber pos_sub_;
  ros::Subscriber vel_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher twist_pub_;
  size_t ros_seq_ = 0;

  landing_target_plugin_t() {}
  ~landing_target_plugin_t() { delete ros_nh_; }

  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
    // Gazebo things
    model_ = parent;
    sdf_ = sdf;
    conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&landing_target_plugin_t::on_update, this));
    prev_sim_time_ = model_->GetWorld()->SimTime().Double();

    // Start ROS thread
    ros_th_ = std::thread(&landing_target_plugin_t::ros_thread, this);
    sleep(1);
  }

  void ros_thread() {
    // Initialize ros node
    if (ros::isInitialized() == false) {
      ROS_FATAL("ROS is not initialized!");
    }
    ros_nh_ = new ros::NodeHandle();

    // Register pos subscriber
    {
      std::string topic_name = "landing_zone/set_position";
      if (sdf_->HasElement("set_position_topic")) {
        topic_name = sdf_->Get<std::string>("set_position_topic");
      }
      pos_sub_ = ros_nh_->subscribe(topic_name,
                                    1,
                                    &landing_target_plugin_t::position_callback,
                                    this);
    }

    // Register vel subscriber
    {
      std::string topic_name = "landing_zone/set_velocity";
      if (sdf_->HasElement("set_velocity_topic")) {
        topic_name = sdf_->Get<std::string>("set_velocity_topic");
      }
      vel_sub_ = ros_nh_->subscribe(topic_name,
                                    1,
                                    &landing_target_plugin_t::velocity_callback,
                                    this);
    }

    // Register pose publisher
    {
      std::string topic_name = "landing_zone/pose";
      if (sdf_->HasElement("pose_topic")) {
        topic_name = sdf_->Get<std::string>("pose_topic");
      }
      pose_pub_ = ros_nh_->advertise<geometry_msgs::PoseStamped>(topic_name, 1);
    }

    // Register twist publisher
    {
      std::string topic_name = "landing_zone/twist";
      if (sdf_->HasElement("twist_topic")) {
        topic_name = sdf_->Get<std::string>("twist_topic");
      }
      twist_pub_ =
          ros_nh_->advertise<geometry_msgs::TwistStamped>(topic_name, 1);
    }
  }

  void on_update() {
    publish_pose();
    publish_twist();
    ros_seq_++;
  }

  void position_callback(const geometry_msgs::Vector3::ConstPtr &msg) {
    ignition::math::Pose3d T_WR = model_->WorldPose();
    ignition::math::Vector3d pos{msg->x, msg->y, msg->z};
    T_WR.Pos() = pos;
    model_->SetWorldPose(T_WR);
  }

  void velocity_callback(const geometry_msgs::Vector3::ConstPtr &msg) {
    ignition::math::Vector3d vel{msg->x, msg->y, msg->z};
    model_->SetLinearVel(vel);
  }

  void publish_pose() {
    const double time = model_->GetWorld()->SimTime().Double();
    const ignition::math::Pose3d T_WR = model_->WorldPose();
    const proto::vec3_t r_WR = convert(T_WR.Pos());
    const proto::quat_t q_WR = convert(T_WR.Rot());

    geometry_msgs::PoseStamped msg;
    msg.header.seq = ros_seq_;
    msg.header.stamp = ros::Time(time);
    msg.header.frame_id = "mav";
    msg.pose.position.x = r_WR(0);
    msg.pose.position.y = r_WR(1);
    msg.pose.position.z = r_WR(2);
    msg.pose.orientation.w = q_WR.w();
    msg.pose.orientation.x = q_WR.x();
    msg.pose.orientation.y = q_WR.y();
    msg.pose.orientation.z = q_WR.z();

    pose_pub_.publish(msg);
  }

  void publish_twist() {
    const double time = model_->GetWorld()->SimTime().Double();
    const proto::vec3_t linear_velocity = convert(model_->WorldLinearVel());
    const ignition::math::Pose3d T_WR = model_->WorldPose();
    const proto::vec3_t angular_velocity = convert(model_->WorldAngularVel());

    geometry_msgs::TwistStamped msg;
    msg.header.seq = ros_seq_;
    msg.header.stamp = ros::Time(time);
    msg.header.frame_id = "mav";
    msg.twist.linear.x = linear_velocity(0);
    msg.twist.linear.y = linear_velocity(1);
    msg.twist.linear.z = linear_velocity(2);
    msg.twist.angular.x = angular_velocity(0);
    msg.twist.angular.y = angular_velocity(1);
    msg.twist.angular.z = angular_velocity(2);

    twist_pub_.publish(msg);
  }
};

GZ_REGISTER_MODEL_PLUGIN(landing_target_plugin_t)
} // namespace gazebo
