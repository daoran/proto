#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// GLOBAL VARIABLES
mavros_msgs::State mav_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
  mav_state = *msg;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "proto_px4_hover_node");
	ros::NodeHandle nh;
	ros::Rate rate(20.0);

  // Subscribers and publishers
	auto topic_state = "mavros/state";
	auto topic_local_pos = "mavros/setpoint_position/local";
	auto state_sub = nh.subscribe<mavros_msgs::State>(topic_state, 10, state_cb);
	auto local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_local_pos, 10);

	// Wait for FCU connection
	ROS_INFO("Waiting for FCU...");
	while (ros::ok() && !mav_state.connected){
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("Connected to FCU!");

	// Hover position
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 1.5;

	// Send a few setpoints before starting
	ROS_INFO("Sending a few hover setpoints..");
	for(int i = 100; ros::ok() && i > 0; i--){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}

	// Send hover position command
	ROS_INFO("Hovering...");
	while (ros::ok()){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
