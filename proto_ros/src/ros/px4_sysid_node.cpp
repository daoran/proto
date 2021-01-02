#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// GLOBAL VARIABLES
mavros_msgs::State mav_state;
geometry_msgs::PoseStamped mav_pose;
#define HOVER 0
#define ROLL_LEFT 1
#define ROLL_RIGHT 2
#define PITCH_FORWARD 3
#define PITCH_BACK 4
int sysid_state = HOVER;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
  mav_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  mav_pose = *msg;
}

bool waypoint_reached(const geometry_msgs::PoseStamped &mav_pose,
                      const geometry_msgs::PoseStamped &waypoint,
                      const double threshold=0.1) {
  const double dx = mav_pose.pose.position.x - waypoint.pose.position.x;
  const double dy = mav_pose.pose.position.y - waypoint.pose.position.y;
  const double dz = mav_pose.pose.position.z - waypoint.pose.position.z;
  const double dist = sqrt(dx * dx + dy * dy + dz * dz);  // Euclidean distance
  if (dist < threshold) {
    return true;
  } else {
    return false;
  }
}

void move_to(ros::Rate rate,
             const ros::Publisher &pos_pub,
             const geometry_msgs::PoseStamped &mav_pose,
             const geometry_msgs::PoseStamped &wp_pose,
             const ros::Duration &hover_duration=ros::Duration(5.0)) {
  bool timer_started = false;
  ros::Time time_prev;

  // Loop until conditions met
	while (ros::ok()){
	  auto time_now = ros::Time::now();

    // Check if waypoint has been reached
    if (waypoint_reached(mav_pose, wp_pose)) {
      ROS_INFO("Waypoint reached!");
      if (timer_started == false) {
        time_prev = ros::Time::now();
        timer_started = true;
      }
    }

    // Calculate hover time elasped
    ros::Duration time_elasped{0};
    if (timer_started) {
      time_elasped = time_now - time_prev;
    }

    // Exit hover loop if hovered enough time
    if (timer_started && time_elasped > hover_duration) {
      ROS_INFO("Hover duraction reached!");
      return;
    }

    // Publish hover pose
		pos_pub.publish(wp_pose);
		ros::spinOnce();
		rate.sleep();
	}
}

void hover(ros::Rate rate,
           const ros::Publisher &pos_pub,
           const geometry_msgs::PoseStamped &mav_pose,
           const ros::Duration &hover_duration=ros::Duration(5.0)) {
  geometry_msgs::PoseStamped hover_pose;
  hover_pose.pose.position.x = 0.0;
  hover_pose.pose.position.y = 0.0;
  hover_pose.pose.position.z = 1.5;
  move_to(rate, pos_pub, mav_pose, hover_pose, hover_duration);
}

void roll_left(ros::Rate rate,
               const ros::Publisher &pos_pub,
               const geometry_msgs::PoseStamped &mav_pose,
               const ros::Duration &hover_duration=ros::Duration(5.0)) {
  geometry_msgs::PoseStamped wp_pose;
  wp_pose.pose.position.x = 0.0;
  wp_pose.pose.position.y = 0.5;
  wp_pose.pose.position.z = 1.5;
  move_to(rate, pos_pub, mav_pose, wp_pose, hover_duration);
}

void roll_right(ros::Rate rate,
                const ros::Publisher &pos_pub,
                const geometry_msgs::PoseStamped &mav_pose,
                const ros::Duration &hover_duration=ros::Duration(5.0)) {
  geometry_msgs::PoseStamped wp_pose;
  wp_pose.pose.position.x = 0.0;
  wp_pose.pose.position.y = -0.5;
  wp_pose.pose.position.z = 1.5;
  move_to(rate, pos_pub, mav_pose, wp_pose, hover_duration);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "proto_hover_node");
	ros::NodeHandle nh;
	ros::Rate rate(20.0);

  // Subscribers and publishers
	auto topic_state = "mavros/state";
	auto topic_setpoint = "mavros/setpoint_position/local";
	auto topic_pose = "mavros/local_position/pose";
	auto state_sub = nh.subscribe<mavros_msgs::State>(topic_state, 1, state_cb);
	auto pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_pose, 1, pose_cb);
	auto pos_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_setpoint, 10);

	// Wait for FCU connection
	ROS_INFO("Waiting for FCU...");
	while (ros::ok() && !mav_state.connected){
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("Connected to FCU!");

	// Sysid
	ROS_INFO("Moving to hover position for 10 seconds");
	hover(rate, pos_pub, mav_pose, ros::Duration(10));

	ROS_INFO("Roll left!");
	roll_left(rate, pos_pub, mav_pose);

	ROS_INFO("Back to hover point!");
	hover(rate, pos_pub, mav_pose, ros::Duration(2));

	ROS_INFO("Roll right!");
	roll_right(rate, pos_pub, mav_pose);

	ROS_INFO("Back to hover point!");
	hover(rate, pos_pub, mav_pose, ros::Duration(2));

	// Hover practically forever
	ROS_INFO("Done!");
	ROS_INFO("Hovering forever! Please take back control!");
	hover(rate, pos_pub, mav_pose, ros::Duration(100000));

	return 0;
}
