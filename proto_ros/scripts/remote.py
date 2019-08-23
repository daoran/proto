#!/usr/bin/env python
from math import pi
from time import sleep
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import rospy


class ROSNode(object):
    def __init__(self):
        self.pubs = {}
        self.subs = {}

    def register_publisher(self, topic, msg_type, queue_size=1):
        self.pubs[topic] = rospy.Publisher(topic,
                                           msg_type,
                                           queue_size=queue_size)

    def register_subscriber(self, topic, msg_type, callback):
        self.subs[topic] = rospy.Subscriber(topic, msg_type, callback)


class LandingZone(ROSNode):
    def __init__(self):
        super(LandingZone, self).__init__()
        self.position_topic = "/landing_zone/set_position"
        self.velocity_topic = "/landing_zone/set_velocity"
        self.register_publisher(self.position_topic, Vector3)
        self.register_publisher(self.velocity_topic, Vector3)

    def set_position(self, pos):
        msg = Vector3()
        msg.x = pos[0]
        msg.y = pos[1]
        msg.z = pos[2]
        self.pubs[self.position_topic].publish(msg)

    def set_velocity(self, vel):
        msg = Vector3()
        msg.x = vel[0]
        msg.y = vel[1]
        msg.z = vel[2]
        self.pubs[self.velocity_topic].publish(msg)


class VISensor(ROSNode):
    def __init__(self):
        super(VISensor, self).__init__()
        self.pose = None
        self.pose_topic = "/visensor/pose"
        self.pose_set_topic = "/visensor/pose/set"

        self.register_publisher(self.pose_set_topic, Pose)
        self.register_subscriber(self.pose_topic, PoseStamped, self.pose_cb)

    def pose_cb(self, msg):
        self.pose = msg

    def set_orientation(self, q_WS):
        msg = Pose()
        msg.position.x = self.pose.pose.position.x
        msg.position.y = self.pose.pose.position.y
        msg.position.z = self.pose.pose.position.z
        msg.orientation.w = q_WS[0]
        msg.orientation.x = q_WS[1]
        msg.orientation.y = q_WS[2]
        msg.orientation.z = q_WS[3]
        self.pubs[self.pose_set_topic].publish(msg)

    def set_position(self, r_WS):
        msg = Pose()
        msg.position.x = r_WS[0]
        msg.position.y = r_WS[1]
        msg.position.z = r_WS[2]
        msg.orientation.w = self.pose.pose.orientation.w
        msg.orientation.x = self.pose.pose.orientation.x
        msg.orientation.y = self.pose.pose.orientation.y
        msg.orientation.z = self.pose.pose.orientation.z
        self.pubs[self.pose_set_topic].publish(msg)

    def set_pose(self, r_WS, q_WS):
        msg = Pose()
        msg.position.x = r_WS[0]
        msg.position.y = r_WS[1]
        msg.position.z = r_WS[2]
        msg.orientation.w = q_WS[0]
        msg.orientation.x = q_WS[1]
        msg.orientation.y = q_WS[2]
        msg.orientation.z = q_WS[3]
        self.pubs[self.pose_set_topic].publish(msg)



def deg2rad(d):
    return d * pi / 180.0


def rad2deg(r):
    return r * 180.0 / pi


if __name__ == "__main__":
    rospy.init_node("proto_remote")
    lz = LandingZone()
    vi_sensor = VISensor()
    rospy.sleep(1.0)
    # rospy.spin()

    vi_sensor.set_position([1.0, 0.0, 1.0])

    # lz.set_velocity([1.0, 0.0, 0.0]);
    # lz.set_position([1.0, 0.0, 0.0]);
