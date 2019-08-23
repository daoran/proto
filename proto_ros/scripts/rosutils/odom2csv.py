#!/usr/bin/env python
import sys
import rosbag
import rospy

import rosutils.std_msgs as std_msgs
import rosutils.geometry_msgs as geometry_msgs


def print_usage():
    print("Usage: odom2csv.py <ros bag> <ros topic> <output path>")
    print("Example: odom2csv.py record.bag /robot/pose robot_images.csv")


def check_topic_type(bag):
    info = bag.get_type_and_topic_info()
    msg_type = info.topics[topic].msg_type
    supported_msgs = ["nav_msgs/Odometry"]

    if msg_type not in supported_msgs:
        err_msg = "odom2csv only supports %s!" % " or ".join(supported_msgs)
        raise RuntimeError(err_msg)


def odometry_to_string(msg):
    header, data = std_msgs.header_to_string(msg.header)

    pos_header, pos_data = geometry_msgs.pose_to_string(msg.pose.pose)
    header += "," + pos_header
    data += "," + pos_header

    return [header, data]


if __name__ == "__main__":
    # Check CLI args
    if len(sys.argv) != 4:
        print_usage()
        exit(-1)

    # Parse CLI args
    bag = rosbag.Bag(sys.argv[1], 'r')
    topic = sys.argv[2]
    output_path = sys.argv[3]

    # Check if topic is in bag
    info = bag.get_type_and_topic_info()
    if topic not in info.topics:
        raise RuntimeError("Opps! topic not in bag!")

    # Check if topic type is supported
    check_topic_type(bag)
    # Output csv file
    # -- Output header
    csv_file = open(output_path, "w")
    topic, msg, t = next(bag.read_messages(topics=[topic]))
    header, data = odom_msg_to_string(msg)
    csv_file.write(header + "\n")
    # -- Output data
    time = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        header, data = odom_msg_to_string(msg)
        csv_file.write(data + "\n")
        csv_file.flush()
    csv_file.close()

    # import numpy as np
    # import matplotlib.pylab as plt
    # from mpl_toolkits.mplot3d import Axes3D
    #
    # header = ["seq", "frame_id", "secs", "nsecs",
    #           "px","py","pz","qw","qx","qy","qz"]
    # data = np.genfromtxt(output_path,
    #                      delimiter=',',
    #                      skip_header=1,
    #                      names=header)
    #
    # plt.plot(data["seq"], data["px"], label="North")
    # plt.plot(data["seq"], data["py"], label="East")
    # plt.plot(data["seq"], -1 * data["pz"], label="Height")
    # plt.xlabel("Message Sequence")
    # plt.ylabel("Displacement [m]")
    # plt.legend(loc=0)
    #
    # # fig = plt.figure()
    # # ax = fig.add_subplot(111, projection='3d')
    # # ax.plot(data["px"], data["py"], -1 * data["pz"])
    # # ax.axis("equal")
    # # ax.set_xlabel("North [m]")
    # # ax.set_ylabel("East [m]")
    # # ax.set_zlabel("Height [m]")
    # # ax.xlim([-0.1, 0.1])
    # # ax.ylim([-0.1, 0.1])
    # # ax.zlim([-0.1, 1.0])
    #
    # # plt.show()
    # plt.savefig("position.png")
