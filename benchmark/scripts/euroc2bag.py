""" Script to convert EuRoC dataset format to ROS1 ROS Bag """
#!/usr/bin/env python
import sys

from ros import rosbag
from sensor_msgs.msg import Image


def print_usage():
    """ Print usage """
    print("Usage: euroc2bag.py <data dir> <ros bag> <time_delay>")
    print("Example: euroc2bag.py /data data.bag 1e-5")


def write_imu_messages(bag, imu_csv):
    """ Write IMU messages """
    pass


def write_image_messages(bag, cam_idx, image_dir):
    """ Write camera image messages """
    # img_msg = Image()
    # img_msg.header.stamp = Stamp
    # img_msg.width = im_left.size[0]
    # img_msg.height = im_left.size[1]
    # img_msg.encoding = "rgb8"
    # img_msg.header.frame_id = "camera"
    # img_msg_data = [pix for pixdata in im.getdata() for pix in pixdata]
    # img_msg_.data = img_msg_data
    # bag.write(f"camera{cam_idx}/image_raw", img_msg, Stamp)
    pass


if __name__ == "__main__":
    # Check CLI args
    if len(sys.argv) != 3:
        print_usage()
        sys.exit(-1)
