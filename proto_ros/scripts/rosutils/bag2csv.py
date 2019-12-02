#!/usr/bin/env python
import sys
import rosbag
import rospy


def print_usage():
    print("Usage: bag2csv.py <ros bag> <ros topic> <output path>")
    print("Example: bag2csv.py record.bag /robot/pose robot_images.csv")


def check_topic_type(bag):
    info = bag.get_type_and_topic_info()
    msg_type = info.topics[topic].msg_type
    supported_msgs = ["geometry_msgs/TransformStamped",
                      "geometry_msgs/PoseStamped"]

    if msg_type not in supported_msgs:
        err_msg = "bag2csv only supports %s!" % " or ".join(supported_msgs)
        raise RuntimeError(err_msg)


def vector3_str(msg):
    return ",".join([str(msg.x), str(msg.y), str(msg.z)])


def quaternion_str(msg):
    return ",".join([str(msg.w), str(msg.x), str(msg.y), str(msg.z)])


def transform_str(msg):
    header = "rx,ry,rz,qw,qx,qy,qz"
    translation = vector3_str(msg.translation)
    rotation = quaternion_str(msg.rotation)
    data = translation + "," + rotation
    return [header, data]


def header_str(msg):
    header = "seq,frame_id,secs,nsecs"

    seq = msg.seq
    frame_id = msg.frame_id
    secs = msg.stamp.secs
    nsecs = msg.stamp.nsecs
    timestamp = rospy.Time(secs, nsecs)

    ts = str(timestamp.to_nsec())
    secs = str(ts[0:10])
    nsecs = str(ts[10:19])

    data = ",".join([str(seq), str(frame_id), secs, nsecs])

    return [header, data]


def transform_stamped_str(msg):
    header, data = header_str(msg.header)
    tf_header, tf_data = transform_str(msg.transform)
    header += "," + tf_header
    data += "," + tf_data
    return [header, data]


def pose_stamped_str(msg):
    header, data = header_str(msg.header)

    header += "px,py,pz,qw,qx,qy,qz"
    data += str(msg.position.x)
    data += str(msg.position.y)
    data += str(msg.position.z)
    data += str(msg.orientation.w)
    data += str(msg.orientation.x)
    data += str(msg.orientation.y)
    data += str(msg.orientation.z)

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
    # header, data = transform_stamped_str(msg)
    # csv_file.write(header + "\n")
    # -- Output data
    time = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        print(msg)
        # header, data = transform_stamped_str(msg)
        # csv_file.write(data + "\n")
        # csv_file.flush()
    csv_file.close()
