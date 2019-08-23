def point_to_string(msg):
    header = "px,py,pz"
    data = ",".join([str(msg.w), str(msg.x), str(msg.y), str(msg.z)])
    return (header, data)


def quaternion_to_string(msg):
    header = "qw,qx,qy,qz"
    data = ",".join([str(msg.w), str(msg.x), str(msg.y), str(msg.z)])
    return (header, data)


def pose_to_string(msg):
    pos_header, pos_data = point_to_string(msg.position)
    orientation_header, orientation_data = point_to_string(msg.position)

    header = pos_header + "," + orientation_header
    data = pos_data + "," + orientation_data
    return (header, data)
