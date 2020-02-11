#!/usr/bin/env python
import sys
import math
from math import atan2
from math import asin
from math import degrees
from collections import namedtuple

import numpy as np
from scipy import signal
import matplotlib
import matplotlib.pylab as plt
matplotlib.rcParams['mathtext.fontset'] = 'custom'
matplotlib.rcParams['mathtext.rm'] = 'Bitstream Vera Sans'
matplotlib.rcParams['mathtext.it'] = 'Bitstream Vera Sans:italic'
matplotlib.rcParams['mathtext.bf'] = 'Bitstream Vera Sans:bold'

import rosbag
import rospy

topic_battery = "/mavros/battery"
topic_mocap = "/vicon/ucl/ucl"
topic_setpoint = "/mavros/setpoint_raw/attitude"
topic_timesync = "/mavros/timesync"


def quat2euler(q):
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    qw2 = qw * qw
    qx2 = qx * qx
    qy2 = qy * qy
    qz2 = qz * qz

    x = atan2(2 * (qx * qw + qz * qy), (qw2 - qx2 - qy2 + qz2))
    y = asin(2 * (qy * qw - qx * qz))
    z = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2))

    return [x, y, z]


def check_topic_exists(bag, topic):
    info = bag.get_type_and_topic_info()
    if topic not in info.topics:
        raise RuntimeError("Opps! topic [%s] not in bag!" % topic)


def parse_pose_stamped_data(bag, topic):
    first_ts = 0
    timestamps = []
    r_WB = []
    q_WB = []
    euler_WB = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_nsec()
        pos = msg.pose.pose.position
        rot = msg.pose.pose.orientation

        first_ts = ts if first_ts == 0 else first_ts
        timestamps.append(ts)
        r_WB.append([pos.x, pos.y, pos.z])
        q_WB.append([rot.w, rot.x, rot.y, rot.z])
        euler_WB.append(quat2euler([rot.w, rot.x, rot.y, rot.z]))

    timestamps = np.array(timestamps)
    r_WB = np.array(r_WB)
    euler_WB = np.rad2deg(np.array(euler_WB))

    PoseData = namedtuple("PoseData", "first_ts ts r_WB euler_WB")
    return PoseData(first_ts, timestamps, r_WB, euler_WB)


def parse_setpoint_data(bag, topic):
    first_ts = 0
    timestamps = []
    euler_WB = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_nsec()
        first_ts = ts if first_ts == 0 else first_ts

        timestamps.append(ts)
        q = msg.orientation
        euler = quat2euler([q.w, q.x, q.y, q.z])
        euler_WB.append(euler)

    timestamps = np.array(timestamps)
    euler_WB = np.rad2deg(np.array(euler_WB))

    SetpointData = namedtuple("SetpointData", "first_ts ts euler_WB")
    return SetpointData(first_ts, timestamps, euler_WB)


def parse_transform_stamped_data(bag, topic):
    first_ts = 0
    timestamps = []
    r_WB = []
    q_WB = []
    euler_WB = []

    for _, msg, t in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_nsec()
        pos = msg.transform.translation
        rot = msg.transform.rotation

        first_ts = ts if first_ts == 0 else first_ts
        timestamps.append(ts)
        r_WB.append([pos.x, pos.y, pos.z])
        q_WB.append([rot.w, rot.x, rot.y, rot.z])
        euler_WB.append(quat2euler([rot.w, rot.x, rot.y, rot.z]))


    timestamps = np.array(timestamps)
    r_WB = np.array(r_WB)
    euler_WB = np.rad2deg(np.array(euler_WB))

    TransformData = namedtuple("TransformData", "first_ts ts r_WB euler_WB")
    return TransformData(first_ts, timestamps, r_WB, euler_WB)


def parse_imu_data(bag, topic):
    first_ts = 0
    imu_ts = []
    imu_euler_WB = []
    imu_w_WB = []
    imu_a_WB = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_nsec()
        first_ts = ts if first_ts == 0 else first_ts
        imu_ts.append(ts)

        q = msg.orientation
        euler = quat2euler([q.w, q.x, q.y, q.z])
        imu_euler_WB.append(euler)

        w = msg.angular_velocity
        imu_w_WB.append([w.x, w.y, w.z])

        a = msg.linear_acceleration
        imu_a_WB.append([a.x, a.y, a.z])

    imu_ts = np.array(imu_ts)
    imu_euler_WB = np.rad2deg(np.array(imu_euler_WB))
    imu_w_WB = np.array(imu_w_WB)
    imu_a_WB = np.array(imu_a_WB)

    ImuData = namedtuple("ImuData", "first_ts ts euler_WB w_WB a_WB")
    return ImuData(first_ts, imu_ts, euler_WB, w_WB, a_WB)


def parse_timesync_data(bag, topic):
    first_ts = 0
    timesync_ts = []
    remote_timestamp_ns = []
    observed_offset_ns = []
    estimated_offset_ns = []
    round_trip_time_ms = []

    for _, msg, t in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_nsec()
        first_ts = ts if first_ts == 0 else first_ts
        timesync_ts.append(ts)
        remote_timestamp_ns.append(msg.remote_timestamp_ns)
        observed_offset_ns.append(msg.observed_offset_ns)
        estimated_offset_ns.append(msg.estimated_offset_ns)
        round_trip_time_ms.append(msg.round_trip_time_ms)

    timesync_ts = np.array(timesync_ts)
    remote_timestamp_ns = np.array(remote_timestamp_ns)
    observed_offset_ns = np.array(observed_offset_ns)
    estimated_offset_ns = np.array(estimated_offset_ns)
    round_trip_time_ms = np.array(round_trip_time_ms)

    field_names = ["first_ts"]
    field_names.append("timesync_ts")
    field_names.append("remote_timestamp_ns")
    field_names.append("observed_offset_ns")
    field_names.append("estimated_offset_ns")
    field_names.append("round_trip_time_ms")
    field_names = " ".join(field_names)

    TimeSyncData = namedtuple("TimeSyncData", field_names)
    return TimeSyncData(first_ts,
                        timesync_ts,
                        remote_timestamp_ns,
                        observed_offset_ns,
                        estimated_offset_ns,
                        round_trip_time_ms)

def lerp(a, b, t):
    return a * (1.0 - t) + b * t


def lerp_data(lerp_ts, target_ts, target_data):
    result_ts = []
    result_data = []

    t0 = target_ts[0]
    t1 = 0
    v0 = target_data[0]
    v1 = [0.0, 0.0, 0.0]

    # Loop through target signal
    lerp_idx = 0;
    for i in range(1, len(target_ts)):
        ts = target_ts[i]
        data = target_data[i]

        # Interpolate
        do_interp = ((ts - lerp_ts[lerp_idx]) * 1e-9) > 0
        if do_interp:
            t1 = ts
            v1 = data

            while lerp_idx < len(lerp_ts):
                if t1 < lerp_ts[lerp_idx]:
                    break

                # Calculate lerp alpha
                num = (lerp_ts[lerp_idx] - t0) * 1e-9
                den = (t1 - t0) * 1e-9
                alpha = num / den

                # Lerp and add to results
                result_data.append(lerp(v0, v1, alpha))
                result_ts.append(lerp_ts[lerp_idx])
                lerp_idx += 1

        # Shift interpolation end point to start point
        t0 = t1
        v0 = v1

        # Reset interpolation end point
        t1 = 0
        v1 = [0.0, 0.0, 0.0]

    return (np.array(result_ts), np.array(result_data))


if __name__ == "__main__":
    # ROS bag and topics
    bag_path = sys.argv[1]

    # Open bag and check if topics exists
    print("Opening ROS bag [%s]" % bag_path)
    bag = rosbag.Bag(bag_path, "r")
    # check_topic_exists(bag, topic_battery)
    # check_topic_exists(bag, topic_mocap)
    # check_topic_exists(bag, topic_setpoint)

    # Parse data
    print("Parsing ROS bag ...")
    battery_data = parse_battery_data(bag, topic_battery)
    # tf_data = parse_transform_stamped_data(bag, topic_mocap)
    # setpoint_data = parse_setpoint_data(bag, topic_setpoint)

    print("Fitting battery model ...")
    time = (battery_data.ts - battery_data.first_ts) * 1e-9
    voltage = battery_data.voltage
    battery_fit = np.polyfit(time, voltage, 1)
    battery_model = np.poly1d(battery_fit)
    print("Polynomial coefficients (highest order first): %s" % str(battery_fit))

    print("Visualize battery model")
    model_y = battery_model(np.linspace(0, 1000, len(time)))
    plt.plot(time, voltage)
    plt.plot(time, model_y)
    plt.xlabel("Time [s]")
    plt.ylabel("Voltage [v]")
    plt.show()
