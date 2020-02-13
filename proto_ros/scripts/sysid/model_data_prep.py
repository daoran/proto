#!/usr/bin/env python
import sys
import math
from math import atan2
from math import asin
from math import degrees
from collections import namedtuple

import numpy as np
from scipy import signal

import rosbag
import rospy

from common import *


if __name__ == "__main__":
    # ROS bag and topics
    bag_path = sys.argv[1]
    topic_mocap = "/vicon/ucl/ucl"
    topic_imu = "/mavros/imu/data"
    topic_setpoint_att = "/mavros/setpoint_raw/target_attitude"
    topic_setpoint_thr = "/mavros/setpoint_raw/thrust"

    # Open bag and check if topics exists
    print("Opening ROS bag [%s]" % bag_path)
    bag = rosbag.Bag(bag_path, "r")
    check_topic_exists(bag, topic_mocap)
    check_topic_exists(bag, topic_imu)
    check_topic_exists(bag, topic_setpoint_att)

    # Parse data
    print("Parsing ROS bag ...")
    mocap = parse_transform_stamped_data(bag, topic_mocap)
    ref_att = parse_setpoint_attitude_data(bag, topic_setpoint_att)
    # setpoint_thr = parse_setpoint_thrust_data(bag, topic_thrust)
    imu = parse_imu_data(bag, topic_imu)

    # Find first timestamp across multiple data sources and convert from
    # nano-seconds to seconds
    first_ts = np.min([mocap.first_ts, ref_att.first_ts, imu.first_ts])
    mocap_time = (mocap.ts - first_ts) * 1e-9
    ref_time = (ref_att.ts - first_ts) * 1e-9
    imu_time = (imu.ts - first_ts) * 1e-9

    # Resample data
    g = 9.81
    t_min = 20.0
    t_max = 36.0
    dt = 1e-3
    t_new = np.linspace(t_min, t_max, 1 / dt)
    thrust_ff = 0.5675  # Feed forward Thrust

    mocap_time, mocap_r = lerp_data(t_new, mocap_time, mocap.r)
    mocap_time, mocap_euler = lerp_data(t_new, mocap_time, mocap.euler)
    ref_time, ref_att = lerp_data(t_new, ref_time, ref_att.euler)
    imu_time, imu_w = lerp_data(t_new, imu_time, imu.w)
    imu_time, imu_a = lerp_data(t_new, imu_time, imu.a)

    # Compute Linear Velocities
    r_dot = np.diff(mocap_r.transpose()) / dt
    r_dot = np.append(r_dot, np.zeros((3, 1)), 1)
    r_dot = r_dot.transpose()
    # plot_velocities(mocap_time, r_dot[:, 0], r_dot[:, 1], r_dot[:, 2])
    # plt.show()

    mocap_time = mocap_time.reshape(-1, 1)
    mocap = np.append(mocap_time, mocap_r, 1);
    mocap = np.append(mocap, mocap_euler, 1);
    r_dot = np.append(mocap_time, r_dot, 1);

    ref_time = ref_time.reshape(-1, 1)
    ref = np.append(ref_time, ref_att, 1);

    imu_time = imu_time.reshape(-1, 1)
    imu = np.append(imu_w, imu_a, 1);
    imu = np.append(imu_time, imu, 1);

    np.savetxt('/tmp/mocap.csv', mocap, delimiter=',')
    np.savetxt('/tmp/r_dot.csv', r_dot, delimiter=',')
    np.savetxt('/tmp/ref.csv', ref, delimiter=',')
    np.savetxt('/tmp/imu.csv', imu, delimiter=',')

    # Estimate pitch transfer function
    # theta_data = iddata(detrend(theta_ts.Data), detrend(theta_ref_ts.Data), Ts);
    # pitch_tf = tfest(theta_data, sys_order,0);
    # compare(theta_data,pitch_tf);

    # Compute velocity in the N frame (i.e. body frame of the drone)

    # Visualize data
    # plot_displacement(mocap_time, mocap.r[:, 0], mocap.r[:, 1], mocap.r[:, 2])
    # plot_setpoints(ref_time, ref_att.euler[:, 0], ref_att.euler[:, 1])
    # plot_imu(imu_time, imu.w, imu.a)
    # plt.show()
