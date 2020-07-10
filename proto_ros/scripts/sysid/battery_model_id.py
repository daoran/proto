#!/usr/bin/env python
import sys

import rospy
import rosbag

import matplotlib
import matplotlib.pylab as plt
matplotlib.rcParams['mathtext.fontset'] = 'custom'
matplotlib.rcParams['mathtext.rm'] = 'Bitstream Vera Sans'
matplotlib.rcParams['mathtext.it'] = 'Bitstream Vera Sans:italic'
matplotlib.rcParams['mathtext.bf'] = 'Bitstream Vera Sans:bold'

from common import *


if __name__ == "__main__":
    # ROS bag and topics
    bag_path = sys.argv[1]
    topic_battery = "/mavros/battery"

    # Open bag and check if topics exists
    print("Opening ROS bag [%s]" % bag_path)
    bag = rosbag.Bag(bag_path, "r")
    check_topic_exists(bag, topic_battery)

    # Parse data
    print("Parsing ROS bag ...")
    battery = parse_battery_data(bag, topic_battery)

    # Fit battery model
    print("Fitting battery model ...")
    time = (battery.ts - battery.first_ts) * 1e-9
    voltage = battery.voltage
    battery_fit = np.polyfit(time, voltage, 1)
    battery_model = np.poly1d(battery_fit)
    print("Polynomial coeffs (highest order first): %s" % str(battery_fit))

    # Visualize battery model
    print("Visualize battery model")
    model_y = battery_model(np.linspace(0, 1000, len(time)))
    plt.plot(time, voltage)
    plt.plot(time, model_y)
    plt.xlabel("Time [s]")
    plt.ylabel("Voltage [v]")
    plt.show()
