#!/usr/bin/env python
import sys

import rospy
import rosbag

from scipy.interpolate import interp1d

import matplotlib
import matplotlib.pylab as plt
matplotlib.rcParams['mathtext.fontset'] = 'custom'
matplotlib.rcParams['mathtext.rm'] = 'Bitstream Vera Sans'
matplotlib.rcParams['mathtext.it'] = 'Bitstream Vera Sans:italic'
matplotlib.rcParams['mathtext.bf'] = 'Bitstream Vera Sans:bold'

from common import *


def find_nearest(array, value):
    idx = (np.abs(array - value)).argmin()
    return array[idx]

class SelectStartEnd:
    def __init__(self, imu, thrust):
        self.coords = []

        self.ts_start = None
        self.ts_end = None
        self.ts_start_idx = None
        self.ts_end_idx = None

        self.cid = None
        self.fig = plt.figure()
        self.imu = imu
        self.thrust = thrust

    def find_nearest(self, array, value):
        idx = (np.abs(array - value)).argmin()
        return array[idx]

    def onclick(self, event):
        self.coords.append((event.xdata, event.ydata))

        if len(self.coords) == 1:
            xy = self.coords[-1]
            t = (self.thrust.ts - self.thrust.first_ts) * 1e-9
            ts_idx = np.where(t == (find_nearest(t, xy[0])))[0][0]
            self.ts_start = self.thrust.ts[ts_idx]
            self.ts_start_idx = ts_idx
            print("Set ts_start: [%.2fs]" % t[ts_idx])

        if len(self.coords) == 2:
            xy = self.coords[-1]
            t = (self.thrust.ts - self.thrust.first_ts) * 1e-9
            ts_idx = np.where(t == (find_nearest(t, xy[0])))[0][0]
            self.ts_end = self.thrust.ts[ts_idx]
            self.ts_end_idx = ts_idx
            print("Set ts_end: [%.2fs]" % t[ts_idx])
            self.fig.canvas.mpl_disconnect(self.cid)
            plt.close()

    def show(self):
        print("Click on graph for start and end time!")
        ax = self.fig.add_subplot(111)
        thrust_time = (self.thrust.ts - self.thrust.first_ts) * 1e-9
        ax.plot(thrust_time, self.thrust.thrust)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Normalized Thrust")
        ax.set_title("Thrust data")
        # -- Figure misc
        self.fig.subplots_adjust(hspace=0.4)
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        plt.show(1)

        return {"ts_start": self.ts_start,
                "ts_end": self.ts_end,
                "ts_start_idx": self.ts_start_idx,
                "ts_end_idx": self.ts_end_idx}


if __name__ == "__main__":
    # ROS bag and topics
    bag_path = sys.argv[1]
    topic_battery = "/mavros/battery"
    topic_thrust = "/mavros/setpoint_raw/attitude"
    topic_imu = "/mavros/imu/data"

    # Open bag and check if topics exists
    print("Opening ROS bag [%s]" % bag_path)
    bag = rosbag.Bag(bag_path, "r")
    check_topic_exists(bag, topic_battery)

    # Parse data
    print("Parsing ROS bag ...")
    battery = parse_battery_data(bag, topic_battery)
    thrust = parse_setpoint_thrust_data(bag, topic_thrust)
    imu = parse_imu_data(bag, topic_imu)

    # Select the start and end of the hover test
    print("Select start and end hover time ...")
    select = SelectStartEnd(imu, thrust)
    select_ts = select.show()

    # Find a time range that is:
    # - within range of both voltage and trust data
    # - within ranage of select start and end time
    ts = []
    for ts_k in thrust.ts:
        if ts_k < battery.ts[0]:
            continue
        if ts_k > battery.ts[-1]:
            continue
        if ts_k < select_ts["ts_start"]:
            continue
        if ts_k > select_ts["ts_end"]:
            break
        ts.append(ts_k)

    # Sychronize both voltage and thrust data by resampling both datasets
    print("Synchronizing data ...")
    voltage_fn = interp1d(battery.ts, battery.voltage)
    thrust_fn = interp1d(thrust.ts, thrust.thrust)
    voltage_data = voltage_fn(ts)
    thrust_data = thrust_fn(ts)

    # Plot resampled data
    # # -- Plot resampled voltage data
    # first_ts = min(ts[0], battery.ts[0])
    # plt.figure()
    # plt.plot((battery.ts - first_ts) * 1e-9, battery.voltage, label="original")
    # plt.plot((ts - first_ts) * 1e-9, voltage_fn(ts), label="interpolated")
    # plt.title("Resample Voltage Data")
    # plt.legend(loc=0)
    # # -- Plot resampled thrust data
    # first_ts = min(ts[0], thrust.ts[0])
    # plt.figure(2)
    # plt.plot((thrust.ts - first_ts) * 1e-9, thrust.thrust, label="original")
    # plt.plot((ts - first_ts) * 1e-9, thrust_fn(ts), label="interpolated")
    # plt.title("Resample Thrust Data")
    # plt.legend(loc=0)
    # plt.show()

    # Fit battery model
    print("Fitting battery model ...")
    voltage_thrust_fit = np.polyfit(voltage_data, thrust_data, 1)
    voltage_thrust_model = np.poly1d(voltage_thrust_fit)
    print("Polynomial coeffs (highest order first): %s" % str(voltage_thrust_fit))

    # Visualize voltage model
    plt.plot(voltage_data, thrust_data, ".", markersize=2)
    plt.plot(voltage_data, voltage_thrust_model(voltage_data), color="red")
    plt.xlabel("Voltage [v]")
    plt.ylabel("Thrust")
    plt.title("Battery model")
    plt.show()
