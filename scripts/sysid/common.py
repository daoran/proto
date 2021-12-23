from math import atan2
from math import asin
from collections import namedtuple

import matplotlib
import matplotlib.pylab as plt
matplotlib.rcParams['mathtext.fontset'] = 'custom'
matplotlib.rcParams['mathtext.rm'] = 'Bitstream Vera Sans'
matplotlib.rcParams['mathtext.it'] = 'Bitstream Vera Sans:italic'
matplotlib.rcParams['mathtext.bf'] = 'Bitstream Vera Sans:bold'

import numpy as np


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


def crop_data(time, data, t_min, t_max):
    crop_time = []
    crop_data = []

    idx = 0
    for t in time:
        if t >= t_min and t <= t_max:
            crop_time.append(t)
            crop_data.append(data[idx])
        idx += 1

    return (np.array(crop_time), np.array(crop_data))


def lerp(a, b, t):
    return a * (1.0 - t) + b * t


def slerp(p0, p1, t):
    omega = np.arccos(np.dot(p0/np.norm(p0), p1/np.norm(p1)))
    so = np.sin(omega)
    return np.sin((1.0-t)*omega) / so * p0 + np.sin(t*omega)/so * p1


def lerp_data(lerp_ts, target_ts, target_data, lerp_func=lerp):
    result_ts = []
    result_data = []

    t0 = target_ts[0]
    t1 = 0
    v0 = target_data[0]
    v1 = np.array([0.0, 0.0, 0.0])

    # Loop through target signal
    lerp_idx = 0;
    for i in range(1, len(target_ts)):
        ts = target_ts[i]
        data = target_data[i]

        # Are we done lerping?
        if (lerp_idx + 1) > len(lerp_ts):
            break

        # Interpolate
        t_diff = (ts - lerp_ts[lerp_idx]) * 1e-9
        do_interp = t_diff > 0
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
                result_data.append(lerp_func(v0, v1, alpha))
                result_ts.append(lerp_ts[lerp_idx])
                lerp_idx += 1

            # Shift interpolation end point to start point
            t0 = t1
            v0 = v1

            # Reset interpolation end point
            t1 = 0
            v1 = np.array([0.0, 0.0, 0.0])

    return (np.array(result_ts), np.array(result_data))


def check_topic_exists(bag, topic):
    info = bag.get_type_and_topic_info()
    if topic not in info.topics:
        raise RuntimeError("Opps! topic [%s] not in bag!" % topic)


def parse_battery_data(bag, topic):
    first_ts = 0
    timestamps = []

    voltage = []                  # Voltage in Volts (Mandatory)
    current = []                  # Negative when discharging (A)
    charge = []                   # Current charge in Ah
    capacity = []                 # Capacity in Ah (last full capacity)
    design_capacity = []          # Capacity in Ah (design capacity)
    percentage = []               # Charge percentage on 0 to 1 range
    power_supply_status = []      # The charging status as reported.
    power_supply_health = []      # The battery health metric.
    power_supply_technology = []  # The battery chemistry.
    present = []                  # True if the battery is present

    for topic, msg, t in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_nsec()
        first_ts = ts if first_ts == 0 else first_ts
        timestamps.append(ts)

        voltage.append(msg.voltage)
        current.append(msg.current)
        charge.append(msg.charge)
        capacity.append(msg.capacity)
        design_capacity.append(msg.design_capacity)
        percentage.append(msg.percentage)
        power_supply_status.append(msg.power_supply_status)
        power_supply_health.append(msg.power_supply_health)
        power_supply_technology.append(msg.power_supply_technology)
        present.append(msg.present)

    timestamps = np.array(timestamps)
    voltage = np.array(voltage)
    current = np.array(current)
    charge = np.array(charge)
    capacity = np.array(capacity)
    design_capacity = np.array(design_capacity)
    percentage = np.array(percentage)
    power_supply_status = np.array(power_supply_status)
    power_supply_health = np.array(power_supply_health)
    power_supply_technology = np.array(power_supply_technology)
    present = np.array(present)

    field_names = ["first_ts"]
    field_names.append("ts")
    field_names.append("voltage")
    field_names.append("current")
    field_names.append("charge")
    field_names.append("capacity")
    field_names.append("design_capacity")
    field_names.append("percentage")
    field_names.append("power_supply_status")
    field_names.append("power_supply_health")
    field_names.append("power_supply_technology")
    field_names.append("present")
    field_names = " ".join(field_names)

    BatteryData = namedtuple("BatteryData", field_names)
    return BatteryData(first_ts,
                       timestamps,
                       voltage,
                       current,
                       charge,
                       capacity,
                       design_capacity,
                       percentage,
                       power_supply_status,
                       power_supply_health,
                       power_supply_technology,
                       present)


def parse_pose_stamped_data(bag, topic):
    first_ts = 0
    timestamps = []
    r = []
    q = []
    euler = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_nsec()
        pos = msg.pose.pose.position
        rot = msg.pose.pose.orientation

        first_ts = ts if first_ts == 0 else first_ts
        timestamps.append(ts)
        r.append([pos.x, pos.y, pos.z])
        q.append([rot.w, rot.x, rot.y, rot.z])
        euler.append(quat2euler([rot.w, rot.x, rot.y, rot.z]))

    timestamps = np.array(timestamps)
    r = np.array(r)
    euler = np.rad2deg(np.array(euler))

    PoseData = namedtuple("PoseData", "first_ts ts r euler")
    return PoseData(first_ts, timestamps, r, euler)


def parse_setpoint_attitude_data(bag, topic):
    first_ts = 0
    timestamps = []
    euler = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_nsec()
        first_ts = ts if first_ts == 0 else first_ts

        timestamps.append(ts)
        q = msg.orientation
        euler.append(quat2euler([q.w, q.x, q.y, q.z]))

    timestamps = np.array(timestamps)
    euler = np.rad2deg(np.array(euler))

    SetpointData = namedtuple("SetpointData", "first_ts ts euler")
    return SetpointData(first_ts, timestamps, euler)


def parse_setpoint_thrust_data(bag, topic):
    first_ts = 0
    timestamps = []
    thrust = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_nsec()
        first_ts = ts if first_ts == 0 else first_ts
        timestamps.append(ts)
        thrust.append(msg.thrust)

    timestamps = np.array(timestamps)
    thrust = np.array(thrust)

    ThrustData = namedtuple("ThrustData", "first_ts ts thrust")
    return ThrustData(first_ts, timestamps, thrust)


def parse_transform_stamped_data(bag, topic):
    first_ts = 0
    timestamps = []
    r = []
    q = []
    euler = []

    for _, msg, t in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_nsec()
        pos = msg.transform.translation
        rot = msg.transform.rotation

        first_ts = ts if first_ts == 0 else first_ts
        timestamps.append(ts)
        r.append([pos.x, pos.y, pos.z])
        q.append([rot.w, rot.x, rot.y, rot.z])
        euler.append(quat2euler([rot.w, rot.x, rot.y, rot.z]))

    timestamps = np.array(timestamps)
    r = np.array(r)
    euler = np.rad2deg(np.array(euler))

    TransformData = namedtuple("TransformData", "first_ts ts r euler")
    return TransformData(first_ts, timestamps, r, euler)


def parse_imu_data(bag, topic):
    first_ts = 0
    imu_ts = []
    imu_euler = []
    imu_w = []
    imu_a = []

    for topic, msg, t in bag.read_messages(topics=[topic]):
        ts = msg.header.stamp.to_nsec()
        first_ts = ts if first_ts == 0 else first_ts
        imu_ts.append(ts)

        q = msg.orientation
        imu_euler.append(quat2euler([q.w, q.x, q.y, q.z]))

        w = msg.angular_velocity
        imu_w.append([w.x, w.y, w.z])

        a = msg.linear_acceleration
        imu_a.append([a.x, a.y, a.z])

    imu_ts = np.array(imu_ts)
    imu_euler = np.rad2deg(np.array(imu_euler))
    imu_w = np.array(imu_w)
    imu_a = np.array(imu_a)

    ImuData = namedtuple("ImuData", "first_ts ts euler w a")
    return ImuData(first_ts, imu_ts, imu_euler, imu_w, imu_a)


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


def plot_displacement(time, x, y, z):
    plt.figure()
    plt.suptitle("Displacement")

    plt.subplot(211)
    plt.plot(time, x, 'r-', label='x')
    plt.plot(time, y, 'g-', label='y')
    plt.xlabel("Time [s]")
    plt.ylabel("Displacement [m]")
    plt.legend(loc=0)

    plt.subplot(212)
    plt.plot(time, z, 'b-', label='z')
    plt.xlabel("Time [s]")
    plt.ylabel("Altitude [m]")
    plt.legend(loc=0)


def plot_velocities(time, x, y, z):
    plt.figure()
    plt.suptitle("Velocity")

    plt.plot(time, x, 'r-', label='x')
    plt.plot(time, y, 'g-', label='y')
    plt.plot(time, z, 'b-', label='z')

    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [ms^-1]")
    plt.legend(loc=0)


def plot_setpoints(time, x, y):
    plt.figure()
    plt.suptitle("Setpoints")

    plt.plot(time, x, 'r-', label='x')
    plt.plot(time, y, 'g-', label='y')
    plt.xlabel("Time [s]")
    plt.ylabel("Attitude [deg]")
    plt.legend(loc=0)


def plot_imu(time, w, a):
    plt.figure()
    plt.suptitle("IMU - Angular Velocity")

    plt.plot(time, w[:, 0], 'r-', label='x')
    plt.plot(time, w[:, 1], 'g-', label='y')
    plt.plot(time, w[:, 2], 'b-', label='z')
    plt.xlabel("Time [s]")
    plt.ylabel("Angular Velocity [rad s^-1]")
    plt.legend(loc=0)

    plt.figure()
    plt.suptitle("IMU - Acceleration")

    plt.plot(time, a[:, 0], 'r-', label='x')
    plt.plot(time, a[:, 1], 'g-', label='y')
    plt.plot(time, a[:, 2], 'b-', label='z')
    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [ms^-2]")
    plt.legend(loc=0)
