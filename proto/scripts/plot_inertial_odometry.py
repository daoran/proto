import pandas as pd
import numpy as np
import matplotlib.pylab as plt

from proto import *


def load_data(csv_path):
  """ Load data """
  csv_data = pd.read_csv(csv_path)

  timestamps = []
  pose_data = []
  vel_data = []
  ba_data = []
  bg_data = []

  for row_idx in range(csv_data.shape[0]):
    # Timestamp
    pose_ts = csv_data["#ts"][row_idx]
    timestamps.append(pose_ts)

    # IMU Pose T_WS
    # -- Translation
    rx = csv_data["rx"][row_idx]
    ry = csv_data["ry"][row_idx]
    rz = csv_data["rz"][row_idx]
    r = np.array([rx, ry, rz])
    # -- Rotation
    qw = csv_data["qw"][row_idx]
    qx = csv_data["qx"][row_idx]
    qy = csv_data["qy"][row_idx]
    qz = csv_data["qz"][row_idx]
    q = np.array([qw, qx, qy, qz])
    # -- Add to pose list
    pose_data.append(tf(q, r))

    # Velocity v_WS
    vx = csv_data["vx"][row_idx]
    vy = csv_data["vy"][row_idx]
    vz = csv_data["vz"][row_idx]
    vel_data.append(np.array([vx, vy, vz]))

    # Accel Biases ba
    ba_x = csv_data["ba_x"][row_idx]
    ba_y = csv_data["ba_y"][row_idx]
    ba_z = csv_data["ba_z"][row_idx]
    ba_data.append(np.array([ba_x, ba_y, ba_z]))

    # Gyro Biases ba
    bg_x = csv_data["bg_x"][row_idx]
    bg_y = csv_data["bg_y"][row_idx]
    bg_z = csv_data["bg_z"][row_idx]
    bg_data.append(np.array([bg_x, bg_y, bg_z]))

  return {"timestamps": timestamps,
          "poses": pose_data,
          "velocities": vel_data,
          "accel_biases": ba_data,
          "gyro_biases": bg_data}


def extract_translations(poses):
  """ Extract translations """
  return np.array([tf_trans(pose) for pose in poses])


if __name__ == "__main__":
  gnd_csv = "/tmp/imu_odom-gnd.csv"
  init_csv = "/tmp/imu_odom-init.csv"
  est_csv = "/tmp/imu_odom-est.csv"
  gnd_data = load_data(gnd_csv)
  init_data = load_data(init_csv)
  est_data = load_data(est_csv)

  pos_gnd = extract_translations(gnd_data["poses"])
  pos_init = extract_translations(init_data["poses"])
  pos_est = extract_translations(est_data["poses"])

  plt.plot(pos_gnd[:, 0], pos_gnd[:, 1], 'r-', label="Ground Truth")
  plt.plot(pos_init[:, 0], pos_init[:, 1], 'b-', label="Initial")
  plt.plot(pos_est[:, 0], pos_est[:, 1], 'g-', label="Estimated")
  plt.xlabel("Displacement [m]")
  plt.ylabel("Displacement [m]")
  plt.legend(loc=0)
  plt.axis("equal")
  plt.show()
