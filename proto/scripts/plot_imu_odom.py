#!/usr/bin/env python3
import pandas as pd
import matplotlib.pylab as plt

if __name__ == "__main__":
  odom_gnd = pd.read_csv("/tmp/imu_odom-gnd.csv")
  odom_init = pd.read_csv("/tmp/imu_odom-init.csv")
  odom_est = pd.read_csv("/tmp/imu_odom-est.csv")

  gnd_x = odom_gnd["x"].to_numpy()
  gnd_y = odom_gnd["y"].to_numpy()
  gnd_z = odom_gnd["z"].to_numpy()
  gnd_vx = odom_gnd["vx"].to_numpy()
  gnd_vy = odom_gnd["vy"].to_numpy()
  gnd_vz = odom_gnd["vz"].to_numpy()

  init_x = odom_init["x"].to_numpy()
  init_y = odom_init["y"].to_numpy()
  init_z = odom_init["z"].to_numpy()
  init_vx = odom_init["vx"].to_numpy()
  init_vy = odom_init["vy"].to_numpy()
  init_vz = odom_init["vz"].to_numpy()

  est_x = odom_est["x"].to_numpy()
  est_y = odom_est["y"].to_numpy()
  est_z = odom_est["z"].to_numpy()
  est_vx = odom_est["vx"].to_numpy()
  est_vy = odom_est["vy"].to_numpy()
  est_vz = odom_est["vz"].to_numpy()

  # Plot Position
  plt.figure()
  plt.plot(gnd_x, gnd_y, 'r--', label="Ground Truth")
  plt.plot(init_x, init_y, 'g-', label="Initial")
  plt.plot(est_x, est_y, 'b-', label="Estimate")
  plt.legend(loc=0)

  # Plot Velocity
  plt.figure()
  plt.subplot(311)
  plt.plot(gnd_vx, 'r--', label="Ground Truth")
  plt.plot(init_vx, 'g-', label="Initial")
  plt.plot(est_vx, 'b-', label="Estimate")
  plt.title("Velocity - X-axis")
  plt.legend(loc=0)

  plt.subplot(312)
  plt.plot(gnd_vy, 'r--', label="Ground Truth")
  plt.plot(init_vy, 'g-', label="Initial")
  plt.plot(est_vy, 'b-', label="Estimate")
  plt.title("Velocity - Y-axis")
  plt.legend(loc=0)

  plt.subplot(313)
  plt.plot(gnd_vz, 'r--', label="Ground Truth")
  plt.plot(init_vz, 'g-', label="Initial")
  plt.plot(est_vz, 'b-', label="Estimate")
  plt.title("Velocity - Z-axis")
  plt.legend(loc=0)

  plt.show()
