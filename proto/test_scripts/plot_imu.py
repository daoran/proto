#!/usr/bin/env python3
import pandas
import matplotlib.pylab as plt

if __name__ == "__main__":
  est = pandas.read_csv("/tmp/imu_est.csv")
  gnd = pandas.read_csv("/tmp/imu_gnd.csv")

  plt.figure()
  plt.plot(est['rx'], est['ry'], 'r-', label="Estimate")
  plt.plot(gnd['rx'], gnd['ry'], 'r--', label="Ground-Truth")
  plt.xlabel("x [m]")
  plt.ylabel("y [m]")
  plt.legend(loc=0)
  plt.show()

  plt.figure()
  plt.plot(est['ts'] * 1e-9, est['rx'], 'r-')
  plt.plot(est['ts'] * 1e-9, est['ry'], 'g-')
  plt.plot(est['ts'] * 1e-9, est['rz'], 'b-')
  plt.plot(gnd['ts'] * 1e-9, gnd['rx'], 'r--')
  plt.plot(gnd['ts'] * 1e-9, gnd['ry'], 'g--')
  plt.plot(gnd['ts'] * 1e-9, gnd['rz'], 'b--')
  plt.xlabel("Time [s]")
  plt.ylabel("Displacement [m]")
  plt.show()
