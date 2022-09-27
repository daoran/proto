#!/usr/bin/env python3
"""
Firmware Debugger
"""
import sys
import time
import subprocess
import importlib
import traceback
import argparse

try:
  import serial
  import numpy as np
  import pyqtgraph as pg
  from PyQt5.QtCore import QTimer
  import matplotlib.pylab as plt
except ImportError:
  print(traceback.format_exc())


def install_package(package):
  """ Install package """
  if importlib.util.find_spec(package) is None:
    subprocess.check_call([sys.executable, "-m", "pip", "install", package])


def parse_serial_data_line(line):
  """ Parse serial data """
  data = {}

  line = line.strip()
  if len(line) == 0:
    return data

  if type(line) is str:
    for el in line.split(" "):
      key = el.split(":")[0].strip()
      val = float(el.split(":")[1].strip())
      data[key] = val
  else:
    for el in line.split(b" "):
      key = el.split(b":")[0].strip().decode("ascii")
      val = float(el.split(b":")[1].strip())
      data[key] = val

  data['ts'] = data['ts'] * 1e-6

  return data


def record_telemetry_data(save_path, record_time):
  """ Record Telemetry Data """
  s = serial.Serial()
  s.port = '/dev/ttyACM0'
  s.baudrate = 921600
  s.timeout = 0.1
  s.open()

  if s.is_open is True:
    print("Connected to FCU ...")
  else:
    print("Failed to connect to FCU ...")
    sys.exit(-1)

  time.sleep(2)
  line = s.readline()
  if line is None:
    return

  time_start = time.time()
  data = []
  while line:
    data.append(line.decode("ascii"))
    line = s.readline()

    if (time.time() - time_start) > record_time:
      print((time.time() - time_start))
      print("BREAK!")
      break

  log_file = open(save_path, "w")
  for data_line in data:
    log_file.write(data_line)


def analyze_timestamps():
  """ Analyze Timestamps """
  import numpy as np
  import matplotlib.pylab as plt

  log_file = "/tmp/debugger.log"
  record_duration = 10.0

  # Record telemetry data
  record_telemetry_data(log_file, record_duration)

  # Parse log file
  timestamps = []
  for line in open(log_file, "r"):
    data = parse_serial_data_line(line)
    timestamps.append(data['ts'])
  timestamps = np.array(timestamps)

  # Calculate rates
  rates = []
  start_idx = 100
  for k in range(start_idx, len(timestamps)):
    ts_k = timestamps[k]
    ts_km1 = timestamps[k - 1]
    dt = ts_k - ts_km1
    rates.append(1.0 / dt)

  # Plot
  plt.plot(timestamps[start_idx:], rates, 'ro')
  plt.ylim([0, 1000])
  plt.xlabel("Time [s]")
  plt.xlabel("Rate [Hz]")
  plt.show()


class RTLinePlot:
  """ Real-Time Line Plot """
  def __init__(self, win, title, x_key, y_keys, x_label, y_label, **kwargs):
    self.plot = win.addPlot(title=title)
    self.plot.addLegend()
    self.plot.setLabel("bottom", x_label)
    self.plot.setLabel("left", y_label)
    self.plot.setDownsampling(mode='peak')
    self.plot.setClipToView(True)

    self.y_min = kwargs.get("y_min")
    self.y_max = kwargs.get("y_max")
    self.y_padding = kwargs.get("y_padding", 10)
    if self.y_min and self.y_max:
      self.plot.setLimits(yMin=self.y_min, yMax=self.y_max)
      self.plot.setYRange(self.y_min, self.y_max, padding=self.y_padding)

    self.win_size = kwargs.get("win_size", 1000)
    self.colors = kwargs.get("colors", ["r", "g", "b", "c", "y", "m"])
    self.x_key = x_key
    self.y_keys = y_keys
    self.data = {}
    self.curves = {}

    self.data[x_key] = []
    for i, y_key in enumerate(self.y_keys):
      self.data[y_key] = []
      self.curves[y_key] = self.plot.plot(pen=self.colors[i],
                                          name=y_key,
                                          skipFiniteCheck=True)

  def set_xlink(self, rt_plot):
    """ Set xlink """
    self.plot.setXLink(rt_plot.plot)

  def update_data(self, data):
    """ Update Data """
    self.data[self.x_key].append(data[self.x_key])
    for y_key in self.y_keys:
      if y_key in data:
        self.data[y_key].append(data[y_key])

  def update_plots(self):
    """ Update Plots """
    for y_key in self.y_keys:
      x_window = self.data[self.x_key][-self.win_size:]
      y_window = self.data[y_key][-self.win_size:]
      self.curves[y_key].setData(x_window, y_window)
      if self.y_min and self.y_max:
        self.plot.setLimits(yMin=self.y_min, yMax=self.y_max)
        self.plot.setYRange(self.y_min, self.y_max, padding=self.y_padding)


class FirmwareDebugger:
  """ Firmware Debugger """
  def __init__(self, **kwargs):
    self.log_file = kwargs.get("log_file")
    self.offline_mode = (self.log_file is not None)
    self.win_size = -1 if self.offline_mode else 1000
    self.s = None  # Serial
    self.app = None
    self.win = None
    self.timestamps = []
    self.last_plotted = None

    self._setup_gui()
    if self.log_file:
      self._load_log_data(self.log_file)
    else:
      self.log = open("/tmp/debugger.log", "w")
      self._setup_uart()
      self._start_plots()

  def _setup_uart(self):
    """ Setup UART comms """
    # Setup serial communication
    self.serial = serial.Serial()
    self.serial.port = '/dev/ttyACM0'
    self.serial.baudrate = 115200
    self.serial.timeout = 0
    self.serial.open()

    if self.serial.is_open is True:
      print("Connected to FCU ...")
    else:
      print("Failed to connect to FCU ...")
      sys.exit(-1)

  def _setup_gui(self):
    """ Setup GUI """
    self.app = pg.mkQApp("Firmware Debugger")
    self.win = pg.GraphicsLayoutWidget(show=True, title="Firmware Debugger")
    self.win.resize(1910, 768)
    pg.setConfigOptions(antialias=True)

    title = "Gyroscope"
    x_key = "ts"
    y_keys = ["gyro_x", "gyro_y", "gyro_z"]
    x_label = "Time [s]"
    y_label = "Angular Velocity [rad / s]"
    kwargs = {"y_min": -5.0, "y_max": 5.0, "win_size": self.win_size}
    self.accel_plot = RTLinePlot(
        self.win,
        title,
        x_key,
        y_keys,
        x_label,
        y_label,
        **kwargs,
    )

    title = "Accelerometer"
    x_key = "ts"
    y_keys = ["accel_x", "accel_y", "accel_z"]
    x_label = "Time [s]"
    y_label = "Acceleration [m / s^2]"
    kwargs = {"y_min": -12.0, "y_max": 12.0, "win_size": self.win_size}
    self.gyro_plot = RTLinePlot(
        self.win,
        title,
        x_key,
        y_keys,
        x_label,
        y_label,
        **kwargs,
    )
    self.gyro_plot.set_xlink(self.accel_plot)

    title = "Attitude"
    x_key = "ts"
    y_keys = [
        "roll",
        "pitch",
        "yaw",
        "roll_desired",
        "pitch_desired",
        "yaw_desired",
    ]
    x_label = "Time [s]"
    y_label = "Attitude [deg]"
    kwargs = {"y_min": -60.0, "y_max": 60.0, "win_size": self.win_size}
    self.attitude_plot = RTLinePlot(
        self.win,
        title,
        x_key,
        y_keys,
        x_label,
        y_label,
        **kwargs,
    )
    self.attitude_plot.set_xlink(self.gyro_plot)

    self.win.nextRow()
    title = "SBUS"
    x_key = "ts"
    y_keys = ["ch[0]", "ch[1]", "ch[2]", "ch[3]"]
    x_label = "Time [s]"
    y_label = "SBUS Value"
    kwargs = {"y_min": 175, "y_max": 1850, "win_size": self.win_size}
    self.sbus_plot = RTLinePlot(
        self.win,
        title,
        x_key,
        y_keys,
        x_label,
        y_label,
        **kwargs,
    )
    self.sbus_plot.set_xlink(self.attitude_plot)

    title = "Motor Outputs"
    x_key = "ts"
    y_keys = ["outputs[0]", "outputs[1]", "outputs[2]", "outputs[3]"]
    x_label = "Time [s]"
    y_label = "Motor Thrust [%]"
    kwargs = {"y_min": 0.0, "y_max": 1.0, "win_size": self.win_size}
    self.motors_plot = RTLinePlot(
        self.win,
        title,
        x_key,
        y_keys,
        x_label,
        y_label,
        **kwargs,
    )
    self.motors_plot.set_xlink(self.attitude_plot)

    title = "Control Rate"
    x_key = "ts"
    y_keys = ["rate"]
    x_label = "Time [s]"
    y_label = "Rate [Hz]"
    # kwargs = {"y_min": 0.0, "y_max": 1000.0}
    self.rate_plot = RTLinePlot(
        self.win,
        title,
        x_key,
        y_keys,
        x_label,
        y_label,
        **kwargs,
    )
    self.rate_plot.set_xlink(self.motors_plot)

  def _start_plots(self):
    """ Start plots """
    timer = QTimer()
    timer.timeout.connect(self._update_plots)
    timer.start()
    pg.exec()

  def _load_log_data(self, log_file):
    """ Load log data """
    # Load log data to plots
    for line in open(log_file, "r"):
      data = parse_serial_data_line(line)
      self.timestamps.append(data['ts'])
      self.sbus_plot.update_data(data)
      self.gyro_plot.update_data(data)
      self.accel_plot.update_data(data)
      self.attitude_plot.update_data(data)
      self.motors_plot.update_data(data)

    for k, dt in enumerate(np.diff(self.timestamps)):
      ts = self.timestamps[k]
      rate = 1.0 / dt
      self.rate_plot.update_data({"ts": ts, "rate": rate})

    # Update plots
    self.sbus_plot.update_plots()
    self.gyro_plot.update_plots()
    self.accel_plot.update_plots()
    self.attitude_plot.update_plots()
    self.motors_plot.update_plots()
    self.rate_plot.update_plots()
    self.last_plotted = time.time()

    # Run GUI
    pg.exec()

  def _update_plots(self):
    """ Update plots """
    # Read serial buffer
    line = self.serial.readline()
    if line is None:
      return

    while line:
      data = parse_serial_data_line(line)
      self.timestamps.append(data['ts'])

      self.sbus_plot.update_data(data)
      self.gyro_plot.update_data(data)
      self.accel_plot.update_data(data)
      self.attitude_plot.update_data(data)
      self.motors_plot.update_data(data)

      if len(self.timestamps) > 2:
        ts_km1 = self.timestamps[-2]
        ts_k = self.timestamps[-1]
        dt = ts_k - ts_km1
        rate = 1.0 / dt
        self.rate_plot.update_data({"ts": ts_k, "rate": rate})

      self.log.write(line.decode("ascii"))
      line = self.serial.readline()

    # Update plots
    time_now = time.time()
    if self.last_plotted is None or (time_now - self.last_plotted) > 0.05:
      self.sbus_plot.update_plots()
      self.gyro_plot.update_plots()
      self.accel_plot.update_plots()
      self.attitude_plot.update_plots()
      self.motors_plot.update_plots()
      self.rate_plot.update_plots()
      self.last_plotted = time.time()


if __name__ == "__main__":
  install_package("pyserial")
  install_package("numpy")
  install_package("pyqtgraph")

  # parser = argparse.ArgumentParser()
  # parser.add_argument("--log_file", nargs="?", default=None)
  # args = parser.parse_args()
  # FirmwareDebugger(log_file=args.log_file)
