#!/usr/bin/env python3
import sys
import time

import serial
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from PyQt5.QtCore import QTimer


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

    self.win_size = kwargs.get("win_size", 100)
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

  def update(self, data):
    """ Update """
    self.data[self.x_key].append(data[self.x_key])
    for y_key in self.y_keys:
      self.data[y_key].append(data[y_key])
      x_window = self.data[self.x_key][-self.win_size:]
      y_window = self.data[y_key][-self.win_size:]
      self.curves[y_key].setData(x_window, y_window)

      self.plot.setLimits(yMin=self.y_min, yMax=self.y_max)
      self.plot.setYRange(self.y_min, self.y_max, padding=self.y_padding)


class FirmwareDebugger:
  """ Firmware Debugger """
  def __init__(self):
    self.s = None  # Serial
    self.app = None
    self.win = None

    self._setup_uart()
    self._setup_gui()
    self._start_plots()

  def _setup_uart(self):
    """ Setup UART comms """
    # Setup serial communication
    self.serial = serial.Serial()
    self.serial.port = '/dev/ttyACM0'
    self.serial.baudrate = 115200
    self.serial.timeout = 10
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
    self.win.resize(1920, 600)
    pg.setConfigOptions(antialias=True)

    title = "SBUS"
    x_key = "ts"
    y_keys = ["ch[0]", "ch[1]", "ch[2]", "ch[3]"]
    x_label = "Time [s]"
    y_label = "SBUS Value"
    kwargs = {"y_min": 175, "y_max": 1850}
    self.sbus_plot = RTLinePlot(
        self.win,
        title,
        x_key,
        y_keys,
        x_label,
        y_label,
        **kwargs,
    )

    title = "Gyroscope"
    x_key = "ts"
    y_keys = ["gyro_x", "gyro_y", "gyro_z"]
    x_label = "Time [s]"
    y_label = "Angular Velocity [rad / s]"
    kwargs = {"y_min": -5.0, "y_max": 5.0}
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
    kwargs = {"y_min": -12.0, "y_max": 12.0}
    self.gyro_plot = RTLinePlot(
        self.win,
        title,
        x_key,
        y_keys,
        x_label,
        y_label,
        **kwargs,
    )

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
    kwargs = {"y_min": -60.0, "y_max": 60.0}
    self.attitude_plot = RTLinePlot(
        self.win,
        title,
        x_key,
        y_keys,
        x_label,
        y_label,
        **kwargs,
    )

    title = "Motor Outputs"
    x_key = "ts"
    y_keys = ["outputs[0]", "outputs[1]", "outputs[2]", "outputs[3]"]
    x_label = "Time [s]"
    y_label = "Motor Thrust [%]"
    kwargs = {"y_min": 0.0, "y_max": 1.0}
    self.motors_plot = RTLinePlot(
        self.win,
        title,
        x_key,
        y_keys,
        x_label,
        y_label,
        **kwargs,
    )

  def _start_plots(self):
    """ Start plots """
    timer = QTimer()
    timer.timeout.connect(self._update_plots)
    timer.start()
    pg.exec()

  def _update_plots(self):
    """ Update plots """
    data = self.parse_serial_data(self.serial.readline())
    data['ts'] = data['ts'] * 1e-6
    self.sbus_plot.update(data)
    self.gyro_plot.update(data)
    self.accel_plot.update(data)
    self.attitude_plot.update(data)
    self.motors_plot.update(data)

  @staticmethod
  def parse_serial_data(line):
    """ Parse serial data """
    line = line.strip()

    data = {}
    for el in line.split(b" "):
      key = el.split(b":")[0].strip().decode("ascii")
      val = float(el.split(b":")[1].strip())
      data[key] = val

    return data


if __name__ == "__main__":
  FirmwareDebugger()
