#!/bin/env octave -qf
graphics_toolkit("fltk");

% Simulate camera measurements
cam0_freq = 20.0;
cam0_dt = 1.0 / cam0_freq;
cam0_ts = 0:cam0_dt:1.0;

cam0_ts_ns = cam0_ts * 1e9;
cam0_file = fopen("/tmp/test_measurement-cam0.csv", "w");
for k = 1:length(cam0_ts)
  fprintf(cam0_file, "%d\n", uint64(cam0_ts_ns(k)));
endfor
fclose(cam0_file);

% Simulate gyroscope measurements
gyro_freq = 400.0;
gyro_dt = 1.0 / gyro_freq;
gyro_ts = 0:gyro_dt:1.0;
gyro_x = sin(2 * pi * gyro_ts) + 0.0;
gyro_y = sin(2 * pi * gyro_ts) + 0.1;
gyro_z = sin(2 * pi * gyro_ts) + 0.2;

gyro_ts_ns = gyro_ts * 1e9;
gyro_file = fopen("/tmp/test_measurement-gyro.csv", "w");
for k = 1:length(gyro_ts)
  fprintf(gyro_file,
          "%d,%f,%f,%f\n",
          uint64(gyro_ts_ns(k)),
          gyro_x(k),
          gyro_y(k),
          gyro_z(k));
endfor
fclose(gyro_file);

% Simulate accelerometer measurements
accel_freq = 250.0;
accel_dt = 1.0 / accel_freq;
accel_ts = 0:accel_dt:1.0;
accel_x = 10 * sin(2 * pi * accel_ts) + 0.0;
accel_y = 2 * sin(2 * pi * accel_ts) + 0.1;
accel_z = 2 * sin(2 * pi * accel_ts) + 0.2;

accel_ts_ns = accel_ts * 1e9;
accel_file = fopen("/tmp/test_measurement-accel.csv", "w");
for k = 1:length(accel_ts)
  fprintf(accel_file,
          "%d,%f,%f,%f\n",
          uint64(accel_ts_ns(k)),
          accel_x(k),
          accel_y(k),
          accel_z(k));
endfor
fclose(accel_file);

% % Plot measurements
% figure();
%
% subplot(211);
% hold on;
% plot(gyro_ts, gyro_x, "r-", "linewidth", 2);
% plot(gyro_ts, gyro_y, "g-", "linewidth", 2);
% plot(gyro_ts, gyro_z, "b-", "linewidth", 2);
% legend("x", "y", "z");
% title("Gyroscope measurements");
%
% subplot(212);
% hold on;
% plot(accel_ts, accel_x, "r-", "linewidth", 2);
% plot(accel_ts, accel_y, "g-", "linewidth", 2);
% plot(accel_ts, accel_z, "b-", "linewidth", 2);
% legend("x", "y", "z");
% title("Accelerometer measurements");
%
% ginput();
