function compare_gyro_measurements(timeline, data);
  figure(2);
  subplot(211);
  gyro0_ts = timeline_flatten_gyro0_ts(timeline);
  gyro0_data = timeline_flatten_gyro0_data(timeline);
  plot_measurements("Gyroscope Data",
                    gyro0_ts,
                    gyro0_data,
                    "rad s^-1");
  subplot(212);
  plot_measurements("Interpolated Gyroscope Data",
                    data.gyro0.ts,
                    data.gyro0.data,
                    "rad s^-1");
endfunction
