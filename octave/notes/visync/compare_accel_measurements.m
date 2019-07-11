function compare_accel_measurements(timeline, data)
  figure(3);
  subplot(211);
  accel0_ts = timeline_flatten_accel0_ts(timeline);
  accel0_data = timeline_flatten_accel0_data(timeline);
  yunit = "ms^{-1}";
  plot_measurements("Accelerometer Data", accel0_ts, accel0_data, yunit);

  subplot(212);
  accel0_ts = data.accel0.ts;
  accel0_data = data.accel0.data;
  plot_measurements("Interpolated Accelerometer Data", accel0_ts, accel0_data, yunit);
endfunction
