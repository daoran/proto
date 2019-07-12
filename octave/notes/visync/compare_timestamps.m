function compare_timestamps(timeline, data)
  figure(1);
  subplot(211);
  plot_timestamps("Un-Synchronized Measurements",
                  timeline_flatten_cam0_ts(timeline),
                  timeline_flatten_cam1_ts(timeline),
                  timeline_flatten_accel0_ts(timeline),
                  timeline_flatten_gyro0_ts(timeline))
  subplot(212);
  plot_timestamps("Synchronized (Interpolated) Measurements",
                  data.cam0_ts,
                  data.cam1_ts,
                  data.accel0.ts,
                  data.gyro0.ts);
endfunction
