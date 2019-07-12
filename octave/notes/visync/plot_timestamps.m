function plot_timestamps(plot_title, cam0_ts, cam1_ts, accel0_ts, gyro0_ts)
  % -- Sensors
  cam0_plot = 0.8;
  cam1_plot = 0.6;
  accel0_plot = 0.4;
  gyro0_plot = 0.2;
  cam0_y = cam0_plot * ones(length(cam0_ts), 1);
  cam1_y = cam1_plot * ones(length(cam1_ts), 1);
  accel0_y = accel0_plot * ones(length(accel0_ts), 1);
  gyro0_y = gyro0_plot * ones(length(gyro0_ts), 1);
  % -- Plot sensor timestamps
  hold on;
  plot(cam0_ts, cam0_y, "rs", "linewidth", 2.0, "markerfacecolor", "r");
  plot(cam1_ts, cam1_y, "rs", "linewidth", 2.0, "markerfacecolor", "r");
  plot(accel0_ts, accel0_y, "bs", "linewidth", 2.0, "markerfacecolor", "b");
  plot(gyro0_ts, gyro0_y, "gs", "linewidth", 2.0, "markerfacecolor", "g");
  hold off;
  % -- Plot properties
  sensors = {"gyro0", "accel0", "cam1", "cam0"};
  y_tick = [gyro0_plot, accel0_plot, cam1_plot, cam0_plot];
  set(gca, "ytick", y_tick, "yticklabel", sensors);
  xlim([0, 0.20]);
  ylim([0.15, 0.85]);
  % ylim([0, 0.5]);
  title(plot_title);
  xlabel("Time [s]");
  ylabel("Sensors");
endfunction
