function plot_imu(sim_data)
  figure();
  clf;

  subplot(211);
  hold on;
  plot(sim_data.time, sim_data.imu_acc(1, :), "r-", "linewidth", 2);
  plot(sim_data.time, sim_data.imu_acc(2, :), "g-", "linewidth", 2);
  plot(sim_data.time, sim_data.imu_acc(3, :), "b-", "linewidth", 2);
  title("Accelerometer");
  xlabel("Time [s]");
  ylabel("Acceleration [m s^-2]");
  xlim([0, sim_data.time(end)]);

  subplot(212);
  hold on;
  plot(sim_data.time, sim_data.imu_gyr(1, :), "r-", "linewidth", 2);
  plot(sim_data.time, sim_data.imu_gyr(2, :), "g-", "linewidth", 2);
  plot(sim_data.time, sim_data.imu_gyr(3, :), "b-", "linewidth", 2);
  title("Gyroscope");
  xlabel("Time [s]");
  ylabel("Angular Velocity [rad s^-1]");
  xlim([0, sim_data.time(end)]);
endfunction
