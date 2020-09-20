function plot_imu(data)
  figure();
  clf;

  subplot(211);
  hold on;
  plot(data.oxts.time, data.oxts.a_B(1, :), "r-", "linewidth", 2);
  plot(data.oxts.time, data.oxts.a_B(2, :), "g-", "linewidth", 2);
  plot(data.oxts.time, data.oxts.a_B(3, :), "b-", "linewidth", 2);
  title("Accelerometer");
  xlabel("x [m]");
  ylabel("y [m]");
  xlim([0, data.oxts.time(end)]);

  subplot(212);
  hold on;
  plot(data.oxts.time, data.oxts.w_B(1, :), "r-", "linewidth", 2);
  plot(data.oxts.time, data.oxts.w_B(2, :), "g-", "linewidth", 2);
  plot(data.oxts.time, data.oxts.w_B(3, :), "b-", "linewidth", 2);
  title("Gyroscope");
  xlabel("x [m]");
  ylabel("y [m]");
  xlim([0, data.oxts.time(end)]);
endfunction
