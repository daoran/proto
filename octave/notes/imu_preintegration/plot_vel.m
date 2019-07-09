function plot_vel(data, traj_vel)
  figure();
  clf;

  subplot(311);
  hold on;
  plot(data.oxts.time, traj_vel(1, :), 'b-', "linewidth", 2);
  plot(data.oxts.time, data.oxts.v_G(1, :), 'r-', "linewidth", 2);
  title("Velocity in x-direction");
  xlabel("time [s]");
  ylabel("velocity [m/s]");
  xlim([0, data.oxts.time(end)]);

  subplot(312);
  hold on;
  plot(data.oxts.time, traj_vel(2, :), 'b-', "linewidth", 2);
  plot(data.oxts.time, data.oxts.v_G(2, :), 'r-', "linewidth", 2);
  title("Velocity in y-direction");
  xlabel("time [s]");
  ylabel("velocity [m/s]");
  xlim([0, data.oxts.time(end)]);

  subplot(313);
  hold on;
  plot(data.oxts.time, traj_vel(3, :), 'b-', "linewidth", 2);
  plot(data.oxts.time, data.oxts.v_G(3, :), 'r-', "linewidth", 2);
  title("Velocity in z-direction");
  xlabel("time [s]");
  ylabel("velocity [m/s]");
  xlim([0, data.oxts.time(end)]);
endfunction
