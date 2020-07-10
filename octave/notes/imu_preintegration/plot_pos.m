function plot_pos(data, traj_pos)
  figure();
  clf;

  subplot(4, 1, [3 1]);
  hold on;
  plot(traj_pos(1, :), traj_pos(2, :), 'b-', "linewidth", 2);
  plot(data.oxts.p_G(1, :), data.oxts.p_G(2, :), 'r-', "linewidth", 2);
  axis("equal");
  title("Position X-Y");
  xlabel("x [m]");
  ylabel("y [m]");
  legend("Estimated", "Ground Truth");

  subplot(4, 1, 4);
  hold on;
  plot(data.oxts.time, traj_pos(3, :), 'b-', "linewidth", 2);
  plot(data.oxts.time, data.oxts.p_G(1, :), 'r-', "linewidth", 2);
  xlim([0, data.oxts.time(end)]);
  title("Altitude");
  xlabel("time [s]");
  ylabel("z [m]");
  legend("Estimated", "Ground Truth");
endfunction
