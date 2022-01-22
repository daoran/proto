function plot_poses(gnd, est=0)
  figure();
  clf;

  subplot(4, 1, [3 1]);
  hold on;
  plot(gnd.pos(1, :), gnd.pos(2, :), 'r-', "linewidth", 2);
  plot(est.pos(1, :), est.pos(2, :), 'r--', "linewidth", 2);
  axis("equal");
  title("Position X-Y");
  xlabel("x [m]");
  ylabel("y [m]");
  legend("Ground Truth", "Estimated");

  subplot(4, 1, 4);
  hold on;
  plot(gnd.time, gnd.att(3, :), 'r-', "linewidth", 2);
  plot(est.time, est.att(3, :), 'r--', "linewidth", 2);
  xlim([0, max(gnd.time(end), est.time(end))]);
  title("Altitude");
  xlabel("time [s]");
  ylabel("z [m]");
  legend("Ground Truth", "Estimated");
endfunction
