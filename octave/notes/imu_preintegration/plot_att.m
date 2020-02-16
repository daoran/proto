function plot_att(data, traj_att)
  figure();
  clf;

  subplot(311);
  hold on;
  plot(data.oxts.time, rad2deg(traj_att(1, :)), 'b-', "linewidth", 2);
  plot(data.oxts.time, rad2deg(data.oxts.rpy(1, :)), 'r-', "linewidth", 2);
  title("Attitude in x");
  xlabel("time [s]");
  ylabel("Attitude [deg]");
  xlim([0, data.oxts.time(end)]);

  subplot(312);
  hold on;
  plot(data.oxts.time, rad2deg(traj_att(2, :)), 'b-', "linewidth", 2);
  plot(data.oxts.time, rad2deg(data.oxts.rpy(2, :)), 'r-', "linewidth", 2);
  title("Attitude in y");
  xlabel("time [s]");
  ylabel("Attitude [deg]");
  xlim([0, data.oxts.time(end)]);

  subplot(313);
  hold on;
  plot(data.oxts.time, rad2deg(traj_att(3, :)), 'b-', "linewidth", 2);
  plot(data.oxts.time, rad2deg(data.oxts.rpy(3, :)), 'r-', "linewidth", 2);
  title("Attitude in z");
  xlabel("time [s]");
  ylabel("Attitude [deg]");
  xlim([0, data.oxts.time(end)]);
endfunction
