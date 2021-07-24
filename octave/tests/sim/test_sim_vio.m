addpath(genpath("proto"));
graphics_toolkit("fltk");

sim_data = sim_vio(0.5, 1.0);

show_plots = 1;
if show_plots
  % Plot scene
  figure();
  hold on;

  % Sensor and camera poses
  T_SC0 = sim_data.T_SC0;
  nb_poses = length(sim_data.timeline);
  interval = int32(nb_poses / 10);
  for k = 1:interval:nb_poses
    event = sim_data.timeline(k);
    T_WS = event.imu_pose;
    T_WC0 = T_WS * T_SC0;
    draw_frame(T_WS, 0.1);
    draw_frame(T_WC0, 0.1);
  endfor

  % Features
  features = sim_data.features;
  scatter3(features(:, 1), features(:, 2), features(:, 3), 'filled');

  % Plot settings
  axis('equal');
  xlabel('x [m]');
  ylabel('y [m]');
  zlabel('z [m]');
  view(3);

  ginput()
end
