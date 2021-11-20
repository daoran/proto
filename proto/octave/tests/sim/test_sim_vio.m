addpath(genpath("proto"));
graphics_toolkit("fltk");

% Simulate imu data
save_path = "/tmp/test_sim_vio.data";
if length(glob(save_path)) == 0
  circle_r = 0.5;
  circle_velocity = 1.0;
  sim_data = sim_vio(circle_r, circle_velocity)
  save("-binary", save_path, "sim_data");
else
  load(save_path);
endif


save_sim_vio(sim_data, "/tmp/sim_vio");


show_plots = 0;
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
