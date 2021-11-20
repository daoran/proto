addpath(genpath("proto"));
graphics_toolkit("fltk");

sim_data = sim_vo(0.5, 1.0);

show_plots = 1;
if show_plots
  % Plot scene
  figure();
  hold on;

  nb_poses = length(sim_data.cam_poses);
  interval = int32(nb_poses / 10);
  for k = 1:interval:nb_poses
    T_WS = sim_data.cam_poses{k};
    draw_frame(T_WS, 0.1);
  endfor

  features = sim_data.features;
  scatter3(features(:, 1), features(:, 2), features(:, 3), 'filled');

  axis('equal');
  xlabel('x [m]');
  ylabel('y [m]');
  zlabel('z [m]');
  view(3);

  ginput()
end
