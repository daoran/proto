addpath(genpath("proto"));

camera = camera_init([680; 480], 90.0);
calib_target = calib_target_init();
data = trajectory_simulate(camera, calib_target);

debug = false;
if debug
  figure(1);
  hold on;
  grid on;
  trajectory_plot(data);
  xlabel("x");
  ylabel("y");
  zlabel("z");
  axis "equal";
  view(3);
  ginput();
end
