addpath(genpath("proto"));
graphics_toolkit("fltk");

% Create camera
cam_idx = 0;
image_width = 640;
image_height = 480;
resolution = [image_width; image_height];
fov = 60.0;
fx = focal_length(image_width, fov);
fy = focal_length(image_height, fov);
cx = image_width / 2;
cy = image_height / 2;
proj_params = [fx; fy; cx; cy];
dist_params = [-0.01; 0.01; 1e-4; 1e-4];
camera = pinhole_radtan4_init(cam_idx, resolution, proj_params, dist_params);

% Calibration target
calib_target = calib_target_init(nb_rows=4, nb_cols=4, tag_size=0.2);

% Simulate trajectory
data = trajectory_simulate(camera, calib_target);

% debug = true;
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
