addpath(genpath("proto"));
graphics_toolkit("fltk");

% Calibration target
calib_target = calib_target_init();
C_WT = euler321(deg2rad([90.0, 0.0, -90.0]));
r_WT = [0.0; 0.0; 0.0];
T_WT = tf(C_WT, r_WT);

% Generate poses
[poses, calib_center] = calib_generate_poses(calib_target);
calib_center_W = dehomogeneous(T_WT * homogeneous(calib_center));
assert(size(calib_center) == [3, 1]);
assert(length(poses) > 1);

% Visualize
debug = false;
% debug = true;
if debug
  figure();
  hold on;

  calib_target_draw(calib_target, T_WT);
  for i = 1:length(poses)
    T_TC = poses{i};
    T_WC = T_WT * T_TC;
    draw_frame(T_WC, 0.04);
  endfor
  scatter3(calib_center_W(1), calib_center_W(2), calib_center_W(3));

  axis equal;
  view(3);
  xlabel("x [m]");
  ylabel("y [m]");
  zlabel("z [m]");
  ginput();
endif
