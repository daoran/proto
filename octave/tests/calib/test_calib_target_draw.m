addpath(genpath("prototype"));

calib_target = calib_target_init();

C_WT = euler321(deg2rad([90.0, 0.0, -90.0]));
r_WT = zeros(3, 1);
T_WT = tf(C_WT, r_WT);

% Plot grid
debug = false;
% debug = true;
if debug == true
  figure();
  hold on;
  calib_target_draw(calib_target, T_WT)
  view(3);
  xlim([-0.5, 0.5]);
  ylim([-0.5, 0.5]);
  zlim([-0.5, 0.5]);
  ginput();
endif
