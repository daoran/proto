#!/usr/bin/octave-cli
script_path = fileparts(mfilename('fullpath'));
script_path = strcat(script_path, "/../..");
addpath(genpath(script_path));


% Settings
# calib_data_dir = "/data/euroc_mav/cam_april/";
# aprilgrid_file = "/data/euroc_mav/cam_april/aprilgrid/1403709383737837056.csv";
calib_data_dir = "/data/euroc_mav/imu_april";
aprilgrid_file = "/data/euroc_mav/imu_april/aprilgrid/cam0/1404733405732800000.csv";


% Load calibration data
data = load_euroc(calib_data_dir);
% -- Initialize world sensor pose
g = [0.0; 0.0; -9.81];
a_m = data.imu.a_RS_S(1:3, 1);
C_WS = quat2rot(vecs2quat(a_m, -g));
r_WS = [0.0; 0.0; 0.0];
T_WS = tf(C_WS, r_WS);
% -- Transform from sensor to cam0
T_SC0 = [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975;
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768;
         -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949;
         0.0, 0.0, 0.0, 1.0]


% Load aprilgrid data
[retval, aprilgrid] = load_aprilgrid(aprilgrid_file);
% -- Transform from camera to world frame
hp_C0F = [aprilgrid.points_CF; ones(1, columns(aprilgrid.points_CF))];
p_W = T_WS * T_SC0 * hp_C0F;


% Plot
for i = 1:10:data.cam0.nb_images
  % Show camera image
  figure(1);
  hold on;
  imshow(imread(data.cam0.image_paths{i}));

  % Show frames and AprilGrid
  figure(2);
  clf;
  hold on;
  draw_frame(T_WS, 0.2);          % Sensor frame
  draw_frame(T_WS * T_SC0, 0.2);  % Camera frame
  draw_points(p_W);
  view(3);
  axis("equal");
  xlabel("x");
  ylabel("y");
  zlabel("z");
  ginput();
end
