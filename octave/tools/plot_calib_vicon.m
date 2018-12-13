#!/usr/bin/octave-cli
script_path = fileparts(mfilename('fullpath'));
script_path = strcat(script_path, "/../..");
addpath(genpath(script_path));

% Settings
calib_data_dir = "/data/marker_calib";
aprilgrid_file = strcat(calib_data_dir, "/aprilgrid0/data/1544464038096008539.csv");
grid_csv = strcat(calib_data_dir, "/aprilgrid0/data.csv");
C_MC = euler321([deg2rad(-180.0), deg2rad(0.0), deg2rad(-90.0)]);
r_MC = [0.0; 0.0; 0.1];
T_MC = tf(C_MC, r_MC);

% Load calibration data and form marker poses in world frame
calib_data = load_calib(calib_data_dir);
T_WM = {};
for i = 1:calib_data.marker0.nb_measurements
  C_WM = quat2rot(calib_data.marker0.q_RS_S{i});
  r_WM = calib_data.marker0.p_RS_S{i};
  T_WM{i} = tf(C_WM, r_WM);
end

# origin = calib_data.marker0.p_RS_S{1};
origin = T_WM{1};
x_min = origin(1) - 5.0;
x_max = origin(1) + 5.0;
y_min = origin(2) - 5.0;
y_max = origin(2) + 5.0;
z_min = origin(3) - 5.0;
z_max = origin(3) + 5.0;

% Load aprilgrid data and intialize AprilGrid corner points in world frame
[_, aprilgrid] = load_aprilgrid(aprilgrid_file);
T_CF = aprilgrid.T_CF;
hp_CF = [aprilgrid.points_CF; ones(1, columns(aprilgrid.points_CF))];
points_W = T_WM{1} * T_MC * hp_CF;
T_WF = T_WM{1} * T_MC * T_CF;

% Load grid data
[grid_ts, grid_paths] = textread(grid_csv, ...
                                "%f %s", ...
                                "delimiter", ",", ...
                                "headerlines", 1);
grid_data = {};
grid_idx = 1;

for i = 1:rows(grid_paths)
  grid_paths{i} = strcat(calib_data_dir, "/aprilgrid0/data/", grid_paths{i});
  [retval, grid] = load_aprilgrid(grid_paths{i});
  if retval != -1
    grid_data{grid_idx} = grid;
    grid_idx += 1;
  end
end

t0 = min(grid_data{1}.ts, calib_data.marker0.ts(1));
r_CF = [];
grid_time = [];

for i = 1:columns(grid_data)
  grid_time = [grid_time, (grid_data{i}.ts - t0) * 1.0e-9];
  r_CF = [r_CF, tf_trans(grid_data{i}.T_CF)];
end

r_WM = [];
marker_time = [];
for i = 1:calib_data.marker0.nb_measurements
  marker_time = [marker_time, (calib_data.marker0.ts(i) - t0) * 1e-9];
  r_WM = [r_WM, calib_data.marker0.p_RS_S{i}];
end

r_CF_offset = r_CF(3, 1)
r_WM_offset = r_WM(3, 1)

% Plot time delay between camera and vicon
figure(1);
hold on;
plot(grid_time, r_CF(3, 1:end) - r_CF_offset, 'r', 'linewidth', 2.0);
plot(marker_time, r_WM(3, 1:end) - r_WM_offset, 'b', 'linewidth', 2.0);
ginput();

% Plot
% for i = 1:calib_data.marker0.nb_measurements
%   % Show camera image
%   figure(1);
%   hold on;
%   clf;
%   imshow(imread(calib_data.cam0.image_paths{i}));
%
%   % Show frames and AprilGrid
%   figure(2);
%   clf;
%   hold on;
%   draw_frame(T_WM{i});         % Marker frame
%   # draw_frame(T_WM{i} * T_MC);  % Camera frame
%   draw_points(points_W);
%   view(3);
%   axis("equal");
%   xlabel("x");
%   ylabel("y");
%   zlabel("z");
%   xlim([x_min, x_max])
%   ylim([y_min, y_max])
%   zlim([z_min, z_max])
%   ginput();
% end
