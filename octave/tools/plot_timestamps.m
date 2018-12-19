#!/usr/bin/octave-cli
script_path = fileparts(mfilename('fullpath'));
script_path = strcat(script_path, "/../..");
addpath(genpath(script_path));

% Settings
calib_data_dir = "/data/marker_calib";
cam0_csv = strcat(calib_data_dir, "/cam0/data.csv");
cam0_data_dir = strcat(calib_data_dir, "/cam0/data");
marker0_csv = strcat(calib_data_dir, "/marker0/data.csv");

% Load cam0 data
[cam0_ts, cam0_images] = textread(
        cam0_csv, ...
        "%f %s", ...
        "delimiter", ",", ...
        "headerlines", 1);
nb_cam0_images = rows(cam0_images);
for i = 1:nb_cam0_images
  cam0_images(i, 1) = strcat(cam0_data_dir, "/", cam0_images{i});
end

% Load marker0 data
% [marker0_ts, rx, ry, rz, qw, qx, qy, qz] = textread(
%     marker0_csv, ...
%     "%u64 %f %f %f %f %f %f %f", ...
%     "delimiter", ",", ...
%     "headerlines", 1 ...
% );
% % -- Form quaternion and translation
% q_WS_vicon = [qw, qx, qy, qz]';
% r_WS_vicon = [rx, ry, rz]';

fid = fopen(marker0_csv, "r");
marker0_csv
data = textscan(
  fid,
  "%s %f %f %f %f %f %f %f",
  "delimiter", ",",
  "headerlines", 1
);
fclose(fid);
% printf("%s\n", data{1}(1));
typeinfo(data{1}(1))
% printf("%f\n", data{1}(1));
printf("1544716471483808681");


% t0 = cam0_ts(1);
% time = (cam0_ts - (t0 * ones(nb_cam0_images, 1))) * 1e-9;
% diff = [];
% for i = 2:nb_cam0_images
%   t_prev = time(i - 1);
%   t_now = time(i);
%   diff = [diff; t_now - t_prev];
% end
% printf("cam0 ts mean diff: %f\n", mean(diff));
% printf("cam0 ts diff variance: %f\n", var(diff));

% t0 = marker0_ts(1);
% printf("%u64\n", t0);
% printf("1544716471483808681");
% nb_timestamps = rows(marker0_ts);
% time = (marker0_ts - (t0 * ones(nb_timestamps, 1))) * 1e-9;
% diff = [];
% for i = 2:nb_timestamps
%   t_prev = time(i - 1);
%   t_now = time(i);
%   diff = [diff; t_now - t_prev];
% end
% printf("marker0 ts mean diff: %f\n", mean(diff));
% printf("marker0 ts diff variance: %f\n", var(diff));

% figure(1);
% plot(time, ones(nb_cam0_images, 1), "o");
% ginput();
