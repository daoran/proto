#!/usr/bin/octave-cli
script_path = fileparts(mfilename('fullpath'));
script_path = strcat(script_path, "/../..");
addpath(genpath(script_path));

% Settings
calib_data_dir = "/data/marker_calib";
cam0_csv = strcat(calib_data_dir, "/cam0/data.csv");
cam0_data_dir = strcat(calib_data_dir, "/cam0/data");


[cam0_ts, cam0_images] = textread(
        cam0_csv, ...
        "%f %s", ...
        "delimiter", ",", ...
        "headerlines", 1);
nb_cam0_images = rows(cam0_images);
for i = 1:nb_cam0_images
  cam0_images(i, 1) = strcat(cam0_data_dir, "/", cam0_images{i});
end


t0 = cam0_ts(1);
time = (cam0_ts - (t0 * ones(nb_cam0_images, 1))) * 1e-9;
for i = 2:nb_cam0_images
  t_prev = time(i - 1);
  t_now = time(i);
  diff = t_now - t_prev;
end


figure(1);
plot(time, ones(nb_cam0_images, 1), "o");
ginput();
