#!/usr/bin/octave-cli
script_path = fileparts(mfilename('fullpath'));
script_path = strcat(script_path, "/../..");
addpath(genpath(script_path));
graphics_toolkit("fltk");

% data_path = "/data/euroc_mav/imu_april/mav0/aprilgrid0/cam0/data";
% data_path = "/data/pointgrey/mono-1/grid0/cam0/data";
% data_path = "/data/pointgrey/mono-2/grid0/cam0/data";
data_path = "/data/pointgrey/mono-3/grid0/cam0/data";
paths = list_dir(data_path);
cam0_grids = [];
printf("loading aprilgrid data %s\n", data_path);
for i = 1:length(paths)
  csv_file = join_paths([data_path, "/", paths(i).name]);
  [retval, aprilgrid] = load_aprilgrid(csv_file);
  if retval == 0
    cam0_grids = [cam0_grids, aprilgrid];
  endif
endfor

% data_path = "/data/euroc_mav/imu_april/mav0/aprilgrid0/cam1/data";
% paths = list_dir(data_path);
% cam1_grids = [];
% printf("loading aprilgrid data %s\n", data_path);
% for i = 1:length(paths)
%   csv_file = join_paths([data_path, "/", paths(i).name]);
%   [retval, aprilgrid] = load_aprilgrid(csv_file);
%   if retval == 0
%     cam1_grids = [cam1_grids, aprilgrid];
%   endif
% endfor

cam0_kps = [];
for i = 1:length(cam0_grids)
  cam0_kps = [cam0_kps, cam0_grids(i).keypoints];
endfor

f = figure();
hold on;
plot(cam0_kps(1, :), cam0_kps(2, :), "r.", "linewidth", 5.0)
title("Measurement Distribution");
xlabel("Pixel Position [px]")
ylabel("Pixel Position [px]")
print(f, "/tmp/distribution.png", "-dpng");

f = figure();
hist(cam0_kps(1, :), 20);
title("Measurement Histogram: x-axis");
xlabel("Pixel Position [px]")
ylabel("Frequency")
print(f, "/tmp/hist-x_axis.png", "-dpng");

f = figure();
hist(cam0_kps(2, :), 20);
title("Measurement Histogram: y-axis");
xlabel("Pixel Position [px]")
ylabel("Frequency")
print(f, "/tmp/hist-y_axis.png", "-dpng");

% ginput();

% cam0_kps = [];
% for i = 1:length(cam0_grids)
%   cam0_kps = [cam0_kps, cam0_grids(i).keypoints];
% endfor
%
% cam1_kps = [];
% for i = 1:length(cam1_grids)
%   cam1_kps = [cam1_kps, cam1_grids(i).keypoints];
% endfor
%
% figure();
% hold on;
% subplot(121)
% plot(cam0_kps(1, :), cam0_kps(2, :), "ro", "linewidth", 5.0)
% subplot(122)
% plot(cam1_kps(1, :), cam1_kps(2, :), "b.", "linewidth", 5.0)
% ginput();
%
% cam0_pos = [];
% for i = 1:length(cam0_grids)
%   T_CF = cam0_grids(i).T_CF;
%   q_CF = tf_rot(T_CF);
%   r_CF = tf_trans(T_CF);
%   cam0_pos = [cam0_pos, r_CF];
% endfor
%
% cam1_pos = [];
% for i = 1:length(cam1_grids)
%   T_CF = cam1_grids(i).T_CF;
%   q_CF = tf_rot(T_CF);
%   r_CF = tf_trans(T_CF);
%   cam1_pos = [cam1_pos, r_CF];
% endfor
%
% figure();
% hold on;
% plot3(cam0_pos(1, :), cam0_pos(2, :), cam0_pos(3, :), "linewidth", 2.0);
% plot3(cam1_pos(1, :), cam1_pos(2, :), cam1_pos(3, :), "linewidth", 2.0);
% xlabel("x [m]");
% ylabel("y [m]");
% zlabel("z [m]");
% view(3)
% ginput();
