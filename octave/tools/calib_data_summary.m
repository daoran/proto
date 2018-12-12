#!/usr/bin/octave-cli
script_path = fileparts(mfilename('fullpath'));
script_path = strcat(script_path, "/../..");
addpath(genpath(script_path));

% Settings
calib_data_dir = "/tmp/calib/mono";
image_width = 752;
image_height = 480;


% Load AprilGrid data
keypoints = [];
fiducial_pos = [];
fiducial_rpy = [];
time = [];
ts0 = 0;
files = list_files(calib_data_dir);

for i = 1:length(files)
  fname = files(i).name;
  data_path = join_paths(calib_data_dir, fname);
  aprilgrid = load_aprilgrid(data_path);

  keypoints = [keypoints, aprilgrid.keypoints];
  fiducial_pos = [fiducial_pos, aprilgrid.r_WF];
  fiducial_rpy = [fiducial_rpy, rad2deg(quat2euler(aprilgrid.q_WF))];
  if ts0 == 0
    ts0 = aprilgrid.ts;
    time = [0.0];
  else
    time = [time, (aprilgrid.ts - ts0) * 1.0e-9];
  end
end


% Plot keypoints histogram
fig = figure(1);

subplot(2, 1, 1);
hist(keypoints(1, :), 50);
xlim([0, image_width]);
xlabel("x [px]");
ylabel("Frequency");

subplot(2, 1, 2);
hist(keypoints(2, :), 50);
xlim([0, image_height]);
xlabel("y [px]");
ylabel("Frequency");

ginput();


# % Plot position histogram
# figure(1);
# title("Position Histogram");
#
# subplot(3, 1, 1);
# hist(fiducial_pos(1, :), 50);
# xlabel("x [m]");
# ylabel("Frequency");
#
# subplot(3, 1, 2);
# hist(fiducial_pos(2, :), 50);
# xlabel("y [m]");
# ylabel("Frequency");
#
# subplot(3, 1, 3);
# hist(fiducial_pos(2, :), 50);
# xlabel("y [m]");
# ylabel("Frequency");
# ginput();
#
#
# % Plot position over time
# figure(1);
# title("Position");
#
# subplot(3, 1, 1);
# plot(time, fiducial_pos(1, :), "r", "linewidth", 2);
# xlim([0.0, max(time)]);
# xlabel("Time [s]");
# ylabel("x [m]");
#
# subplot(3, 1, 2);
# plot(time, fiducial_pos(2, :), "g", "linewidth", 2);
# xlim([0.0, max(time)]);
# xlabel("Time [s]");
# ylabel("y [m]");
#
# subplot(3, 1, 3);
# plot(time, fiducial_pos(2, :), "b", "linewidth", 2);
# xlim([0.0, max(time)]);
# xlabel("Time [s]");
# ylabel("z [m]");
# ginput();
#
#
# % Plot attitude histogram
# figure(1);
# title("Attitude Histogram");
#
# subplot(3, 1, 1);
# hist(fiducial_rpy(1, :), 18);
# xlabel("Roll [deg]");
# ylabel("Frequency");
#
# subplot(3, 1, 2);
# hist(fiducial_rpy(2, :), 18);
# xlabel("Pitch [deg]");
# ylabel("Frequency");
#
# subplot(3, 1, 3);
# hist(fiducial_rpy(2, :), 18);
# xlabel("Yaw [deg]");
# ylabel("Frequency");
# ginput();
#
#
# % Plot attitude over time
# figure(1);
# title("Attitude");
#
# subplot(3, 1, 1);
# plot(time, fiducial_rpy(1, :), "r", "linewidth", 2);
# xlim([0.0, max(time)]);
# xlabel("Time [s]");
# ylabel("x [m]");
#
# subplot(3, 1, 2);
# plot(time, fiducial_rpy(2, :), "g", "linewidth", 2);
# xlim([0.0, max(time)]);
# xlabel("Time [s]");
# ylabel("y [m]");
#
# subplot(3, 1, 3);
# plot(time, fiducial_rpy(2, :), "b", "linewidth", 2);
# xlim([0.0, max(time)]);
# xlabel("Time [s]");
# ylabel("z [m]");
# ginput();
