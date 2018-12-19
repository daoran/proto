#!/usr/bin/octave-cli
script_path = fileparts(mfilename('fullpath'));
script_path = strcat(script_path, "/../..");
addpath(genpath(script_path));


% Load cloest marker pose csv
csv_path = "/tmp/marker0_timestamps.csv";
[ts, qw, qx, qy, qz, rx, ry, rz] = textread(
    csv_path, ...
    "%f %f %f %f %f %f %f %f", ...
    "delimiter", ",", ...
    "headerlines", 1 ...
);

% Form time [s]
time = [0];
t0 = ts(1);
for i = 2:rows(ts)
  time = [time; (ts(i) - t0) * 1e-9];
end

% Form quaternion and translation
q_WS = [qw, qx, qy, qz]';
r_WS = [rx, ry, rz]';

% Convert quaternion to roll pitch yaw
rpy_WS = [];
for i = 1:columns(q_WS)
  rpy_WS = [rpy_WS, rad2deg(quat2euler(q_WS(1:4, i)))];
end


% Load vicon marker pose csv
calib_data_dir = "/data/marker_calib";
marker0_csv = strcat(calib_data_dir, "/marker0/data.csv");
[ts, rx, ry, rz, qw, qx, qy, qz] = textread(
    marker0_csv, ...
    "%f %f %f %f %f %f %f %f", ...
    "delimiter", ",", ...
    "headerlines", 1 ...
);

% Form time [s]
time_vicon = [0];
t0 = ts(1);
for i = 2:rows(ts)
  time_vicon = [time_vicon; (ts(i) - t0) * 1e-9];
end

% Form quaternion and translation
q_WS_vicon = [qw, qx, qy, qz]';
r_WS_vicon = [rx, ry, rz]';

% Convert quaternion to roll pitch yaw
rpy_WS_vicon = [];
for i = 1:columns(q_WS_vicon)
  rpy_WS_vicon = [rpy_WS_vicon, rad2deg(quat2euler(q_WS_vicon(1:4, i)))];
end

% Plot translations
figure(1);
hold on;
plot(time(1:end), r_WS(1, 1:end), "r", "linewidth", 2);
plot(time(1:end), r_WS(2, 1:end), "g", "linewidth", 2);
plot(time(1:end), r_WS(3, 1:end), "b", "linewidth", 2);
legend("x", "y", "z");

% Plot rotations
figure(2);
hold on;
plot(time(1:end), rpy_WS(1, 1:end), "r", "linewidth", 2);
plot(time(1:end), rpy_WS(2, 1:end), "g", "linewidth", 2);
plot(time(1:end), rpy_WS(3, 1:end), "b", "linewidth", 2);
legend("x", "y", "z");
ginput();

% % Plot rotations
% figure(3);
% hold on;
% plot(time_vicon(1:end), rpy_WS_vicon(1, 1:end), "r", "linewidth", 2);
% plot(time(1:end), rpy_WS(1, 1:end), "g");
% % plot(time_vicon(1:end), rpy_WS_vicon(2, 1:end), "g", "linewidth", 2);
% % plot(time_vicon(1:end), rpy_WS_vicon(3, 1:end), "b", "linewidth", 2);
% legend("x", "y", "z");
% xlim([0, time_vicon(end)]);
% ginput();
