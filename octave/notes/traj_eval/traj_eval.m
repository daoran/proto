#!/bin/octave-cli
addpath(genpath("proto"));
graphics_toolkit("fltk");

est_data = "notes/traj_eval/MH_01_estimate.txt";
gnd_data = "notes/traj_eval/MH_01_groundtruth.txt";

% Parse data
fmt = "%f %f %f %f %f %f %f %f";
delim = " ";
[ts, rx, ry, rz, qx, qy, qz, qw] = textread(est_data, fmt,
                                            "delimiter", delim,
                                            "commentstyle", "shell");
est_quat = [qw, qx, qy, qz];
est_pos = [rx, ry, rz];

[gnd_ts, gnd_rx, gnd_ry, gnd_rz, gnd_qx, gnd_qy, gnd_qz, gnd_qw] = textread(
  gnd_data, fmt,
  "delimiter", delim,
  "commentstyle", "shell"
);

gnd_quat = [gnd_qw, gnd_qx, gnd_qy, gnd_qz];
gnd_pos = [gnd_rx, gnd_ry, gnd_rz];

figure(1);
hold on;
gnd_q = gnd_quat(1, 1:4)';
gnd_r = gnd_pos(1, 1:3)';
T = tf(gnd_q, gnd_r);
for k = 1:20:length(ts)
  est_q = est_quat(k, 1:4)';
  est_r = est_pos(k, 1:3)';
  est_pose = tf(est_q, est_r);
  draw_frame(T * est_pose, 0.3);
end
axis "equal";
xlabel("x");
ylabel("y");
zlabel("z");
view(3);

figure(2);
hold on;
for k = 1:200:length(gnd_ts)
  gnd_q = gnd_quat(k, 1:4)';
  gnd_r = gnd_pos(k, 1:3)';
  gnd_pose = tf(gnd_q, gnd_r);
  draw_frame(gnd_pose, 0.3);
end
axis "equal";
xlabel("x");
ylabel("y");
zlabel("z");
view(3);

ginput();
