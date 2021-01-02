#!/usr/bin/env octave-cli
addpath(genpath("octave"));
graphics_toolkit("gnuplot");

% save_dir = "/tmp/mav_traj-2020-11-16-19-14-19";
% csv_file = "/tmp/mav_tf-2020-11-16-19-14-19.csv";

args = argv();
save_dir = args{1}
csv_file = args{2}

mkdir(save_dir);
csv = csvread(csv_file, 1, 0);

nb_poses = rows(csv);
r_WB = csv(1:end, 5:7)';
q_WB = csv(1:end, 8:11)';

poses = {};

for i = 1:nb_poses
  pos = r_WB(1:3, i);
  rot = quat2rot(q_WB(1:4, i));
  poses{i} = tf(rot, pos);
endfor

% for i = 1:10
for i = 1:nb_poses
  fname = sprintf("%s/traj%03d.png", save_dir, i);
  if exist(fname, "file")
    continue;
  endif

  figure("visible", "off");
  clf;
  hold on;
  draw_frame(poses{i}, 0.3);
  axis("equal");
  xlabel("x");
  ylabel("y");
  zlabel("z");
  xlim([-2.0, 2.0]);
  ylim([-2.0, 2.0]);
  zlim([0.0, 4.0]);
  view(3);

  printf("Saving plot [%s]\n", fname);
  print("-dpng", "-r100", fname);
endfor
% ginput;
