addpath(genpath("proto"));
graphics_toolkit("fltk");

data_path = "/data/results/euroc/orbslam3/MH_01/stamped_traj_estimate.txt"
data_path2 = "/data/results/euroc/vins_mono/MH_01/stamped_traj_estimate.txt"

% T_CS = [
%   0.01496650221625856, 0.9996699324639489, -0.02088132991299131, 0.06866596603701511;
%   -0.9998849881448637, 0.015014414167788204, 0.002139591022748003, -0.015907823174948457;
%   0.0024524057488988452, 0.02084690611871645, 0.9997796718334099, -0.0024599422338853166;
%   0.0, 0.0, 0.0, 1.0
% ]

% T = eye(4);
% T(1:3, 1:3) = euler321([deg2rad(-90.0); deg2rad(0.0); deg2rad(-90.0)]);

T = eye(4);
T(1:3, 1:3) = quat2rot([0.534108,-0.153029,-0.827383,-0.082152]);

function data = load_data(file_path)
  fid = fopen(file_path, "r");
  fmt_str = "%s %f %f %f %f %f %f %f";
  data = textscan(fid, fmt_str, "delimiter", " ", "headerlines", 1);
  fclose(fid);
endfunction

function poses = load_poses(file_path)
  data = load_data(file_path);

  poses = {};
  for i = 1:length(data{1})
    r = [data{2}(i); data{3}(i); data{4}(i)];
    q = [data{8}(i); data{5}(i); data{6}(i); data{7}(i)];
    poses{i} = tf(q, r);
  endfor
endfunction

poses = load_poses(data_path);
poses2 = load_poses(data_path2);

% ts = [];
% timestamps = csv_data{1};
% for i = 1:length(csv_data{1})
%   ts = [ts; str2uint64(timestamps{i})];
% endfor


figure(1);
hold on;
for i = 1:10:1000
  draw_frame(poses{i});
endfor
xlabel("x");
ylabel("y");
zlabel("z");
axis equal
view(3);

figure(2);
hold on
for i = 1:10:1000
  draw_frame(poses2{i});
endfor
xlabel("x");
ylabel("y");
zlabel("z");
axis equal
view(3);

ginput();
