#!/usr/bin/octave
addpath(genpath("proto"));
addpath(genpath("notes"));
graphics_toolkit("fltk");

sim_data = sim_imu(0.5, 1.0);
% N = length(sim_data.time);
start_idx = 20;
N = 5;
imu_ts = sim_data.time(start_idx:start_idx+N);
imu_acc = sim_data.imu_acc(:, start_idx:start_idx+N);
imu_gyr = sim_data.imu_gyr(:, start_idx:start_idx+N);
g = [0.0; 0.0; 9.81];
pose_i = sim_data.poses{start_idx};
sb_i = [sim_data.vel(:, start_idx); 1e-2 * eye(6, 1)];
pose_j = sim_data.poses{start_idx+N};
sb_j = [sim_data.vel(:, start_idx+N); 2e-2 * eye(6, 1)];

% Evaluate imu factor
[r, jacs] = imu_factor_eval(imu_ts, imu_acc, imu_gyr, g, pose_i, sb_i, pose_j, sb_j);

% Check jacobians
step = 1e-8;

% -- Check jacobian w.r.t pose i (DONE!)
fdiff = zeros(15, 6);

for i = 1:6
  pose_i_diff = perturb_pose(pose_i, step / 2, i);
  [r_fwd, _] = imu_factor_eval(imu_ts, imu_acc, imu_gyr, g, pose_i_diff, sb_i, pose_j, sb_j);

  pose_i_diff = perturb_pose(pose_i, -step / 2, i);
  [r_bwd, _] = imu_factor_eval(imu_ts, imu_acc, imu_gyr, g, pose_i_diff, sb_i, pose_j, sb_j);

  fdiff(:, i) = (r_fwd - r_bwd) / step;
endfor

if max(max(abs(jacs{1} - fdiff))) > 1e-4
  jacs{1}
  fdiff
  jacs{1} - fdiff
else
  printf("J_pose_i Passed!\n");
endif

% -- Check jacobian w.r.t sb i
fdiff = zeros(15, 9);
for i = 1:9
  sb_i_diff = sb_i;
  sb_i_diff(i) += step;
  [r_diff, _] = imu_factor_eval(imu_ts, imu_acc, imu_gyr, g, pose_i, sb_i_diff, pose_j, sb_j);
  fdiff(:, i) = (r_diff - r) / step;
endfor

if max(max(abs(jacs{2} - fdiff))) > 1e-4
  jacs{2}
  fdiff
  jacs{2} - fdiff
else
  printf("J_sb_i Passed!\n");
endif

% -- Check jacobian w.r.t pose j
fdiff = zeros(15, 6);
for i = 1:6
  pose_j_diff = perturb_pose(pose_j, step, i);
  [r_diff, _] = imu_factor_eval(imu_ts, imu_acc, imu_gyr, g, pose_i, sb_i, pose_j_diff, sb_j);
  fdiff(:, i) = (r_diff - r) / step;
endfor

if max(max(abs(jacs{3} - fdiff))) > 1e-4
  jacs{3}
  fdiff
  jacs{3} - fdiff
else
  printf("J_pose_j Passed!\n");
endif

% -- Check jacobian w.r.t sb j (DONE!)
fdiff = zeros(15, 9);
for i = 1:9
  sb_j_diff = sb_j;
  sb_j_diff(i) += step;
  [r_diff, _] = imu_factor_eval(imu_ts, imu_acc, imu_gyr, g, pose_i, sb_i, pose_j, sb_j_diff);
  fdiff(:, i) = (r_diff - r) / step;
endfor

if max(max(abs(jacs{4} - fdiff))) > 1e-4
  printf("J_sb_j Failed!\n");
  jacs{4}
  fdiff
  jacs{4} - fdiff
else
  printf("J_sb_j Passed!\n");
endif
