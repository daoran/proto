#!/usr/bin/octave
addpath(genpath("proto"));
addpath(genpath("notes"));
graphics_toolkit("fltk");

sim_data = sim_imu(0.5, 1.0);
start_idx = 10;
end_idx = start_idx + 1;

ts_i = sim_data.imu_time(start_idx);
ts_j = sim_data.imu_time(end_idx);

acc_i = sim_data.imu_acc(:, start_idx);
gyr_i = sim_data.imu_gyr(:, start_idx);
acc_j = sim_data.imu_acc(:, end_idx);
gyr_j = sim_data.imu_gyr(:, end_idx);

g = [0.0; 0.0; 9.81];

pose_i = sim_data.imu_poses{start_idx};
sb_i = [sim_data.imu_vel(:, start_idx); 1e-2 * eye(6, 1)];

pose_j = sim_data.imu_poses{end_idx};
sb_j = [sim_data.imu_vel(:, end_idx); 1e-2 * eye(6, 1)];

% Evaluate imu midpoint integration
r_i = zeros(3, 1);
v_i = zeros(3, 1);
C_i = eye(3, 3);
ba_i = sb_i(4:6);
bg_i = sb_i(7:9);
[F, r_j, v_j, C_j, ba_j, bg_j] = imu_midpoint_integrate(ts_i, ts_j,
                                                        acc_i, gyr_i,
                                                        acc_j, gyr_j,
                                                        r_i, v_i, C_i, ba_i, bg_i, g);

% Check jacobians
step_size = 1e-8;

% ----------------------------- CHECK dr JACOBIANS ---------------------------%
% -- Check dr w.r.t r_i
fdiff = zeros(3, 3);
for i = 1:3
  r_i_diff = r_i;
  r_i_diff(i) += step_size;
  [_, r_j_diff, _, _, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i_diff, v_i, C_i, ba_i, bg_i, g);
  fdiff(:, i) = (r_j_diff - r_j) / step_size;
endfor

if max(max(abs(F(1:3, 1:3) - fdiff))) > 1e-4
  printf("Failed [dr / r_i]\n");
  F(1:3, 1:3)
  fdiff
  F(1:3, 1:3) - fdiff
else
  printf("Passed [dr / r_i]\n");
endif

% -- Check dr jacobian w.r.t v_i
fdiff = zeros(3, 3);
for i = 1:3
  v_i_diff = v_i;
  v_i_diff(i) += step_size;
  [_, r_j_diff, _, _, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i, v_i_diff, C_i, ba_i, bg_i, g);
  fdiff(:, i) = (r_j_diff - r_j) / step_size;
endfor

if max(max(abs(F(1:3, 4:6) - fdiff))) > 1e-4
  printf("Failed [dr / v_i]\n");
  F(1:3, 4:6)
  fdiff
  F(1:3, 4:6) - fdiff
else
  printf("Passed [dr / v_i]\n");
endif

% -- Check dr jacobian w.r.t C_i
fdiff = zeros(3, 3);
for i = 1:3
  C_i_diff = perturb_rot(C_i, step_size, i);
  [_, r_j_diff, _, _, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i, v_i, C_i_diff, ba_i, bg_i, g);
  fdiff(:, i) = (r_j_diff - r_j) / step_size;
endfor

if max(max(abs(F(1:3, 7:9) - fdiff))) > 1e-3
  printf("Failed [dr / C_i]\n");
  F(1:3, 7:9)
  fdiff
  F(1:3, 7:9) - fdiff
else
  printf("Passed [dr / C_i]\n");
endif

% -- Check dr jacobian w.r.t ba_i
fdiff = zeros(3, 3);
for i = 1:3
  ba_i_diff = ba_i;
  ba_i_diff(i) += step_size;
  [_, r_j_diff, _, _, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i, v_i, C_i, ba_i_diff, bg_i, g);
  fdiff(:, i) = (r_j_diff - r_j) / step_size;
endfor

if max(max(abs(F(1:3, 10:12) - fdiff))) > 1e-4
  printf("Failed [dr / ba_i]\n");
  F(1:3, 10:12)
  fdiff
  F(1:3, 10:12) - fdiff
else
  printf("Passed [dr / ba_i]\n");
endif

% -- Check dr jacobian w.r.t bg_i
fdiff = zeros(3, 3);
for i = 1:3
  bg_i_diff = bg_i;
  bg_i_diff(i) += step_size;
  [_, r_j_diff, _, _, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i, v_i, C_i, ba_i, bg_i_diff, g);
  fdiff(:, i) = (r_j_diff - r_j) / step_size;
endfor

if max(max(abs(F(1:3, 13:15) - fdiff))) > 1e-4
  printf("Failed [dr / bg_i]\n");
  jac = F(1:3, 13:15)
  fdiff
  F(1:3, 13:15) - fdiff
else
  printf("Passed [dr / bg_i]\n");
endif

% ----------------------------- CHECK dv JACOBIANS ---------------------------%
% -- Check dv w.r.t r_i
fdiff = zeros(3, 3);
for i = 1:3
  r_i_diff = r_i;
  r_i_diff(i) += step_size;
  [_, _, v_j_diff, _, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i_diff, v_i, C_i, ba_i, bg_i, g);
  fdiff(:, i) = (v_j_diff - v_j) / step_size;
endfor

if max(max(abs(F(4:6, 1:3) - fdiff))) > 1e-4
  printf("Failed [dv / r_i]\n");
  jac = F(4:6, 1:3)
  fdiff
  F(4:6, 1:3) - fdiff
else
  printf("Passed [dv / r_i]\n");
endif

% -- Check dv jacobian w.r.t v_i
fdiff = zeros(3, 3);
for i = 1:3
  v_i_diff = v_i;
  v_i_diff(i) += step_size;
  [_, _, v_j_diff, _, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i, v_i_diff, C_i, ba_i, bg_i, g);
  fdiff(:, i) = (v_j_diff - v_j) / step_size;
endfor

if max(max(abs(F(4:6, 4:6) - fdiff))) > 1e-4
  printf("Failed [dv / v_i]\n");
  jac = F(4:6, 4:6)
  fdiff
  F(4:6, 4:6) - fdiff
else
  printf("Passed [dv / v_i]\n");
endif

% -- Check dv jacobian w.r.t C_i
fdiff = zeros(3, 3);
for i = 1:3
  C_i_diff = perturb_rot(C_i, step_size, i);
  [_, _, v_j_diff, _, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i, v_i, C_i_diff, ba_i, bg_i, g);
  fdiff(:, i) = (v_j_diff - v_j) / step_size;
endfor

if max(max(abs(F(4:6, 7:9) - fdiff))) > 1e-3
  printf("Failed [dv / C_i]\n");
  jac = F(4:6, 7:9)
  fdiff
  F(4:6, 7:9) - fdiff
else
  printf("Passed [dv / C_i]\n");
endif

% -- Check dv jacobian w.r.t ba_i
fdiff = zeros(3, 3);
for i = 1:3
  ba_i_diff = ba_i;
  ba_i_diff(i) += step_size;
  [_, _, v_j_diff, _, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i, v_i, C_i, ba_i_diff, bg_i, g);
  fdiff(:, i) = (v_j_diff - v_j) / step_size;
endfor

if max(max(abs(F(4:6, 10:12) - fdiff))) > 1e-4
  printf("Failed [dv / ba_i]\n");
  jac = F(4:6, 10:12)
  fdiff
  F(4:6, 10:12) - fdiff
else
  printf("Passed [dv / ba_i]\n");
endif

% -- Check dv jacobian w.r.t bg_i
fdiff = zeros(3, 3);
for i = 1:3
  bg_i_diff = bg_i;
  bg_i_diff(i) += step_size;
  [_, _, v_j_diff, _, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i, v_i, C_i, ba_i, bg_i_diff, g);
  fdiff(:, i) = (v_j_diff - v_j) / step_size;
endfor

if max(max(abs(F(4:6, 13:15) - fdiff))) > 1e-4
  printf("Failed [dv / bg_i]\n");
  jac = F(4:6, 13:15)
  fdiff
  F(4:6, 13:15) - fdiff
else
  printf("Passed [dv / bg_i]\n");
endif

% ----------------------------- CHECK dC JACOBIANS ---------------------------%
% -- Check dC w.r.t r_i
fdiff = zeros(3, 3);
for i = 1:3
  r_i_diff = r_i;
  r_i_diff(i) += step_size;
  [_, _, _, C_j_diff, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i_diff, v_i, C_i, ba_i, bg_i, g);
  fdiff(:, i) = Log(C_j' * C_j_diff) / step_size;
endfor

if max(max(abs(F(7:9, 1:3) - fdiff))) > 1e-4
  printf("Failed [dC / r_i]\n");
  jac = F(7:9, 1:3)
  fdiff
  F(7:9, 1:3) - fdiff
else
  printf("Passed [dC / r_i]\n");
endif

% -- Check dC jacobian w.r.t v_i
fdiff = zeros(3, 3);
for i = 1:3
  v_i_diff = v_i;
  v_i_diff(i) += step_size;
  [_, _, _, C_j_diff, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i, v_i_diff, C_i, ba_i, bg_i, g);
  fdiff(:, i) = Log(C_j' * C_j_diff) / step_size;
endfor

if max(max(abs(F(7:9, 4:6) - fdiff))) > 1e-4
  printf("Failed [dC / v_i]\n");
  jac = F(7:9, 4:6)
  fdiff
  F(7:9, 4:6) - fdiff
else
  printf("Passed [dC / v_i]\n");
endif

% -- Check dC jacobian w.r.t C_i
fdiff = zeros(3, 3);
for i = 1:3
  C_i_diff = perturb_rot(C_i, step_size, i);
  [_, _, _, C_j_diff, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i, v_i, C_i_diff, ba_i, bg_i, g);
  fdiff(:, i) = Log(C_j' * C_j_diff) / step_size;
endfor

if max(max(abs(F(7:9, 7:9) - fdiff))) > 1e-3
  printf("Failed [dC / C_i]\n");
  jac = F(7:9, 7:9)
  fdiff
  F(7:9, 7:9) - fdiff
else
  printf("Passed [dC / C_i]\n");
endif

% -- Check dC jacobian w.r.t ba_i
fdiff = zeros(3, 3);
for i = 1:3
  ba_i_diff = ba_i;
  ba_i_diff(i) += step_size;
  [_, _, _, C_j_diff, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i, v_i, C_i, ba_i_diff, bg_i, g);
  fdiff(:, i) = Log(C_j' * C_j_diff) / step_size;
endfor

if max(max(abs(F(7:9, 10:12) - fdiff))) > 1e-4
  printf("Failed [dC / ba_i]\n");
  jac = F(7:9, 10:12)
  fdiff
  F(7:9, 10:12) - fdiff
else
  printf("Passed [dC / ba_i]\n");
endif

% -- Check dC jacobian w.r.t bg_i
fdiff = zeros(3, 3);
for i = 1:3
  bg_i_diff = bg_i;
  bg_i_diff(i) += step_size;
  [_, _, _, C_j_diff, _, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                     acc_i, gyr_i,
                                                     acc_j, gyr_j,
                                                     r_i, v_i, C_i, ba_i, bg_i_diff, g);
  fdiff(:, i) = Log(C_j' * C_j_diff) / step_size;
endfor

if max(max(abs(F(7:9, 13:15) - fdiff))) > 1e-4
  printf("Failed [dC / bg_i]\n");
  jac = F(7:9, 13:15)
  fdiff
  F(7:9, 13:15) - fdiff
else
  printf("Passed [dC / bg_i]\n");
endif

% ----------------------------- CHECK dba JACOBIANS ---------------------------%
% -- Check dba w.r.t r_i
fdiff = zeros(3, 3);
for i = 1:3
  r_i_diff = r_i;
  r_i_diff(i) += step_size;
  [_, _, _, _, ba_j_diff, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                      acc_i, gyr_i,
                                                      acc_j, gyr_j,
                                                      r_i_diff, v_i, C_i, ba_i, bg_i, g);
  fdiff(:, i) = (ba_j_diff - ba_j) / step_size;
endfor

if max(max(abs(F(10:12, 1:3) - fdiff))) > 1e-4
  printf("Failed [dba / r_i]\n");
  jac = F(10:12, 1:3)
  fdiff
  F(10:12, 1:3) - fdiff
else
  printf("Passed [dba / r_i]\n");
endif

% -- Check dba jacobian w.r.t v_i
fdiff = zeros(3, 3);
for i = 1:3
  v_i_diff = v_i;
  v_i_diff(i) += step_size;
  [_, _, _, _, ba_j_diff, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                      acc_i, gyr_i,
                                                      acc_j, gyr_j,
                                                      r_i, v_i_diff, C_i, ba_i, bg_i, g);
  fdiff(:, i) = (ba_j_diff - ba_j) / step_size;
endfor

if max(max(abs(F(10:12, 4:6) - fdiff))) > 1e-4
  printf("Failed [dba / v_i]\n");
  jac = F(10:12, 4:6)
  fdiff
  F(10:12, 4:6) - fdiff
else
  printf("Passed [dba / v_i]\n");
endif

% -- Check dba jacobian w.r.t C_i
fdiff = zeros(3, 3);
for i = 1:3
  C_i_diff = perturb_rot(C_i, step_size, i);
  [_, _, _, _, ba_j_diff, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                      acc_i, gyr_i,
                                                      acc_j, gyr_j,
                                                      r_i, v_i, C_i_diff, ba_i, bg_i, g);
  fdiff(:, i) = (ba_j_diff - ba_j) / step_size;
endfor

if max(max(abs(F(10:12, 7:9) - fdiff))) > 1e-3
  printf("Failed [dba / C_i]\n");
  jac = F(10:12, 7:9)
  fdiff
  F(10:12, 7:9) - fdiff
else
  printf("Passed [dba / C_i]\n");
endif

% -- Check dba jacobian w.r.t ba_i
fdiff = zeros(3, 3);
for i = 1:3
  ba_i_diff = ba_i;
  ba_i_diff(i) += step_size;
  [_, _, _, _, ba_j_diff, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                      acc_i, gyr_i,
                                                      acc_j, gyr_j,
                                                      r_i, v_i, C_i, ba_i_diff, bg_i, g);
  fdiff(:, i) = (ba_j_diff - ba_j) / step_size;
endfor

if max(max(abs(F(10:12, 10:12) - fdiff))) > 1e-4
  printf("Failed [dba / ba_i]\n");
  jac = F(10:12, 10:12)
  fdiff
  F(10:12, 10:12) - fdiff
else
  printf("Passed [dba / ba_i]\n");
endif

% -- Check dba jacobian w.r.t bg_i
fdiff = zeros(3, 3);
for i = 1:3
  bg_i_diff = bg_i;
  bg_i_diff(i) += step_size;
  [_, _, _, _, ba_j_diff, _] = imu_midpoint_integrate(ts_i, ts_j,
                                                      acc_i, gyr_i,
                                                      acc_j, gyr_j,
                                                      r_i, v_i, C_i, ba_i, bg_i_diff, g);
  fdiff(:, i) = (ba_j_diff - ba_j) / step_size;
endfor

if max(max(abs(F(10:12, 13:15) - fdiff))) > 1e-4
  printf("Failed [dba / bg_i]\n");
  jac = F(10:12, 13:15)
  fdiff
  F(10:12, 13:15) - fdiff
else
  printf("Passed [dba / bg_i]\n");
endif

% ----------------------------- CHECK dbg JACOBIANS ---------------------------%
% -- Check dbg w.r.t r_i
fdiff = zeros(3, 3);
for i = 1:3
  r_i_diff = r_i;
  r_i_diff(i) += step_size;
  [_, _, _, _, _, bg_j_diff] = imu_midpoint_integrate(ts_i, ts_j,
                                                      acc_i, gyr_i,
                                                      acc_j, gyr_j,
                                                      r_i_diff, v_i, C_i, ba_i, bg_i, g);
  fdiff(:, i) = (bg_j_diff - bg_j) / step_size;
endfor

if max(max(abs(F(13:15, 1:3) - fdiff))) > 1e-4
  printf("Failed [dbg / r_i]\n");
  jac = F(13:15, 1:3)
  fdiff
  F(13:15, 1:3) - fdiff
else
  printf("Passed [dbg / r_i]\n");
endif

% -- Check dbg jacobian w.r.t v_i
fdiff = zeros(3, 3);
for i = 1:3
  v_i_diff = v_i;
  v_i_diff(i) += step_size;
  [_, _, _, _, _, bg_j_diff] = imu_midpoint_integrate(ts_i, ts_j,
                                                      acc_i, gyr_i,
                                                      acc_j, gyr_j,
                                                      r_i, v_i_diff, C_i, ba_i, bg_i, g);
  fdiff(:, i) = (bg_j_diff - bg_j) / step_size;
endfor

if max(max(abs(F(13:15, 4:6) - fdiff))) > 1e-4
  printf("Failed [dbg / v_i]\n");
  jac = F(13:15, 4:6)
  fdiff
  F(13:15, 4:6) - fdiff
else
  printf("Passed [dbg / v_i]\n");
endif

% -- Check dbg jacobian w.r.t C_i
fdiff = zeros(3, 3);
for i = 1:3
  C_i_diff = perturb_rot(C_i, step_size, i);
  [_, _, _, _, _, bg_j_diff] = imu_midpoint_integrate(ts_i, ts_j,
                                                      acc_i, gyr_i,
                                                      acc_j, gyr_j,
                                                      r_i, v_i, C_i_diff, ba_i, bg_i, g);
  fdiff(:, i) = (bg_j_diff - bg_j) / step_size;
endfor

if max(max(abs(F(13:15, 7:9) - fdiff))) > 1e-3
  printf("Failed [dbg / C_i]\n");
  jac = F(13:15, 7:9)
  fdiff
  F(13:15, 7:9) - fdiff
else
  printf("Passed [dbg / C_i]\n");
endif

% -- Check dbg jacobian w.r.t ba_i
fdiff = zeros(3, 3);
for i = 1:3
  ba_i_diff = ba_i;
  ba_i_diff(i) += step_size;
  [_, _, _, _, _, bg_j_diff] = imu_midpoint_integrate(ts_i, ts_j,
                                                      acc_i, gyr_i,
                                                      acc_j, gyr_j,
                                                      r_i, v_i, C_i, ba_i_diff, bg_i, g);
  fdiff(:, i) = (bg_j_diff - bg_j) / step_size;
endfor

if max(max(abs(F(13:15, 10:12) - fdiff))) > 1e-4
  printf("Failed [dbg / ba_i]\n");
  jac = F(13:15, 10:12)
  fdiff
  F(13:15, 10:12) - fdiff
else
  printf("Passed [dbg / ba_i]\n");
endif

% -- Check dbg jacobian w.r.t bg_i
fdiff = zeros(3, 3);
for i = 1:3
  bg_i_diff = bg_i;
  bg_i_diff(i) += step_size;
  [_, _, _, _, _, bg_j_diff] = imu_midpoint_integrate(ts_i, ts_j,
                                                      acc_i, gyr_i,
                                                      acc_j, gyr_j,
                                                      r_i, v_i, C_i, ba_i, bg_i_diff, g);
  fdiff(:, i) = (bg_j_diff - bg_j) / step_size;
endfor

if max(max(abs(F(13:15, 13:15) - fdiff))) > 1e-4
  printf("Failed [dbg / bg_i]\n");
  jac = F(13:15, 13:15)
  fdiff
  F(13:15, 13:15) - fdiff
else
  printf("Passed [dbg / bg_i]\n");
endif
