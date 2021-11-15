addpath(genpath("proto"));

sim_data = sim_imu(0.5, 1.0);
window_size = 10
g = [0.0; 0.0; 9.81];

i = 1;
start_idx = i;
end_idx = i + window_size - 1;

imu_buf = {};
imu_buf.ts = sim_data.imu_time(start_idx:end_idx);
imu_buf.acc = sim_data.imu_acc(:, start_idx:end_idx);
imu_buf.gyr = sim_data.imu_gyr(:, start_idx:end_idx);

imu_params = {};
imu_params.noise_acc = 0.08;    % accelerometer measurement noise stddev.
imu_params.noise_gyr = 0.004;   % gyroscope measurement noise stddev.
imu_params.noise_ba = 0.00004;  % accelerometer bias random work noise stddev.
imu_params.noise_bg = 2.0e-6;   % gyroscope bias random work noise stddev.

param_ids = [1, 2, 3, 4];
vel_i = sim_data.imu_vel(:, start_idx);
ba_i = 0.01 * ones(3, 1);
bg_i = 0.001 * ones(3, 1);
sb_i = sb_init(imu_buf.ts(1), vel_i, bg_i, ba_i);

factor_euler = imu_factor_init(param_ids, imu_buf, imu_params, sb_i, "euler");
factor_midpoint = imu_factor_init(param_ids, imu_buf, imu_params, sb_i, "midpoint");

compare_matrices("Euler", factor_euler.state_F, "Midpoint", factor_midpoint.state_F);
% compare_matrices("Euler", factor_euler.state_P, "Midpoint", factor_midpoint.state_P);
% compare_matrices("Euler", factor_euler.dr, "Midpoint", factor_midpoint.dr);
% compare_matrices("Euler", factor_euler.dv, "Midpoint", factor_midpoint.dv);
% compare_matrices("Euler", factor_euler.dC, "Midpoint", factor_midpoint.dC);
% compare_matrices("Euler", factor_euler.ba, "Midpoint", factor_midpoint.ba);
% compare_matrices("Euler", factor_euler.bg, "Midpoint", factor_midpoint.bg);
ginput();


% figure(1);
% subplot(211);
% imagesc(factor_euler.state_F);
% subplot(212);
% imagesc(factor_midpoint.state_F);
% ginput();

% assert(abs(factor_euler.dr - factor_midpoint.dr) < 1e-4);
% assert(abs(factor_euler.dv - factor_midpoint.dv) < 1e-4);
% assert(abs(max(max(factor_euler.dC - factor_midpoint.dC))) < 1e-4);
% assert(abs(factor_euler.ba - factor_midpoint.ba) < 1e-4);
% assert(abs(factor_euler.bg - factor_midpoint.bg) < 1e-4);
% assert(abs(factor_euler.g - factor_midpoint.g) < 1e-4);
% assert(abs(max(max(factor_euler.state_F - factor_midpoint.state_F))) < 1e-4);
% assert(abs(max(max(factor_euler.state_P - factor_midpoint.state_P))) < 1e-4);
